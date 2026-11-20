`timescale 1ns / 1ps

// =============================================================================
// tb_program
// Simulacion de un programa RV32I completo (121 instrucciones) sobre el
// procesador (core = fetch + DecExe) conectado a la memoria RTL del proyecto
// (HDL/memory.vhd). El programa se carga por el puerto A con el core en reset.
//
// El programa cubre todos los tipos de instruccion:
//   0x10000-0x100A8 : U (lui, auipc) + ALU tipo I y tipo R, firma XOR en x22 / x23
//   0x100AC-0x1010C : stores y loads (sw, sh, sb, lw, lh, lhu, lb, lbu), firma en x24
//   0x10110-0x10168 : bifurcaciones (beq, bne, blt, bge, bltu, bgeu). x26 cuenta
//                 las veces que se llega a "addi x26, x26, 1". Acaba en 1 en un
//                 procesador correcto: la bgeu de 0x10164 tiene offset +4 y tanto
//                 si salta como si no se ejecuta 0x10168
//   0x1016C-0x10188 : llamadas jal / retorno jalr. x27 cuenta los retornos (= 2)
//   0x1018C-0x101DC : casos limite (-1, -2048, 2047, desplazamientos de 31) y firma
//                 final en x28 (= 0x123552E9)
//   0x101E0     : halt (jal x0, 0)
//
// Verificacion:
//   1. Modelo de referencia (ISS) en el propio testbench. Antes de simular se
//      ejecuta el programa completo en el modelo (ejecucion "golden") para
//      obtener el estado final esperado de registros y memoria.
//   2. Comprobacion en lockstep: en cada ciclo se identifica la instruccion que
//      esta en el decoder (su PC es la direccion que la memoria muestreo en el
//      flanco anterior, ya que la lectura es sincrona) y se compara con el
//      modelo:
//        - flujo de control : el PC de la instruccion es el que toca
//        - decodificacion   : rdata1 / rdata2, wAlu, we, wl, we_b, be_b
//        - saltos           : pc_load y pc_target
//        - write-back       : valor escrito en rd y ninguna otra escritura
//        - memoria          : palabra escrita por los stores
//   3. Estado final: banco de registros completo y memoria de datos frente a
//      la ejecucion golden.
//
// Con RESYNC = 1, cuando el DUT se equivoca el modelo adopta el valor del DUT
// (registro destino, salvo que sea X, o PC) para que cada instruccion se juzgue
// por separado y un error no se propague a todas las siguientes. El estado final se compara
// siempre con la ejecucion golden (sin resincronizar).
//
// Mapa de memoria: la memoria se direcciona por palabras y el core genera
// direcciones de byte de ADDR_WIDTH = 17 bits. El programa se carga en PROG_BASE
// = 0x10000 (PC inicial del core) y coloca los datos en 0x10010000
// (auipc x20, 0x10000 + addi -176), que al truncar a 17 bits es 0x10000:
// los stores sobrescriben las primeras palabras del programa (ya ejecutadas).
// El modelo de referencia aplica el mismo truncado.
//
// DUT: por defecto HDL/core.vhd. Con +define+USE_CORE_AUX se usa core_aux.
// Ejecucion (desde la raiz del proyecto): do TEST/run_tb_program.do
// =============================================================================
module tb_program;

  localparam int MEM_WIDTH  = 32;
  localparam int NUM_BYTES  = 4;
  localparam int ADDR_REG   = 5;
  localparam int ADDR_WIDTH = 17;
  localparam int DEPTH      = 2**(ADDR_WIDTH-2);   // palabras accesibles con direcciones de byte

  localparam int  PROG_WORDS = 121;
  localparam logic [31:0] PROG_BASE = 32'h0001_0000;  // = PC_RESET del core: direccion de la primera instruccion
  localparam int  MAX_CYCLES = 600;   // limite de ciclos de simulacion
  localparam int  MAX_ERRORS = 200;   // se detiene la simulacion al alcanzar este numero de errores
  localparam int  LOAD_LAT   = 2;     // flancos maximos para que el dato de un load llegue a rd
  localparam int  STORE_LAT  = 3;     // flancos maximos para que un store llegue a la memoria
  localparam bit  RESYNC     = 1'b1;  // el modelo adopta el valor del DUT tras un error
  localparam bit  TRACE      = 1'b1;  // traza de cada instruccion ejecutada

  localparam logic [31:0] HALT = 32'h0000_006F;   // jal x0, 0

  // Opcodes RV32I (7 bits)
  localparam logic [6:0] OP_R      = 7'b0110011;
  localparam logic [6:0] OP_I      = 7'b0010011;
  localparam logic [6:0] OP_LOAD   = 7'b0000011;
  localparam logic [6:0] OP_STORE  = 7'b0100011;
  localparam logic [6:0] OP_BRANCH = 7'b1100011;
  localparam logic [6:0] OP_JAL    = 7'b1101111;
  localparam logic [6:0] OP_JALR   = 7'b1100111;
  localparam logic [6:0] OP_LUI    = 7'b0110111;
  localparam logic [6:0] OP_AUIPC  = 7'b0010111;

  // Reloj y reset
  logic clk;
  logic nRst;

  // Puertos del core
  logic [MEM_WIDTH-1:0]  rdata_a, rdata_b;
  logic [ADDR_WIDTH-1:0] addr_a,  addr_b;
  logic [MEM_WIDTH-1:0]  wdata_a, wdata_b;
  logic                  we_a,    we_b;
  logic [NUM_BYTES-1:0]  be_a,    be_b;

  // Puertos de la memoria: el testbench controla el puerto A durante la carga
  // del programa (core en reset) y el core los controla despues.
  // Memoria direccionada por palabras: direccion de byte -> addr[ADDR_WIDTH-1:2]
  logic                  load_en;
  logic [ADDR_WIDTH-1:0] tb_addr;
  logic [MEM_WIDTH-1:0]  tb_wdata;
  logic                  tb_we;
  logic [ADDR_WIDTH-1:0] mem_addr_a, mem_addr_b;
  logic [MEM_WIDTH-1:0]  mem_wdata_a, mem_wdata_b;
  logic                  mem_we_a, mem_we_b;
  logic [NUM_BYTES-1:0]  mem_be_a, mem_be_b;

  assign mem_addr_a  = load_en ? tb_addr  : {2'b00, addr_a[ADDR_WIDTH-1:2]};
  assign mem_wdata_a = load_en ? tb_wdata : wdata_a;
  assign mem_we_a    = load_en ? tb_we    : we_a;
  assign mem_be_a    = load_en ? 4'b1111  : be_a;
  assign mem_addr_b  = load_en ? '0       : {2'b00, addr_b[ADDR_WIDTH-1:2]};
  assign mem_wdata_b = load_en ? '0       : wdata_b;
  assign mem_we_b    = load_en ? 1'b0     : we_b;
  assign mem_be_b    = load_en ? '0       : be_b;

  // Monitorizacion jerarquica de DecExe
  wire [MEM_WIDTH-1:0]  mon_rdata1    = dut.U1.rdata1;
  wire [MEM_WIDTH-1:0]  mon_rdata2    = dut.U1.rdata2;
  wire [MEM_WIDTH-1:0]  mon_wAlu      = dut.U1.wAlu;
  wire                  mon_we        = dut.U1.we;          // escritura ALU -> banco de registros
  wire                  mon_wl        = dut.U1.wl;          // escritura memoria -> banco (LOAD)
  wire                  mon_we_mem    = dut.U1.aux_we_b;    // escritura en memoria (decoder)
  wire [NUM_BYTES-1:0]  mon_be_mem    = dut.U1.aux_be_b;    // byte enable de memoria (decoder)
  wire                  mon_pc_load   = dut.U1.pc_load;
  wire [ADDR_WIDTH-1:0] mon_pc_target = dut.U1.pc_target;

  // Instruccion valida en el decoder ('0' = burbuja). core_aux no anula instrucciones
`ifdef USE_CORE_AUX
  wire                  mon_valid     = 1'b1;
`else
  wire                  mon_valid     = dut.instr_valid;
`endif

  // PC de la instruccion presente en rdata_a: la memoria es sincrona, asi que
  // es la direccion que muestreo en el ultimo flanco de subida
  logic [31:0] dec_pc;
  always @(posedge clk) dec_pc <= {15'b0, mem_addr_a[ADDR_WIDTH-3:0], 2'b00};

  // ---------------------------------------------------------------------------
  // DUT y memoria
  // ---------------------------------------------------------------------------
`ifdef USE_CORE_AUX
  core_aux #(
`else
  core #(
`endif
    .MEM_WIDTH  (MEM_WIDTH),
    .NUM_BYTES  (NUM_BYTES),
    .ADDR_REG   (ADDR_REG),
    .ADDR_WIDTH (ADDR_WIDTH)
  ) dut (
    .clk     (clk),
    .nRst    (nRst),
    .rdata_a (rdata_a),
    .addr_a  (addr_a),
    .wdata_a (wdata_a),
    .we_a    (we_a),
    .be_a    (be_a),
    .rdata_b (rdata_b),
    .addr_b  (addr_b),
    .wdata_b (wdata_b),
    .we_b    (we_b),
    .be_b    (be_b)
  );

  // Memoria RTL del proyecto (HDL/memory.vhd)
  memory #(
    .MEM_WIDTH  (MEM_WIDTH),
    .NUM_BYTES  (NUM_BYTES),
    .ADDR_WIDTH (ADDR_WIDTH)
  ) u_mem (
    .clk     (clk),
    .addr_a  (mem_addr_a),
    .wdata_a (mem_wdata_a),
    .rdata_a (rdata_a),
    .we_a    (mem_we_a),
    .be_a    (mem_be_a),
    .addr_b  (mem_addr_b),
    .wdata_b (mem_wdata_b),
    .rdata_b (rdata_b),
    .we_b    (mem_we_b),
    .be_b    (mem_be_b)
  );

  // Reloj de 10 ns
  initial clk = 1'b0;
  always #5 clk = ~clk;

  // ---------------------------------------------------------------------------
  // Programa
  // ---------------------------------------------------------------------------
  logic [31:0] prog [0:PROG_WORDS-1];

  task automatic build_program();
    prog[  0] = 32'h00001A37; // 0x10000  lui x20, 0x00001
    prog[  1] = 32'h123455B7; // 0x10004  lui x11, 0x12345
    prog[  2] = 32'h00000197; // 0x10008  auipc x3, 0x00000
    prog[  3] = 32'h0035CAB3; // 0x1000C  xor x21, x11, x3
    prog[  4] = 32'h06400593; // 0x10010  addi x11, x0, 100
    prog[  5] = 32'h0C85A193; // 0x10014  slti x3, x11, 200
    prog[  6] = 32'h0325B213; // 0x10018  sltiu x4, x11, 50
    prog[  7] = 32'h03F5F293; // 0x1001C  andi x5, x11, 63
    prog[  8] = 32'h00F5E313; // 0x10020  ori x6, x11, 15
    prog[  9] = 32'h0FF5C393; // 0x10024  xori x7, x11, 255
    prog[ 10] = 32'h00259413; // 0x10028  slli x8, x11, 2
    prog[ 11] = 32'h0015D493; // 0x1002C  srli x9, x11, 1
    prog[ 12] = 32'h4015D513; // 0x10030  srai x10, x11, 1
    prog[ 13] = 32'h0035CB33; // 0x10034  xor x22, x11, x3
    prog[ 14] = 32'h004B4B33; // 0x10038  xor x22, x22, x4
    prog[ 15] = 32'h005B4B33; // 0x1003C  xor x22, x22, x5
    prog[ 16] = 32'h006B4B33; // 0x10040  xor x22, x22, x6
    prog[ 17] = 32'h007B4B33; // 0x10044  xor x22, x22, x7
    prog[ 18] = 32'h008B4B33; // 0x10048  xor x22, x22, x8
    prog[ 19] = 32'h009B4B33; // 0x1004C  xor x22, x22, x9
    prog[ 20] = 32'h00AB4B33; // 0x10050  xor x22, x22, x10
    prog[ 21] = 32'h06400593; // 0x10054  addi x11, x0, 100
    prog[ 22] = 32'hFF900193; // 0x10058  addi x3, x0, -7
    prog[ 23] = 32'h00100213; // 0x1005C  addi x4, x0, 1
    prog[ 24] = 32'h003582B3; // 0x10060  add x5, x11, x3
    prog[ 25] = 32'h40358333; // 0x10064  sub x6, x11, x3
    prog[ 26] = 32'h00B1A3B3; // 0x10068  slt x7, x3, x11
    prog[ 27] = 32'h00B1B433; // 0x1006C  sltu x8, x3, x11
    prog[ 28] = 32'h0035F4B3; // 0x10070  and x9, x11, x3
    prog[ 29] = 32'h0035E533; // 0x10074  or x10, x11, x3
    prog[ 30] = 32'h0035C5B3; // 0x10078  xor x11, x11, x3
    prog[ 31] = 32'h00459633; // 0x1007C  sll x12, x11, x4
    prog[ 32] = 32'h0045D6B3; // 0x10080  srl x13, x11, x4
    prog[ 33] = 32'h4041D733; // 0x10084  sra x14, x3, x4
    prog[ 34] = 32'h0062CBB3; // 0x10088  xor x23, x5, x6
    prog[ 35] = 32'h007BCBB3; // 0x1008C  xor x23, x23, x7
    prog[ 36] = 32'h008BCBB3; // 0x10090  xor x23, x23, x8
    prog[ 37] = 32'h009BCBB3; // 0x10094  xor x23, x23, x9
    prog[ 38] = 32'h00ABCBB3; // 0x10098  xor x23, x23, x10
    prog[ 39] = 32'h00BBCBB3; // 0x1009C  xor x23, x23, x11
    prog[ 40] = 32'h00CBCBB3; // 0x100A0  xor x23, x23, x12
    prog[ 41] = 32'h00DBCBB3; // 0x100A4  xor x23, x23, x13
    prog[ 42] = 32'h00EBCBB3; // 0x100A8  xor x23, x23, x14
    prog[ 43] = 32'h06400593; // 0x100AC  addi x11, x0, 100
    prog[ 44] = 32'h10000A17; // 0x100B0  auipc x20, 0x10000
    prog[ 45] = 32'hF50A0A13; // 0x100B4  addi x20, x20, -176
    prog[ 46] = 32'h00BA2023; // 0x100B8  sw x11, 0(x20)
    prog[ 47] = 32'h000A2183; // 0x100BC  lw x3, 0(x20)
    prog[ 48] = 32'h0AB00213; // 0x100C0  addi x4, x0, 171
    prog[ 49] = 32'h004A1223; // 0x100C4  sh x4, 4(x20)
    prog[ 50] = 32'h004A1283; // 0x100C8  lh x5, 4(x20)
    prog[ 51] = 32'h004A5303; // 0x100CC  lhu x6, 4(x20)
    prog[ 52] = 32'h004A0423; // 0x100D0  sb x4, 8(x20)
    prog[ 53] = 32'h008A0383; // 0x100D4  lb x7, 8(x20)
    prog[ 54] = 32'h008A4403; // 0x100D8  lbu x8, 8(x20)
    prog[ 55] = 32'h03700493; // 0x100DC  addi x9, x0, 55
    prog[ 56] = 32'h009A2623; // 0x100E0  sw x9, 12(x20)
    prog[ 57] = 32'h00CA2503; // 0x100E4  lw x10, 12(x20)
    prog[ 58] = 32'h0051CC33; // 0x100E8  xor x24, x3, x5
    prog[ 59] = 32'h006C4C33; // 0x100EC  xor x24, x24, x6
    prog[ 60] = 32'h007C4C33; // 0x100F0  xor x24, x24, x7
    prog[ 61] = 32'h008C4C33; // 0x100F4  xor x24, x24, x8
    prog[ 62] = 32'h00AC4C33; // 0x100F8  xor x24, x24, x10
    prog[ 63] = 32'h41800C33; // 0x100FC  sub x24, x0, x24
    prog[ 64] = 32'h04D00593; // 0x10100  addi x11, x0, 77
    prog[ 65] = 32'h00BA2823; // 0x10104  sw x11, 16(x20)
    prog[ 66] = 32'h010A2183; // 0x10108  lw x3, 16(x20)
    prog[ 67] = 32'h00018CB3; // 0x1010C  add x25, x3, x0
    prog[ 68] = 32'h00000D13; // 0x10110  addi x26, x0, 0
    prog[ 69] = 32'h06400593; // 0x10114  addi x11, x0, 100
    prog[ 70] = 32'hFF900193; // 0x10118  addi x3, x0, -7
    prog[ 71] = 32'h00100213; // 0x1011C  addi x4, x0, 1
    prog[ 72] = 32'h06400593; // 0x10120  addi x11, x0, 100
    prog[ 73] = 32'h00B58463; // 0x10124  beq x11, x11, 8 (-> 0x1012c)
    prog[ 74] = 32'h001D0D13; // 0x10128  addi x26, x26, 1
    prog[ 75] = 32'h02B59E63; // 0x1012C  bne x11, x11, 60 (-> 0x10168)
    prog[ 76] = 32'h00359463; // 0x10130  bne x11, x3, 8 (-> 0x10138)
    prog[ 77] = 32'h001D0D13; // 0x10134  addi x26, x26, 1
    prog[ 78] = 32'h00B1C463; // 0x10138  blt x3, x11, 8 (-> 0x10140)
    prog[ 79] = 32'h001D0D13; // 0x1013C  addi x26, x26, 1
    prog[ 80] = 32'h0235C463; // 0x10140  blt x11, x3, 40 (-> 0x10168)
    prog[ 81] = 32'h0035D463; // 0x10144  bge x11, x3, 8 (-> 0x1014c)
    prog[ 82] = 32'h001D0D13; // 0x10148  addi x26, x26, 1
    prog[ 83] = 32'h00B1DE63; // 0x1014C  bge x3, x11, 28 (-> 0x10168)
    prog[ 84] = 32'h00B26463; // 0x10150  bltu x4, x11, 8 (-> 0x10158)
    prog[ 85] = 32'h001D0D13; // 0x10154  addi x26, x26, 1
    prog[ 86] = 32'h0045E863; // 0x10158  bltu x11, x4, 16 (-> 0x10168)
    prog[ 87] = 32'h0045F463; // 0x1015C  bgeu x11, x4, 8 (-> 0x10164)
    prog[ 88] = 32'h001D0D13; // 0x10160  addi x26, x26, 1
    prog[ 89] = 32'h00B27263; // 0x10164  bgeu x4, x11, 4 (-> 0x10168)
    prog[ 90] = 32'h001D0D13; // 0x10168  addi x26, x26, 1
    prog[ 91] = 32'h00000D93; // 0x1016C  addi x27, x0, 0
    prog[ 92] = 32'h014000EF; // 0x10170  jal x1, 20 (-> 0x10184)
    prog[ 93] = 32'h001D8D93; // 0x10174  addi x27, x27, 1
    prog[ 94] = 32'h010000EF; // 0x10178  jal x1, 16 (-> 0x10188)
    prog[ 95] = 32'h001D8D93; // 0x1017C  addi x27, x27, 1
    prog[ 96] = 32'h00C0006F; // 0x10180  jal x0, 12 (-> 0x1018c)
    prog[ 97] = 32'h00008067; // 0x10184  jalr x0, 0(x1)
    prog[ 98] = 32'h00008067; // 0x10188  jalr x0, 0(x1)
    prog[ 99] = 32'hFFF00593; // 0x1018C  addi x11, x0, -1
    prog[100] = 32'h80000193; // 0x10190  addi x3, x0, -2048
    prog[101] = 32'h7FF00213; // 0x10194  addi x4, x0, 2047
    prog[102] = 32'hFFF00293; // 0x10198  addi x5, x0, -1
    prog[103] = 32'h41F2D293; // 0x1019C  srai x5, x5, 31
    prog[104] = 32'hFFF00313; // 0x101A0  addi x6, x0, -1
    prog[105] = 32'h01F35313; // 0x101A4  srli x6, x6, 31
    prog[106] = 32'h00100393; // 0x101A8  addi x7, x0, 1
    prog[107] = 32'h01F39393; // 0x101AC  slli x7, x7, 31
    prog[108] = 32'h0035CE33; // 0x101B0  xor x28, x11, x3
    prog[109] = 32'h004E4E33; // 0x101B4  xor x28, x28, x4
    prog[110] = 32'h005E4E33; // 0x101B8  xor x28, x28, x5
    prog[111] = 32'h006E4E33; // 0x101BC  xor x28, x28, x6
    prog[112] = 32'h007E4E33; // 0x101C0  xor x28, x28, x7
    prog[113] = 32'h417E0E33; // 0x101C4  sub x28, x28, x23
    prog[114] = 32'h018E0E33; // 0x101C8  add x28, x28, x24
    prog[115] = 32'h019E0E33; // 0x101CC  add x28, x28, x25
    prog[116] = 32'h01AE0E33; // 0x101D0  add x28, x28, x26
    prog[117] = 32'h01BE0E33; // 0x101D4  add x28, x28, x27
    prog[118] = 32'h016E0E33; // 0x101D8  add x28, x28, x22
    prog[119] = 32'h015E0E33; // 0x101DC  add x28, x28, x21
    prog[120] = 32'h0000006F; // 0x101E0  jal x0, 0 (-> 0x101e0)
  endtask

  // ---------------------------------------------------------------------------
  // Modelo de referencia (ISS RV32I)
  // ---------------------------------------------------------------------------
  logic [31:0] m_x   [0:31];      // banco de registros del modelo
  logic [31:0] m_mem [0:DEPTH-1]; // memoria del modelo (palabras)
  logic [31:0] m_pc;

  // Resultado esperado de una instruccion
  typedef struct {
    string       txt;        // desensamblado
    logic [4:0]  rs1, rs2, rd;
    bit          use_rs1, use_rs2;
    logic [31:0] v1, v2;     // valores esperados de rs1 / rs2
    bit          we;         // el decoder debe activar we (escritura desde la ALU)
    bit          wr_rd;      // la instruccion escribe un valor en rd (rd != 0)
    logic [31:0] rd_val;
    bit          chk_alu;    // comprobar wAlu
    logic [31:0] alu;
    bit          is_load, is_store, jump;
    logic [31:0] addr;       // direccion efectiva (load / store)
    int          idx;        // indice de palabra en memoria
    logic [3:0]  be;
    logic [31:0] st_word;    // palabra resultante tras el store
    logic [31:0] next_pc;
    bit          illegal;
  } exp_t;

  function automatic string rname(input logic [4:0] r);
    return $sformatf("x%0d", r);
  endfunction

  function automatic string disasm(input logic [31:0] pc, input logic [31:0] i);
    logic [31:0] imm_i, imm_s, imm_b, imm_j;
    string n;
    imm_i = {{20{i[31]}}, i[31:20]};
    imm_s = {{20{i[31]}}, i[31:25], i[11:7]};
    imm_b = {{19{i[31]}}, i[31], i[7], i[30:25], i[11:8], 1'b0};
    imm_j = {{11{i[31]}}, i[31], i[19:12], i[20], i[30:21], 1'b0};
    case (i[6:0])
      OP_LUI:   return $sformatf("lui   %s, 0x%05H", rname(i[11:7]), i[31:12]);
      OP_AUIPC: return $sformatf("auipc %s, 0x%05H", rname(i[11:7]), i[31:12]);
      OP_JAL:   return $sformatf("jal   %s, %0d (-> 0x%05H)", rname(i[11:7]), $signed(imm_j), pc + imm_j);
      OP_JALR:  return $sformatf("jalr  %s, %0d(%s)", rname(i[11:7]), $signed(imm_i), rname(i[19:15]));
      OP_BRANCH: begin
        case (i[14:12])
          3'b000: n = "beq "; 3'b001: n = "bne "; 3'b100: n = "blt ";
          3'b101: n = "bge "; 3'b110: n = "bltu"; 3'b111: n = "bgeu";
          default: n = "b???";
        endcase
        return $sformatf("%s  %s, %s, %0d (-> 0x%05H)", n, rname(i[19:15]), rname(i[24:20]), $signed(imm_b), pc + imm_b);
      end
      OP_LOAD: begin
        case (i[14:12])
          3'b000: n = "lb "; 3'b001: n = "lh "; 3'b010: n = "lw ";
          3'b100: n = "lbu"; 3'b101: n = "lhu"; default: n = "l??";
        endcase
        return $sformatf("%s   %s, %0d(%s)", n, rname(i[11:7]), $signed(imm_i), rname(i[19:15]));
      end
      OP_STORE: begin
        case (i[14:12])
          3'b000: n = "sb"; 3'b001: n = "sh"; 3'b010: n = "sw"; default: n = "s?";
        endcase
        return $sformatf("%s    %s, %0d(%s)", n, rname(i[24:20]), $signed(imm_s), rname(i[19:15]));
      end
      OP_I: begin
        case (i[14:12])
          3'b000: n = "addi "; 3'b001: n = "slli "; 3'b010: n = "slti "; 3'b011: n = "sltiu";
          3'b100: n = "xori "; 3'b101: n = i[30] ? "srai " : "srli "; 3'b110: n = "ori  ";
          default: n = "andi ";
        endcase
        if (i[13:12] == 2'b01)
          return $sformatf("%s %s, %s, %0d", n, rname(i[11:7]), rname(i[19:15]), i[24:20]);
        return $sformatf("%s %s, %s, %0d", n, rname(i[11:7]), rname(i[19:15]), $signed(imm_i));
      end
      OP_R: begin
        case (i[14:12])
          3'b000: n = i[30] ? "sub " : "add "; 3'b001: n = "sll "; 3'b010: n = "slt ";
          3'b011: n = "sltu"; 3'b100: n = "xor "; 3'b101: n = i[30] ? "sra " : "srl ";
          3'b110: n = "or  "; default: n = "and ";
        endcase
        return $sformatf("%s  %s, %s, %s", n, rname(i[11:7]), rname(i[19:15]), rname(i[24:20]));
      end
      default: return $sformatf("??? (0x%08H)", i);
    endcase
  endfunction

  function automatic logic [31:0] alu_op(input logic [2:0] f3, input bit alt,
                                         input logic [31:0] a, input logic [31:0] b);
    case (f3)
      3'b000: return alt ? a - b : a + b;
      3'b001: return a << b[4:0];
      3'b010: return ($signed(a) < $signed(b)) ? 32'd1 : 32'd0;
      3'b011: return (a < b) ? 32'd1 : 32'd0;
      3'b100: return a ^ b;
      3'b101: return alt ? 32'($signed(a) >>> b[4:0]) : a >> b[4:0];
      3'b110: return a | b;
      default: return a & b;
    endcase
  endfunction

  function automatic int word_idx(input logic [31:0] byte_addr);
    return int'(byte_addr[ADDR_WIDTH-1:2]);
  endfunction

  // Ejecuta una instruccion en el modelo (actualiza m_x y m_mem) y devuelve lo esperado
  function automatic exp_t iss_step(input logic [31:0] pc, input logic [31:0] i);
    exp_t        e;
    logic [31:0] imm_i, imm_s, imm_b, imm_u, imm_j, a, b, w;
    logic [2:0]  f3;
    logic [4:0]  sh;
    bit          taken;

    imm_i = {{20{i[31]}}, i[31:20]};
    imm_s = {{20{i[31]}}, i[31:25], i[11:7]};
    imm_b = {{19{i[31]}}, i[31], i[7], i[30:25], i[11:8], 1'b0};
    imm_u = {i[31:12], 12'b0};
    imm_j = {{11{i[31]}}, i[31], i[19:12], i[20], i[30:21], 1'b0};
    f3    = i[14:12];

    e = '{default: 0};
    e.txt     = disasm(pc, i);
    e.rs1     = i[19:15];
    e.rs2     = i[24:20];
    e.rd      = i[11:7];
    e.v1      = (e.rs1 == 0) ? 32'h0 : m_x[e.rs1];
    e.v2      = (e.rs2 == 0) ? 32'h0 : m_x[e.rs2];
    e.next_pc = pc + 4;
    a = e.v1;
    b = e.v2;

    case (i[6:0])
      OP_LUI: begin
        e.we = 1; e.rd_val = imm_u; e.chk_alu = 1; e.alu = imm_u;
      end
      OP_AUIPC: begin
        e.we = 1; e.rd_val = pc + imm_u;
      end
      OP_JAL: begin
        e.we = 1; e.rd_val = pc + 4; e.jump = 1; e.next_pc = pc + imm_j;
      end
      OP_JALR: begin
        e.use_rs1 = 1;
        e.we = 1; e.rd_val = pc + 4; e.jump = 1; e.next_pc = (a + imm_i) & ~32'd1;
      end
      OP_BRANCH: begin
        e.use_rs1 = 1; e.use_rs2 = 1;
        case (f3)
          3'b000:  taken = (a == b);
          3'b001:  taken = (a != b);
          3'b100:  taken = ($signed(a) <  $signed(b));
          3'b101:  taken = ($signed(a) >= $signed(b));
          3'b110:  taken = (a <  b);
          3'b111:  taken = (a >= b);
          default: begin taken = 0; e.illegal = 1; end
        endcase
        e.jump = taken;
        if (taken) e.next_pc = pc + imm_b;
      end
      OP_LOAD: begin
        e.use_rs1 = 1; e.is_load = 1;
        e.addr = a + imm_i; e.chk_alu = 1; e.alu = e.addr;
        e.idx  = word_idx(e.addr);
        w = m_mem[e.idx] >> (8 * e.addr[1:0]);
        case (f3)
          3'b000:  begin e.rd_val = {{24{w[7]}},  w[7:0]};  e.be = 4'b0001; end // lb
          3'b001:  begin e.rd_val = {{16{w[15]}}, w[15:0]}; e.be = 4'b0011; end // lh
          3'b010:  begin e.rd_val = w;                      e.be = 4'b1111; end // lw
          3'b100:  begin e.rd_val = {24'b0, w[7:0]};        e.be = 4'b0001; end // lbu
          3'b101:  begin e.rd_val = {16'b0, w[15:0]};       e.be = 4'b0011; end // lhu
          default: e.illegal = 1;
        endcase
      end
      OP_STORE: begin
        e.use_rs1 = 1; e.use_rs2 = 1; e.is_store = 1;
        e.addr = a + imm_s; e.chk_alu = 1; e.alu = e.addr;
        e.idx  = word_idx(e.addr);
        case (f3)
          3'b000:  e.be = 4'b0001;
          3'b001:  e.be = 4'b0011;
          3'b010:  e.be = 4'b1111;
          default: e.illegal = 1;
        endcase
        e.be = e.be << e.addr[1:0];
        w    = b << (8 * e.addr[1:0]);
        e.st_word = m_mem[e.idx];
        for (int k = 0; k < 4; k++)
          if (e.be[k]) e.st_word[8*k +: 8] = w[8*k +: 8];
        m_mem[e.idx] = e.st_word;
      end
      OP_I: begin
        e.use_rs1 = 1;
        sh = i[24:20];
        e.we = 1; e.chk_alu = 1;
        e.rd_val = alu_op(f3, (f3 == 3'b101) && i[30], a, (f3[1:0] == 2'b01) ? {27'b0, sh} : imm_i);
        e.alu = e.rd_val;
      end
      OP_R: begin
        e.use_rs1 = 1; e.use_rs2 = 1;
        e.we = 1; e.chk_alu = 1;
        e.rd_val = alu_op(f3, i[30], a, b);
        e.alu = e.rd_val;
      end
      default: e.illegal = 1;
    endcase

    e.wr_rd = (e.we || e.is_load) && (e.rd != 0);
    if (e.wr_rd) m_x[e.rd] = e.rd_val;
    return e;
  endfunction

  // El modelo parte del contenido real de memory.vhd tras la carga (programa
  // cargado + contenido inicial propio de la memoria, p. ej. su programa de
  // ejemplo en las palabras 0..9)
  task automatic model_reset();
    foreach (m_x[k])   m_x[k]   = 32'h0;
    for (int k = 0; k < DEPTH; k++) m_mem[k] = u_mem.mem[k];
    m_pc = PROG_BASE;
  endtask

  // Carga del programa por el puerto A de memory.vhd (el core esta en reset)
  task automatic load_memory();
    load_en = 1'b1;
    tb_we   = 1'b1;
    for (int k = 0; k < PROG_WORDS; k++) begin
      @(negedge clk);
      tb_addr  = word_idx(PROG_BASE) + k;
      tb_wdata = prog[k];
    end
    @(negedge clk);       // la ultima palabra se escribe en el flanco anterior
    tb_we   = 1'b0;
    load_en = 1'b0;
    for (int k = 0; k < PROG_WORDS; k++)
      if (u_mem.mem[word_idx(PROG_BASE) + k] !== prog[k])
        fail("CARGA", $sformatf("mem[0x%05H] = 0x%08H, esperado 0x%08H",
                                PROG_BASE + 4*k, u_mem.mem[word_idx(PROG_BASE) + k], prog[k]));
    $display("  [CARGA] %0d instrucciones escritas por el puerto A en 0x%05H..0x%05H",
             PROG_WORDS, PROG_BASE, PROG_BASE + 4*(PROG_WORDS-1));
  endtask

  // ---------------------------------------------------------------------------
  // Ejecucion golden: estado final esperado
  // ---------------------------------------------------------------------------
  logic [31:0] g_x [0:31];
  logic [31:0] g_mem [int];   // palabras escritas por los stores
  int          g_steps;
  logic [31:0] g_halt_pc;

  task automatic golden_run();
    exp_t        e;
    logic [31:0] i;
    model_reset();
    g_steps = 0;
    forever begin
      i = m_mem[word_idx(m_pc)];
      e = iss_step(m_pc, i);
      g_steps++;
      if (e.is_store) g_mem[e.idx] = 32'h0;
      if (i === HALT || e.illegal || g_steps >= MAX_CYCLES) break;
      m_pc = e.next_pc;
    end
    g_halt_pc = m_pc;
    foreach (m_x[k]) g_x[k] = m_x[k];
    foreach (g_mem[k]) g_mem[k] = m_mem[k];
    $display("  [GOLDEN] %0d instrucciones ejecutadas en el modelo, halt en PC = 0x%05H", g_steps, g_halt_pc);
    $display("  [GOLDEN] x26 (contador de bifurcaciones) = %0d | x27 (retornos jal/jalr) = %0d | x28 (firma) = 0x%08H",
             g_x[26], g_x[27], g_x[28]);
  endtask

  // ---------------------------------------------------------------------------
  // Registro de errores
  // ---------------------------------------------------------------------------
  int n_errors;
  int err_cnt [string];
  int n_checked;

  task automatic fail(input string cat, input string msg);
    n_errors++;
    if (err_cnt.exists(cat)) err_cnt[cat]++;
    else                     err_cnt[cat] = 1;
    $error("[%s] %s", cat, msg);
  endtask

  function automatic logic [31:0] dut_reg(input logic [4:0] r);
    return dut.U1.U0.regs[r];
  endfunction

  // Loads pendientes de escribir en rd (no cuentan como escritura inesperada)
  int pend_load [0:31];

  // ---------------------------------------------------------------------------
  // Comprobaciones en segundo plano: load (dato en rd) y store (memoria)
  // ---------------------------------------------------------------------------
  // El resultado se observa en el flanco que cierra el ciclo cyc_cnt - 1
  task automatic check_load(input logic [4:0] r, input logic [31:0] v, input logic [31:0] pc, input string txt,
                            input string cls, input int t_fetch);
    logic [31:0] got;
    for (int k = 0; k < LOAD_LAT; k++) begin
      @(posedge clk); #1;
      got = dut_reg(r);
      if (got === v) break;
    end
    if (got === v) add_lat(cls, cyc_cnt - 1 - t_fetch + 1);
    else           add_unmeasured(cls);
    #1 pend_load[r]--;
    if (got !== v)
      fail("LOAD", $sformatf("PC=0x%05H %s: %s esp 0x%08H, obt 0x%08H", pc, txt, rname(r), v, got));
  endtask

  task automatic check_store(input int idx, input logic [31:0] v, input logic [31:0] pc, input string txt,
                             input string cls, input int t_fetch);
    logic [31:0] got;
    for (int k = 0; k < STORE_LAT; k++) begin
      @(posedge clk); #1;
      got = u_mem.mem[idx];
      if (got === v) break;
    end
    if (got === v) add_lat(cls, cyc_cnt - 1 - t_fetch + 1);
    else           add_unmeasured(cls);
    if (got !== v)
      fail("STORE", $sformatf("PC=0x%05H %s: mem[0x%05H] esp 0x%08H, obt 0x%08H", pc, txt, idx << 2, v, got));
  endtask

  // ---------------------------------------------------------------------------
  // Medida de ciclos de reloj
  //   cyc_cnt       : ciclo actual (0 = primer ciclo tras soltar el reset)
  //   dec_fetch_cyc : ciclo en el que se presento en addr_a la direccion de la
  //                   instruccion que esta ahora en el decoder (fetch)
  //   latencia      : ciclos desde el fetch hasta que el resultado es visible
  //                   (registro escrito, memoria escrita o salto resuelto)
  //   burbujas      : ciclos anulados tras la instruccion (salto tomado)
  // ---------------------------------------------------------------------------
  int cyc_cnt = 0;
  int dec_fetch_cyc = 0;
  always @(posedge clk) begin
    dec_fetch_cyc <= cyc_cnt;
    if (nRst) cyc_cnt <= cyc_cnt + 1;
  end

  typedef struct {
    int n;          // instrucciones ejecutadas de este tipo
    int n_lat;      // instrucciones con latencia medida (resultado correcto)
    int lat_min, lat_max, lat_sum;
    int bub;        // burbujas provocadas
  } stat_t;
  stat_t  st [string];
  string  cls_order [$];
  int     n_bubbles;
  int     halt_cycle;

  function automatic string iclass(input logic [31:0] i, input bit taken);
    case (i[6:0])
      OP_R:      return "R (ALU)";
      OP_I:      return "I (ALU imm)";
      OP_LOAD:   return "LOAD";
      OP_STORE:  return "STORE";
      OP_BRANCH: return taken ? "BRANCH tomado" : "BRANCH no tomado";
      OP_JAL:    return "JAL";
      OP_JALR:   return "JALR";
      OP_LUI:    return "LUI";
      OP_AUIPC:  return "AUIPC";
      default:   return "ILEGAL";
    endcase
  endfunction

  task automatic stat_touch(input string cls);
    if (!st.exists(cls)) begin
      st[cls] = '{n: 0, n_lat: 0, lat_min: 1 << 30, lat_max: 0, lat_sum: 0, bub: 0};
      cls_order.push_back(cls);
    end
  endtask

  task automatic add_lat(input string cls, input int lat);
    stat_touch(cls);
    st[cls].n_lat++;
    st[cls].lat_sum += lat;
    if (lat < st[cls].lat_min) st[cls].lat_min = lat;
    if (lat > st[cls].lat_max) st[cls].lat_max = lat;
  endtask

  task automatic add_unmeasured(input string cls);
    stat_touch(cls);  // la instruccion cuenta en n pero sin latencia medida
  endtask

  task automatic report_cycles();
    int total;
    total = halt_cycle + 1;
    $display("\n--- CICLOS DE RELOJ ---");
    $display("  Ciclos totales (reset -> halt en el decoder): %0d", total);
    $display("  Instrucciones ejecutadas: %0d | burbujas: %0d | CPI = %0.2f",
             n_checked, n_bubbles, real'(total) / real'(n_checked));
    if (st.exists("RESET"))
      $display("  Burbujas tras el reset (la memoria aun no ha leido la primera instruccion): %0d", st["RESET"].bub);
    $display("  %-17s %5s %18s %12s %16s", "Tipo", "n", "latencia min/max", "burbujas", "ciclos/instr");
    foreach (cls_order[k]) begin
      string c;
      c = cls_order[k];
      if (st[c].n == 0) continue;
      if (st[c].n_lat > 0)
        $display("  %-17s %5d %13d / %-3d %12d %16.2f", c, st[c].n, st[c].lat_min, st[c].lat_max, st[c].bub,
                 1.0 + real'(st[c].bub) / real'(st[c].n));
      else
        $display("  %-17s %5d %18s %12d %16.2f", c, st[c].n, "no medida (falla)", st[c].bub,
                 1.0 + real'(st[c].bub) / real'(st[c].n));
    end
    $display("  latencia     = ciclos desde el fetch hasta que el resultado es visible");
    $display("  ciclos/instr = ciclos que ocupa el procesador (1 + burbujas que provoca)");
  endtask

  // ---------------------------------------------------------------------------
  // Ejecucion en lockstep: DUT frente al modelo, instruccion a instruccion
  // ---------------------------------------------------------------------------
  task automatic run_lockstep(output bit halted);
    exp_t        e;
    logic [31:0] pc, instr, got;
    logic [31:0] snap [0:31];
    int          err_before;
    int          t_fetch, t_dec;
    string       cls, last_cls;
    bit          lat_ok;

    halted   = 0;
    last_cls = "RESET";
    for (int cyc = 0; cyc < MAX_CYCLES; cyc++) begin
      #1; // propagacion combinacional (decoder -> regFile -> alu)
      pc         = dec_pc;
      instr      = rdata_a;
      err_before = n_errors;

      // --- flujo de control ---
      if ($isunknown(pc) || $isunknown(instr)) begin
        fail("FLUJO", $sformatf("ciclo %0d: PC (0x%08H) o instruccion (0x%08H) desconocidos", cyc, pc, instr));
        @(posedge clk); @(negedge clk);
        if (n_errors >= MAX_ERRORS) return;
        continue;
      end

      // --- burbuja: instruccion anulada (primer ciclo o tras un salto tomado) ---
      if (mon_valid === 1'b0) begin
        n_bubbles++;
        stat_touch(last_cls);
        st[last_cls].bub++;
        foreach (snap[k]) snap[k] = dut_reg(k);
        if (mon_we_mem !== 1'b0 || mon_pc_load !== 1'b0)
          fail("BURBUJA", $sformatf("ciclo %0d: la burbuja activa we_b=%b / pc_load=%b", cyc, mon_we_mem, mon_pc_load));
        @(posedge clk); #1;
        for (int k = 1; k < 32; k++)
          if (pend_load[k] == 0 && dut_reg(k) !== snap[k])
            fail("BURBUJA", $sformatf("ciclo %0d: la burbuja escribe x%0d", cyc, k));
        #2;
        if (TRACE)
          $display("[%4d] PC=0x%05H %08H  %-32s (tras %s)", cyc, pc, instr, "-- burbuja (anulada) --", last_cls);
        @(negedge clk);
        continue;
      end

      t_fetch = dec_fetch_cyc;
      t_dec   = cyc_cnt;
      if (pc !== m_pc) begin
        fail("FLUJO", $sformatf("ciclo %0d: se ejecuta PC=0x%05H (%s) pero tocaba PC=0x%05H",
                                cyc, pc, disasm(pc, instr), m_pc));
        m_pc = pc;  // siempre se resincroniza el PC
      end
      if (instr !== m_mem[word_idx(pc)])
        fail("FETCH", $sformatf("PC=0x%05H: instruccion leida 0x%08H, en memoria 0x%08H", pc, instr, m_mem[word_idx(pc)]));

      foreach (snap[k]) snap[k] = dut_reg(k);
      e = iss_step(pc, instr);
      n_checked++;
      cls = iclass(instr, e.jump);
      stat_touch(cls);
      st[cls].n++;
      if (e.illegal) fail("ILEGAL", $sformatf("PC=0x%05H: instruccion no soportada 0x%08H", pc, instr));

      // --- decodificacion / ejecucion ---
      if (e.use_rs1 && mon_rdata1 !== e.v1)
        fail("DECODE", $sformatf("PC=0x%05H %s: rdata1 (%s) esp 0x%08H, obt 0x%08H", pc, e.txt, rname(e.rs1), e.v1, mon_rdata1));
      if (e.use_rs2 && mon_rdata2 !== e.v2)
        fail("DECODE", $sformatf("PC=0x%05H %s: rdata2 (%s) esp 0x%08H, obt 0x%08H", pc, e.txt, rname(e.rs2), e.v2, mon_rdata2));
      if (e.chk_alu && mon_wAlu !== e.alu)
        fail("ALU", $sformatf("PC=0x%05H %s: wAlu esp 0x%08H, obt 0x%08H", pc, e.txt, e.alu, mon_wAlu));
      if (mon_we !== e.we)
        fail("DECODE", $sformatf("PC=0x%05H %s: we esp %b, obt %b", pc, e.txt, e.we, mon_we));
      if (mon_wl !== e.is_load)
        fail("DECODE", $sformatf("PC=0x%05H %s: wl esp %b, obt %b", pc, e.txt, e.is_load, mon_wl));
      if (mon_we_mem !== e.is_store)
        fail("DECODE", $sformatf("PC=0x%05H %s: we_b esp %b, obt %b", pc, e.txt, e.is_store, mon_we_mem));
      if (e.is_store && mon_be_mem !== e.be)
        fail("DECODE", $sformatf("PC=0x%05H %s: be_b esp %b, obt %b", pc, e.txt, e.be, mon_be_mem));

      // --- saltos ---
      if (mon_pc_load !== e.jump)
        fail("SALTO", $sformatf("PC=0x%05H %s: pc_load esp %b, obt %b", pc, e.txt, e.jump, mon_pc_load));
      if (e.jump && mon_pc_target !== e.next_pc[ADDR_WIDTH-1:0])
        fail("SALTO", $sformatf("PC=0x%05H %s: pc_target esp 0x%05H, obt 0x%05H", pc, e.txt, e.next_pc[ADDR_WIDTH-1:0], mon_pc_target));

      // --- comprobaciones con latencia (load / store) ---
      if (e.is_load && e.rd != 0) begin
        pend_load[e.rd]++;
        fork
          automatic logic [4:0]  f_rd  = e.rd;
          automatic logic [31:0] f_val = e.rd_val;
          automatic logic [31:0] f_pc  = pc;
          automatic string       f_txt = e.txt;
          automatic string       f_cls = cls;
          automatic int          f_tf  = t_fetch;
          check_load(f_rd, f_val, f_pc, f_txt, f_cls, f_tf);
        join_none
      end
      if (e.is_store) begin
        fork
          automatic int          f_idx = e.idx;
          automatic logic [31:0] f_val = e.st_word;
          automatic logic [31:0] f_pc  = pc;
          automatic string       f_txt = e.txt;
          automatic string       f_cls = cls;
          automatic int          f_tf  = t_fetch;
          check_store(f_idx, f_val, f_pc, f_txt, f_cls, f_tf);
        join_none
      end
      lat_ok = (n_errors == err_before);   // decodificacion y salto correctos hasta aqui

      // --- write-back ---
      @(posedge clk); #1;
      if (e.wr_rd && !e.is_load) begin
        got = dut_reg(e.rd);
        if (got !== e.rd_val) begin
          fail("WB", $sformatf("PC=0x%05H %s: %s esp 0x%08H, obt 0x%08H", pc, e.txt, rname(e.rd), e.rd_val, got));
          if (RESYNC && !$isunknown(got)) m_x[e.rd] = got;  // nunca se adopta un X
          lat_ok = 0;
        end
      end
      // Latencia de las instrucciones sin memoria: el resultado (rd o salto) se
      // resuelve en el flanco que cierra el ciclo de decodificacion
      if (!e.is_load && !e.is_store) begin
        if (lat_ok) add_lat(cls, t_dec - t_fetch + 1);
        else        add_unmeasured(cls);
      end
      last_cls = cls;
      for (int k = 1; k < 32; k++) begin
        if (e.wr_rd && k == e.rd) continue;
        if (pend_load[k] > 0) continue;
        if (dut_reg(k) !== snap[k])
          fail("WB", $sformatf("PC=0x%05H %s: escritura inesperada en x%0d (0x%08H -> 0x%08H)",
                               pc, e.txt, k, snap[k], dut_reg(k)));
      end

      #2; // deja terminar las comprobaciones de este flanco antes de la traza
      if (TRACE)
        $display("[%4d] PC=0x%05H %08H  %-32s%s%s", cyc, pc, instr, e.txt,
                 e.wr_rd ? $sformatf(" %s=0x%08H", rname(e.rd), e.rd_val) :
                 e.is_store ? $sformatf(" mem[0x%05H]=0x%08H", e.idx << 2, e.st_word) :
                 e.jump ? $sformatf(" -> 0x%05H", e.next_pc) : "",
                 (n_errors == err_before) ? "  OK" : "  <-- ERROR");

      if (instr === HALT) begin
        halted     = 1;
        halt_cycle = t_dec;
        return;
      end
      m_pc = e.next_pc;
      if (n_errors >= MAX_ERRORS) begin
        $display("  [STOP] Se alcanzan %0d errores", MAX_ERRORS);
        return;
      end
      @(negedge clk);
    end
  endtask

  // ---------------------------------------------------------------------------
  // Estado final frente a la ejecucion golden
  // ---------------------------------------------------------------------------
  task automatic check_final_state();
    logic [31:0] val;
    $display("------------------------------------------------------------------");
    repeat (STORE_LAT + 1) @(posedge clk);
    #1;
    for (int k = 0; k < 32; k++) begin
      val = dut_reg(k);
      $display("  x%-2d = 0x%08H (esp 0x%08H)%s", k, val, g_x[k], (val === g_x[k]) ? "" : "  <-- ERROR");
      if (val !== g_x[k])
        fail("FINAL", $sformatf("x%0d esp 0x%08H, obt 0x%08H", k, g_x[k], val));
    end
    foreach (g_mem[k]) begin
      val = u_mem.mem[k];
      $display("  mem[0x%05H] = 0x%08H (esp 0x%08H)%s", k << 2, val, g_mem[k], (val === g_mem[k]) ? "" : "  <-- ERROR");
      if (val !== g_mem[k])
        fail("FINAL", $sformatf("mem[0x%05H] esp 0x%08H, obt 0x%08H", k << 2, g_mem[k], val));
    end
  endtask

  // ---------------------------------------------------------------------------
  // Secuencia principal
  // ---------------------------------------------------------------------------
  initial begin
    bit halted;
    $display("=== INICIO SIMULACION PROGRAMA RV32I (%0d instrucciones) ===", PROG_WORDS);
    nRst      = 1'b0;
    n_errors  = 0;
    n_checked = 0;
    foreach (pend_load[k]) pend_load[k] = 0;

    load_en   = 1'b1;
    tb_we     = 1'b0;
    tb_addr   = '0;
    tb_wdata  = '0;

    // Carga del programa en memory.vhd, ejecucion golden y modelo en el estado inicial
    build_program();
    load_memory();
    golden_run();
    model_reset();

    repeat (3) @(posedge clk);
    @(negedge clk);
    nRst = 1'b1;

    $display("------------------------------------------------------------------");
    run_lockstep(halted);
    if (!halted)
      fail("FLUJO", $sformatf("el procesador no llega al halt (0x%05H) en %0d ciclos", g_halt_pc, MAX_CYCLES));

    $display("\n--- ESTADO FINAL ---");
    check_final_state();

    if (halted) report_cycles();

    $display("------------------------------------------------------------------");
    $display("  Instrucciones comprobadas: %0d (golden: %0d)", n_checked, g_steps);
    if (n_errors == 0)
      $display("=== SIMULACION COMPLETADA: SIN ERRORES ===");
    else begin
      $display("=== SIMULACION COMPLETADA: %0d ERRORES ===", n_errors);
      foreach (err_cnt[c]) $display("    %-7s : %0d", c, err_cnt[c]);
    end
    $finish;
  end

endmodule
