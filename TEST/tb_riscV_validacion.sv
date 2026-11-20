`timescale 1ns / 1ps

// =============================================================================
// tb_riscV_validacion
// Validacion completa del procesador RV32I (core = fetch + DecExe) conectado a
// la memoria RTL del proyecto (HDL/memory.vhd). Testbench unico y autocontenido:
// no necesita scripts, agentes ni ficheros externos.
//
// PLAN DE PRUEBAS: cinco programas, cada uno orientado a una funcionalidad.
//   P1 ENUNCIADO  : el programa de 121 instrucciones proporcionado (el mismo de
//                   tb_program). Comprueba los registros de firma x21..x28.
//   P2 ALU        : las 19 instrucciones aritmetico-logicas con registros e
//                   inmediatos con valores extremos (0, 1, -1, 0x7FFFFFFF,
//                   0x80000000, -2048, 2047), desbordamientos, desplazamientos
//                   de 0 y 31 bits (y 32 -> solo cuentan 5 bits), escrituras en
//                   x0 y cadenas de instrucciones dependientes.
//   P3 MEMORIA    : sw/sh/sb y lw/lh/lhu/lb/lbu en todos los desplazamientos de
//                   byte, extension de signo con 0x7F/0x80/0xFF, offsets
//                   negativos, store seguido de load, load seguido de uso
//                   inmediato (ALU, direccion de otro load, dato de un store,
//                   bifurcacion) y load sobre x0.
//   P4 SALTOS COND: beq/bne/blt/bge/bltu/bgeu tomadas y no tomadas con valores
//                   que distinguen comparacion con signo y sin signo e
//                   igualdad, anulacion de la instruccion siguiente, bucle hacia
//                   atras, bifurcacion dependiente de la instruccion anterior y
//                   saltos encadenados (un salto cuyo destino es otro salto).
//   P5 SALTOS INC.: jal/jalr (enlace usado inmediatamente, destino impar,
//                   rd = rs1, offset negativo, jal x0), subrutinas anidadas con
//                   llamadas hacia atras, lui/auipc con valores extremos.
//
// COMPROBACION AUTOMATICA (lo que hay que mirar: el resumen final)
//   - Programas autocomprobantes (P2..P5): cada prueba compara su resultado con
//     el valor esperado escrito en el propio plan de pruebas (macro CHK) y, si
//     no coincide, incrementa el contador de errores x29. Todos los resultados
//     se acumulan ademas en una firma (x31, XOR + rotacion), de modo que
//     cualquier error se propaga al resultado final. Al terminar, el programa
//     guarda la firma en SIG_ADDR y x29 en ERR_ADDR.
//     El testbench comprueba con aserciones: mem[ERR_ADDR] = 0 y
//     mem[SIG_ADDR] = EXP_SIG del programa.
//   - P1: aserciones sobre los registros de firma del enunciado (x21..x28).
//   - Ademas, como diagnostico, un modelo de referencia (ISS RV32I) dentro del
//     testbench compara ciclo a ciclo cada instruccion (lockstep) y el estado
//     final completo (32 registros + memoria escrita), mide los ciclos (CPI) y
//     la cobertura de instrucciones y de riesgos.
//
// Arquitectura de referencia: core monociclo con memoria sincrona; una burbuja
// tras cada salto tomado (instruccion siguiente anulada) y tras cada load.
// PC inicial = 0x10000 (PC_RESET del core). Datos en DATA_BASE = 0x1F000.
//
// Compilacion (desde la raiz del proyecto, QuestaSim / ModelSim):
//   vlib work
//   vcom -2008 HDL/alu.vhd
//   vcom -2002 HDL/divisor.vhd HDL/regFile.vhd HDL/decoder.vhd HDL/fetch.vhd HDL/DecExe.vhd HDL/core.vhd HDL/memory.vhd
//   vlog -sv TEST/tb_riscV_validacion.sv
//   vsim -c work.tb_riscV_validacion -do "run -all"
// =============================================================================
module tb_riscV_validacion;

  // ---------------------------------------------------------------------------
  // Parametros
  // ---------------------------------------------------------------------------
  localparam int MEM_WIDTH  = 32;
  localparam int NUM_BYTES  = 4;
  localparam int ADDR_REG   = 5;
  localparam int ADDR_WIDTH = 17;
  localparam int DEPTH      = 2**(ADDR_WIDTH-2);   // palabras accesibles con direcciones de byte

  localparam logic [31:0] PROG_BASE = 32'h0001_0000;  // PC_RESET del core
  localparam logic [31:0] DATA_BASE = 32'h0001_F000;  // zona de datos de P3 y resultados
  localparam logic [31:0] SIG_ADDR  = DATA_BASE + 32'h7F0;  // firma de cada programa
  localparam logic [31:0] ERR_ADDR  = DATA_BASE + 32'h7F4;  // contador de errores (x29)

  localparam int  N_PROGS    = 5;
  localparam int  MAX_CYCLES = 1500;  // limite de ciclos por programa
  localparam int  MAX_ERRORS = 100;   // errores del lockstep por programa antes de abortarlo
  localparam int  LOAD_LAT   = 2;     // flancos maximos para que el dato de un load llegue a rd
  localparam int  STORE_LAT  = 3;     // flancos maximos para que un store llegue a la memoria
  localparam bit  RESYNC     = 1'b1;  // tras un error el modelo adopta el valor del DUT
  localparam bit  TRACE      = 1'b0;  // 1: traza de cada instruccion ejecutada

  localparam logic [31:0] HALT = 32'h0000_006F;   // jal x0, 0

  // Firma esperada (x31 al final) de los programas autocomprobantes P2..P5.
  // Si se modifica un programa hay que actualizar su firma: el testbench avisa
  // (error PLAN) con el valor que calcula el modelo de referencia.
  localparam logic [31:0] EXP_SIG [0:N_PROGS-1] = '{
    32'h0000_0000,   // P1 (no usa firma en memoria: ver P1_X21..P1_X28)
    32'hE5A8_58F8,   // P2 ALU
    32'h5A65_6C32,   // P3 MEMORIA
    32'h0000_01EE,   // P4 SALTOS CONDICIONALES
    32'hC9BE_E20D    // P5 SALTOS INCONDICIONALES Y TIPO U
  };

  // Registros de firma esperados del programa del enunciado (P1)
  localparam logic [31:0] P1_X21 = 32'h1235_5008;
  localparam logic [31:0] P1_X22 = 32'h0000_0125;
  localparam logic [31:0] P1_X23 = 32'h7FFF_FF3F;
  localparam logic [31:0] P1_X24 = 32'h0000_00AD;
  localparam logic [31:0] P1_X25 = 32'h0000_004D;
  localparam logic [31:0] P1_X26 = 32'h0000_0001;
  localparam logic [31:0] P1_X27 = 32'h0000_0002;
  localparam logic [31:0] P1_X28 = 32'h1235_52E9;

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

  // funct3 de las bifurcaciones
  localparam logic [2:0] F_BEQ = 3'b000, F_BNE = 3'b001, F_BLT  = 3'b100,
                         F_BGE = 3'b101, F_BLTU = 3'b110, F_BGEU = 3'b111;

  // Registros reservados por los programas autocomprobantes
  localparam int R_BASE = 10;   // puntero a DATA_BASE
  localparam int R_EXP  = 28;   // valor esperado en CHK
  localparam int R_ERR  = 29;   // contador de errores
  localparam int R_TMP  = 30;   // temporal de la rotacion de la firma
  localparam int R_SIG  = 31;   // firma

  // ---------------------------------------------------------------------------
  // Reloj, reset y conexion core <-> memory.vhd
  // ---------------------------------------------------------------------------
  logic clk;
  logic nRst;

  logic [MEM_WIDTH-1:0]  rdata_a, rdata_b;
  logic [ADDR_WIDTH-1:0] addr_a,  addr_b;
  logic [MEM_WIDTH-1:0]  wdata_a, wdata_b;
  logic                  we_a,    we_b;
  logic [NUM_BYTES-1:0]  be_a,    be_b;

  // El testbench controla el puerto A durante la carga de cada programa (core en
  // reset); despues los dos puertos son del core. Memoria direccionada por
  // palabras: direccion de byte -> addr[ADDR_WIDTH-1:2]
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

  core #(
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

  initial clk = 1'b0;
  always #5 clk = ~clk;   // 10 ns

  // Monitorizacion jerarquica de DecExe y del core
  wire [MEM_WIDTH-1:0]  mon_rdata1    = dut.U1.rdata1;
  wire [MEM_WIDTH-1:0]  mon_rdata2    = dut.U1.rdata2;
  wire [MEM_WIDTH-1:0]  mon_wAlu      = dut.U1.wAlu;
  wire                  mon_we        = dut.U1.we;          // escritura ALU -> banco de registros
  wire                  mon_wl        = dut.U1.wl;          // load en ejecucion
  wire                  mon_we_mem    = dut.U1.aux_we_b;    // escritura en memoria (decoder)
  wire [NUM_BYTES-1:0]  mon_be_mem    = dut.U1.aux_be_b;    // byte enable de memoria (decoder)
  wire                  mon_pc_load   = dut.U1.pc_load;
  wire [ADDR_WIDTH-1:0] mon_pc_target = dut.U1.pc_target;
  wire                  mon_valid     = dut.instr_valid;    // '0' = burbuja

  function automatic logic [31:0] dut_reg(input logic [4:0] r);
    return dut.U1.U0.regs[r];
  endfunction

  // PC de la instruccion presente en rdata_a: la memoria es sincrona, asi que es
  // la direccion que muestreo en el ultimo flanco de subida
  logic [31:0] dec_pc;
  always @(posedge clk) dec_pc <= {15'b0, mem_addr_a[ADDR_WIDTH-3:0], 2'b00};

  // Contador de ciclos (0 = primer ciclo tras soltar el reset) y ciclo en el que
  // se presento en addr_a la instruccion que esta ahora en el decoder
  int cyc_cnt;
  int dec_fetch_cyc;
  always @(posedge clk or negedge nRst)
    if (!nRst) begin
      cyc_cnt       <= 0;
      dec_fetch_cyc <= 0;
    end else begin
      dec_fetch_cyc <= cyc_cnt;
      cyc_cnt       <= cyc_cnt + 1;
    end

  // ===========================================================================
  // ENSAMBLADOR: los programas se escriben con una tarea por instruccion
  // ===========================================================================
  logic [31:0] prog [$];
  string       prog_name;

  function automatic int here();                 // indice de la siguiente instruccion
    return prog.size();
  endfunction

  function automatic logic [31:0] pc_of(input int idx);  // direccion de la instruccion idx
    return PROG_BASE + 32'(4 * idx);
  endfunction

  task automatic emit(input logic [31:0] w);
    prog.push_back(w);
  endtask

  // Codificadores
  function automatic logic [31:0] enc_r(input logic [6:0] f7, input int rs2, input int rs1,
                                        input logic [2:0] f3, input int rd);
    return {f7, 5'(rs2), 5'(rs1), f3, 5'(rd), OP_R};
  endfunction
  function automatic logic [31:0] enc_i(input logic [31:0] imm, input int rs1, input logic [2:0] f3,
                                        input int rd, input logic [6:0] op);
    return {imm[11:0], 5'(rs1), f3, 5'(rd), op};
  endfunction
  function automatic logic [31:0] enc_s(input logic [31:0] imm, input int rs2, input int rs1,
                                        input logic [2:0] f3);
    return {imm[11:5], 5'(rs2), 5'(rs1), f3, imm[4:0], OP_STORE};
  endfunction
  function automatic logic [31:0] enc_b(input logic [31:0] imm, input int rs2, input int rs1,
                                        input logic [2:0] f3);
    return {imm[12], imm[10:5], 5'(rs2), 5'(rs1), f3, imm[4:1], imm[11], OP_BRANCH};
  endfunction
  function automatic logic [31:0] enc_u(input logic [19:0] imm20, input int rd, input logic [6:0] op);
    return {imm20, 5'(rd), op};
  endfunction
  function automatic logic [31:0] enc_j(input logic [31:0] imm, input int rd);
    return {imm[20], imm[10:1], imm[11], imm[19:12], 5'(rd), OP_JAL};
  endfunction

  // Tipo R
  task automatic ADD (input int rd, rs1, rs2); emit(enc_r(7'h00, rs2, rs1, 3'b000, rd)); endtask
  task automatic SUB (input int rd, rs1, rs2); emit(enc_r(7'h20, rs2, rs1, 3'b000, rd)); endtask
  task automatic SLL (input int rd, rs1, rs2); emit(enc_r(7'h00, rs2, rs1, 3'b001, rd)); endtask
  task automatic SLT (input int rd, rs1, rs2); emit(enc_r(7'h00, rs2, rs1, 3'b010, rd)); endtask
  task automatic SLTU(input int rd, rs1, rs2); emit(enc_r(7'h00, rs2, rs1, 3'b011, rd)); endtask
  task automatic XOR (input int rd, rs1, rs2); emit(enc_r(7'h00, rs2, rs1, 3'b100, rd)); endtask
  task automatic SRL (input int rd, rs1, rs2); emit(enc_r(7'h00, rs2, rs1, 3'b101, rd)); endtask
  task automatic SRA (input int rd, rs1, rs2); emit(enc_r(7'h20, rs2, rs1, 3'b101, rd)); endtask
  task automatic OR  (input int rd, rs1, rs2); emit(enc_r(7'h00, rs2, rs1, 3'b110, rd)); endtask
  task automatic AND (input int rd, rs1, rs2); emit(enc_r(7'h00, rs2, rs1, 3'b111, rd)); endtask
  // Tipo I (ALU)
  task automatic ADDI (input int rd, rs1, input logic [31:0] imm); emit(enc_i(imm, rs1, 3'b000, rd, OP_I)); endtask
  task automatic SLTI (input int rd, rs1, input logic [31:0] imm); emit(enc_i(imm, rs1, 3'b010, rd, OP_I)); endtask
  task automatic SLTIU(input int rd, rs1, input logic [31:0] imm); emit(enc_i(imm, rs1, 3'b011, rd, OP_I)); endtask
  task automatic XORI (input int rd, rs1, input logic [31:0] imm); emit(enc_i(imm, rs1, 3'b100, rd, OP_I)); endtask
  task automatic ORI  (input int rd, rs1, input logic [31:0] imm); emit(enc_i(imm, rs1, 3'b110, rd, OP_I)); endtask
  task automatic ANDI (input int rd, rs1, input logic [31:0] imm); emit(enc_i(imm, rs1, 3'b111, rd, OP_I)); endtask
  task automatic SLLI (input int rd, rs1, sh); emit(enc_i(32'(sh & 31),         rs1, 3'b001, rd, OP_I)); endtask
  task automatic SRLI (input int rd, rs1, sh); emit(enc_i(32'(sh & 31),         rs1, 3'b101, rd, OP_I)); endtask
  task automatic SRAI (input int rd, rs1, sh); emit(enc_i(32'h400 | (sh & 31),  rs1, 3'b101, rd, OP_I)); endtask
  // Loads y stores
  task automatic LB (input int rd, rs1, input logic [31:0] off); emit(enc_i(off, rs1, 3'b000, rd, OP_LOAD)); endtask
  task automatic LH (input int rd, rs1, input logic [31:0] off); emit(enc_i(off, rs1, 3'b001, rd, OP_LOAD)); endtask
  task automatic LW (input int rd, rs1, input logic [31:0] off); emit(enc_i(off, rs1, 3'b010, rd, OP_LOAD)); endtask
  task automatic LBU(input int rd, rs1, input logic [31:0] off); emit(enc_i(off, rs1, 3'b100, rd, OP_LOAD)); endtask
  task automatic LHU(input int rd, rs1, input logic [31:0] off); emit(enc_i(off, rs1, 3'b101, rd, OP_LOAD)); endtask
  task automatic SB (input int rs2, rs1, input logic [31:0] off); emit(enc_s(off, rs2, rs1, 3'b000)); endtask
  task automatic SH (input int rs2, rs1, input logic [31:0] off); emit(enc_s(off, rs2, rs1, 3'b001)); endtask
  task automatic SW (input int rs2, rs1, input logic [31:0] off); emit(enc_s(off, rs2, rs1, 3'b010)); endtask
  // Tipo U y saltos (offsets en bytes, relativos a la propia instruccion)
  task automatic LUI  (input int rd, input logic [19:0] imm20); emit(enc_u(imm20, rd, OP_LUI));   endtask
  task automatic AUIPC(input int rd, input logic [19:0] imm20); emit(enc_u(imm20, rd, OP_AUIPC)); endtask
  task automatic JAL  (input int rd, input logic [31:0] off);   emit(enc_j(off, rd));               endtask
  task automatic JALR (input int rd, rs1, input logic [31:0] off); emit(enc_i(off, rs1, 3'b000, rd, OP_JALR)); endtask
  task automatic BRANCH(input logic [2:0] f3, input int rs1, rs2, input logic [31:0] off);
    emit(enc_b(off, rs2, rs1, f3));
  endtask
  task automatic BEQ(input int rs1, rs2, input logic [31:0] off); BRANCH(F_BEQ, rs1, rs2, off); endtask
  task automatic BNE(input int rs1, rs2, input logic [31:0] off); BRANCH(F_BNE, rs1, rs2, off); endtask

  // Salto hacia delante: se emite con offset 0 y se completa con patch() en el destino
  task automatic patch(input int at);
    logic [31:0] w, off;
    w   = prog[at];
    off = 32'(4 * (here() - at));
    if (w[6:0] == OP_BRANCH) prog[at] = enc_b(off, w[24:20], w[19:15], w[14:12]);
    else                     prog[at] = enc_j(off, w[11:7]);
  endtask

  // Offset (bytes) desde la instruccion actual hasta la instruccion idx
  function automatic logic [31:0] to(input int idx);
    return 32'(4 * (idx - here()));
  endfunction

  // ---------------------------------------------------------------------------
  // Macros de los programas autocomprobantes
  // ---------------------------------------------------------------------------
  // rd <- v (lui + addi, o solo addi si cabe en 12 bits)
  task automatic LI(input int rd, input logic [31:0] v);
    logic [31:0] lo, hi;
    lo = {{20{v[11]}}, v[11:0]};
    hi = v - lo;
    if (hi != 0) begin
      LUI(rd, hi[31:12]);
      if (lo != 0) ADDI(rd, rd, lo);
    end else
      ADDI(rd, 0, lo);
  endtask

  // Acumula r en la firma: x31 = rotl(x31 ^ r, 1)
  task automatic ACC(input int r);
    XOR (R_SIG, R_SIG, r);
    SLLI(R_TMP, R_SIG, 1);
    SRLI(R_SIG, R_SIG, 31);
    OR  (R_SIG, R_SIG, R_TMP);
  endtask

  // Comprueba r == esperado (si no, x29++) y lo acumula en la firma
  int n_chk;   // pruebas CHK en el programa
  task automatic CHK(input int r, input logic [31:0] expected);
    LI (R_EXP, expected);
    BEQ(r, R_EXP, 8);
    ADDI(R_ERR, R_ERR, 1);
    ACC(r);
    n_chk++;
  endtask

  // Instruccion que nunca debe ejecutarse (camino anulado / destino erroneo)
  task automatic TRAP();
    ADDI(R_ERR, R_ERR, 1);
  endtask

  // Final de un programa autocomprobante: guarda firma y errores y se detiene
  task automatic END_PROGRAM();
    LUI(R_BASE, DATA_BASE[31:12]);
    SW (R_SIG, R_BASE, SIG_ADDR - DATA_BASE);
    SW (R_ERR, R_BASE, ERR_ADDR - DATA_BASE);
    JAL(0, 0);                        // halt
  endtask

  // ===========================================================================
  // PROGRAMAS
  // ===========================================================================

  // ---------------------------------------------------------------------------
  // P1: programa del enunciado (121 instrucciones, el mismo de tb_program)
  // ---------------------------------------------------------------------------
  task automatic build_p1_enunciado();
    logic [31:0] w [0:120] = '{
      32'h00001A37, 32'h123455B7, 32'h00000197, 32'h0035CAB3, 32'h06400593, 32'h0C85A193, 32'h0325B213, 32'h03F5F293,
      32'h00F5E313, 32'h0FF5C393, 32'h00259413, 32'h0015D493, 32'h4015D513, 32'h0035CB33, 32'h004B4B33, 32'h005B4B33,
      32'h006B4B33, 32'h007B4B33, 32'h008B4B33, 32'h009B4B33, 32'h00AB4B33, 32'h06400593, 32'hFF900193, 32'h00100213,
      32'h003582B3, 32'h40358333, 32'h00B1A3B3, 32'h00B1B433, 32'h0035F4B3, 32'h0035E533, 32'h0035C5B3, 32'h00459633,
      32'h0045D6B3, 32'h4041D733, 32'h0062CBB3, 32'h007BCBB3, 32'h008BCBB3, 32'h009BCBB3, 32'h00ABCBB3, 32'h00BBCBB3,
      32'h00CBCBB3, 32'h00DBCBB3, 32'h00EBCBB3, 32'h06400593, 32'h10000A17, 32'hF50A0A13, 32'h00BA2023, 32'h000A2183,
      32'h0AB00213, 32'h004A1223, 32'h004A1283, 32'h004A5303, 32'h004A0423, 32'h008A0383, 32'h008A4403, 32'h03700493,
      32'h009A2623, 32'h00CA2503, 32'h0051CC33, 32'h006C4C33, 32'h007C4C33, 32'h008C4C33, 32'h00AC4C33, 32'h41800C33,
      32'h04D00593, 32'h00BA2823, 32'h010A2183, 32'h00018CB3, 32'h00000D13, 32'h06400593, 32'hFF900193, 32'h00100213,
      32'h06400593, 32'h00B58463, 32'h001D0D13, 32'h02B59E63, 32'h00359463, 32'h001D0D13, 32'h00B1C463, 32'h001D0D13,
      32'h0235C463, 32'h0035D463, 32'h001D0D13, 32'h00B1DE63, 32'h00B26463, 32'h001D0D13, 32'h0045E863, 32'h0045F463,
      32'h001D0D13, 32'h00B27263, 32'h001D0D13, 32'h00000D93, 32'h014000EF, 32'h001D8D93, 32'h010000EF, 32'h001D8D93,
      32'h00C0006F, 32'h00008067, 32'h00008067, 32'hFFF00593, 32'h80000193, 32'h7FF00213, 32'hFFF00293, 32'h41F2D293,
      32'hFFF00313, 32'h01F35313, 32'h00100393, 32'h01F39393, 32'h0035CE33, 32'h004E4E33, 32'h005E4E33, 32'h006E4E33,
      32'h007E4E33, 32'h417E0E33, 32'h018E0E33, 32'h019E0E33, 32'h01AE0E33, 32'h01BE0E33, 32'h016E0E33, 32'h015E0E33,
      32'h0000006F };
    prog_name = "P1 ENUNCIADO";
    foreach (w[k]) emit(w[k]);
  endtask

  // ---------------------------------------------------------------------------
  // P2: ALU (tipo R y tipo I) con valores extremos y cadenas dependientes
  // ---------------------------------------------------------------------------
  task automatic build_p2_alu();
    prog_name = "P2 ALU";
    // Operandos extremos
    LI(1, 32'h7FFF_FFFF);           // x1 = MAX
    LI(2, 32'h8000_0000);           // x2 = MIN
    ADDI(3, 0, -1);                 // x3 = -1
    ADDI(4, 0, 1);                  // x4 = 1
    LI(5, 32'h1234_5678);           // x5 = patron
    ADDI(6, 0, 31);                 // x6 = 31
    ADDI(7, 0, 32);                 // x7 = 32 (desplaza 0: solo cuentan 5 bits)
    ADDI(8, 0, -31);                // x8 = 0xFFFFFFE1 (desplaza 1)

    // add / sub: desbordamiento y signo
    ADD (11, 1, 4);   CHK(11, 32'h8000_0000);   // MAX + 1 = MIN
    ADD (11, 2, 3);   CHK(11, 32'h7FFF_FFFF);   // MIN - 1 = MAX
    SUB (11, 2, 4);   CHK(11, 32'h7FFF_FFFF);   // MIN - 1
    SUB (11, 0, 2);   CHK(11, 32'h8000_0000);   // -MIN = MIN
    SUB (11, 4, 3);   CHK(11, 32'd2);           // 1 - (-1)
    // desplazamientos con registro
    SLL (11, 4, 6);   CHK(11, 32'h8000_0000);
    SLL (11, 5, 7);   CHK(11, 32'h1234_5678);   // 32 -> 0
    SLL (11, 5, 8);   CHK(11, 32'h2468_ACF0);   // 0xFFFFFFE1 -> 1
    SRL (11, 2, 6);   CHK(11, 32'h0000_0001);
    SRL (11, 3, 4);   CHK(11, 32'h7FFF_FFFF);
    SRA (11, 2, 6);   CHK(11, 32'hFFFF_FFFF);
    SRA (11, 1, 6);   CHK(11, 32'h0000_0000);
    SRA (11, 5, 7);   CHK(11, 32'h1234_5678);
    // comparaciones con y sin signo
    SLT (11, 2, 1);   CHK(11, 32'd1);           // MIN < MAX
    SLT (11, 1, 2);   CHK(11, 32'd0);
    SLT (11, 3, 0);   CHK(11, 32'd1);           // -1 < 0
    SLT (11, 4, 4);   CHK(11, 32'd0);           // iguales
    SLTU(11, 2, 1);   CHK(11, 32'd0);           // 0x80000000 > 0x7FFFFFFF sin signo
    SLTU(11, 0, 3);   CHK(11, 32'd1);           // 0 < 0xFFFFFFFF
    SLTU(11, 3, 3);   CHK(11, 32'd0);
    SLTU(11, 0, 4);   CHK(11, 32'd1);           // snez
    // logicas
    XOR (11, 5, 3);   CHK(11, 32'hEDCB_A987);   // not
    OR  (11, 2, 1);   CHK(11, 32'hFFFF_FFFF);
    AND (11, 5, 2);   CHK(11, 32'h0000_0000);
    AND (11, 3, 5);   CHK(11, 32'h1234_5678);

    // tipo I: inmediatos extremos (-2048 y 2047)
    ADDI (11, 1, 1);      CHK(11, 32'h8000_0000);
    ADDI (11, 0, -2048);  CHK(11, 32'hFFFF_F800);
    ADDI (11, 0, 2047);   CHK(11, 32'h0000_07FF);
    ADDI (11, 3, 1);      CHK(11, 32'h0000_0000);
    SLTI (11, 2, 0);      CHK(11, 32'd1);
    SLTI (11, 1, -1);     CHK(11, 32'd0);
    SLTI (11, 3, -1);     CHK(11, 32'd0);       // iguales
    SLTIU(11, 0, -1);     CHK(11, 32'd1);       // 0 < 0xFFFFFFFF
    SLTIU(11, 3, -1);     CHK(11, 32'd0);
    SLTIU(11, 4, 1);      CHK(11, 32'd0);
    SLTIU(11, 0, 1);      CHK(11, 32'd1);       // seqz
    XORI (11, 5, -1);     CHK(11, 32'hEDCB_A987);
    ORI  (11, 0, -2048);  CHK(11, 32'hFFFF_F800);
    ANDI (11, 5, 2047);   CHK(11, 32'h0000_0678);
    ANDI (11, 5, -16);    CHK(11, 32'h1234_5670);
    SLLI (11, 4, 31);     CHK(11, 32'h8000_0000);
    SLLI (11, 5, 0);      CHK(11, 32'h1234_5678);
    SRLI (11, 2, 31);     CHK(11, 32'h0000_0001);
    SRLI (11, 3, 0);      CHK(11, 32'hFFFF_FFFF);
    SRAI (11, 2, 31);     CHK(11, 32'hFFFF_FFFF);
    SRAI (11, 2, 1);      CHK(11, 32'hC000_0000);
    SRAI (11, 1, 30);     CHK(11, 32'h0000_0001);
    LUI  (11, 20'hFFFFF); CHK(11, 32'hFFFF_F000);
    LUI  (11, 20'h80000); CHK(11, 32'h8000_0000);

    // x0 no se modifica y se lee como 0 justo despues de intentar escribirlo
    ADDI(0, 4, 5);  ADD(11, 0, 4);  CHK(11, 32'd1);
    LUI (0, 20'h12345);  ADD(11, 0, 0);  CHK(11, 32'd0);

    // Cadena de 10 instrucciones dependientes: x12 = 2^10
    ADDI(12, 0, 1);
    repeat (10) ADD(12, 12, 12);
    CHK(12, 32'd1024);
    // Cadena mixta: cada resultado es operando de la siguiente
    ADDI(13, 0, 5);       // 5
    SLLI(13, 13, 3);      // 40
    XORI(13, 13, 'h55);   // 125
    SUB (13, 0, 13);      // -125
    SRAI(13, 13, 2);      // -32
    SLT (14, 13, 4);      // 1
    SLTU(15, 13, 4);      // 0
    ADD (16, 13, 14);     // -31
    CHK(16, 32'hFFFF_FFE1);
    CHK(15, 32'd0);

    END_PROGRAM();
  endtask

  // ---------------------------------------------------------------------------
  // P3: memoria (loads y stores) y riesgos con loads
  // ---------------------------------------------------------------------------
  task automatic build_p3_memoria();
    prog_name = "P3 MEMORIA";
    LUI(R_BASE, DATA_BASE[31:12]);  // x10 = 0x1F000
    LI(5, 32'hF1E2_D3C4);           // bytes negativos
    LI(6, 32'h7F01_7F01);           // bytes positivos (0x7F / 0x01)
    LI(7, 32'h0080_7F80);           // frontera 0x80 / 0x7F

    SW(5, R_BASE, 0);
    SW(6, R_BASE, 4);
    SW(7, R_BASE, 8);
    // lw y uso inmediato del dato en la ALU (riesgo load-uso)
    LW(11, R_BASE, 0);  ADDI(12, 11, 1);  CHK(12, 32'hF1E2_D3C5);
    // lb / lbu en los cuatro bytes
    LB (11, R_BASE, 0);  CHK(11, 32'hFFFF_FFC4);
    LB (11, R_BASE, 1);  CHK(11, 32'hFFFF_FFD3);
    LB (11, R_BASE, 2);  CHK(11, 32'hFFFF_FFE2);
    LB (11, R_BASE, 3);  CHK(11, 32'hFFFF_FFF1);
    LBU(11, R_BASE, 0);  CHK(11, 32'h0000_00C4);
    LBU(11, R_BASE, 1);  CHK(11, 32'h0000_00D3);
    LBU(11, R_BASE, 2);  CHK(11, 32'h0000_00E2);
    LBU(11, R_BASE, 3);  CHK(11, 32'h0000_00F1);
    // lh / lhu en las dos medias palabras
    LH (11, R_BASE, 0);  CHK(11, 32'hFFFF_D3C4);
    LH (11, R_BASE, 2);  CHK(11, 32'hFFFF_F1E2);
    LHU(11, R_BASE, 0);  CHK(11, 32'h0000_D3C4);
    LHU(11, R_BASE, 2);  CHK(11, 32'h0000_F1E2);
    // valores positivos y frontera de signo 0x7F / 0x80
    LB (11, R_BASE, 4);  CHK(11, 32'h0000_0001);
    LB (11, R_BASE, 5);  CHK(11, 32'h0000_007F);
    LH (11, R_BASE, 6);  CHK(11, 32'h0000_7F01);
    LB (11, R_BASE, 8);  CHK(11, 32'hFFFF_FF80);
    LB (11, R_BASE, 9);  CHK(11, 32'h0000_007F);
    LB (11, R_BASE, 10); CHK(11, 32'hFFFF_FF80);
    LBU(11, R_BASE, 10); CHK(11, 32'h0000_0080);
    LH (11, R_BASE, 8);  CHK(11, 32'h0000_7F80);
    LH (11, R_BASE, 10); CHK(11, 32'h0000_0080);
    LHU(11, R_BASE, 8);  CHK(11, 32'h0000_7F80);
    // lb con signo seguido de uso inmediato
    LB(11, R_BASE, 3);  ADDI(12, 11, 'h10);  CHK(12, 32'h0000_0001);

    // sb en los cuatro bytes de una palabra
    SW(0, R_BASE, 12);
    SB(5, R_BASE, 12);   // C4
    SB(6, R_BASE, 13);   // 01
    SB(7, R_BASE, 14);   // 80
    SB(5, R_BASE, 15);   // C4
    LW(11, R_BASE, 12);  CHK(11, 32'hC480_01C4);
    // sh en las dos medias palabras y sb que sobrescribe un solo byte
    SW(0, R_BASE, 16);
    SH(5, R_BASE, 16);   // D3C4
    SH(6, R_BASE, 18);   // 7F01
    LW(11, R_BASE, 16);  CHK(11, 32'h7F01_D3C4);
    SB(0, R_BASE, 17);
    LW(11, R_BASE, 16);  CHK(11, 32'h7F01_00C4);
    // store seguido inmediatamente de load de la misma direccion
    SW(6, R_BASE, 20);  LW(11, R_BASE, 20);  CHK(11, 32'h7F01_7F01);
    // dato de store producido por la instruccion anterior
    ADDI(25, 0, 'h55);  SB(25, R_BASE, 36);  LBU(26, R_BASE, 36);  CHK(26, 32'h0000_0055);

    // load seguido de uso inmediato como operando de la ALU (los dos operandos)
    LW(12, R_BASE, 0);  ADD(13, 12, 12);  CHK(13, 32'hE3C5_A788);
    // load seguido de load que usa el dato como direccion (punteros)
    ADDI(14, R_BASE, 24);  SW(14, R_BASE, 28);         // mem[28] = &mem[24]
    LI(15, 32'h1357_9BDF); SW(15, R_BASE, 24);
    LW(16, R_BASE, 28);    LW(17, 16, 0);              CHK(17, 32'h1357_9BDF);
    // load seguido de store que usa el dato
    LW(18, R_BASE, 24);    SW(18, R_BASE, 32);
    LW(19, R_BASE, 32);    CHK(19, 32'h1357_9BDF);
    // load seguido de bifurcacion que usa el dato
    LW(20, R_BASE, 4);     BEQ(20, 6, 8);  TRAP();
    // load sobre x0: no se escribe y x0 se lee como 0 en la instruccion siguiente
    LW(0, R_BASE, 0);      ADD(21, 0, 0);   CHK(21, 32'd0);
    // dos loads seguidos al mismo registro y uso inmediato (gana el segundo)
    LW(21, R_BASE, 0);     LW(21, R_BASE, 4);  ADDI(22, 21, 0);  CHK(22, 32'h7F01_7F01);
    // offsets negativos
    ADDI(23, R_BASE, 64);  SW(5, 23, -4);  LW(24, R_BASE, 60);  CHK(24, 32'hF1E2_D3C4);
    LW(24, 23, -64);       CHK(24, 32'hF1E2_D3C4);

    END_PROGRAM();
  endtask

  // ---------------------------------------------------------------------------
  // P4: bifurcaciones condicionales
  //   tomada    : B rs1, rs2, +12 ; TRAP ; TRAP ; x9++  (x9 cuenta las tomadas)
  //   no tomada : B rs1, rs2, +8  ; x8++               (x8 cuenta las no tomadas)
  // Si una tomada no salta o no se anula la instruccion siguiente, x29 > 0;
  // si una no tomada salta, x8 no llega a su valor esperado.
  // ---------------------------------------------------------------------------
  int n_taken, n_not;

  task automatic B_TAKEN(input logic [2:0] f3, input int rs1, rs2);
    BRANCH(f3, rs1, rs2, 12);
    TRAP();
    TRAP();
    ADDI(9, 9, 1);
    n_taken++;
  endtask

  task automatic B_NOT(input logic [2:0] f3, input int rs1, rs2);
    BRANCH(f3, rs1, rs2, 8);
    ADDI(8, 8, 1);
    n_not++;
  endtask

  task automatic build_p4_saltos_cond();
    int loop, f;
    prog_name = "P4 SALTOS COND";
    n_taken = 0;
    n_not   = 0;
    LI(1, 32'h7FFF_FFFF);   // MAX
    LI(2, 32'h8000_0000);   // MIN
    ADDI(3, 0, -1);         // -1
    ADDI(4, 0, 1);          // 1

    // beq
    B_TAKEN(F_BEQ, 4, 4);   B_TAKEN(F_BEQ, 3, 3);   B_TAKEN(F_BEQ, 0, 0);
    B_NOT  (F_BEQ, 4, 0);   B_NOT  (F_BEQ, 2, 1);
    // bne
    B_TAKEN(F_BNE, 1, 2);   B_TAKEN(F_BNE, 4, 0);
    B_NOT  (F_BNE, 3, 3);   B_NOT  (F_BNE, 2, 2);
    // blt (con signo): MIN < MAX, -1 < 0, iguales no
    B_TAKEN(F_BLT, 2, 1);   B_TAKEN(F_BLT, 3, 0);
    B_NOT  (F_BLT, 1, 2);   B_NOT  (F_BLT, 0, 3);   B_NOT(F_BLT, 4, 4);
    // bge (con signo): iguales si
    B_TAKEN(F_BGE, 1, 2);   B_TAKEN(F_BGE, 4, 4);   B_TAKEN(F_BGE, 0, 3);
    B_NOT  (F_BGE, 2, 1);   B_NOT  (F_BGE, 3, 0);
    // bltu (sin signo): 0x7FFFFFFF < 0x80000000, 0 < 0xFFFFFFFF
    B_TAKEN(F_BLTU, 1, 2);  B_TAKEN(F_BLTU, 0, 3);
    B_NOT  (F_BLTU, 2, 1);  B_NOT  (F_BLTU, 3, 0);  B_NOT(F_BLTU, 4, 4);
    // bgeu (sin signo): iguales si
    B_TAKEN(F_BGEU, 2, 1);  B_TAKEN(F_BGEU, 3, 0);  B_TAKEN(F_BGEU, 4, 4);
    B_NOT  (F_BGEU, 1, 2);  B_NOT  (F_BGEU, 0, 3);

    // bifurcacion cuyos operandos produce la instruccion inmediatamente anterior
    ADDI(14, 0, 7);  ADDI(15, 0, 7);  B_TAKEN(F_BEQ, 14, 15);
    ADDI(15, 15, 1); B_NOT(F_BEQ, 14, 15);

    // saltos encadenados: el destino de un salto tomado es otro salto
    BEQ(0, 0, 8);   TRAP();          // -> bne
    BNE(4, 0, 8);   TRAP();          // -> jal
    JAL(0, 8);      TRAP();
    ADDI(9, 9, 1);  n_taken++;

    // bifurcacion hacia delante sobre un bloque largo (offset grande)
    f = here();  BNE(1, 2, 0);
    repeat (40) TRAP();
    patch(f);
    ADDI(9, 9, 1);  n_taken++;

    // bucle hacia atras: x13 = 10 + 9 + ... + 1 = 55; la bne depende del addi anterior
    ADDI(12, 0, 10);
    ADDI(13, 0, 0);
    loop = here();
    ADD (13, 13, 12);
    ADDI(12, 12, -1);
    BNE (12, 0, to(loop));
    CHK(13, 32'd55);

    CHK(9, 32'(n_taken));
    CHK(8, 32'(n_not));
    END_PROGRAM();
  endtask

  // ---------------------------------------------------------------------------
  // P5: saltos incondicionales (jal / jalr), subrutinas y tipo U
  // ---------------------------------------------------------------------------
  task automatic build_p5_saltos_inc();
    int a, f, sub1, sub2, call3;
    prog_name = "P5 SALTOS INC";

    // jal hacia delante: enlace = PC + 4, las dos siguientes se anulan
    a = here();  JAL(1, 12);  TRAP();  TRAP();
    CHK(1, pc_of(a) + 4);
    // el destino usa el enlace inmediatamente
    a = here();  JAL(1, 8);   TRAP();  ADD(11, 1, 0);
    CHK(11, pc_of(a) + 4);
    // jal x0: no enlaza
    JAL(0, 8);   TRAP();
    ADD(11, 0, 0);  CHK(11, 32'd0);

    // jalr con base calculada por las dos instrucciones anteriores
    a = here();  AUIPC(5, 0);  ADDI(5, 5, 20);  JALR(6, 5, 0);  TRAP();  TRAP();
    CHK(6, pc_of(a + 2) + 4);
    // jalr a una direccion impar: el bit 0 se pone a 0
    a = here();  AUIPC(7, 0);  ADDI(7, 7, 17);  JALR(0, 7, 0);  TRAP();
    ADDI(9, 9, 1);                              // destino: PC(a) + 16
    // jalr con rd = rs1: el destino usa el valor anterior del registro
    a = here();  AUIPC(13, 0);  JALR(13, 13, 12);  TRAP();
    CHK(13, pc_of(a + 1) + 4);
    // jalr con offset negativo
    a = here();  AUIPC(14, 0);  ADDI(14, 14, 36);  JALR(0, 14, -20);  TRAP();
    ADDI(9, 9, 1);                              // destino: PC(a) + 16
    CHK(9, 32'd2);

    // subrutinas (se saltan con jal x0 y se llaman hacia atras)
    f = here();  JAL(0, 0);
    sub1 = here();
    ADDI(20, 20, 1);
    JALR(0, 1, 0);                              // retorno
    sub2 = here();                              // llama a sub1 (anidada)
    ADDI(18, 1, 0);                             // guarda el enlace
    JAL (1, to(sub1));
    ADDI(20, 20, 16);
    ADDI(1, 18, 0);                             // restaura el enlace...
    JALR(0, 1, 0);                              // ...y lo usa inmediatamente
    patch(f);
    JAL(1, to(sub1));                           // x20 = 1
    JAL(1, to(sub2));                           // x20 = 1 + 1 + 16 = 18
    call3 = here();
    JAL(1, to(sub1));                           // x20 = 19
    CHK(20, 32'd19);
    CHK(1, pc_of(call3) + 4);

    // lui y auipc con valores extremos
    LUI(11, 20'h00000);                CHK(11, 32'h0000_0000);
    LUI(11, 20'hFFFFF);                CHK(11, 32'hFFFF_F000);
    a = here();  AUIPC(11, 20'hFFFFF); CHK(11, pc_of(a) - 32'h1000);
    a = here();  AUIPC(11, 20'h80000); CHK(11, pc_of(a) + 32'h8000_0000);
    a = here();  AUIPC(11, 20'h00001); ADDI(11, 11, -4);  CHK(11, pc_of(a) + 32'h0FFC);

    END_PROGRAM();
  endtask

  task automatic build_program(input int p);
    prog.delete();
    n_chk = 0;
    case (p)
      0: build_p1_enunciado();
      1: build_p2_alu();
      2: build_p3_memoria();
      3: build_p4_saltos_cond();
      4: build_p5_saltos_inc();
    endcase
  endtask

  // ===========================================================================
  // MODELO DE REFERENCIA (ISS RV32I)
  // ===========================================================================
  logic [31:0] m_x   [0:31];
  logic [31:0] m_mem [0:DEPTH-1];
  logic [31:0] m_pc;

  typedef struct {
    string       txt;        // desensamblado
    string       mnem;       // mnemonico
    logic [4:0]  rs1, rs2, rd;
    bit          use_rs1, use_rs2;
    logic [31:0] v1, v2;
    bit          we;         // el decoder debe activar we
    bit          wr_rd;      // escribe un valor en rd (rd != 0)
    logic [31:0] rd_val;
    bit          chk_alu;
    logic [31:0] alu;
    bit          is_load, is_store, is_branch, jump;
    logic [31:0] addr;
    int          idx;
    logic [3:0]  be;
    logic [31:0] st_word;
    logic [31:0] next_pc;
    bit          illegal;
  } exp_t;

  function automatic string rname(input logic [4:0] r);
    return $sformatf("x%0d", r);
  endfunction

  function automatic string mnemonic(input logic [31:0] i);
    case (i[6:0])
      OP_LUI:    return "lui";
      OP_AUIPC:  return "auipc";
      OP_JAL:    return "jal";
      OP_JALR:   return "jalr";
      OP_BRANCH: case (i[14:12]) 3'b000: return "beq";  3'b001: return "bne";  3'b100: return "blt";
                                 3'b101: return "bge";  3'b110: return "bltu"; 3'b111: return "bgeu";
                                 default: return "b???"; endcase
      OP_LOAD:   case (i[14:12]) 3'b000: return "lb";  3'b001: return "lh";  3'b010: return "lw";
                                 3'b100: return "lbu"; 3'b101: return "lhu"; default: return "l???"; endcase
      OP_STORE:  case (i[14:12]) 3'b000: return "sb";  3'b001: return "sh";  3'b010: return "sw";
                                 default: return "s???"; endcase
      OP_I:      case (i[14:12]) 3'b000: return "addi"; 3'b001: return "slli"; 3'b010: return "slti";
                                 3'b011: return "sltiu"; 3'b100: return "xori";
                                 3'b101: return i[30] ? "srai" : "srli";
                                 3'b110: return "ori"; default: return "andi"; endcase
      OP_R:      case (i[14:12]) 3'b000: return i[30] ? "sub" : "add"; 3'b001: return "sll";
                                 3'b010: return "slt"; 3'b011: return "sltu"; 3'b100: return "xor";
                                 3'b101: return i[30] ? "sra" : "srl";
                                 3'b110: return "or"; default: return "and"; endcase
      default:   return "???";
    endcase
  endfunction

  function automatic string disasm(input logic [31:0] pc, input logic [31:0] i);
    logic [31:0] imm_i, imm_s, imm_b, imm_j;
    string m;
    imm_i = {{20{i[31]}}, i[31:20]};
    imm_s = {{20{i[31]}}, i[31:25], i[11:7]};
    imm_b = {{19{i[31]}}, i[31], i[7], i[30:25], i[11:8], 1'b0};
    imm_j = {{11{i[31]}}, i[31], i[19:12], i[20], i[30:21], 1'b0};
    m = mnemonic(i);
    case (i[6:0])
      OP_LUI, OP_AUIPC: return $sformatf("%-5s %s, 0x%05H", m, rname(i[11:7]), i[31:12]);
      OP_JAL:    return $sformatf("%-5s %s, %0d (-> 0x%05H)", m, rname(i[11:7]), $signed(imm_j), pc + imm_j);
      OP_JALR:   return $sformatf("%-5s %s, %0d(%s)", m, rname(i[11:7]), $signed(imm_i), rname(i[19:15]));
      OP_BRANCH: return $sformatf("%-5s %s, %s, %0d (-> 0x%05H)", m, rname(i[19:15]), rname(i[24:20]), $signed(imm_b), pc + imm_b);
      OP_LOAD:   return $sformatf("%-5s %s, %0d(%s)", m, rname(i[11:7]), $signed(imm_i), rname(i[19:15]));
      OP_STORE:  return $sformatf("%-5s %s, %0d(%s)", m, rname(i[24:20]), $signed(imm_s), rname(i[19:15]));
      OP_I:      if (i[13:12] == 2'b01)
                   return $sformatf("%-5s %s, %s, %0d", m, rname(i[11:7]), rname(i[19:15]), i[24:20]);
                 else
                   return $sformatf("%-5s %s, %s, %0d", m, rname(i[11:7]), rname(i[19:15]), $signed(imm_i));
      OP_R:      return $sformatf("%-5s %s, %s, %s", m, rname(i[11:7]), rname(i[19:15]), rname(i[24:20]));
      default:   return $sformatf("??? (0x%08H)", i);
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

  // Ejecuta una instruccion en el modelo (actualiza m_x y m_mem)
  function automatic exp_t iss_step(input logic [31:0] pc, input logic [31:0] i);
    exp_t        e;
    logic [31:0] imm_i, imm_s, imm_b, imm_u, imm_j, a, b, w;
    logic [2:0]  f3;
    bit          taken;

    imm_i = {{20{i[31]}}, i[31:20]};
    imm_s = {{20{i[31]}}, i[31:25], i[11:7]};
    imm_b = {{19{i[31]}}, i[31], i[7], i[30:25], i[11:8], 1'b0};
    imm_u = {i[31:12], 12'b0};
    imm_j = {{11{i[31]}}, i[31], i[19:12], i[20], i[30:21], 1'b0};
    f3    = i[14:12];

    e = '{default: 0};
    e.txt     = disasm(pc, i);
    e.mnem    = mnemonic(i);
    e.rs1     = i[19:15];
    e.rs2     = i[24:20];
    e.rd      = i[11:7];
    e.v1      = (e.rs1 == 0) ? 32'h0 : m_x[e.rs1];
    e.v2      = (e.rs2 == 0) ? 32'h0 : m_x[e.rs2];
    e.next_pc = pc + 4;
    a = e.v1;
    b = e.v2;

    case (i[6:0])
      OP_LUI:   begin e.we = 1; e.rd_val = imm_u; e.chk_alu = 1; e.alu = imm_u; end
      OP_AUIPC: begin e.we = 1; e.rd_val = pc + imm_u; end
      OP_JAL:   begin e.we = 1; e.rd_val = pc + 4; e.jump = 1; e.next_pc = pc + imm_j; end
      OP_JALR:  begin
        e.use_rs1 = 1;
        e.we = 1; e.rd_val = pc + 4; e.jump = 1; e.next_pc = (a + imm_i) & ~32'd1;
      end
      OP_BRANCH: begin
        e.use_rs1 = 1; e.use_rs2 = 1; e.is_branch = 1;
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
          3'b000:  begin e.rd_val = {{24{w[7]}},  w[7:0]};  e.be = 4'b0001; end
          3'b001:  begin e.rd_val = {{16{w[15]}}, w[15:0]}; e.be = 4'b0011; end
          3'b010:  begin e.rd_val = w;                      e.be = 4'b1111; end
          3'b100:  begin e.rd_val = {24'b0, w[7:0]};        e.be = 4'b0001; end
          3'b101:  begin e.rd_val = {16'b0, w[15:0]};       e.be = 4'b0011; end
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
        // be del decoder (sin desplazar) y palabra resultante en memoria
        w = b << (8 * e.addr[1:0]);
        e.st_word = m_mem[e.idx];
        for (int k = 0; k < 4; k++)
          if (e.be[k] && (k + e.addr[1:0]) < 4) e.st_word[8*(k + e.addr[1:0]) +: 8] = w[8*(k + e.addr[1:0]) +: 8];
        m_mem[e.idx] = e.st_word;
      end
      OP_I: begin
        e.use_rs1 = 1;
        e.we = 1; e.chk_alu = 1;
        e.rd_val = alu_op(f3, (f3 == 3'b101) && i[30], a, (f3[1:0] == 2'b01) ? {27'b0, i[24:20]} : imm_i);
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

  // El modelo parte del contenido real de memory.vhd (tras cargar el programa)
  task automatic model_reset();
    foreach (m_x[k]) m_x[k] = 32'h0;
    for (int k = 0; k < DEPTH; k++) m_mem[k] = u_mem.mem[k];
    m_pc = PROG_BASE;
  endtask

  // ===========================================================================
  // REGISTRO DE ERRORES, COBERTURA Y CICLOS
  // ===========================================================================
  int n_errors;            // errores del programa actual
  int total_errors;        // errores de toda la simulacion
  int err_cnt [string];

  task automatic fail(input string cat, input string msg);
    n_errors++;
    total_errors++;
    if (err_cnt.exists(cat)) err_cnt[cat]++;
    else                     err_cnt[cat] = 1;
    $error("[%s] %s: %s", prog_name, cat, msg);
  endtask

  // Cobertura (sobre la ejecucion golden de todos los programas)
  int cov [string];
  task automatic cov_hit(input string k);
    if (cov.exists(k)) cov[k]++;
    else               cov[k] = 1;
  endtask

  // Ciclos por tipo de instruccion (programa actual)
  typedef struct {
    int n, n_lat, lat_min, lat_max, lat_sum, bub;
  } stat_t;
  stat_t st [string];
  string cls_order [$];
  int    n_bubbles, halt_cycle, n_checked;
  int    pend_load [0:31];

  function automatic string iclass(input logic [31:0] i, input bit taken);
    case (i[6:0])
      OP_R:      return "ALU registros";
      OP_I:      return "ALU inmediato";
      OP_LOAD:   return "load";
      OP_STORE:  return "store";
      OP_BRANCH: return taken ? "bifurcacion tomada" : "bifurcacion no tomada";
      OP_JAL:    return "jal";
      OP_JALR:   return "jalr";
      OP_LUI:    return "lui";
      OP_AUIPC:  return "auipc";
      default:   return "ilegal";
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

  // Resultados por programa (resumen final)
  typedef struct {
    string       name;
    int          instr, cycles, bubbles, errors, chks;
    logic [31:0] sig, err;
    bit          pass;
  } res_t;
  res_t results [0:N_PROGS-1];

  // ===========================================================================
  // CARGA Y EJECUCION GOLDEN
  // ===========================================================================
  // Carga del programa por el puerto A de memory.vhd (core en reset)
  task automatic load_memory();
    load_en = 1'b1;
    tb_we   = 1'b1;
    foreach (prog[k]) begin
      @(negedge clk);
      tb_addr  = word_idx(PROG_BASE) + k;
      tb_wdata = prog[k];
    end
    @(negedge clk);
    tb_we   = 1'b0;
    load_en = 1'b0;
    foreach (prog[k])
      if (u_mem.mem[word_idx(PROG_BASE) + k] !== prog[k])
        fail("CARGA", $sformatf("mem[0x%05H] = 0x%08H, esperado 0x%08H",
                                pc_of(k), u_mem.mem[word_idx(PROG_BASE) + k], prog[k]));
  endtask

  logic [31:0] g_x [0:31];
  logic [31:0] g_mem [int];
  int          g_steps;
  logic [31:0] g_halt_pc;

  task automatic golden_run();
    exp_t        e;
    logic [31:0] i;
    logic [4:0]  prev_rd;
    bit          prev_load, prev_wr;
    model_reset();
    g_mem.delete();
    g_steps   = 0;
    prev_wr   = 0;
    prev_load = 0;
    prev_rd   = 0;
    forever begin
      i = m_mem[word_idx(m_pc)];
      e = iss_step(m_pc, i);
      g_steps++;
      // cobertura de instrucciones, casos de las bifurcaciones y riesgos
      cov_hit(e.mnem);
      if (e.is_branch) cov_hit({e.mnem, e.jump ? " tomada" : " no tomada"});
      if (e.jump)      cov_hit("anulacion tras salto tomado");
      if (prev_wr && ((e.use_rs1 && e.rs1 == prev_rd) || (e.use_rs2 && e.rs2 == prev_rd))) begin
        if (prev_load) cov_hit("load seguido de uso del dato");
        else           cov_hit("dato usado por la instruccion siguiente");
        if (prev_load && e.is_load && e.rs1 == prev_rd)  cov_hit("load -> direccion de load");
        if (prev_load && e.is_store && e.rs2 == prev_rd) cov_hit("load -> dato de store");
        if (prev_load && e.is_branch)                    cov_hit("load -> bifurcacion");
        if (e.is_branch || i[6:0] == OP_JALR)            cov_hit("salto que usa el dato anterior");
      end
      prev_wr   = e.wr_rd;
      prev_load = e.is_load;
      prev_rd   = e.rd;
      if (e.is_store) g_mem[e.idx] = 32'h0;
      if (i === HALT || e.illegal || g_steps >= MAX_CYCLES) break;
      m_pc = e.next_pc;
    end
    g_halt_pc = m_pc;
    foreach (m_x[k])   g_x[k]   = m_x[k];
    foreach (g_mem[k]) g_mem[k] = m_mem[k];
  endtask

  // ===========================================================================
  // EJECUCION EN LOCKSTEP: DUT frente al modelo, instruccion a instruccion
  // ===========================================================================
  // old = valor previo del destino: si ya era el esperado la latencia no se puede medir
  task automatic check_load(input logic [4:0] r, input logic [31:0] v, input logic [31:0] old,
                            input logic [31:0] pc, input string txt, input string cls, input int t_fetch);
    logic [31:0] got;
    for (int k = 0; k < LOAD_LAT; k++) begin
      @(posedge clk); #1;
      got = dut_reg(r);
      if (got === v) break;
    end
    if (got === v && old !== v) add_lat(cls, cyc_cnt - 1 - t_fetch + 1);
    #1 pend_load[r]--;
    if (got !== v)
      fail("LOAD", $sformatf("PC=0x%05H %s: %s esp 0x%08H, obt 0x%08H", pc, txt, rname(r), v, got));
  endtask

  task automatic check_store(input int idx, input logic [31:0] v, input logic [31:0] old,
                             input logic [31:0] pc, input string txt, input string cls, input int t_fetch);
    logic [31:0] got;
    for (int k = 0; k < STORE_LAT; k++) begin
      @(posedge clk); #1;
      got = u_mem.mem[idx];
      if (got === v) break;
    end
    if (got === v && old !== v) add_lat(cls, cyc_cnt - 1 - t_fetch + 1);
    if (got !== v)
      fail("STORE", $sformatf("PC=0x%05H %s: mem[0x%05H] esp 0x%08H, obt 0x%08H", pc, txt, idx << 2, v, got));
  endtask

  task automatic run_lockstep(output bit halted);
    exp_t        e;
    logic [31:0] pc, instr, got;
    logic [31:0] snap [0:31];
    int          err_before, t_fetch, t_dec;
    string       cls, last_cls;
    bit          lat_ok;

    halted   = 0;
    last_cls = "reset";
    for (int cyc = 0; cyc < MAX_CYCLES; cyc++) begin
      #1;  // propagacion combinacional (decoder -> regFile -> alu)
      pc         = dec_pc;
      instr      = rdata_a;
      err_before = n_errors;

      if ($isunknown(pc) || $isunknown(instr)) begin
        fail("FLUJO", $sformatf("ciclo %0d: PC (0x%08H) o instruccion (0x%08H) desconocidos", cyc, pc, instr));
        @(posedge clk); @(negedge clk);
        if (n_errors >= MAX_ERRORS) return;
        continue;
      end

      // --- burbuja: instruccion anulada (primer ciclo, tras salto tomado o load) ---
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
        if (TRACE) $display("[%4d] PC=0x%05H %08H  -- burbuja (tras %s) --", cyc, pc, instr, last_cls);
        @(negedge clk);
        continue;
      end

      t_fetch = dec_fetch_cyc;
      t_dec   = cyc_cnt;
      if (pc !== m_pc) begin
        fail("FLUJO", $sformatf("ciclo %0d: se ejecuta PC=0x%05H (%s) pero tocaba PC=0x%05H",
                                cyc, pc, disasm(pc, instr), m_pc));
        m_pc = pc;
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
          automatic logic [31:0] f_old = snap[e.rd];
          check_load(f_rd, f_val, f_old, f_pc, f_txt, f_cls, f_tf);
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
          automatic logic [31:0] f_old = u_mem.mem[e.idx];
          check_store(f_idx, f_val, f_old, f_pc, f_txt, f_cls, f_tf);
        join_none
      end
      lat_ok = (n_errors == err_before);

      // --- write-back ---
      @(posedge clk); #1;
      if (e.wr_rd && !e.is_load) begin
        got = dut_reg(e.rd);
        if (got !== e.rd_val) begin
          fail("WB", $sformatf("PC=0x%05H %s: %s esp 0x%08H, obt 0x%08H", pc, e.txt, rname(e.rd), e.rd_val, got));
          if (RESYNC && !$isunknown(got)) m_x[e.rd] = got;
          lat_ok = 0;
        end
      end
      if (!e.is_load && !e.is_store && lat_ok) add_lat(cls, t_dec - t_fetch + 1);
      last_cls = cls;
      for (int k = 1; k < 32; k++) begin
        if (e.wr_rd && k == e.rd) continue;
        if (pend_load[k] > 0) continue;
        if (dut_reg(k) !== snap[k])
          fail("WB", $sformatf("PC=0x%05H %s: escritura inesperada en x%0d (0x%08H -> 0x%08H)",
                               pc, e.txt, k, snap[k], dut_reg(k)));
      end

      #2;
      if (TRACE)
        $display("[%4d] PC=0x%05H %08H  %-34s%s%s", cyc, pc, instr, e.txt,
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
        $display("  [STOP] %s: se alcanzan %0d errores", prog_name, MAX_ERRORS);
        return;
      end
      @(negedge clk);
    end
  endtask

  // ===========================================================================
  // COMPROBACION FINAL DE CADA PROGRAMA
  // ===========================================================================
  task automatic check_final(input int p, input bit halted);
    logic [31:0] val, sig, errw;
    bit          pass;
    int          n_before;
    n_before = n_errors;
    repeat (STORE_LAT + 1) @(posedge clk);
    #1;

    if (!halted)
      fail("FLUJO", $sformatf("no llega al halt (0x%05H) en %0d ciclos", g_halt_pc, MAX_CYCLES));

    // Estado completo frente a la ejecucion golden (diagnostico)
    for (int k = 0; k < 32; k++) begin
      val = dut_reg(k);
      if (val !== g_x[k]) fail("FINAL", $sformatf("x%0d esp 0x%08H, obt 0x%08H", k, g_x[k], val));
    end
    foreach (g_mem[k]) begin
      val = u_mem.mem[k];
      if (val !== g_mem[k]) fail("FINAL", $sformatf("mem[0x%05H] esp 0x%08H, obt 0x%08H", k << 2, g_mem[k], val));
    end

    // Comprobacion principal: resultado final del programa
    if (p == 0) begin
      // P1: registros de firma del enunciado
      assert (dut_reg(21) === P1_X21) else fail("FIRMA", $sformatf("x21 esp 0x%08H, obt 0x%08H", P1_X21, dut_reg(21)));
      assert (dut_reg(22) === P1_X22) else fail("FIRMA", $sformatf("x22 esp 0x%08H, obt 0x%08H", P1_X22, dut_reg(22)));
      assert (dut_reg(23) === P1_X23) else fail("FIRMA", $sformatf("x23 esp 0x%08H, obt 0x%08H", P1_X23, dut_reg(23)));
      assert (dut_reg(24) === P1_X24) else fail("FIRMA", $sformatf("x24 esp 0x%08H, obt 0x%08H", P1_X24, dut_reg(24)));
      assert (dut_reg(25) === P1_X25) else fail("FIRMA", $sformatf("x25 esp 0x%08H, obt 0x%08H", P1_X25, dut_reg(25)));
      assert (dut_reg(26) === P1_X26) else fail("FIRMA", $sformatf("x26 esp 0x%08H, obt 0x%08H", P1_X26, dut_reg(26)));
      assert (dut_reg(27) === P1_X27) else fail("FIRMA", $sformatf("x27 esp 0x%08H, obt 0x%08H", P1_X27, dut_reg(27)));
      assert (dut_reg(28) === P1_X28) else fail("FIRMA", $sformatf("x28 esp 0x%08H, obt 0x%08H", P1_X28, dut_reg(28)));
      sig  = dut_reg(28);
      errw = 32'h0;
    end else begin
      sig  = u_mem.mem[word_idx(SIG_ADDR)];
      errw = u_mem.mem[word_idx(ERR_ADDR)];
      // el plan de pruebas es coherente con el repertorio (el modelo no falla ningun CHK)
      assert (g_x[R_ERR] === 32'h0)
        else fail("PLAN", $sformatf("el modelo de referencia falla %0d pruebas CHK: revisar los valores esperados", g_x[R_ERR]));
      assert (g_x[R_SIG] === EXP_SIG[p])
        else fail("PLAN", $sformatf("firma del modelo 0x%08H distinta de EXP_SIG 0x%08H", g_x[R_SIG], EXP_SIG[p]));
      // procesador: ninguna prueba fallida y firma correcta
      assert (errw === 32'h0)
        else fail("FIRMA", $sformatf("%0d pruebas CHK fallidas en el procesador (mem[ERR_ADDR])", errw));
      assert (sig === EXP_SIG[p])
        else fail("FIRMA", $sformatf("firma esp 0x%08H, obt 0x%08H (mem[SIG_ADDR])", EXP_SIG[p], sig));
    end

    pass = (n_errors == 0);
    results[p] = '{name: prog_name, instr: n_checked, cycles: halted ? halt_cycle + 1 : -1,
                   bubbles: n_bubbles, errors: n_errors, chks: n_chk, sig: sig, err: errw, pass: pass};
  endtask

  task automatic report_cycles();
    $display("    %-22s %5s %12s %9s %13s", "tipo", "n", "latencia", "burbujas", "ciclos/instr");
    foreach (cls_order[k]) begin
      string c;
      c = cls_order[k];
      if (st[c].n == 0) continue;
      if (st[c].n_lat > 0)
        $display("    %-22s %5d %8d-%-3d %9d %13.2f", c, st[c].n, st[c].lat_min, st[c].lat_max, st[c].bub,
                 1.0 + real'(st[c].bub) / real'(st[c].n));
      else
        $display("    %-22s %5d %12s %9d %13.2f", c, st[c].n, "no medida", st[c].bub,
                 1.0 + real'(st[c].bub) / real'(st[c].n));
    end
  endtask

  // ===========================================================================
  // UN PROGRAMA COMPLETO: carga, golden, lockstep y comprobacion final
  // ===========================================================================
  task automatic run_program(input int p);
    bit halted;
    nRst = 1'b0;
    build_program(p);
    n_errors   = 0;
    n_checked  = 0;
    n_bubbles  = 0;
    halt_cycle = 0;
    st.delete();
    cls_order.delete();
    foreach (pend_load[k]) pend_load[k] = 0;

    $display("\n==================================================================");
    $display(" %s: %0d instrucciones en 0x%05H..0x%05H", prog_name, prog.size(), PROG_BASE, pc_of(prog.size() - 1));
    $display("==================================================================");
    load_memory();
    golden_run();
    model_reset();
    repeat (3) @(posedge clk);
    @(negedge clk);
    nRst = 1'b1;
    run_lockstep(halted);
    check_final(p, halted);

    if (halted)
      $display("  Ciclos: %0d | instrucciones: %0d (golden %0d) | burbujas: %0d | CPI = %0.2f",
               halt_cycle + 1, n_checked, g_steps, n_bubbles, real'(halt_cycle + 1) / real'(n_checked));
    report_cycles();
    if (p == 0)
      $display("  Firma x28 = 0x%08H (esp 0x%08H)", results[p].sig, P1_X28);
    else
      $display("  Pruebas CHK: %0d | fallidas: %0d | firma = 0x%08H (esp 0x%08H)",
               n_chk, results[p].err, results[p].sig, EXP_SIG[p]);
    $display("  Resultado %s: %s (%0d errores)", prog_name, results[p].pass ? "CORRECTO" : "FALLO", n_errors);
    @(negedge clk);
    nRst = 1'b0;
  endtask

  // ===========================================================================
  // COBERTURA
  // ===========================================================================
  task automatic report_coverage();
    string rv32i [0:36] = '{"lui", "auipc", "jal", "jalr",
                            "beq", "bne", "blt", "bge", "bltu", "bgeu",
                            "lb", "lh", "lw", "lbu", "lhu", "sb", "sh", "sw",
                            "addi", "slti", "sltiu", "xori", "ori", "andi", "slli", "srli", "srai",
                            "add", "sub", "sll", "slt", "sltu", "xor", "srl", "sra", "or", "and"};
    string br [0:5] = '{"beq", "bne", "blt", "bge", "bltu", "bgeu"};
    string hz [0:6] = '{"dato usado por la instruccion siguiente", "load seguido de uso del dato",
                        "load -> direccion de load", "load -> dato de store", "load -> bifurcacion",
                        "salto que usa el dato anterior", "anulacion tras salto tomado"};
    string line;
    int    missing;
    missing = 0;
    $display("\n--- COBERTURA (ejecuciones en los %0d programas) ---", N_PROGS);
    line = "  ";
    foreach (rv32i[k]) begin
      int n;
      n = cov.exists(rv32i[k]) ? cov[rv32i[k]] : 0;
      if (n == 0) missing++;
      line = {line, $sformatf("%-6s %4d   ", rv32i[k], n)};
      if (k % 6 == 5 || k == 36) begin
        $display("%s", line);
        line = "  ";
      end
    end
    foreach (br[k]) begin
      int t, nt;
      t  = cov.exists({br[k], " tomada"})    ? cov[{br[k], " tomada"}]    : 0;
      nt = cov.exists({br[k], " no tomada"}) ? cov[{br[k], " no tomada"}] : 0;
      $display("  %-5s tomada %3d | no tomada %3d", br[k], t, nt);
      if (t == 0 || nt == 0) missing++;
    end
    foreach (hz[k]) begin
      int n;
      n = cov.exists(hz[k]) ? cov[hz[k]] : 0;
      $display("  %-42s %4d", hz[k], n);
      if (n == 0) missing++;
    end
    assert (missing == 0)
      else begin
        total_errors++;
        $error("COBERTURA: %0d instrucciones / casos sin ejercitar", missing);
      end
    if (missing == 0) $display("  Cobertura completa: 37 instrucciones RV32I, las 6 bifurcaciones en sus dos casos y los riesgos.");
  endtask

  // ===========================================================================
  // SECUENCIA PRINCIPAL
  // ===========================================================================
  initial begin
    $display("=== VALIDACION DEL PROCESADOR RISC-V RV32I (core + memory.vhd) ===");
    nRst         = 1'b0;
    load_en      = 1'b1;
    tb_we        = 1'b0;
    tb_addr      = '0;
    tb_wdata     = '0;
    total_errors = 0;

    for (int p = 0; p < N_PROGS; p++) run_program(p);

    report_coverage();

    $display("\n--- RESUMEN ---");
    $display("  %-16s %6s %7s %5s %9s %7s %12s %10s", "programa", "instr", "ciclos", "CPI", "pruebas", "fallos", "firma", "resultado");
    foreach (results[p])
      $display("  %-16s %6d %7d %5.2f %9s %7s   0x%08H %10s", results[p].name, results[p].instr, results[p].cycles,
               (results[p].instr > 0) ? real'(results[p].cycles) / real'(results[p].instr) : 0.0,
               (p == 0) ? "-" : $sformatf("%0d", results[p].chks),
               (p == 0) ? "-" : $sformatf("%0d", results[p].err),
               results[p].sig, results[p].pass ? "CORRECTO" : "FALLO");
    if (total_errors == 0)
      $display("\n=== VALIDACION COMPLETADA: SIN ERRORES ===");
    else begin
      $display("\n=== VALIDACION COMPLETADA: %0d ERRORES ===", total_errors);
      foreach (err_cnt[c]) $display("    %-8s : %0d", c, err_cnt[c]);
    end
    $finish;
  end

endmodule
