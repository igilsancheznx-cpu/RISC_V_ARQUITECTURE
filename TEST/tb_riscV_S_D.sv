`timescale 1ns / 1ps

// =============================================================================
// tb_riscV_S_D
// Test sencillo del sistema completo (core + memory.vhd) CON el bloque divisor.
// Se prueban todas las instrucciones RV32I y las de division de la extension M
// (div, divu, rem, remu) con sus casos limite.
// La arquitectura del divisor se elige con DIV_ARCH (0 serie, 1 paralela,
// 2 segmentada). Por ejemplo: vsim -c work.tb_riscV_S_D -GDIV_ARCH=2 -do "run -all"
//
// Funcionamiento:
//   1. Se escribe un programa en memoria. Cada prueba deja su resultado en x5 y
//      despues lo guarda con "sw x5" en la zona de resultados (RES_BASE).
//      Antes de cada prueba x5 = 0x555, asi una instruccion que no escribe se
//      detecta.
//   2. Se ejecuta el programa hasta llegar al halt (jal x0, 0).
//   3. Se compara cada resultado guardado en memoria con el valor esperado.
//
// Compilar y simular (desde la raiz del proyecto):
//   vlib work
//   vcom -2008 HDL/alu.vhd
//   vcom -2002 HDL/divisor.vhd HDL/regFile.vhd HDL/decoder.vhd HDL/fetch.vhd HDL/DecExe.vhd HDL/core.vhd HDL/memory.vhd
//   vlog -sv TEST/tb_riscV_S_D.sv
//   vsim -c work.tb_riscV_S_D -do "run -all"
// =============================================================================
module tb_riscV_S_D;

  parameter DIV_ARCH = 0;                      // divisor: 0 serie, 1 paralela, 2 segmentada

  localparam [31:0] PROG_BASE = 32'h0001_0000;  // PC inicial del core
  localparam [31:0] RES_BASE  = 32'h0001_F000;  // resultados (x1)
  localparam [31:0] DATO_BASE = 32'h0001_F400;  // datos para loads y stores (x2)
  localparam [31:0] DATO      = 32'h80FF_7F81;  // palabra precargada en DATO_BASE
  localparam integer MAX_CICLOS = 6000;

  // ---------------------------------------------------------------------------
  // Reloj, reset y conexiones
  // ---------------------------------------------------------------------------
  reg clk  = 0;
  reg nRst = 0;
  always #5 clk = ~clk;

  wire [31:0] rdata_a, rdata_b, wdata_a, wdata_b;
  wire [16:0] addr_a, addr_b;
  wire        we_a, we_b;
  wire [3:0]  be_a, be_b;

  // Carga del programa por el puerto A mientras el core esta en reset
  reg        carga  = 1;
  reg [16:0] c_addr = 0;
  reg [31:0] c_dato = 0;
  reg        c_we   = 0;

  core #(.DIV_ARCH(DIV_ARCH)) dut (
    .clk(clk), .nRst(nRst),
    .rdata_a(rdata_a), .addr_a(addr_a), .wdata_a(wdata_a), .we_a(we_a), .be_a(be_a),
    .rdata_b(rdata_b), .addr_b(addr_b), .wdata_b(wdata_b), .we_b(we_b), .be_b(be_b)
  );

  // La memoria se direcciona por palabras: direccion de byte / 4
  memory u_mem (
    .clk(clk),
    .addr_a (carga ? c_addr  : {2'b00, addr_a[16:2]}),
    .wdata_a(carga ? c_dato  : wdata_a),
    .we_a   (carga ? c_we    : we_a),
    .be_a   (carga ? 4'b1111 : be_a),
    .rdata_a(rdata_a),
    .addr_b ({2'b00, addr_b[16:2]}),
    .wdata_b(wdata_b),
    .we_b   (we_b),
    .be_b   (be_b),
    .rdata_b(rdata_b)
  );

  // ---------------------------------------------------------------------------
  // Codificacion de instrucciones
  // ---------------------------------------------------------------------------
  function [31:0] tipoR(input [6:0] f7, input [2:0] f3, input [4:0] rd, input [4:0] rs1, input [4:0] rs2);
    tipoR = {f7, rs2, rs1, f3, rd, 7'b0110011};
  endfunction
  function [31:0] tipoI(input [6:0] op, input [2:0] f3, input [4:0] rd, input [4:0] rs1, input [11:0] imm);
    tipoI = {imm, rs1, f3, rd, op};
  endfunction
  function [31:0] tipoS(input [2:0] f3, input [4:0] rs2, input [4:0] rs1, input [11:0] imm);
    tipoS = {imm[11:5], rs2, rs1, f3, imm[4:0], 7'b0100011};
  endfunction
  function [31:0] tipoB(input [2:0] f3, input [4:0] rs1, input [4:0] rs2, input [12:0] off);
    tipoB = {off[12], off[10:5], rs2, rs1, f3, off[4:1], off[11], 7'b1100011};
  endfunction

  // Tipo R
  function [31:0] ADD (input [4:0] rd, rs1, rs2); ADD  = tipoR(7'h00, 3'b000, rd, rs1, rs2); endfunction
  function [31:0] SUB (input [4:0] rd, rs1, rs2); SUB  = tipoR(7'h20, 3'b000, rd, rs1, rs2); endfunction
  function [31:0] SLL (input [4:0] rd, rs1, rs2); SLL  = tipoR(7'h00, 3'b001, rd, rs1, rs2); endfunction
  function [31:0] SLT (input [4:0] rd, rs1, rs2); SLT  = tipoR(7'h00, 3'b010, rd, rs1, rs2); endfunction
  function [31:0] SLTU(input [4:0] rd, rs1, rs2); SLTU = tipoR(7'h00, 3'b011, rd, rs1, rs2); endfunction
  function [31:0] XOR (input [4:0] rd, rs1, rs2); XOR  = tipoR(7'h00, 3'b100, rd, rs1, rs2); endfunction
  function [31:0] SRL (input [4:0] rd, rs1, rs2); SRL  = tipoR(7'h00, 3'b101, rd, rs1, rs2); endfunction
  function [31:0] SRA (input [4:0] rd, rs1, rs2); SRA  = tipoR(7'h20, 3'b101, rd, rs1, rs2); endfunction
  function [31:0] OR  (input [4:0] rd, rs1, rs2); OR   = tipoR(7'h00, 3'b110, rd, rs1, rs2); endfunction
  function [31:0] AND (input [4:0] rd, rs1, rs2); AND  = tipoR(7'h00, 3'b111, rd, rs1, rs2); endfunction
  function [31:0] DIV (input [4:0] rd, rs1, rs2); DIV  = tipoR(7'h01, 3'b100, rd, rs1, rs2); endfunction
  function [31:0] DIVU(input [4:0] rd, rs1, rs2); DIVU = tipoR(7'h01, 3'b101, rd, rs1, rs2); endfunction
  function [31:0] REM (input [4:0] rd, rs1, rs2); REM  = tipoR(7'h01, 3'b110, rd, rs1, rs2); endfunction
  function [31:0] REMU(input [4:0] rd, rs1, rs2); REMU = tipoR(7'h01, 3'b111, rd, rs1, rs2); endfunction

  // Tipo I (aritmeticas)
  function [31:0] ADDI (input [4:0] rd, rs1, input [11:0] imm); ADDI  = tipoI(7'b0010011, 3'b000, rd, rs1, imm); endfunction
  function [31:0] SLTI (input [4:0] rd, rs1, input [11:0] imm); SLTI  = tipoI(7'b0010011, 3'b010, rd, rs1, imm); endfunction
  function [31:0] SLTIU(input [4:0] rd, rs1, input [11:0] imm); SLTIU = tipoI(7'b0010011, 3'b011, rd, rs1, imm); endfunction
  function [31:0] XORI (input [4:0] rd, rs1, input [11:0] imm); XORI  = tipoI(7'b0010011, 3'b100, rd, rs1, imm); endfunction
  function [31:0] ORI  (input [4:0] rd, rs1, input [11:0] imm); ORI   = tipoI(7'b0010011, 3'b110, rd, rs1, imm); endfunction
  function [31:0] ANDI (input [4:0] rd, rs1, input [11:0] imm); ANDI  = tipoI(7'b0010011, 3'b111, rd, rs1, imm); endfunction
  function [31:0] SLLI (input [4:0] rd, rs1, sh); SLLI = tipoI(7'b0010011, 3'b001, rd, rs1, {7'h00, sh}); endfunction
  function [31:0] SRLI (input [4:0] rd, rs1, sh); SRLI = tipoI(7'b0010011, 3'b101, rd, rs1, {7'h00, sh}); endfunction
  function [31:0] SRAI (input [4:0] rd, rs1, sh); SRAI = tipoI(7'b0010011, 3'b101, rd, rs1, {7'h20, sh}); endfunction

  // Loads, stores y saltos
  function [31:0] LB  (input [4:0] rd, rs1, input [11:0] imm); LB  = tipoI(7'b0000011, 3'b000, rd, rs1, imm); endfunction
  function [31:0] LH  (input [4:0] rd, rs1, input [11:0] imm); LH  = tipoI(7'b0000011, 3'b001, rd, rs1, imm); endfunction
  function [31:0] LW  (input [4:0] rd, rs1, input [11:0] imm); LW  = tipoI(7'b0000011, 3'b010, rd, rs1, imm); endfunction
  function [31:0] LBU (input [4:0] rd, rs1, input [11:0] imm); LBU = tipoI(7'b0000011, 3'b100, rd, rs1, imm); endfunction
  function [31:0] LHU (input [4:0] rd, rs1, input [11:0] imm); LHU = tipoI(7'b0000011, 3'b101, rd, rs1, imm); endfunction
  function [31:0] SB  (input [4:0] rs2, rs1, input [11:0] imm); SB  = tipoS(3'b000, rs2, rs1, imm); endfunction
  function [31:0] SH  (input [4:0] rs2, rs1, input [11:0] imm); SH  = tipoS(3'b001, rs2, rs1, imm); endfunction
  function [31:0] SW  (input [4:0] rs2, rs1, input [11:0] imm); SW  = tipoS(3'b010, rs2, rs1, imm); endfunction
  function [31:0] BEQ (input [4:0] rs1, rs2, input [12:0] off); BEQ  = tipoB(3'b000, rs1, rs2, off); endfunction
  function [31:0] BNE (input [4:0] rs1, rs2, input [12:0] off); BNE  = tipoB(3'b001, rs1, rs2, off); endfunction
  function [31:0] BLT (input [4:0] rs1, rs2, input [12:0] off); BLT  = tipoB(3'b100, rs1, rs2, off); endfunction
  function [31:0] BGE (input [4:0] rs1, rs2, input [12:0] off); BGE  = tipoB(3'b101, rs1, rs2, off); endfunction
  function [31:0] BLTU(input [4:0] rs1, rs2, input [12:0] off); BLTU = tipoB(3'b110, rs1, rs2, off); endfunction
  function [31:0] BGEU(input [4:0] rs1, rs2, input [12:0] off); BGEU = tipoB(3'b111, rs1, rs2, off); endfunction
  function [31:0] JALR(input [4:0] rd, rs1, input [11:0] imm); JALR = tipoI(7'b1100111, 3'b000, rd, rs1, imm); endfunction
  function [31:0] JAL (input [4:0] rd, input [20:0] off);
    JAL = {off[20], off[10:1], off[11], off[19:12], rd, 7'b1101111};
  endfunction
  function [31:0] LUI  (input [4:0] rd, input [19:0] imm); LUI   = {imm, rd, 7'b0110111}; endfunction
  function [31:0] AUIPC(input [4:0] rd, input [19:0] imm); AUIPC = {imm, rd, 7'b0010111}; endfunction

  // ---------------------------------------------------------------------------
  // Construccion del programa
  // ---------------------------------------------------------------------------
  reg [31:0] prog [0:511];      // instrucciones del programa
  integer    n = 0;             // numero de instrucciones escritas

  reg [31:0] esperado [0:127];  // resultado esperado de cada prueba
  string     nombre   [0:127];  // nombre de cada prueba
  integer    k = 0;             // numero de pruebas

  // Escribe una instruccion en la siguiente posicion del programa
  task poner(input [31:0] instr);
    prog[n] = instr;
    n = n + 1;
  endtask

  // PC de la siguiente instruccion que se va a escribir
  function [31:0] PC();
    PC = PROG_BASE + 4*n;
  endfunction

  // Guarda x5 en la zona de resultados y apunta el valor esperado
  task guardar(input string nom, input [31:0] esp);
    poner(SW(5, 1, 4*k));
    nombre[k]   = nom;
    esperado[k] = esp;
    k = k + 1;
  endtask

  // Prueba de una instruccion que escribe en x5
  task probar(input string nom, input [31:0] instr, input [31:0] esp);
    poner(ADDI(5, 0, 12'h555));   // valor inicial conocido
    poner(instr);
    guardar(nom, esp);
  endtask

  // Prueba de una bifurcacion con desplazamiento +8: si salta, se salta
  // "addi x5, x0, 0" y x5 queda a 1
  task bifurcacion(input string nom, input [31:0] instr, input tomado);
    poner(ADDI(5, 0, 1));
    poner(instr);
    poner(ADDI(5, 0, 0));
    guardar(nom, tomado);
  endtask

  task programa_pruebas();
    reg [31:0] pc;

    // Registros de trabajo
    poner(LUI (1, 20'h0001F));      // x1  = 0x0001F000 (resultados)
    poner(ADDI(2, 1, 12'h400));     // x2  = 0x0001F400 (datos)
    poner(ADDI(10, 0, -7));         // x10 = -7
    poner(ADDI(11, 0, 100));        // x11 = 100
    poner(LUI (12, 20'h80000));     // x12 = 0x80000000
    poner(ADDI(13, 0, 3));          // x13 = 3
    poner(ADDI(14, 0, -1));         // x14 = -1

    // Tipo U
    probar("lui",               LUI(5, 20'h12345),     32'h12345000);
    probar("lui (bit 31)",      LUI(5, 20'hFFFFF),     32'hFFFFF000);
    poner(ADDI(5, 0, 12'h555));
    pc = PC();
    poner(AUIPC(5, 20'h00001));
    guardar("auipc",            pc + 32'h1000);

    // Tipo I aritmeticas
    probar("addi",              ADDI (5, 11, -200),    32'hFFFFFF9C);
    probar("addi (-2048)",      ADDI (5, 0, -2048),    32'hFFFFF800);
    probar("slti (si)",         SLTI (5, 10, -6),      32'd1);
    probar("slti (igual)",      SLTI (5, 11, 100),     32'd0);
    probar("sltiu (imm = -1)",  SLTIU(5, 11, -1),      32'd1);
    probar("xori (not)",        XORI (5, 11, -1),      32'hFFFFFF9B);
    probar("ori",               ORI  (5, 11, 12'h0F0), 32'h000000F4);
    probar("andi",              ANDI (5, 14, 12'h7FF), 32'h000007FF);
    probar("slli",              SLLI (5, 11, 4),       32'h00000640);
    probar("srli",              SRLI (5, 12, 4),       32'h08000000);
    probar("srai",              SRAI (5, 12, 4),       32'hF8000000);

    // Tipo R
    probar("add",               ADD (5, 11, 10),       32'h0000005D);
    probar("add (desborde)",    ADD (5, 12, 12),       32'h00000000);
    probar("sub",               SUB (5, 10, 11),       32'hFFFFFF95);
    probar("sub (0 - 2^31)",    SUB (5, 0, 12),        32'h80000000);
    probar("sll",               SLL (5, 13, 13),       32'h00000018);
    probar("sll (rs2 = -7)",    SLL (5, 13, 10),       32'h06000000);
    probar("slt (si)",          SLT (5, 12, 11),       32'd1);
    probar("slt (no)",          SLT (5, 11, 10),       32'd0);
    probar("sltu",              SLTU(5, 12, 11),       32'd0);
    probar("xor",               XOR (5, 10, 11),       32'hFFFFFF9D);
    probar("srl",               SRL (5, 12, 13),       32'h10000000);
    probar("sra",               SRA (5, 12, 13),       32'hF0000000);
    probar("or",                OR  (5, 10, 11),       32'hFFFFFFFD);
    probar("and",               AND (5, 10, 11),       32'h00000060);
    poner(ADDI(0, 0, 123));
    probar("escritura en x0",   ADD (5, 0, 0),         32'h00000000);

    // Loads (DATO = 0x80FF7F81 en DATO_BASE)
    probar("lw",                LW (5, 2, 0),          32'h80FF7F81);
    probar("lb  (byte 0)",      LB (5, 2, 0),          32'hFFFFFF81);
    probar("lb  (byte 1)",      LB (5, 2, 1),          32'h0000007F);
    probar("lbu (byte 0)",      LBU(5, 2, 0),          32'h00000081);
    probar("lbu (byte 3)",      LBU(5, 2, 3),          32'h00000080);
    probar("lh  (media 0)",     LH (5, 2, 0),          32'h00007F81);
    probar("lh  (media 1)",     LH (5, 2, 2),          32'hFFFF80FF);
    probar("lhu (media 1)",     LHU(5, 2, 2),          32'h000080FF);
    poner(ADDI(5, 0, 12'h555));
    poner(LW  (6, 2, 0));           // load seguido de su uso
    poner(ADD (5, 6, 0));
    guardar("lw + add (load-use)", 32'h80FF7F81);

    // Stores (se leen con lw)
    poner(SW(10, 2, 4));
    probar("sw",                LW(5, 2, 4),           32'hFFFFFFF9);
    poner(SB(11, 2, 5));
    probar("sb  (byte 1)",      LW(5, 2, 4),           32'hFFFF64F9);
    poner(SH(13, 2, 6));
    probar("sh  (media 1)",     LW(5, 2, 4),           32'h000364F9);

    // Bifurcaciones (x10 = -7, x11 = 100)
    bifurcacion("beq  (tomado)",    BEQ (11, 11, 8), 1);
    bifurcacion("beq  (no tomado)", BEQ (11, 10, 8), 0);
    bifurcacion("bne  (tomado)",    BNE (11, 10, 8), 1);
    bifurcacion("bne  (no tomado)", BNE (11, 11, 8), 0);
    bifurcacion("blt  (tomado)",    BLT (10, 11, 8), 1);
    bifurcacion("blt  (no tomado)", BLT (11, 10, 8), 0);
    bifurcacion("bge  (tomado)",    BGE (11, 10, 8), 1);
    bifurcacion("bge  (igual)",     BGE (11, 11, 8), 1);
    bifurcacion("bge  (no tomado)", BGE (10, 11, 8), 0);
    bifurcacion("bltu (tomado)",    BLTU(11, 10, 8), 1);
    bifurcacion("bltu (no tomado)", BLTU(10, 11, 8), 0);
    bifurcacion("bgeu (tomado)",    BGEU(10, 11, 8), 1);
    bifurcacion("bgeu (no tomado)", BGEU(11, 10, 8), 0);

    // jal: x5 = PC + 4 y se salta la instruccion siguiente
    pc = PC();
    poner(JAL (5, 8));
    poner(ADDI(5, 0, 0));
    guardar("jal",              pc + 4);

    // jalr: salta a x7 + 12 (se salta una instruccion) y x5 = PC + 4
    pc = PC();
    poner(AUIPC(7, 0));
    poner(JALR (5, 7, 12));
    poner(ADDI (5, 0, 0));
    guardar("jalr",             pc + 8);

    // Division (x10 = -7, x11 = 100, x12 = 0x80000000, x13 = 3, x14 = -1)
    probar("div  100 / 3",        DIV (5, 11, 13),     32'h00000021);
    probar("rem  100 % 3",        REM (5, 11, 13),     32'h00000001);
    probar("div  -7 / 3",         DIV (5, 10, 13),     32'hFFFFFFFE);
    probar("rem  -7 % 3",         REM (5, 10, 13),     32'hFFFFFFFF);
    probar("div  100 / -7",       DIV (5, 11, 10),     32'hFFFFFFF2);
    probar("rem  100 % -7",       REM (5, 11, 10),     32'h00000002);
    probar("div  -7 / 100",       DIV (5, 10, 11),     32'h00000000);
    probar("rem  -7 % 100",       REM (5, 10, 11),     32'hFFFFFFF9);
    probar("divu 0xFFFFFFF9 / 3", DIVU(5, 10, 13),     32'h55555553);
    probar("remu 0xFFFFFFF9 % 3", REMU(5, 10, 13),     32'h00000000);
    probar("remu 100 % 3",        REMU(5, 11, 13),     32'h00000001);
    probar("divu 100 / 0xF..F9",  DIVU(5, 11, 10),     32'h00000000);
    probar("div  entre 0",        DIV (5, 11, 0),      32'hFFFFFFFF);
    probar("rem  entre 0",        REM (5, 11, 0),      32'h00000064);
    probar("divu entre 0",        DIVU(5, 11, 0),      32'hFFFFFFFF);
    probar("remu entre 0",        REMU(5, 10, 0),      32'hFFFFFFF9);
    probar("div  -2^31 / -1",     DIV (5, 12, 14),     32'h80000000);
    probar("rem  -2^31 % -1",     REM (5, 12, 14),     32'h00000000);
    poner(ADDI(5, 0, 12'h555));
    poner(DIV (6, 11, 13));         // division seguida del uso del resultado
    poner(ADD (5, 6, 6));
    guardar("div + add (dependencia)", 32'h00000042);
  endtask

  // ---------------------------------------------------------------------------
  // Carga de una palabra en memoria por el puerto A
  // ---------------------------------------------------------------------------
  task escribir(input [31:0] direccion, input [31:0] dato);
    @(negedge clk);
    c_addr = {2'b00, direccion[16:2]};
    c_dato = dato;
    c_we   = 1;
    @(negedge clk);
    c_we   = 0;
  endtask

  // ---------------------------------------------------------------------------
  // Contador de ciclos y deteccion del halt
  // ---------------------------------------------------------------------------
  reg [31:0] pc_halt;
  integer    ciclo = 0;
  integer    ciclo_halt = 0;

  always @(posedge clk) begin
    if (nRst == 1) begin
      ciclo = ciclo + 1;
      if (ciclo_halt == 0 && dut.pc_instr == pc_halt[16:0])
        ciclo_halt = ciclo;
    end
  end

  // ---------------------------------------------------------------------------
  // Test
  // ---------------------------------------------------------------------------
  integer    i;
  integer    errores;
  reg [31:0] obtenido;

  initial begin
    if (DIV_ARCH == 0) $display("=== tb_riscV_S_D: RV32I + division SERIE ===");
    if (DIV_ARCH == 1) $display("=== tb_riscV_S_D: RV32I + division PARALELA ===");
    if (DIV_ARCH == 2) $display("=== tb_riscV_S_D: RV32I + division SEGMENTADA ===");

    // 1. Programa: pruebas + halt
    programa_pruebas();
    pc_halt = PC();
    poner(JAL(0, 0));
    $display("Programa: %0d instrucciones, %0d pruebas", n, k);

    // 2. Carga en memoria (core en reset)
    for (i = 0; i < n; i = i + 1)
      escribir(PROG_BASE + 4*i, prog[i]);
    escribir(DATO_BASE, DATO);
    @(negedge clk);
    carga = 0;
    nRst  = 1;

    // 3. Ejecucion hasta el halt
    while (ciclo_halt == 0 && ciclo < MAX_CICLOS)
      @(posedge clk);
    repeat (5) @(posedge clk);
    if (ciclo_halt == 0)
      $display("ERROR: el procesador no llega al halt en %0d ciclos", MAX_CICLOS);
    else
      $display("Halt alcanzado en el ciclo %0d", ciclo_halt);

    // 4. Comprobacion de resultados
    errores = 0;
    for (i = 0; i < k; i = i + 1) begin
      obtenido = u_mem.mem[(RES_BASE >> 2) + i];
      if (obtenido === esperado[i])
        $display("  OK     %-24s 0x%08h", nombre[i], obtenido);
      else begin
        $display("  ERROR  %-24s 0x%08h (esperado 0x%08h)", nombre[i], obtenido, esperado[i]);
        errores = errores + 1;
      end
    end
    if (ciclo_halt == 0)
      errores = errores + 1;

    if (errores == 0)
      $display("=== RESULTADO: CORRECTO (%0d pruebas) ===", k);
    else
      $display("=== RESULTADO: %0d ERRORES ===", errores);
    $finish;
  end

endmodule
