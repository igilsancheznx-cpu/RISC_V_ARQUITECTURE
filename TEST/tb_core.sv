`timescale 1ns / 1ps

// =============================================================================
// tb_core
// Comprobacion completa del procesador (core = fetch + DecExe) conectado a la
// memoria de doble puerto (memory.vhd):
//   - Puerto A: lectura de instrucciones (fetch)
//   - Puerto B: lectura / escritura de datos (loads y stores)
//
// El testbench carga un programa RV32I en la memoria mientras el core esta en
// reset y despues sigue su ejecucion instruccion a instruccion:
//   FASE 1: inicializacion de registros (addi, lui)
//   FASE 2: ALU tipo R    (add, sub, sll, slt, sltu, xor, srl, sra, or, and)
//   FASE 3: ALU tipo I    (addi, slti, sltiu, xori, ori, andi, slli, srli, srai)
//   FASE 4: stores tipo S (sb, sh, sw)      -> se comprueba la memoria
//   FASE 5: loads tipo I  (lb, lh, lw, lbu, lhu) -> desde la memoria real
//   FASE 6: bifurcaciones tipo B (beq, bne, blt, bge, bltu, bgeu)
//           tomadas, no tomadas y bucle con salto hacia atras (solo TEST_JUMPS = 1)
//   FASE 7: tipos U y J   (lui, auipc, jal, jalr) + bucle final (halt)
//           (el jalr usa auipc para formar el destino: el codigo es reubicable)
//           con TEST_JUMPS = 0: solo lui y auipc, ejecucion secuencial
//   FASE 8: estado final  (banco de registros completo y memoria de datos)
//
// Cada instruccion se identifica por su codigo al llegar al decoder. Tras un
// salto tomado se colocan instrucciones "trampa" (addi x31, x31, 1): si alguna
// se ejecuta, x31 deja de valer 0 y se detecta en la FASE 8.
//
// La memoria es por defecto el agente TEST/mem_agent.sv (mismo comportamiento
// que memory.vhd + monitor y comprobaciones). Para usar la memoria RTL:
//   vlog -sv +define+USE_RTL_MEM tb_core.sv
//
// El programa se carga a partir de PROG_BASE = 0x10000 (PC_RESET del core). Los
// PC que aparecen en las tareas y en los mensajes son relativos a PROG_BASE.
//
// La memoria se direcciona por palabras y el core genera direcciones de byte
// (PC + 4), por lo que el testbench conecta addr[ADDR_WIDTH-1:2] a la memoria.
// =============================================================================
module tb_core;

  localparam int MEM_WIDTH  = 32;
  localparam int NUM_BYTES  = 4;
  localparam int ADDR_REG   = 5;
  localparam int ADDR_WIDTH = 17;

  // 1: programa completo con bifurcaciones y saltos (beq..bgeu, jal, jalr, halt)
  // 0: programa secuencial sin saltos (no se comprueban pc_load / pc_target)
  localparam bit TEST_JUMPS = 1'b1;

  localparam int DECODE_TIMEOUT = 20; // ciclos maximos esperando una instruccion en el decoder
  localparam int WB_CYCLES      = 1;  // ciclos para escribir el resultado de la ALU en el banco
  localparam int LOAD_CYCLES    = 3;  // ciclos para escribir el dato de un load en el banco
  localparam int STORE_CYCLES   = 3;  // ciclos para que un store llegue a la memoria

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

  // Programa y datos
  localparam int PROG_WORDS = 77;            // PROG_BASE + 0x000 .. 0x130
  // Direccion de la primera instruccion = PC_RESET del core (0x10000). Las
  // direcciones de las tareas (pc) son relativas a PROG_BASE
  localparam logic [31:0] PROG_BASE = 32'h0001_0000;
  localparam logic [31:0] DATA_BASE = 32'h0000_0400;
  localparam logic [31:0] DATA_INIT = 32'hCAFE_F08A;
  localparam logic [31:0] TRAP      = {12'd1, 5'd31, 3'b000, 5'd31, OP_I}; // addi x31, x31, 1

  // Reloj y reset
  logic clk;
  logic nRst;

  // Puertos del core
  logic [MEM_WIDTH-1:0]  rdata_a, rdata_b;
  logic [ADDR_WIDTH-1:0] addr_a,  addr_b;
  logic [MEM_WIDTH-1:0]  wdata_a, wdata_b;
  logic                  we_a,    we_b;
  logic [NUM_BYTES-1:0]  be_a,    be_b;

  // Puertos de la memoria (multiplexados: testbench durante la carga, core despues)
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

  // Monitorización jerárquica de señales intermedias entre bloques
  wire [ADDR_WIDTH-1:0] mon_pc        = dut.U0.pc_out;      // PC de la etapa fetch
  wire [MEM_WIDTH-1:0]  mon_instr     = dut.U1.rdata_a;     // instruccion que llega al decoder
  wire [4:0]            mon_opcode    = dut.U1.U1.opcode;
  wire [4:0]            mon_rs1       = dut.U1.rs1;
  wire [4:0]            mon_rs2       = dut.U1.rs2;
  wire [4:0]            mon_rd        = dut.U1.rd;
  wire [MEM_WIDTH-1:0]  mon_imm       = dut.U1.imm;
  wire [NUM_BYTES-1:0]  mon_aloup     = dut.U1.aloup;
  wire [MEM_WIDTH-1:0]  mon_rdata1    = dut.U1.rdata1;
  wire [MEM_WIDTH-1:0]  mon_rdata2    = dut.U1.rdata2;
  wire [MEM_WIDTH-1:0]  mon_wAlu      = dut.U1.wAlu;
  wire                  mon_eq        = dut.U1.eq;
  wire                  mon_lt        = dut.U1.lt;
  wire                  mon_ltu       = dut.U1.ltu;
  wire                  mon_we        = dut.U1.we;          // escritura en banco de registros desde la ALU
  wire                  mon_wl        = dut.U1.wl;          // escritura en banco de registros desde memoria (LOAD)
  wire                  mon_we_mem    = dut.U1.aux_we_b;    // escritura en memoria (decoder)
  wire [NUM_BYTES-1:0]  mon_be_mem    = dut.U1.aux_be_b;    // byte enable de memoria (decoder)
  wire                  mon_pc_load   = dut.U1.pc_load;     // salto: carga pc_target en el PC
  wire [ADDR_WIDTH-1:0] mon_pc_target = dut.U1.pc_target;   // direccion destino del salto

  // Programa, modelo esperado del banco de registros
  logic [31:0] prog     [0:PROG_WORDS-1];
  logic [31:0] exp_regs [0:31];

  // Instancia del DUT
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

  // Instancia de la memoria
  //   por defecto : agente de memoria en SystemVerilog (TEST/mem_agent.sv)
  //   +define+USE_RTL_MEM : memoria RTL (HDL/memory.vhd)
`ifdef USE_RTL_MEM
  memory #(
    .MEM_WIDTH  (MEM_WIDTH),
    .NUM_BYTES  (NUM_BYTES),
    .ADDR_WIDTH (ADDR_WIDTH)
  ) u_mem (
`else
  mem_agent #(
    .MEM_WIDTH  (MEM_WIDTH),
    .NUM_BYTES  (NUM_BYTES),
    .ADDR_WIDTH (ADDR_WIDTH),
    .VERBOSE    (1)
  ) u_mem (
`endif
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
  // Funciones de codificacion de instrucciones RV32I
  // ---------------------------------------------------------------------------
  function automatic logic [31:0] enc_r(input logic [6:0] f7, input logic [4:0] rs2,
                                        input logic [4:0] rs1, input logic [2:0] f3,
                                        input logic [4:0] rd);
    return {f7, rs2, rs1, f3, rd, OP_R};
  endfunction

  function automatic logic [31:0] enc_i(input logic [31:0] imm, input logic [4:0] rs1,
                                        input logic [2:0] f3, input logic [4:0] rd,
                                        input logic [6:0] op);
    return {imm[11:0], rs1, f3, rd, op};
  endfunction

  function automatic logic [31:0] enc_s(input logic [31:0] imm, input logic [4:0] rs2,
                                        input logic [4:0] rs1, input logic [2:0] f3);
    return {imm[11:5], rs2, rs1, f3, imm[4:0], OP_STORE};
  endfunction

  function automatic logic [31:0] enc_b(input logic [31:0] imm, input logic [4:0] rs2,
                                        input logic [4:0] rs1, input logic [2:0] f3);
    return {imm[12], imm[10:5], rs2, rs1, f3, imm[4:1], imm[11], OP_BRANCH};
  endfunction

  function automatic logic [31:0] enc_u(input logic [19:0] imm20, input logic [4:0] rd,
                                        input logic [6:0] op);
    return {imm20, rd, op};
  endfunction

  function automatic logic [31:0] enc_j(input logic [31:0] imm, input logic [4:0] rd);
    return {imm[20], imm[10:1], imm[11], imm[19:12], rd, OP_JAL};
  endfunction

  // ---------------------------------------------------------------------------
  // Programa de prueba (direccion de byte -> prog[pc/4])
  // ---------------------------------------------------------------------------
  task automatic build_program();
    begin
      foreach (prog[i]) prog[i] = 32'h0;

      // FASE 1: inicializacion de registros
      prog['h000>>2] = enc_i(32'd20,     5'd0, 3'b000, 5'd1, OP_I);   // addi x1, x0, 20
      prog['h004>>2] = enc_i(32'd5,      5'd0, 3'b000, 5'd2, OP_I);   // addi x2, x0, 5
      prog['h008>>2] = enc_i(-32'd10,    5'd0, 3'b000, 5'd3, OP_I);   // addi x3, x0, -10
      prog['h00C>>2] = enc_i(32'd255,    5'd0, 3'b000, 5'd4, OP_I);   // addi x4, x0, 255
      prog['h010>>2] = enc_i(32'd20,     5'd0, 3'b000, 5'd5, OP_I);   // addi x5, x0, 20
      prog['h014>>2] = enc_i(DATA_BASE,  5'd0, 3'b000, 5'd7, OP_I);   // addi x7, x0, 0x400
      prog['h018>>2] = enc_u(20'h12345,  5'd8, OP_LUI);               // lui  x8, 0x12345
      prog['h01C>>2] = enc_i(32'h678,    5'd8, 3'b000, 5'd8, OP_I);   // addi x8, x8, 0x678

      // FASE 2: ALU tipo R
      prog['h020>>2] = enc_r(7'b0000000, 5'd2, 5'd1, 3'b000, 5'd10);  // add  x10, x1, x2
      prog['h024>>2] = enc_r(7'b0100000, 5'd2, 5'd1, 3'b000, 5'd11);  // sub  x11, x1, x2
      prog['h028>>2] = enc_r(7'b0000000, 5'd2, 5'd1, 3'b001, 5'd12);  // sll  x12, x1, x2
      prog['h02C>>2] = enc_r(7'b0000000, 5'd2, 5'd3, 3'b010, 5'd13);  // slt  x13, x3, x2
      prog['h030>>2] = enc_r(7'b0000000, 5'd2, 5'd3, 3'b011, 5'd14);  // sltu x14, x3, x2
      prog['h034>>2] = enc_r(7'b0000000, 5'd4, 5'd1, 3'b100, 5'd15);  // xor  x15, x1, x4
      prog['h038>>2] = enc_r(7'b0000000, 5'd2, 5'd3, 3'b101, 5'd16);  // srl  x16, x3, x2
      prog['h03C>>2] = enc_r(7'b0100000, 5'd2, 5'd3, 3'b101, 5'd17);  // sra  x17, x3, x2
      prog['h040>>2] = enc_r(7'b0000000, 5'd4, 5'd1, 3'b110, 5'd18);  // or   x18, x1, x4
      prog['h044>>2] = enc_r(7'b0000000, 5'd4, 5'd1, 3'b111, 5'd19);  // and  x19, x1, x4

      // FASE 3: ALU tipo I
      prog['h048>>2] = enc_i(-32'd5,     5'd1, 3'b000, 5'd20, OP_I);  // addi  x20, x1, -5
      prog['h04C>>2] = enc_i(32'd5,      5'd3, 3'b010, 5'd21, OP_I);  // slti  x21, x3, 5
      prog['h050>>2] = enc_i(-32'd1,     5'd1, 3'b011, 5'd22, OP_I);  // sltiu x22, x1, -1
      prog['h054>>2] = enc_i(32'h0F0,    5'd1, 3'b100, 5'd23, OP_I);  // xori  x23, x1, 0xF0
      prog['h058>>2] = enc_i(32'h100,    5'd1, 3'b110, 5'd24, OP_I);  // ori   x24, x1, 0x100
      prog['h05C>>2] = enc_i(32'h00F,    5'd4, 3'b111, 5'd25, OP_I);  // andi  x25, x4, 0x0F
      prog['h060>>2] = enc_i(32'h003,    5'd1, 3'b001, 5'd26, OP_I);  // slli  x26, x1, 3
      prog['h064>>2] = enc_i(32'h004,    5'd3, 3'b101, 5'd27, OP_I);  // srli  x27, x3, 4
      prog['h068>>2] = enc_i(32'h404,    5'd3, 3'b101, 5'd28, OP_I);  // srai  x28, x3, 4

      // FASE 4: stores (base x7 = 0x400, dato x8 = 0x12345678)
      prog['h06C>>2] = enc_s(32'd4,      5'd8, 5'd7, 3'b000);         // sb x8, 4(x7)
      prog['h070>>2] = enc_s(32'd8,      5'd8, 5'd7, 3'b001);         // sh x8, 8(x7)
      prog['h074>>2] = enc_s(-32'd4,     5'd8, 5'd7, 3'b010);         // sw x8, -4(x7)

      // FASE 5: loads (mem[0x400] = 0xCAFEF08A precargado por el testbench)
      prog['h078>>2] = enc_i(32'd0,      5'd7, 3'b000, 5'd29, OP_LOAD); // lb  x29, 0(x7)
      prog['h07C>>2] = enc_i(32'd0,      5'd7, 3'b001, 5'd30, OP_LOAD); // lh  x30, 0(x7)
      prog['h080>>2] = enc_i(32'd0,      5'd7, 3'b010, 5'd9,  OP_LOAD); // lw  x9,  0(x7)
      prog['h084>>2] = enc_i(32'd0,      5'd7, 3'b100, 5'd6,  OP_LOAD); // lbu x6,  0(x7)
      prog['h088>>2] = enc_i(32'd0,      5'd7, 3'b101, 5'd10, OP_LOAD); // lhu x10, 0(x7)
      prog['h08C>>2] = enc_i(-32'd4,     5'd7, 3'b010, 5'd11, OP_LOAD); // lw  x11, -4(x7) (lee el sw)
      prog['h090>>2] = enc_i(32'd4,      5'd7, 3'b010, 5'd12, OP_LOAD); // lw  x12, 4(x7)  (lee el sb)
      prog['h094>>2] = enc_i(32'd8,      5'd7, 3'b010, 5'd13, OP_LOAD); // lw  x13, 8(x7)  (lee el sh)

      if (!TEST_JUMPS) begin
        // FASE 7 sin saltos: tipos U. El resto de la memoria queda a 0
        prog['h098>>2] = enc_u(20'hABCDE, 5'd22, OP_LUI);             // lui   x22, 0xABCDE
        prog['h09C>>2] = enc_u(20'h00001, 5'd23, OP_AUIPC);           // auipc x23, 0x00001
        prog['h0A0>>2] = enc_i(32'd77, 5'd0, 3'b000, 5'd27, OP_I);    // addi  x27, x0, 77
        return;
      end

      // FASE 6: bifurcaciones tomadas (+8 salta la trampa)
      prog['h098>>2] = enc_b(32'd8, 5'd5, 5'd1, 3'b000);              // beq  x1, x5, +8
      prog['h09C>>2] = TRAP;
      prog['h0A0>>2] = enc_b(32'd8, 5'd2, 5'd1, 3'b001);              // bne  x1, x2, +8
      prog['h0A4>>2] = TRAP;
      prog['h0A8>>2] = enc_b(32'd8, 5'd2, 5'd3, 3'b100);              // blt  x3, x2, +8
      prog['h0AC>>2] = TRAP;
      prog['h0B0>>2] = enc_b(32'd8, 5'd2, 5'd1, 3'b101);              // bge  x1, x2, +8
      prog['h0B4>>2] = TRAP;
      prog['h0B8>>2] = enc_b(32'd8, 5'd3, 5'd2, 3'b110);              // bltu x2, x3, +8
      prog['h0BC>>2] = TRAP;
      prog['h0C0>>2] = enc_b(32'd8, 5'd2, 5'd3, 3'b111);              // bgeu x3, x2, +8
      prog['h0C4>>2] = TRAP;
      // bifurcaciones no tomadas (se ejecuta la instruccion siguiente)
      prog['h0C8>>2] = enc_b(32'd8, 5'd2, 5'd1, 3'b000);              // beq  x1, x2, +8
      prog['h0CC>>2] = enc_i(32'd1, 5'd0, 3'b000, 5'd14, OP_I);       // addi x14, x0, 1
      prog['h0D0>>2] = enc_b(32'd8, 5'd5, 5'd1, 3'b001);              // bne  x1, x5, +8
      prog['h0D4>>2] = enc_i(32'd2, 5'd0, 3'b000, 5'd15, OP_I);       // addi x15, x0, 2
      prog['h0D8>>2] = enc_b(32'd8, 5'd3, 5'd2, 3'b100);              // blt  x2, x3, +8
      prog['h0DC>>2] = enc_i(32'd3, 5'd0, 3'b000, 5'd16, OP_I);       // addi x16, x0, 3
      prog['h0E0>>2] = enc_b(32'd8, 5'd2, 5'd3, 3'b101);              // bge  x3, x2, +8
      prog['h0E4>>2] = enc_i(32'd4, 5'd0, 3'b000, 5'd17, OP_I);       // addi x17, x0, 4
      prog['h0E8>>2] = enc_b(32'd8, 5'd2, 5'd3, 3'b110);              // bltu x3, x2, +8
      prog['h0EC>>2] = enc_i(32'd5, 5'd0, 3'b000, 5'd18, OP_I);       // addi x18, x0, 5
      prog['h0F0>>2] = enc_b(32'd8, 5'd3, 5'd2, 3'b111);              // bgeu x2, x3, +8
      prog['h0F4>>2] = enc_i(32'd6, 5'd0, 3'b000, 5'd19, OP_I);       // addi x19, x0, 6
      // bucle con salto hacia atras: 3 iteraciones
      prog['h0F8>>2] = enc_i(32'd3,  5'd0,  3'b000, 5'd20, OP_I);     // addi x20, x0, 3
      prog['h0FC>>2] = enc_i(-32'd1, 5'd20, 3'b000, 5'd20, OP_I);     // loop: addi x20, x20, -1
      prog['h100>>2] = enc_b(-32'd4, 5'd0, 5'd20, 3'b001);            // bne  x20, x0, loop

      // FASE 7: tipos U y J
      prog['h104>>2] = enc_u(20'hABCDE, 5'd22, OP_LUI);               // lui   x22, 0xABCDE
      prog['h108>>2] = enc_u(20'h00001, 5'd23, OP_AUIPC);             // auipc x23, 0x00001
      prog['h10C>>2] = enc_j(32'd12, 5'd24);                          // jal   x24, +12
      prog['h110>>2] = TRAP;
      prog['h114>>2] = TRAP;
      prog['h118>>2] = enc_u(20'h00000, 5'd25, OP_AUIPC);              // auipc x25, 0  -> x25 = PROG_BASE + 0x118
      prog['h11C>>2] = enc_i(32'd20,  5'd25, 3'b000, 5'd26, OP_JALR); // jalr  x26, 20(x25) -> PROG_BASE + 0x12C
      prog['h120>>2] = TRAP;
      prog['h124>>2] = TRAP;
      prog['h128>>2] = TRAP;
      prog['h12C>>2] = enc_i(32'd77,  5'd0, 3'b000, 5'd27, OP_I);     // addi  x27, x0, 77
      prog['h130>>2] = enc_j(32'd0, 5'd0);                            // halt: jal x0, 0
    end
  endtask

  // ---------------------------------------------------------------------------
  // Carga del programa y de los datos en la memoria por el puerto A
  // (el core esta en reset y el testbench controla la memoria)
  // ---------------------------------------------------------------------------
  task automatic load_memory();
    begin
`ifndef USE_RTL_MEM
      // Agente: carga por backdoor (sin consumir ciclos)
      for (int i = 0; i < PROG_WORDS; i++) u_mem.backdoor_write((PROG_BASE >> 2) + i, prog[i]);
      u_mem.backdoor_write(DATA_BASE >> 2, DATA_INIT);
      load_en = 1'b0;
      $display("  [LOAD] (agente) %0d instrucciones cargadas en 0x%05H..0x%05H", PROG_WORDS, PROG_BASE, PROG_BASE + (PROG_WORDS-1)*4);
      $display("  [LOAD] (agente) Dato 0x%08H cargado en 0x%03H", DATA_INIT, DATA_BASE);
      return;
`endif
      load_en = 1'b1;
      tb_we   = 1'b1;
      for (int i = 0; i < PROG_WORDS; i++) begin
        @(negedge clk);
        tb_addr  = (PROG_BASE >> 2) + i;
        tb_wdata = prog[i];
      end
      @(negedge clk);
      tb_addr  = DATA_BASE >> 2;
      tb_wdata = DATA_INIT;
      @(negedge clk);
      tb_we    = 1'b0;
      load_en  = 1'b0;
      $display("  [LOAD] %0d instrucciones cargadas en 0x%05H..0x%05H", PROG_WORDS, PROG_BASE, PROG_BASE + (PROG_WORDS-1)*4);
      $display("  [LOAD] Dato 0x%08H cargado en 0x%03H", DATA_INIT, DATA_BASE);
    end
  endtask

  // ---------------------------------------------------------------------------
  // Espera a que la instruccion de la direccion pc llegue al decoder
  // ---------------------------------------------------------------------------
  task automatic wait_decode(
    input  logic [31:0] pc,
    input  string       test_name,
    output bit          found
  );
    begin
      found = 1'b0;
      for (int i = 0; i < DECODE_TIMEOUT; i++) begin
        @(negedge clk);
        if (mon_instr === prog[pc>>2]) begin
          found = 1'b1;
          break;
        end
      end
      #2; // Propagación combinacional: decoder -> regFile -> alu
      $display("------------------------------------------------------------------");
      assert (found)
        else $error("Fallo en %s: la instruccion 0x%08H (PC=0x%03H) no llega al decoder en %0d ciclos (PC fetch=0x%05H, instr=0x%08H)",
                    test_name, prog[pc>>2], pc, DECODE_TIMEOUT, mon_pc, mon_instr);
    end
  endtask

  // ---------------------------------------------------------------------------
  // Comprueba (en segundo plano) que el registro rd llega a valer exp_val
  // en un maximo de n_cycles flancos tras la decodificacion
  // ---------------------------------------------------------------------------
  task automatic check_reg(
    input logic [4:0]  reg_addr,
    input logic [31:0] exp_val,
    input string       test_name,
    input int          n_cycles
  );
    logic [31:0] val;
    begin
      for (int i = 0; i < n_cycles; i++) begin
        @(posedge clk);
        #1;
        val = dut.U1.U0.regs[reg_addr];
        if (val === exp_val) break;
      end
      $display("  Registro destino -> x%0d = 0x%08H (%0d)", reg_addr, val, $signed(val));
      assert (val === exp_val)
        else $error("Fallo escritura x%0d en %s: esp 0x%08H, obt 0x%08H", reg_addr, test_name, exp_val, val);
    end
  endtask

  // ---------------------------------------------------------------------------
  // Comprueba (en segundo plano) que la palabra de memoria llega a valer exp_word
  // ---------------------------------------------------------------------------
  task automatic check_mem(
    input logic [31:0] byte_addr,
    input logic [31:0] exp_word,
    input string       test_name,
    input int          n_cycles
  );
    logic [31:0] val;
    begin
      for (int i = 0; i < n_cycles; i++) begin
        @(posedge clk);
        #1;
        val = u_mem.mem[byte_addr >> 2];
        if (val === exp_word) break;
      end
      $display("  Memoria          -> mem[0x%03H] = 0x%08H", byte_addr, val);
      assert (val === exp_word)
        else $error("Fallo memoria en %s: mem[0x%03H] esp 0x%08H, obt 0x%08H", test_name, byte_addr, exp_word, val);
    end
  endtask

  // ---------------------------------------------------------------------------
  // Instruccion ALU (tipo R o tipo I): comprueba la ALU y despues que el
  // resultado se ha escrito en el registro destino
  // ---------------------------------------------------------------------------
  task automatic test_alu(
    input logic [31:0] pc,
    input string       test_name,
    input logic [4:0]  exp_opcode,
    input logic [31:0] exp_rdata1,
    input logic [31:0] exp_rdata2,
    input logic [31:0] exp_wAlu,
    input logic        exp_eq,
    input logic        exp_lt
  );
    bit          found;
    logic [4:0]  rd;
    begin
      rd = prog[pc>>2][11:7];
      if (rd != 0) exp_regs[rd] = exp_wAlu;   // modelo esperado (aunque la instruccion no llegue)
      wait_decode(pc, test_name, found);
      if (found) begin
        $display("[%s] PC = 0x%03H | Instr = 0x%08H | opcode = %b | aloup = %b",
                 test_name, pc, prog[pc>>2], mon_opcode, mon_aloup);
        $display("  Registros leidos -> rs1(x%0d)=0x%08H, rs2(x%0d)=0x%08H",
                 mon_rs1, mon_rdata1, mon_rs2, mon_rdata2);
        $display("  Salida ALU       -> wAlu=0x%08H (%0d) | eq=%b, lt=%b, ltu=%b",
                 mon_wAlu, $signed(mon_wAlu), mon_eq, mon_lt, mon_ltu);

        assert (mon_opcode  === exp_opcode) else $error("Fallo opcode en %s", test_name);
        assert (mon_rdata1  === exp_rdata1) else $error("Fallo rdata1 en %s: esp 0x%08H, obt 0x%08H", test_name, exp_rdata1, mon_rdata1);
        assert (mon_rdata2  === exp_rdata2) else $error("Fallo rdata2 en %s: esp 0x%08H, obt 0x%08H", test_name, exp_rdata2, mon_rdata2);
        assert (mon_wAlu    === exp_wAlu)   else $error("Fallo wAlu en %s: esp 0x%08H, obt 0x%08H", test_name, exp_wAlu, mon_wAlu);
        assert (mon_eq      === exp_eq)     else $error("Fallo flag eq en %s", test_name);
        assert (mon_lt      === exp_lt)     else $error("Fallo flag lt en %s", test_name);
        if (TEST_JUMPS)
          assert (mon_pc_load === 1'b0)     else $error("Fallo en %s: no debe saltar (pc_load=%b)", test_name, mon_pc_load);

        check_reg(rd, exp_wAlu, test_name, WB_CYCLES);
      end
    end
  endtask

  // ---------------------------------------------------------------------------
  // Bifurcacion (tipo B): compara rs1 con rs2, offset en imm, decision del
  // salto (pc_load) y destino (pc_target). No escribe en registros ni memoria
  // ---------------------------------------------------------------------------
  task automatic test_branch(
    input logic [31:0] pc,
    input string       test_name,
    input logic [31:0] exp_rdata1,
    input logic [31:0] exp_rdata2,
    input logic [31:0] exp_imm,
    input logic        exp_eq,
    input logic        exp_lt,
    input logic        exp_ltu,
    input logic        exp_taken
  );
    bit          found;
    logic [31:0] target;
    begin
      wait_decode(pc, test_name, found);
      if (found) begin
        target = PROG_BASE + pc + exp_imm;  // destino absoluto
        $display("[%s] PC = 0x%03H | Instr = 0x%08H | opcode = %b | imm = 0x%08H (%0d)",
                 test_name, pc, prog[pc>>2], mon_opcode, mon_imm, $signed(mon_imm));
        $display("  Registros leidos -> rs1(x%0d)=0x%08H, rs2(x%0d)=0x%08H | eq=%b, lt=%b, ltu=%b",
                 mon_rs1, mon_rdata1, mon_rs2, mon_rdata2, mon_eq, mon_lt, mon_ltu);
        $display("  Salto            -> pc_load=%b (esp %b), pc_target=0x%05H (esp 0x%05H)",
                 mon_pc_load, exp_taken, mon_pc_target, exp_taken ? target : PROG_BASE + pc + 4);

        assert (mon_opcode  === 5'b11000)   else $error("Fallo opcode en %s", test_name);
        assert (mon_rdata1  === exp_rdata1) else $error("Fallo rdata1 en %s: esp 0x%08H, obt 0x%08H", test_name, exp_rdata1, mon_rdata1);
        assert (mon_rdata2  === exp_rdata2) else $error("Fallo rdata2 en %s: esp 0x%08H, obt 0x%08H", test_name, exp_rdata2, mon_rdata2);
        assert (mon_imm     === exp_imm)    else $error("Fallo imm en %s: esp 0x%08H, obt 0x%08H", test_name, exp_imm, mon_imm);
        assert (mon_eq      === exp_eq)     else $error("Fallo flag eq en %s", test_name);
        assert (mon_lt      === exp_lt)     else $error("Fallo flag lt en %s", test_name);
        assert (mon_ltu     === exp_ltu)    else $error("Fallo flag ltu en %s", test_name);
        assert (mon_we      === 1'b0)       else $error("Fallo en %s: no debe escribir en registros", test_name);
        assert (mon_we_mem  === 1'b0)       else $error("Fallo en %s: no debe escribir en memoria", test_name);
        assert (mon_pc_load === exp_taken)  else $error("Fallo decision de salto en %s: pc_load esp %b, obt %b", test_name, exp_taken, mon_pc_load);
        if (exp_taken)
          assert (mon_pc_target === target[ADDR_WIDTH-1:0])
            else $error("Fallo destino de salto en %s: esp 0x%05H, obt 0x%05H", test_name, target[ADDR_WIDTH-1:0], mon_pc_target);
      end
    end
  endtask

  // ---------------------------------------------------------------------------
  // Store (tipo S): direccion = rs1 + imm, dato = rs2, enables de memoria
  // y despues se comprueba el contenido de la memoria
  // ---------------------------------------------------------------------------
  task automatic test_store(
    input logic [31:0] pc,
    input string       test_name,
    input logic [31:0] exp_addr,
    input logic [31:0] exp_data,
    input logic [3:0]  exp_be,
    input logic [31:0] exp_mem_word
  );
    bit found;
    begin
      wait_decode(pc, test_name, found);
      if (found) begin
        $display("[%s] PC = 0x%03H | Instr = 0x%08H | opcode = %b", test_name, pc, prog[pc>>2], mon_opcode);
        $display("  Hacia memoria    -> addr=0x%08H, dato=0x%08H, we_b=%b, be_b=%b",
                 mon_wAlu, mon_rdata2, mon_we_mem, mon_be_mem);

        assert (mon_opcode === 5'b01000) else $error("Fallo opcode en %s", test_name);
        assert (mon_wAlu   === exp_addr) else $error("Fallo direccion en %s: esp 0x%08H, obt 0x%08H", test_name, exp_addr, mon_wAlu);
        assert (mon_rdata2 === exp_data) else $error("Fallo dato en %s: esp 0x%08H, obt 0x%08H", test_name, exp_data, mon_rdata2);
        assert (mon_we_mem === 1'b1)     else $error("Fallo we_b en %s: esp 1, obt %b", test_name, mon_we_mem);
        assert (mon_be_mem === exp_be)   else $error("Fallo be_b en %s: esp %b, obt %b", test_name, exp_be, mon_be_mem);
        assert (mon_we     === 1'b0)     else $error("Fallo en %s: un store no debe escribir en registros", test_name);

        fork
          check_mem(exp_addr, exp_mem_word, test_name, STORE_CYCLES);
        join_none
        @(posedge clk);
      end
    end
  endtask

  // ---------------------------------------------------------------------------
  // Load (tipo I): direccion = rs1 + imm, el dato viene de la memoria real
  // por el puerto B. Despues se comprueba el valor del registro destino.
  // ---------------------------------------------------------------------------
  task automatic test_load(
    input logic [31:0] pc,
    input string       test_name,
    input logic [31:0] exp_addr,
    input logic [31:0] exp_rd_val
  );
    bit         found;
    logic [4:0] rd;
    begin
      rd = prog[pc>>2][11:7];
      exp_regs[rd] = exp_rd_val;               // modelo esperado (aunque la instruccion no llegue)
      wait_decode(pc, test_name, found);
      if (found) begin
        $display("[%s] PC = 0x%03H | Instr = 0x%08H | opcode = %b", test_name, pc, prog[pc>>2], mon_opcode);
        $display("  Memoria          -> addr=0x%08H, mem=0x%08H", mon_wAlu, u_mem.mem[exp_addr >> 2]);

        assert (mon_opcode === 5'b00000) else $error("Fallo opcode en %s", test_name);
        assert (mon_wAlu   === exp_addr) else $error("Fallo direccion en %s: esp 0x%08H, obt 0x%08H", test_name, exp_addr, mon_wAlu);
        assert (mon_we_mem === 1'b0)     else $error("Fallo en %s: un load no debe escribir en memoria", test_name);
        assert (mon_wl     === 1'b1)     else $error("Fallo wl en %s: un load debe escribir el dato de memoria en registros", test_name);
        assert (mon_we     === 1'b0)     else $error("Fallo we en %s: un load no debe escribir la salida de la ALU en registros", test_name);

        fork
          check_reg(rd, exp_rd_val, test_name, LOAD_CYCLES);
        join_none
        @(posedge clk);
      end
    end
  endtask

  // ---------------------------------------------------------------------------
  // Tipos U y J (lui, auipc, jal, jalr): decodificacion (opcode, rd, imm),
  // salto (pc_load / pc_target) y valor escrito en rd (imm, PC+imm o PC+4)
  // ---------------------------------------------------------------------------
  task automatic test_upper_jump(
    input logic [31:0] pc,
    input string       test_name,
    input logic [4:0]  exp_opcode,
    input logic [31:0] exp_imm,
    input logic [31:0] exp_rd_val,
    input logic        exp_jump,
    input logic [31:0] exp_target
  );
    bit         found;
    logic [4:0] rd;
    begin
      rd = prog[pc>>2][11:7];
      if (rd != 0) exp_regs[rd] = exp_rd_val;  // modelo esperado (aunque la instruccion no llegue)
      wait_decode(pc, test_name, found);
      if (found) begin
        $display("[%s] PC = 0x%03H | Instr = 0x%08H | opcode = %b | rd = x%0d | imm = 0x%08H | we = %b",
                 test_name, pc, prog[pc>>2], mon_opcode, mon_rd, mon_imm, mon_we);
        $display("  Salto            -> pc_load=%b (esp %b), pc_target=0x%05H (esp 0x%05H)",
                 mon_pc_load, exp_jump, mon_pc_target, exp_jump ? exp_target : PROG_BASE + pc + 4);

        assert (mon_opcode  === exp_opcode) else $error("Fallo opcode en %s", test_name);
        assert (mon_rd      === rd)         else $error("Fallo rd en %s: esp x%0d, obt x%0d", test_name, rd, mon_rd);
        assert (mon_imm     === exp_imm)    else $error("Fallo imm en %s: esp 0x%08H, obt 0x%08H", test_name, exp_imm, mon_imm);
        assert (mon_we      === 1'b1)       else $error("Fallo en %s: debe escribir en registros", test_name);
        assert (mon_we_mem  === 1'b0)       else $error("Fallo en %s: no debe escribir en memoria", test_name);
        if (TEST_JUMPS)
          assert (mon_pc_load === exp_jump) else $error("Fallo pc_load en %s: esp %b, obt %b", test_name, exp_jump, mon_pc_load);
        if (exp_jump)
          assert (mon_pc_target === exp_target[ADDR_WIDTH-1:0])
            else $error("Fallo destino de salto en %s: esp 0x%05H, obt 0x%05H", test_name, exp_target[ADDR_WIDTH-1:0], mon_pc_target);

        if (rd != 0)
          check_reg(rd, exp_rd_val, test_name, WB_CYCLES);
        else
          @(posedge clk);
      end
    end
  endtask

  // ---------------------------------------------------------------------------
  // Estado final: banco de registros completo, trampas y memoria de datos
  // ---------------------------------------------------------------------------
  task automatic check_final_state();
    logic [31:0] val;
    begin
      $display("------------------------------------------------------------------");
      repeat (10) @(posedge clk);
      #1;
      if (TEST_JUMPS) begin
        // El procesador debe quedarse en el bucle final (jal x0, 0 en 0x130).
        // Cada salto tomado anula la instruccion siguiente, asi que en el decoder
        // se alternan el halt y una burbuja (NOP): se observan dos ciclos
        bit halt_seen;
        halt_seen = 1'b0;
        for (int c = 0; c < 2; c++) begin
          $display("  [HALT] PC fetch = 0x%05H | instr en decoder = 0x%08H", mon_pc, mon_instr);
          if (mon_instr === prog['h130>>2]) halt_seen = 1'b1;
          else assert (mon_instr === 32'h0000_0013)
            else $error("Fallo HALT: instruccion inesperada en el bucle final (instr=0x%08H)", mon_instr);
          assert (mon_pc <= PROG_BASE + 'h138 && mon_pc >= PROG_BASE + 'h130)
            else $error("Fallo HALT: PC fuera del bucle final (PC=0x%05H)", mon_pc);
          @(posedge clk); #1;
        end
        assert (halt_seen)
          else $error("Fallo HALT: el procesador no esta en el bucle final (no se ve jal x0, 0)");
      end else
        $display("  [FIN] PC fetch = 0x%05H (sin saltos el PC sigue avanzando por memoria vacia)", mon_pc);

      // Trampas: x31 cuenta las instrucciones que no debian ejecutarse
      val = dut.U1.U0.regs[31];
      assert (val === 32'd0)
        else $error("Fallo bifurcaciones: se han ejecutado %0d instrucciones trampa (x31)", val);

      // Banco de registros completo
      for (int i = 0; i < 32; i++) begin
        val = dut.U1.U0.regs[i];
        $display("  x%0d\t= 0x%08H (esp 0x%08H)%s", i, val, exp_regs[i], (val === exp_regs[i]) ? "" : "  <-- ERROR");
        assert (val === exp_regs[i])
          else $error("Fallo estado final x%0d: esp 0x%08H, obt 0x%08H", i, exp_regs[i], val);
      end

      // Memoria de datos
      check_mem(DATA_BASE - 4, 32'h1234_5678, "FINAL sw",    1);
      check_mem(DATA_BASE,     DATA_INIT,     "FINAL dato",  1);
      check_mem(DATA_BASE + 4, 32'h0000_0078, "FINAL sb",    1);
      check_mem(DATA_BASE + 8, 32'h0000_5678, "FINAL sh",    1);
`ifndef USE_RTL_MEM
      u_mem.dump((DATA_BASE >> 2) - 1, (DATA_BASE >> 2) + 2);
      u_mem.report();
`endif
    end
  endtask

  initial begin
    $display("=== INICIO SIMULACION CORE: FETCH, DECEXE Y MEMORIA ===");

    // 1. Reset inicial y carga del programa en memoria
    nRst     = 1'b0;
    load_en  = 1'b1;
    tb_we    = 1'b0;
    tb_addr  = '0;
    tb_wdata = '0;
    foreach (exp_regs[i]) exp_regs[i] = 32'h0;
    build_program();
    load_memory();
    @(negedge clk);
    nRst = 1'b1;

    // =========================================================================
    // FASE 1: INICIALIZACION DE REGISTROS
    // =========================================================================
    $display("\n--- FASE 1: INICIALIZACION DE REGISTROS ---");
    test_alu('h000, "I ADDI x1, x0, 20",    5'b00100, 32'd0, 32'd0, 32'd20,  1'b0, 1'b1);
    test_alu('h004, "I ADDI x2, x0, 5",     5'b00100, 32'd0, 32'd0, 32'd5,   1'b0, 1'b1);
    test_alu('h008, "I ADDI x3, x0, -10",   5'b00100, 32'd0, 32'd0, -32'd10, 1'b0, 1'b0);
    test_alu('h00C, "I ADDI x4, x0, 255",   5'b00100, 32'd0, 32'd0, 32'd255, 1'b0, 1'b1);
    test_alu('h010, "I ADDI x5, x0, 20",    5'b00100, 32'd0, 32'd0, 32'd20,  1'b0, 1'b1);
    test_alu('h014, "I ADDI x7, x0, 0x400", 5'b00100, 32'd0, 32'd0, DATA_BASE, 1'b0, 1'b1);
    test_upper_jump('h018, "U LUI  x8, 0x12345", 5'b01101, 32'h1234_5000, 32'h1234_5000, 1'b0, 32'h0);
    test_alu('h01C, "I ADDI x8, x8, 0x678", 5'b00100, 32'h1234_5000, 32'd0, 32'h1234_5678, 1'b0, 1'b0);

    // =========================================================================
    // FASE 2: ALU TIPO R
    // =========================================================================
    $display("\n--- FASE 2: ALU TIPO R ---");
    test_alu('h020, "R ADD  x10, x1, x2", 5'b01100, 32'd20,  32'd5,         32'd25,         1'b0, 1'b0);
    test_alu('h024, "R SUB  x11, x1, x2", 5'b01100, 32'd20,  32'd5,         32'd15,         1'b0, 1'b0);
    test_alu('h028, "R SLL  x12, x1, x2", 5'b01100, 32'd20,  32'd5,         32'd640,        1'b0, 1'b0);
    test_alu('h02C, "R SLT  x13, x3, x2", 5'b01100, -32'd10, 32'd5,         32'd1,          1'b0, 1'b1);
    test_alu('h030, "R SLTU x14, x3, x2", 5'b01100, -32'd10, 32'd5,         32'd0,          1'b0, 1'b1);
    test_alu('h034, "R XOR  x15, x1, x4", 5'b01100, 32'd20,  32'h0000_00FF, 32'h0000_00EB, 1'b0, 1'b1);
    test_alu('h038, "R SRL  x16, x3, x2", 5'b01100, -32'd10, 32'd5,         32'h07FF_FFFF, 1'b0, 1'b1);
    test_alu('h03C, "R SRA  x17, x3, x2", 5'b01100, -32'd10, 32'd5,         32'hFFFF_FFFF, 1'b0, 1'b1);
    test_alu('h040, "R OR   x18, x1, x4", 5'b01100, 32'd20,  32'h0000_00FF, 32'h0000_00FF, 1'b0, 1'b1);
    test_alu('h044, "R AND  x19, x1, x4", 5'b01100, 32'd20,  32'h0000_00FF, 32'h0000_0014, 1'b0, 1'b1);

    // =========================================================================
    // FASE 3: ALU TIPO I (rdata2 = x0 y los flags comparan rs1 con imm)
    // =========================================================================
    $display("\n--- FASE 3: ALU TIPO I ---");
    test_alu('h048, "I ADDI  x20, x1, -5",    5'b00100, 32'd20,        32'd0, 32'd15,         1'b0, 1'b0);
    test_alu('h04C, "I SLTI  x21, x3, 5",     5'b00100, -32'd10,       32'd0, 32'd1,          1'b0, 1'b1);
    test_alu('h050, "I SLTIU x22, x1, -1",    5'b00100, 32'd20,        32'd0, 32'd1,          1'b0, 1'b0);
    test_alu('h054, "I XORI  x23, x1, 0xF0",  5'b00100, 32'd20,        32'd0, 32'h0000_00E4, 1'b0, 1'b1);
    test_alu('h058, "I ORI   x24, x1, 0x100", 5'b00100, 32'd20,        32'd0, 32'h0000_0114, 1'b0, 1'b1);
    test_alu('h05C, "I ANDI  x25, x4, 0x0F",  5'b00100, 32'h0000_00FF, 32'd0, 32'h0000_000F, 1'b0, 1'b0);
    test_alu('h060, "I SLLI  x26, x1, 3",     5'b00100, 32'd20,        32'd0, 32'd160,        1'b0, 1'b0);
    test_alu('h064, "I SRLI  x27, x3, 4",     5'b00100, -32'd10,       32'd0, 32'h0FFF_FFFF, 1'b0, 1'b1);
    test_alu('h068, "I SRAI  x28, x3, 4",     5'b00100, -32'd10,       32'd0, 32'hFFFF_FFFF, 1'b0, 1'b1);

    // =========================================================================
    // FASE 4: STORES TIPO S (base x7 = 0x400, dato x8 = 0x12345678)
    // =========================================================================
    $display("\n--- FASE 4: STORES TIPO S ---");
    test_store('h06C, "S SB x8, 4(x7)",  32'h0000_0404, 32'h1234_5678, 4'b0001, 32'h0000_0078);
    test_store('h070, "S SH x8, 8(x7)",  32'h0000_0408, 32'h1234_5678, 4'b0011, 32'h0000_5678);
    test_store('h074, "S SW x8, -4(x7)", 32'h0000_03FC, 32'h1234_5678, 4'b1111, 32'h1234_5678);

    // =========================================================================
    // FASE 5: LOADS TIPO I (mem[0x400] = 0xCAFEF08A)
    // =========================================================================
    $display("\n--- FASE 5: LOADS TIPO I ---");
    test_load('h078, "L LB  x29, 0(x7)",  32'h0000_0400, 32'hFFFF_FF8A);
    test_load('h07C, "L LH  x30, 0(x7)",  32'h0000_0400, 32'hFFFF_F08A);
    test_load('h080, "L LW  x9, 0(x7)",   32'h0000_0400, 32'hCAFE_F08A);
    test_load('h084, "L LBU x6, 0(x7)",   32'h0000_0400, 32'h0000_008A);
    test_load('h088, "L LHU x10, 0(x7)",  32'h0000_0400, 32'h0000_F08A);
    test_load('h08C, "L LW  x11, -4(x7)", 32'h0000_03FC, 32'h1234_5678);
    test_load('h090, "L LW  x12, 4(x7)",  32'h0000_0404, 32'h0000_0078);
    test_load('h094, "L LW  x13, 8(x7)",  32'h0000_0408, 32'h0000_5678);

    if (!TEST_JUMPS) begin
      // =======================================================================
      // FASE 6: sin saltos (TEST_JUMPS = 0) -> no se ejecuta
      // FASE 7: TIPOS U (lui, auipc) en ejecucion secuencial
      // =======================================================================
      $display("\n--- FASE 6: BIFURCACIONES DESACTIVADAS (TEST_JUMPS = 0) ---");
      $display("\n--- FASE 7: TIPOS U (sin saltos) ---");
      test_upper_jump('h098, "U LUI   x22, 0xABCDE", 5'b01101, 32'hABCD_E000, 32'hABCD_E000, 1'b0, 32'h0);
      test_upper_jump('h09C, "U AUIPC x23, 0x00001", 5'b00101, 32'h0000_1000, PROG_BASE + 32'h109C, 1'b0, 32'h0);
      test_alu       ('h0A0, "I ADDI  x27, x0, 77",  5'b00100, 32'd0, 32'd0, 32'd77, 1'b0, 1'b1);
    end else begin

    // =========================================================================
    // FASE 6: BIFURCACIONES TIPO B
    // =========================================================================
    $display("\n--- FASE 6: BIFURCACIONES TOMADAS ---");
    test_branch('h098, "B BEQ  x1, x5, +8 (tomado)", 32'd20,  32'd20,  32'd8, 1'b1, 1'b0, 1'b0, 1'b1);
    test_branch('h0A0, "B BNE  x1, x2, +8 (tomado)", 32'd20,  32'd5,   32'd8, 1'b0, 1'b0, 1'b0, 1'b1);
    test_branch('h0A8, "B BLT  x3, x2, +8 (tomado)", -32'd10, 32'd5,   32'd8, 1'b0, 1'b1, 1'b0, 1'b1);
    test_branch('h0B0, "B BGE  x1, x2, +8 (tomado)", 32'd20,  32'd5,   32'd8, 1'b0, 1'b0, 1'b0, 1'b1);
    test_branch('h0B8, "B BLTU x2, x3, +8 (tomado)", 32'd5,   -32'd10, 32'd8, 1'b0, 1'b0, 1'b1, 1'b1);
    test_branch('h0C0, "B BGEU x3, x2, +8 (tomado)", -32'd10, 32'd5,   32'd8, 1'b0, 1'b1, 1'b0, 1'b1);

    $display("\n--- FASE 6: BIFURCACIONES NO TOMADAS ---");
    test_branch('h0C8, "B BEQ  x1, x2, +8 (no tomado)", 32'd20,  32'd5,   32'd8, 1'b0, 1'b0, 1'b0, 1'b0);
    test_alu   ('h0CC, "I ADDI x14, x0, 1", 5'b00100, 32'd0, 32'd0, 32'd1, 1'b0, 1'b1);
    test_branch('h0D0, "B BNE  x1, x5, +8 (no tomado)", 32'd20,  32'd20,  32'd8, 1'b1, 1'b0, 1'b0, 1'b0);
    test_alu   ('h0D4, "I ADDI x15, x0, 2", 5'b00100, 32'd0, 32'd0, 32'd2, 1'b0, 1'b1);
    test_branch('h0D8, "B BLT  x2, x3, +8 (no tomado)", 32'd5,   -32'd10, 32'd8, 1'b0, 1'b0, 1'b1, 1'b0);
    test_alu   ('h0DC, "I ADDI x16, x0, 3", 5'b00100, 32'd0, 32'd0, 32'd3, 1'b0, 1'b1);
    test_branch('h0E0, "B BGE  x3, x2, +8 (no tomado)", -32'd10, 32'd5,   32'd8, 1'b0, 1'b1, 1'b0, 1'b0);
    test_alu   ('h0E4, "I ADDI x17, x0, 4", 5'b00100, 32'd0, 32'd0, 32'd4, 1'b0, 1'b1);
    test_branch('h0E8, "B BLTU x3, x2, +8 (no tomado)", -32'd10, 32'd5,   32'd8, 1'b0, 1'b1, 1'b0, 1'b0);
    test_alu   ('h0EC, "I ADDI x18, x0, 5", 5'b00100, 32'd0, 32'd0, 32'd5, 1'b0, 1'b1);
    test_branch('h0F0, "B BGEU x2, x3, +8 (no tomado)", 32'd5,   -32'd10, 32'd8, 1'b0, 1'b0, 1'b1, 1'b0);
    test_alu   ('h0F4, "I ADDI x19, x0, 6", 5'b00100, 32'd0, 32'd0, 32'd6, 1'b0, 1'b1);

    $display("\n--- FASE 6: BUCLE CON SALTO HACIA ATRAS (3 iteraciones) ---");
    test_alu   ('h0F8, "I ADDI x20, x0, 3",           5'b00100, 32'd0, 32'd0, 32'd3, 1'b0, 1'b1);
    test_alu   ('h0FC, "I ADDI x20, x20, -1 (iter 1)", 5'b00100, 32'd3, 32'd0, 32'd2, 1'b0, 1'b0);
    test_branch('h100, "B BNE  x20, x0, -4 (tomado)",    32'd2, 32'd0, -32'd4, 1'b0, 1'b0, 1'b0, 1'b1);
    test_alu   ('h0FC, "I ADDI x20, x20, -1 (iter 2)", 5'b00100, 32'd2, 32'd0, 32'd1, 1'b0, 1'b0);
    test_branch('h100, "B BNE  x20, x0, -4 (tomado)",    32'd1, 32'd0, -32'd4, 1'b0, 1'b0, 1'b0, 1'b1);
    test_alu   ('h0FC, "I ADDI x20, x20, -1 (iter 3)", 5'b00100, 32'd1, 32'd0, 32'd0, 1'b0, 1'b0);
    test_branch('h100, "B BNE  x20, x0, -4 (no tomado)", 32'd0, 32'd0, -32'd4, 1'b1, 1'b0, 1'b0, 1'b0);

    // =========================================================================
    // FASE 7: TIPOS U Y J
    // =========================================================================
    $display("\n--- FASE 7: TIPOS U Y J ---");
    test_upper_jump('h104, "U LUI   x22, 0xABCDE", 5'b01101, 32'hABCD_E000, 32'hABCD_E000, 1'b0, 32'h0);
    test_upper_jump('h108, "U AUIPC x23, 0x00001", 5'b00101, 32'h0000_1000, PROG_BASE + 32'h1108, 1'b0, 32'h0);
    test_upper_jump('h10C, "J JAL   x24, +12",     5'b11011, 32'd12,        PROG_BASE + 'h110, 1'b1, PROG_BASE + 'h118);
    test_upper_jump('h118, "U AUIPC x25, 0",       5'b00101, 32'h0,         PROG_BASE + 'h118, 1'b0, 32'h0);
    test_upper_jump('h11C, "I JALR  x26, 20(x25)", 5'b11001, 32'd20,        PROG_BASE + 'h120, 1'b1, PROG_BASE + 'h12C);
    test_alu       ('h12C, "I ADDI  x27, x0, 77",  5'b00100, 32'd0, 32'd0, 32'd77, 1'b0, 1'b1);
    test_upper_jump('h130, "J JAL   x0, 0 (halt)", 5'b11011, 32'd0,         32'h0,         1'b1, PROG_BASE + 'h130);
    end // TEST_JUMPS

    // =========================================================================
    // FASE 8: ESTADO FINAL
    // =========================================================================
    $display("\n--- FASE 8: ESTADO FINAL ---");
    check_final_state();

    #20;
    $display("------------------------------------------------------------------");
    $display("=== SIMULACION COMPLETADA (revisar los mensajes ** Error) ===");
    $finish;
  end

endmodule
