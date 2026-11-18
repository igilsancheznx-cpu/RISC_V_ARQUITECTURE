`timescale 1ns / 1ps

// =============================================================================
// tb_AluMemoryRegFileDec
// Comprobacion completa del bloque DecExe (decoder + regFile + alu) con todos
// los opcodes de RV32I:
//   FASE 1: precarga de registros
//   FASE 2: ALU tipo R   (add, sub, sll, slt, sltu, xor, srl, sra, or, and)
//   FASE 3: ALU tipo I   (addi, slti, sltiu, xori, ori, andi, slli, srli, srai)
//   FASE 4: saltos tipo B (beq, bne, blt, bge, bltu, bgeu)
//   FASE 5: stores tipo S (sb, sh, sw)
//   FASE 6: loads tipo I  (lb, lh, lw, lbu, lhu)
//   FASE 7: tipos U y J   (lui, auipc, jal, jalr)
//
// La memoria aun no esta conectada a DecExe: el testbench hace de memoria.
//   - Stores: se comprueba la direccion (wAlu), el dato (rdata2) y los
//             enables de memoria que genera el decoder (we_b, be_b).
//   - Loads:  el testbench pone el dato leido de memoria en rdata_b y se
//             comprueba que llega al registro destino.
// =============================================================================
module tb_AluMemoryRegFileDec;

  localparam int REG_WIDTH  = 32;
  localparam int ADDR_REGS  = 5;
  localparam int ADDR_WIDTH = 17;
  localparam int ALU_DECOD  = 4;

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

  // Puertos de DecExe
  logic                  clk;
  logic                  nRst;
  logic [REG_WIDTH-1:0]  rdata_a;
  logic [REG_WIDTH-1:0]  rdata_b;
  logic [ADDR_WIDTH-1:0] addr_b;
  logic [REG_WIDTH-1:0]  wdata_b;
  logic                  we_b;
  logic [ALU_DECOD-1:0]  be_b;

  // Monitorización jerárquica de señales intermedias entre bloques
  wire [4:0]           mon_opcode = dut.U1.opcode;
  wire [4:0]           mon_rs1    = dut.rs1;
  wire [4:0]           mon_rs2    = dut.rs2;
  wire [4:0]           mon_rd     = dut.rd;
  wire [REG_WIDTH-1:0] mon_imm    = dut.imm;
  wire [ALU_DECOD-1:0] mon_aloup  = dut.aloup;
  wire [REG_WIDTH-1:0] mon_rdata1 = dut.rdata1;
  wire [REG_WIDTH-1:0] mon_rdata2 = dut.rdata2;
  wire [REG_WIDTH-1:0] mon_wAlu   = dut.wAlu;
  wire                 mon_eq     = dut.eq;
  wire                 mon_lt     = dut.lt;
  wire                 mon_ltu    = dut.ltu;
  wire                 mon_we     = dut.we;        // escritura en banco de registros desde la ALU
  wire                 mon_wl     = dut.wl;        // escritura en banco de registros desde memoria (LOAD)
  wire                 mon_we_mem = dut.aux_we_b;  // escritura en memoria (decoder)
  wire [ALU_DECOD-1:0] mon_be_mem = dut.aux_be_b;  // byte enable de memoria (decoder)

  // Señal auxiliar
  logic [REG_WIDTH-1:0] preload_val_static;

  // Instancia del DUT
  DecExe #(
    .REG_WIDTH  (REG_WIDTH),
    .ADDR_REGS  (ADDR_REGS),
    .ADDR_WIDTH (ADDR_WIDTH),
    .ALU_DECOD  (ALU_DECOD)
  ) dut (
    .clk     (clk),
    .nRst    (nRst),
    .rdata_a (rdata_a),
    .rdata_b (rdata_b),
    .addr_b  (addr_b),
    .wdata_b (wdata_b),
    .we_b    (we_b),
    .be_b    (be_b)
  );

  // Reloj de 10 ns
  initial clk = 1'b0;
  always #5 clk = ~clk;

  // ---------------------------------------------------------------------------
  // Tarea auxiliar para precargar registros en U0 (regFile)
  // ---------------------------------------------------------------------------
  task automatic preload_reg(
    input logic [4:0]  reg_addr,
    input logic [31:0] reg_val
  );
    begin
      @(negedge clk);
      // Inyectamos una instrucción tipo I dummy cuyo rd sea reg_addr
      preload_val_static = reg_val;
      rdata_a = {12'd0, 5'd0, 3'b000, reg_addr, OP_I};
      force dut.we   = 1'b1;
      force dut.wAlu = preload_val_static;
      @(posedge clk);
      #1;
      release dut.we;
      release dut.wAlu;
      $display("  [PRELOAD] Registro x%0d <= 0x%08H (%0d)", reg_addr, reg_val, $signed(reg_val));
    end
  endtask

  // ---------------------------------------------------------------------------
  // Comprueba el valor de un registro leyendolo por el puerto 1:
  // add x0, xi, x0 -> rdata1 = xi (rd = x0, no modifica el banco)
  // ---------------------------------------------------------------------------
  task automatic check_reg(
    input logic [4:0]  reg_addr,
    input logic [31:0] exp_val,
    input string       test_name
  );
    begin
      @(negedge clk);
      rdata_a = {7'b0000000, 5'd0, reg_addr, 3'b000, 5'd0, OP_R};
      #2;
      $display("  Registro destino -> x%0d = 0x%08H (%0d)", reg_addr, mon_rdata1, $signed(mon_rdata1));
      assert (mon_rdata1 === exp_val)
        else $error("Fallo escritura x%0d en %s: esp 0x%08H, obt 0x%08H", reg_addr, test_name, exp_val, mon_rdata1);
      @(posedge clk);
    end
  endtask

  // ---------------------------------------------------------------------------
  // Tarea de comprobación de instrucciones operando sobre los registros precargados
  // (opcode, operandos leidos, salida de la ALU y flags de comparacion)
  // ---------------------------------------------------------------------------
  task automatic test_instr_with_regs(
    input logic [31:0] instruction_word,
    input string       test_name,
    input logic [4:0]  exp_opcode,
    input logic [31:0] exp_rdata1,
    input logic [31:0] exp_rdata2,
    input logic [31:0] exp_wAlu,
    input logic        exp_eq,
    input logic        exp_lt
  );
    begin
      @(negedge clk);
      rdata_a = instruction_word;
      #2; // Propagación combinacional: decoder -> regFile -> alu

      $display("------------------------------------------------------------------");
      $display("[%s] Instr = 0x%08H | opcode = %b | aloup = %b",
               test_name, instruction_word, mon_opcode, mon_aloup);
      $display("  Registros leidos -> rs1(x%0d)=0x%08H, rs2(x%0d)=0x%08H",
               mon_rs1, mon_rdata1, mon_rs2, mon_rdata2);
      $display("  Salida ALU       -> wAlu=0x%08H (%0d) | eq=%b, lt=%b, ltu=%b",
               mon_wAlu, $signed(mon_wAlu), mon_eq, mon_lt, mon_ltu);

      assert (mon_opcode === exp_opcode) else $error("Fallo opcode en %s", test_name);
      assert (mon_rdata1 === exp_rdata1) else $error("Fallo rdata1 en %s: esp 0x%08H, obt 0x%08H", test_name, exp_rdata1, mon_rdata1);
      assert (mon_rdata2 === exp_rdata2) else $error("Fallo rdata2 en %s: esp 0x%08H, obt 0x%08H", test_name, exp_rdata2, mon_rdata2);
      assert (mon_wAlu   === exp_wAlu)   else $error("Fallo wAlu en %s: esp 0x%08H, obt 0x%08H", test_name, exp_wAlu, mon_wAlu);
      assert (mon_eq     === exp_eq)     else $error("Fallo flag eq en %s", test_name);
      assert (mon_lt     === exp_lt)     else $error("Fallo flag lt en %s", test_name);

      @(posedge clk);
    end
  endtask

  // ---------------------------------------------------------------------------
  // Instruccion ALU (tipo R o tipo I): comprueba la ALU y despues que el
  // resultado se ha escrito en el registro destino
  // ---------------------------------------------------------------------------
  task automatic test_alu(
    input logic [31:0] instruction_word,
    input string       test_name,
    input logic [4:0]  exp_opcode,
    input logic [31:0] exp_rdata1,
    input logic [31:0] exp_rdata2,
    input logic [31:0] exp_wAlu,
    input logic        exp_eq,
    input logic        exp_lt
  );
    begin
      test_instr_with_regs(instruction_word, test_name, exp_opcode,
                           exp_rdata1, exp_rdata2, exp_wAlu, exp_eq, exp_lt);
      check_reg(instruction_word[11:7], exp_wAlu, test_name);
    end
  endtask

  // ---------------------------------------------------------------------------
  // Salto condicional (tipo B): compara rs1 con rs2, offset en imm
  // y no debe escribir ni en registros ni en memoria
  // ---------------------------------------------------------------------------
  task automatic test_branch(
    input logic [31:0] instruction_word,
    input string       test_name,
    input logic [31:0] exp_rdata1,
    input logic [31:0] exp_rdata2,
    input logic [31:0] exp_imm,
    input logic        exp_eq,
    input logic        exp_lt,
    input logic        exp_ltu
  );
    begin
      @(negedge clk);
      rdata_a = instruction_word;
      #2;

      $display("------------------------------------------------------------------");
      $display("[%s] Instr = 0x%08H | opcode = %b | imm = 0x%08H (%0d)",
               test_name, instruction_word, mon_opcode, mon_imm, $signed(mon_imm));
      $display("  Registros leidos -> rs1(x%0d)=0x%08H, rs2(x%0d)=0x%08H | eq=%b, lt=%b, ltu=%b",
               mon_rs1, mon_rdata1, mon_rs2, mon_rdata2, mon_eq, mon_lt, mon_ltu);

      assert (mon_opcode === 5'b11000) else $error("Fallo opcode en %s", test_name);
      assert (mon_rdata1 === exp_rdata1) else $error("Fallo rdata1 en %s: esp 0x%08H, obt 0x%08H", test_name, exp_rdata1, mon_rdata1);
      assert (mon_rdata2 === exp_rdata2) else $error("Fallo rdata2 en %s: esp 0x%08H, obt 0x%08H", test_name, exp_rdata2, mon_rdata2);
      assert (mon_imm    === exp_imm)    else $error("Fallo imm en %s: esp 0x%08H, obt 0x%08H", test_name, exp_imm, mon_imm);
      assert (mon_eq     === exp_eq)     else $error("Fallo flag eq en %s", test_name);
      assert (mon_lt     === exp_lt)     else $error("Fallo flag lt en %s", test_name);
      assert (mon_ltu    === exp_ltu)    else $error("Fallo flag ltu en %s", test_name);
      assert (mon_we     === 1'b0)       else $error("Fallo en %s: no debe escribir en registros", test_name);
      assert (mon_we_mem === 1'b0)       else $error("Fallo en %s: no debe escribir en memoria", test_name);

      @(posedge clk);
    end
  endtask

  // ---------------------------------------------------------------------------
  // Store (tipo S): direccion = rs1 + imm, dato = rs2, enables de memoria
  // y no debe escribir en el banco de registros
  // ---------------------------------------------------------------------------
  task automatic test_store(
    input logic [31:0] instruction_word,
    input string       test_name,
    input logic [31:0] exp_addr,
    input logic [31:0] exp_data,
    input logic [3:0]  exp_be
  );
    begin
      @(negedge clk);
      rdata_a = instruction_word;
      #2;

      $display("------------------------------------------------------------------");
      $display("[%s] Instr = 0x%08H | opcode = %b", test_name, instruction_word, mon_opcode);
      $display("  Hacia memoria    -> addr=0x%08H, dato=0x%08H, we_b=%b, be_b=%b",
               mon_wAlu, mon_rdata2, mon_we_mem, mon_be_mem);

      assert (mon_opcode === 5'b01000) else $error("Fallo opcode en %s", test_name);
      assert (mon_wAlu   === exp_addr) else $error("Fallo direccion en %s: esp 0x%08H, obt 0x%08H", test_name, exp_addr, mon_wAlu);
      assert (mon_rdata2 === exp_data) else $error("Fallo dato en %s: esp 0x%08H, obt 0x%08H", test_name, exp_data, mon_rdata2);
      assert (mon_we_mem === 1'b1)     else $error("Fallo we_b en %s: esp 1, obt %b", test_name, mon_we_mem);
      assert (mon_be_mem === exp_be)   else $error("Fallo be_b en %s: esp %b, obt %b", test_name, exp_be, mon_be_mem);
      assert (mon_we     === 1'b0)     else $error("Fallo en %s: un store no debe escribir en registros", test_name);

      @(posedge clk);
    end
  endtask

  // ---------------------------------------------------------------------------
  // Load (tipo I): direccion = rs1 + imm. El testbench hace de memoria y pone
  // mem_data en rdata_b. Despues se comprueba el valor del registro destino.
  // ---------------------------------------------------------------------------
  task automatic test_load(
    input logic [31:0] instruction_word,
    input string       test_name,
    input logic [31:0] exp_addr,
    input logic [31:0] mem_data,
    input logic [31:0] exp_rd_val
  );
    begin
      @(negedge clk);
      rdata_a = instruction_word;
      rdata_b = mem_data;   // dato devuelto por la memoria
      #2;

      $display("------------------------------------------------------------------");
      $display("[%s] Instr = 0x%08H | opcode = %b", test_name, instruction_word, mon_opcode);
      $display("  Memoria          -> addr=0x%08H, dato leido=0x%08H", mon_wAlu, mem_data);

      assert (mon_opcode === 5'b00000) else $error("Fallo opcode en %s", test_name);
      assert (mon_wAlu   === exp_addr) else $error("Fallo direccion en %s: esp 0x%08H, obt 0x%08H", test_name, exp_addr, mon_wAlu);
      assert (mon_we_mem === 1'b0)     else $error("Fallo en %s: un load no debe escribir en memoria", test_name);
      assert (mon_wl     === 1'b1)     else $error("Fallo wl en %s: un load debe escribir el dato de memoria en registros", test_name);
      assert (mon_we     === 1'b0)     else $error("Fallo we en %s: un load no debe escribir la salida de la ALU en registros", test_name);

      @(posedge clk);
      #1;
      rdata_b = 32'h0;
      check_reg(instruction_word[11:7], exp_rd_val, test_name);
    end
  endtask

  // ---------------------------------------------------------------------------
  // Tipos U y J (lui, auipc, jal, jalr): comprueba la decodificacion
  // (opcode, rd, imm) y que se habilita la escritura en el banco
  // ---------------------------------------------------------------------------
  task automatic test_upper_jump(
    input logic [31:0] instruction_word,
    input string       test_name,
    input logic [4:0]  exp_opcode,
    input logic [31:0] exp_imm
  );
    begin
      @(negedge clk);
      rdata_a = instruction_word;
      #2;

      $display("------------------------------------------------------------------");
      $display("[%s] Instr = 0x%08H | opcode = %b | rd = x%0d | imm = 0x%08H | we = %b",
               test_name, instruction_word, mon_opcode, mon_rd, mon_imm, mon_we);

      assert (mon_opcode === exp_opcode)              else $error("Fallo opcode en %s", test_name);
      assert (mon_rd     === instruction_word[11:7])  else $error("Fallo rd en %s: esp x%0d, obt x%0d", test_name, instruction_word[11:7], mon_rd);
      assert (mon_imm    === exp_imm)                 else $error("Fallo imm en %s: esp 0x%08H, obt 0x%08H", test_name, exp_imm, mon_imm);
      assert (mon_we     === 1'b1)                    else $error("Fallo en %s: debe escribir en registros", test_name);
      assert (mon_we_mem === 1'b0)                    else $error("Fallo en %s: no debe escribir en memoria", test_name);

      @(posedge clk);
    end
  endtask

  initial begin
    $display("=== INICIO SIMULACION DECEXE: ALU, BANCO DE REGISTROS Y MEMORIA ===");

    // 1. Reset inicial
    nRst    = 1'b0;
    rdata_a = 32'h0;
    rdata_b = 32'h0;
    repeat (2) @(posedge clk);
    nRst = 1'b1;

    // =========================================================================
    // FASE 1: PRECARGA DE REGISTROS
    // =========================================================================
    $display("\n--- FASE 1: PRECARGA DE REGISTROS ---");
    preload_reg(5'd1, 32'd20);          // x1 = 20
    preload_reg(5'd2, 32'd5);           // x2 = 5
    preload_reg(5'd3, -32'd10);         // x3 = -10 (0xFFFF_FFF6)
    preload_reg(5'd4, 32'h0000_00FF);   // x4 = 255
    preload_reg(5'd5, 32'd20);          // x5 = 20
    preload_reg(5'd7, 32'h0000_0100);   // x7 = 0x100 (direccion base de memoria)
    preload_reg(5'd8, 32'h1234_5678);   // x8 = dato para los stores

    // =========================================================================
    // FASE 2: ALU TIPO R  -> {funct7, rs2, rs1, funct3, rd, opcode}
    // =========================================================================
    $display("\n--- FASE 2: ALU TIPO R ---");

    // add x10, x1, x2 -> 20 + 5 = 25
    test_alu({7'b0000000, 5'd2, 5'd1, 3'b000, 5'd10, OP_R}, "R ADD  x10, x1, x2",
             5'b01100, 32'd20, 32'd5, 32'd25, 1'b0, 1'b0);

    // sub x11, x1, x2 -> 20 - 5 = 15
    test_alu({7'b0100000, 5'd2, 5'd1, 3'b000, 5'd11, OP_R}, "R SUB  x11, x1, x2",
             5'b01100, 32'd20, 32'd5, 32'd15, 1'b0, 1'b0);

    // sll x12, x1, x2 -> 20 << 5 = 640
    test_alu({7'b0000000, 5'd2, 5'd1, 3'b001, 5'd12, OP_R}, "R SLL  x12, x1, x2",
             5'b01100, 32'd20, 32'd5, 32'd640, 1'b0, 1'b0);

    // slt x13, x3, x2 -> (-10 < 5) = 1
    test_alu({7'b0000000, 5'd2, 5'd3, 3'b010, 5'd13, OP_R}, "R SLT  x13, x3, x2",
             5'b01100, -32'd10, 32'd5, 32'd1, 1'b0, 1'b1);

    // sltu x14, x3, x2 -> (0xFFFFFFF6 <u 5) = 0
    test_alu({7'b0000000, 5'd2, 5'd3, 3'b011, 5'd14, OP_R}, "R SLTU x14, x3, x2",
             5'b01100, -32'd10, 32'd5, 32'd0, 1'b0, 1'b1);

    // xor x15, x1, x4 -> 0x14 ^ 0xFF = 0xEB
    test_alu({7'b0000000, 5'd4, 5'd1, 3'b100, 5'd15, OP_R}, "R XOR  x15, x1, x4",
             5'b01100, 32'd20, 32'h0000_00FF, 32'h0000_00EB, 1'b0, 1'b1);

    // srl x16, x3, x2 -> 0xFFFFFFF6 >> 5 = 0x07FFFFFF
    test_alu({7'b0000000, 5'd2, 5'd3, 3'b101, 5'd16, OP_R}, "R SRL  x16, x3, x2",
             5'b01100, -32'd10, 32'd5, 32'h07FF_FFFF, 1'b0, 1'b1);

    // sra x17, x3, x2 -> -10 >>> 5 = -1
    test_alu({7'b0100000, 5'd2, 5'd3, 3'b101, 5'd17, OP_R}, "R SRA  x17, x3, x2",
             5'b01100, -32'd10, 32'd5, 32'hFFFF_FFFF, 1'b0, 1'b1);

    // or x18, x1, x4 -> 0x14 | 0xFF = 0xFF
    test_alu({7'b0000000, 5'd4, 5'd1, 3'b110, 5'd18, OP_R}, "R OR   x18, x1, x4",
             5'b01100, 32'd20, 32'h0000_00FF, 32'h0000_00FF, 1'b0, 1'b1);

    // and x19, x1, x4 -> 0x14 & 0xFF = 0x14
    test_alu({7'b0000000, 5'd4, 5'd1, 3'b111, 5'd19, OP_R}, "R AND  x19, x1, x4",
             5'b01100, 32'd20, 32'h0000_00FF, 32'h0000_0014, 1'b0, 1'b1);

    // =========================================================================
    // FASE 3: ALU TIPO I  -> {imm[11:0], rs1, funct3, rd, opcode}
    // rdata2 = x0 = 0 y los flags eq/lt comparan rs1 con imm (entrada op2 de la ALU)
    // =========================================================================
    $display("\n--- FASE 3: ALU TIPO I ---");

    // addi x20, x1, -5 -> 20 + (-5) = 15
    test_alu({12'hFFB, 5'd1, 3'b000, 5'd20, OP_I}, "I ADDI  x20, x1, -5",
             5'b00100, 32'd20, 32'd0, 32'd15, 1'b0, 1'b0);

    // slti x21, x3, 5 -> (-10 < 5) = 1
    test_alu({12'd5, 5'd3, 3'b010, 5'd21, OP_I}, "I SLTI  x21, x3, 5",
             5'b00100, -32'd10, 32'd0, 32'd1, 1'b0, 1'b1);

    // sltiu x22, x1, -1 -> (20 <u 0xFFFFFFFF) = 1
    test_alu({12'hFFF, 5'd1, 3'b011, 5'd22, OP_I}, "I SLTIU x22, x1, -1",
             5'b00100, 32'd20, 32'd0, 32'd1, 1'b0, 1'b0);

    // xori x23, x1, 0xF0 -> 0x14 ^ 0xF0 = 0xE4
    test_alu({12'h0F0, 5'd1, 3'b100, 5'd23, OP_I}, "I XORI  x23, x1, 0xF0",
             5'b00100, 32'd20, 32'd0, 32'h0000_00E4, 1'b0, 1'b1);

    // ori x24, x1, 0x100 -> 0x14 | 0x100 = 0x114
    test_alu({12'h100, 5'd1, 3'b110, 5'd24, OP_I}, "I ORI   x24, x1, 0x100",
             5'b00100, 32'd20, 32'd0, 32'h0000_0114, 1'b0, 1'b1);

    // andi x25, x4, 0x0F -> 0xFF & 0x0F = 0x0F
    test_alu({12'h00F, 5'd4, 3'b111, 5'd25, OP_I}, "I ANDI  x25, x4, 0x0F",
             5'b00100, 32'h0000_00FF, 32'd0, 32'h0000_000F, 1'b0, 1'b0);

    // slli x26, x1, 3 -> 20 << 3 = 160
    test_alu({7'b0000000, 5'd3, 5'd1, 3'b001, 5'd26, OP_I}, "I SLLI  x26, x1, 3",
             5'b00100, 32'd20, 32'd0, 32'd160, 1'b0, 1'b0);

    // srli x27, x3, 4 -> 0xFFFFFFF6 >> 4 = 0x0FFFFFFF
    test_alu({7'b0000000, 5'd4, 5'd3, 3'b101, 5'd27, OP_I}, "I SRLI  x27, x3, 4",
             5'b00100, -32'd10, 32'd0, 32'h0FFF_FFFF, 1'b0, 1'b1);

    // srai x28, x3, 4 -> -10 >>> 4 = -1  (imm = 0x404)
    test_alu({7'b0100000, 5'd4, 5'd3, 3'b101, 5'd28, OP_I}, "I SRAI  x28, x3, 4",
             5'b00100, -32'd10, 32'd0, 32'hFFFF_FFFF, 1'b0, 1'b1);

    // =========================================================================
    // FASE 4: SALTOS TIPO B -> {imm[12], imm[10:5], rs2, rs1, funct3, imm[4:1], imm[11], opcode}
    // offset +16 -> imm[4:1] = 4'b1000 | offset -8 -> todos los bits altos a 1, imm[4:1] = 4'b1100
    // =========================================================================
    $display("\n--- FASE 4: SALTOS TIPO B ---");

    // beq x1, x5, +16 -> 20 == 20 -> salta
    test_branch({1'b0, 6'b000000, 5'd5, 5'd1, 3'b000, 4'b1000, 1'b0, OP_BRANCH}, "B BEQ  x1, x5, +16",
                32'd20, 32'd20, 32'd16, 1'b1, 1'b0, 1'b0);

    // bne x1, x2, +16 -> 20 != 5 -> salta
    test_branch({1'b0, 6'b000000, 5'd2, 5'd1, 3'b001, 4'b1000, 1'b0, OP_BRANCH}, "B BNE  x1, x2, +16",
                32'd20, 32'd5, 32'd16, 1'b0, 1'b0, 1'b0);

    // blt x3, x2, +16 -> -10 < 5 -> salta
    test_branch({1'b0, 6'b000000, 5'd2, 5'd3, 3'b100, 4'b1000, 1'b0, OP_BRANCH}, "B BLT  x3, x2, +16",
                -32'd10, 32'd5, 32'd16, 1'b0, 1'b1, 1'b0);

    // bge x1, x2, +16 -> 20 >= 5 -> salta
    test_branch({1'b0, 6'b000000, 5'd2, 5'd1, 3'b101, 4'b1000, 1'b0, OP_BRANCH}, "B BGE  x1, x2, +16",
                32'd20, 32'd5, 32'd16, 1'b0, 1'b0, 1'b0);

    // bltu x2, x3, -8 -> 5 <u 0xFFFFFFF6 -> salta
    test_branch({1'b1, 6'b111111, 5'd3, 5'd2, 3'b110, 4'b1100, 1'b1, OP_BRANCH}, "B BLTU x2, x3, -8",
                32'd5, -32'd10, -32'd8, 1'b0, 1'b0, 1'b1);

    // bgeu x3, x2, -8 -> 0xFFFFFFF6 >=u 5 -> salta
    test_branch({1'b1, 6'b111111, 5'd2, 5'd3, 3'b111, 4'b1100, 1'b1, OP_BRANCH}, "B BGEU x3, x2, -8",
                -32'd10, 32'd5, -32'd8, 1'b0, 1'b1, 1'b0);

    // =========================================================================
    // FASE 5: STORES TIPO S -> {imm[11:5], rs2, rs1, funct3, imm[4:0], opcode}
    // base x7 = 0x100, dato x8 = 0x12345678
    // =========================================================================
    $display("\n--- FASE 5: STORES TIPO S ---");

    // sb x8, 4(x7) -> mem[0x104] <= byte 0
    test_store({7'b0000000, 5'd8, 5'd7, 3'b000, 5'd4, OP_STORE}, "S SB x8, 4(x7)",
               32'h0000_0104, 32'h1234_5678, 4'b0001);

    // sh x8, 8(x7) -> mem[0x108] <= media palabra
    test_store({7'b0000000, 5'd8, 5'd7, 3'b001, 5'd8, OP_STORE}, "S SH x8, 8(x7)",
               32'h0000_0108, 32'h1234_5678, 4'b0011);

    // sw x8, -4(x7) -> mem[0x0FC] <= palabra completa
    test_store({7'b1111111, 5'd8, 5'd7, 3'b010, 5'b11100, OP_STORE}, "S SW x8, -4(x7)",
               32'h0000_00FC, 32'h1234_5678, 4'b1111);

    // =========================================================================
    // FASE 6: LOADS TIPO I -> {imm[11:0], rs1, funct3, rd, opcode}
    // base x7 = 0x100, la "memoria" (testbench) devuelve 0xCAFEF08A
    // =========================================================================
    $display("\n--- FASE 6: LOADS TIPO I ---");

    // lb x29, 0(x7) -> extension de signo del byte 0x8A
    test_load({12'd0, 5'd7, 3'b000, 5'd29, OP_LOAD}, "L LB  x29, 0(x7)",
              32'h0000_0100, 32'hCAFE_F08A, 32'hFFFF_FF8A);

    // lh x30, 0(x7) -> extension de signo de la media palabra 0xF08A
    test_load({12'd0, 5'd7, 3'b001, 5'd30, OP_LOAD}, "L LH  x30, 0(x7)",
              32'h0000_0100, 32'hCAFE_F08A, 32'hFFFF_F08A);

    // lw x31, 0(x7) -> palabra completa
    test_load({12'd0, 5'd7, 3'b010, 5'd31, OP_LOAD}, "L LW  x31, 0(x7)",
              32'h0000_0100, 32'hCAFE_F08A, 32'hCAFE_F08A);

    // lbu x29, 4(x7) -> extension con ceros del byte 0x8A
    test_load({12'd4, 5'd7, 3'b100, 5'd29, OP_LOAD}, "L LBU x29, 4(x7)",
              32'h0000_0104, 32'hCAFE_F08A, 32'h0000_008A);

    // lhu x30, 8(x7) -> extension con ceros de la media palabra 0xF08A
    test_load({12'd8, 5'd7, 3'b101, 5'd30, OP_LOAD}, "L LHU x30, 8(x7)",
              32'h0000_0108, 32'hCAFE_F08A, 32'h0000_F08A);

    // =========================================================================
    // FASE 7: TIPOS U Y J
    // auipc, jal y jalr escriben PC + algo en rd: DecExe no tiene el PC,
    // por lo que se comprueba la decodificacion (opcode, rd, imm, we)
    // =========================================================================
    $display("\n--- FASE 7: TIPOS U Y J ---");

    // lui x10, 0x12345 -> x10 = 0x12345000
    test_upper_jump({20'h12345, 5'd10, OP_LUI}, "U LUI   x10, 0x12345",
                    5'b01101, 32'h1234_5000);
    check_reg(5'd10, 32'h1234_5000, "U LUI   x10, 0x12345");

    // auipc x11, 0x00001 -> x11 = PC + 0x1000
    test_upper_jump({20'h00001, 5'd11, OP_AUIPC}, "U AUIPC x11, 0x00001",
                    5'b00101, 32'h0000_1000);

    // jal x12, +16 -> {imm[20], imm[10:1], imm[11], imm[19:12], rd, opcode}
    test_upper_jump({1'b0, 10'b0000001000, 1'b0, 8'h00, 5'd12, OP_JAL}, "J JAL   x12, +16",
                    5'b11011, 32'd16);

    // jalr x13, 12(x7) -> destino del salto = x7 + 12 = 0x10C (calculado en la ALU)
    test_upper_jump({12'd12, 5'd7, 3'b000, 5'd13, OP_JALR}, "I JALR  x13, 12(x7)",
                    5'b11001, 32'd12);
    test_instr_with_regs({12'd12, 5'd7, 3'b000, 5'd13, OP_JALR}, "I JALR  destino x7 + 12",
                         5'b11001, 32'h0000_0100, 32'd0, 32'h0000_010C, 1'b0, 1'b0);

    #20;
    $display("------------------------------------------------------------------");
    $display("=== SIMULACION COMPLETADA (revisar los mensajes ** Error) ===");
    $finish;
  end

endmodule
