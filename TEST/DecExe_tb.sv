`timescale 1ns / 1ps

module DecExe_tb;

  localparam int REG_WIDTH  = 32;
  localparam int ADDR_REGS  = 5;
  localparam int ADDR_WIDTH = 17;
  localparam int ALU_DECOD  = 4;

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

  // Tarea auxiliar para precargar registros en U0 (regFile)
  task automatic preload_reg(
    input logic [4:0]  reg_addr,
    input logic [31:0] reg_val
  );
    begin
      @(negedge clk);
      // Inyectamos una instrucción tipo I dummy cuyo rd sea reg_addr
      preload_val_static = reg_val;
      rdata_a = {12'd0, 5'd0, 3'b000, reg_addr, 7'b0010011};
      force dut.we   = 1'b1;
      force dut.wAlu = preload_val_static;;
      @(posedge clk);
      #1;
      release dut.we;
      release dut.wAlu;
      $display("  [PRELOAD] Registro x%0d <= 0x%08H (%0d)", reg_addr, reg_val, $signed(reg_val));
    end
  endtask

  // Tarea de comprobación de instrucciones operando sobre los registros precargados
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

  initial begin
    $display("=== INICIO SIMULACION DECEXE CON PRECARGA DE REGISTROS ===");

    // 1. Reset inicial
    nRst    = 1'b0;
    rdata_a = 32'h0;
    rdata_b = 32'h0;
    repeat (2) @(posedge clk);
    nRst = 1'b1;

    // 2. Precarga de 5 registros en el banco (x1 a x5)
    $display("\n--- FASE 1: PRECARGA DE 5 REGISTROS ---");
    preload_reg(5'd1, 32'd20);          // x1 = 20  (0x0000_0014)
    preload_reg(5'd2, 32'd5);           // x2 = 5   (0x0000_0005)
    preload_reg(5'd3, -32'd10);         // x3 = -10 (0xFFFF_FFF6)
    preload_reg(5'd4, 32'h0000_00FF);   // x4 = 255 (0x0000_00FF)
    preload_reg(5'd5, 32'd20);          // x5 = 20  (0x0000_0014)

    // 3. Ejecución de instrucciones usando los datos almacenados en x1..x5
    $display("\n--- FASE 2: COMPROBACION DE INSTRUCCIONES ---");

    // R-Type ADD: add x6, x1, x2 -> 20 + 5 = 25 (0x19) | eq=0, lt=0
    test_instr_with_regs(32'h0020_8333, "R-Type ADD (x1 + x2)",
                         5'b01100, 32'd20, 32'd5, 32'd25, 1'b0, 1'b0);

    // R-Type SUB: sub x7, x1, x2 -> 20 - 5 = 15 (0x0F) | eq=0, lt=0
    test_instr_with_regs(32'h4020_83B3, "R-Type SUB (x1 - x2)",
                         5'b01100, 32'd20, 32'd5, 32'd15, 1'b0, 1'b0);

    // R-Type AND: and x8, x1, x4 -> 0x14 & 0xFF = 0x14 | eq=0, lt=0
    test_instr_with_regs(32'h0040_F433, "R-Type AND (x1 & x4)",
                         5'b01100, 32'd20, 32'h0000_00FF, 32'h0000_0014, 1'b0, 1'b0);

    // R-Type SLT: slt x9, x3, x2 -> (-10 < 5) = 1 | eq=0, lt=1
    test_instr_with_regs(32'h0021_A4B3, "R-Type SLT (x3 < x2)",
                         5'b01100, -32'd10, 32'd5, 32'd1, 1'b0, 1'b1);

    // B-Type BEQ: beq x1, x5, +16 -> compara x1 (20) y x5 (20) -> eq=1, lt=0
    test_instr_with_regs(32'h0050_8863, "B-Type BEQ (x1 == x5)",
                         5'b11000, 32'd20, 32'd20, 32'd40, 1'b1, 1'b0);

    // S-Type SW: sw x4, 12(x1) -> lee base x1=20 y dato x4=0xFF (NOTA: ALU suma rdata1+rdata2 porque falta mux imm)
    test_instr_with_regs(32'h0040_A623, "S-Type SW (base x1, dato x4)",
                         5'b01000, 32'd20, 32'h0000_00FF, 32'd275, 1'b0, 1'b0);

    #20;
    $display("------------------------------------------------------------------");
    $display("=== SIMULACION COMPLETADA ===");
    $finish;
  end

endmodule