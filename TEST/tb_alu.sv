`timescale 1ns / 1ps

module tb_alu;

  localparam int ALU_LENGTH = 32;
  localparam int ALU_DECOD  = 4;

  // Estímulos[cite: 2]
  logic [ALU_LENGTH-1:0] op1;
  logic [ALU_LENGTH-1:0] op2;
  logic [ALU_DECOD-1:0]  aloup;

  // Salidas[cite: 2]
  logic                  eq;
  logic                  lt;
  logic                  ltu;
  logic [ALU_LENGTH-1:0] aluOut;

  // Instancia del DUT (alu.vhd)[cite: 2]
  alu #(
    .ALU_LENGTH (ALU_LENGTH),
    .ALU_DECOD  (ALU_DECOD)
  ) dut (
    .op1    (op1),
    .op2    (op2),
    .aloup  (aloup),
    .eq     (eq),
    .lt     (lt),
    .ltu    (ltu),
    .aluOut (aluOut)
  );

  task automatic test_alu_op(
    input string       op_name,
    input logic [3:0]  in_aloup,
    input logic [31:0] in_op1,
    input logic [31:0] in_op2,
    input logic [31:0] exp_out,
    input logic        exp_eq,
    input logic        exp_lt,
    input logic        exp_ltu
  );
    begin
      op1   = in_op1;
      op2   = in_op2;
      aloup = in_aloup;
      #5;

      $display("[%s | aloup=%b] op1=0x%08H, op2=0x%08H -> aluOut=0x%08H | eq=%b, lt=%b, ltu=%b",
               op_name, aloup, op1, op2, aluOut, eq, lt, ltu);

      assert (aluOut === exp_out) else $error("Fallo aluOut en %s: esp 0x%08H, obt 0x%08H", op_name, exp_out, aluOut);
      assert (eq     === exp_eq)  else $error("Fallo flag eq en %s: esp %b, obt %b", op_name, exp_eq, eq);
      assert (lt     === exp_lt)  else $error("Fallo flag lt en %s: esp %b, obt %b", op_name, exp_lt, lt);
      assert (ltu    === exp_ltu) else $error("Fallo flag ltu en %s: esp %b, obt %b", op_name, exp_ltu, ltu);
    end
  endtask

  initial begin
    $display("=== INICIO TESTBENCH ALU ===");

    // 1. 0000: Suma signed (15 + (-5) = 10) | 15 > -5 signed (lt=0), 0x0000000F < 0xFFFFFFFB unsigned (ltu=1)[cite: 2]
    test_alu_op("ADD ", 4'b0000, 32'd15, -32'd5, 32'd10, 1'b0, 1'b0, 1'b1);

    // 2. 1000: Resta signed (-5 - 15 = -20) | -5 < 15 signed (lt=1), 0xFFFFFFFB > 0x0000000F unsigned (ltu=0)[cite: 2]
    test_alu_op("SUB ", 4'b1000, -32'd5, 32'd15, -32'd20, 1'b0, 1'b1, 1'b0);

    // 3. 0001: Shift Left Logical (1 << 4 = 16)[cite: 2]
    test_alu_op("SLL ", 4'b0001, 32'h0000_0001, 32'd4, 32'h0000_0010, 1'b0, 1'b1, 1'b1);

    // 4. 0010: Set Less Than Signed (-10 < 5 -> 1)[cite: 2]
    test_alu_op("SLT ", 4'b0010, -32'd10, 32'd5, 32'd1, 1'b0, 1'b1, 1'b0);

    // 5. 0011: Set Less Than Unsigned (-10 [0xFFFFFFF6] < 5 -> 0)[cite: 2]
    test_alu_op("SLTU", 4'b0011, -32'd10, 32'd5, 32'd0, 1'b0, 1'b1, 1'b0);

    // 6. 0100: XOR[cite: 2]
    // ltu = 1: 0xAAAA5555 <u 0xFFFF0000
    test_alu_op("XOR ", 4'b0100, 32'hAAAA_5555, 32'hFFFF_0000, 32'h5555_5555, 1'b0, 1'b1, 1'b1);

    // 7. 0101: Shift Right Logical (0x8000_0000 >> 4 = 0x0800_0000)[cite: 2]
    test_alu_op("SRL ", 4'b0101, 32'h8000_0000, 32'd4, 32'h0800_0000, 1'b0, 1'b1, 1'b0);

    // 8. 1101: Shift Right Arithmetic (0x8000_0000 >>> 4 = 0xF800_0000)[cite: 2]
    test_alu_op("SRA ", 4'b1101, 32'h8000_0000, 32'd4, 32'hF800_0000, 1'b0, 1'b1, 1'b0);

    // 9. 0110: OR[cite: 2]
    test_alu_op("OR  ", 4'b0110, 32'h1234_0000, 32'h0000_5678, 32'h1234_5678, 1'b0, 1'b0, 1'b0);

    // 10. 0111: AND (y prueba de igualdad eq=1)[cite: 2]
    test_alu_op("AND ", 4'b0111, 32'hCAFE_BABE, 32'hCAFE_BABE, 32'hCAFE_BABE, 1'b1, 1'b0, 1'b0);

    // 11. When others: Código no definido (ej. 4'b1111 -> 0)[cite: 2]
    test_alu_op("OTHR", 4'b1111, 32'h1234_5678, 32'h8765_4321, 32'h0000_0000, 1'b0, 1'b0, 1'b1);

    #10;
    $display("=== FIN TESTBENCH ALU ===");
    $finish;
  end

endmodule