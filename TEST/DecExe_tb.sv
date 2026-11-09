`timescale 1ns / 1ps

module tb_decoder;

  localparam int REG_WIDTH = 32;
  localparam int ALU_DECOD = 4;

  // Estimulos
  logic [REG_WIDTH-1:0]   instr;

  // Salidas del DUT
  logic [4:0]             rs1;
  logic [4:0]             rs2;
  logic [4:0]             rd;
  logic [REG_WIDTH-1:0]   imm;
  logic                   ena_imm;
  logic [ALU_DECOD-1:0]   aloup;
  logic                   we_b;
  logic [3:0]             be_b;

  // Instancia del decodificador puramente combinacional
  decoder #(
    .REG_WIDTH(REG_WIDTH),
    .ALU_DECOD(ALU_DECOD)
  ) dut (
    .instr    (instr),
    .rs1      (rs1),
    .rs2      (rs2),
    .rd       (rd),
    .imm      (imm),
    .ena_imm  (ena_imm),
    .aloup    (aloup),
    .we_b     (we_b),
    .be_b     (be_b)
  );

  // Tarea de verificacion combinacional
  task automatic test_instr(
    input logic [31:0] instruction_word,
    input string       test_name,
    input logic [4:0]  exp_rd,
    input logic [4:0]  exp_rs1,
    input logic [4:0]  exp_rs2,
    input logic [31:0] exp_imm,
    input logic        exp_ena_imm,
    input logic [3:0]  exp_aloup,
    input logic        exp_we_b
  );
    begin
      instr = instruction_word;
      #5; // Retardo para permitir la propagacion combinacional

      $display("--------------------------------------------------");
      $display("[CHECK: %s] Instr = 0x%08H", test_name, instruction_word);
      $display("  Obtenido -> rd:x%0d, rs1:x%0d, rs2:x%0d, imm:0x%08H, aloup:%b, we:%b", 
               rd, rs1, rs2, imm, aloup, we_b);

      // Comprobaciones
      assert (rd === exp_rd)       else $error("Fallo rd: esperado %0d, obtenido %0d", exp_rd, rd);
      assert (rs1 === exp_rs1)     else $error("Fallo rs1: esperado %0d, obtenido %0d", exp_rs1, rs1);
      assert (rs2 === exp_rs2)     else $error("Fallo rs2: esperado %0d, obtenido %0d", exp_rs2, rs2);
      assert (imm === exp_imm)     else $error("Fallo imm: esperado 0x%08H, obtenido 0x%08H", exp_imm, imm);
      assert (ena_imm === exp_ena_imm) else $error("Fallo ena_imm");
      assert (aloup === exp_aloup) else $error("Fallo aloup: esperado %b, obtenido %b", exp_aloup, aloup);
      assert (we_b === exp_we_b)   else $error("Fallo we_b");
    end
  endtask

  initial begin
    $display("=== INICIO TEST DECODIFICADOR COMBINACIONAL ===");

    // 1. R-Type: sub x3, x1, x2 (funct7=0100000, funct3=000) -> aloup = 4'b1000
    test_instr(32'h4020_81B3, "R-Type (SUB)",  5'd3, 5'd1, 5'd2, 32'h0, 1'b0, 4'b1000, 1'b0);

    // 2. I-Type ALU: addi x5, x1, -4 (imm = 0xFFFFFFFC) -> aloup = 4'b0000
    test_instr(32'hFFC0_8293, "I-Type (ADDI)", 5'd5, 5'd1, 5'd0, 32'hFFFF_FFFC, 1'b1, 4'b0000, 1'b0);

    // 3. I-Type LOAD: lw x6, 8(x2) (imm = 8)
    test_instr(32'h0081_2303, "I-Type (LW)",   5'd6, 5'd2, 5'd0, 32'h0000_0008, 1'b1, 4'b0000, 1'b0);

    // 4. S-Type STORE: sw x4, 12(x2) (imm = 12, we_b = 1)
    test_instr(32'h0041_2623, "S-Type (SW)",   5'd0, 5'd2, 5'd4, 32'h0000_000C, 1'b1, 4'b0000, 1'b1);

    // 5. B-Type BRANCH: beq x1, x2, -8 (imm = 0xFFFFFFF8)
    test_instr(32'hFE20_8CE3, "B-Type (BEQ)",  5'd0, 5'd1, 5'd2, 32'hFFFF_FFF8, 1'b0, 4'b0000, 1'b0);

    // 6. J-Type: jal x1, 16 (imm = 16)
    test_instr(32'h0100_00EF, "J-Type (JAL)",  5'd1, 5'd0, 5'd0, 32'h0000_0010, 1'b0, 4'b0000, 1'b0);

    // 7. I-Type: jalr x1, 0(x5) (imm = 0)
    test_instr(32'h0002_80E7, "I-Type (JALR)", 5'd1, 5'd5, 5'd0, 32'h0000_0000, 1'b0, 4'b0000, 1'b0);

    // 8. U-Type: lui x7, 0x12345 (imm = 0x12345000)
    test_instr(32'h1234_53B7, "U-Type (LUI)",  5'd7, 5'd0, 5'd0, 32'h1234_5000, 1'b0, 4'b0000, 1'b0);

    // 9. U-Type: auipc x8, 0x00002 (imm = 0x00002000)
    test_instr(32'h0000_2417, "U-Type (AUIPC)",5'd8, 5'd0, 5'd0, 32'h0000_2000, 1'b0, 4'b0000, 1'b0);

    #10;
    $display("--------------------------------------------------");
    $display("=== SIMULACION COMPLETADA SIN ERRORES ===");
    $finish;
  end

endmodule