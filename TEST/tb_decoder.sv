`timescale 1ns / 1ps

module tb_decoder;

  localparam int REG_WIDTH = 32;
  localparam int ALU_DECOD = 4;

  // Estímulos
  logic                 clk;
  logic                 nRst;
  logic [REG_WIDTH-1:0] instr;

  // Salidas del DUT
  logic [4:0]           rs1;
  logic [4:0]           rs2;
  logic [4:0]           rd;
  logic [REG_WIDTH-1:0] imm;
  logic                 ena_imm;
  logic [ALU_DECOD-1:0] aloup;
  logic                 we_b;
  logic [ALU_DECOD-1:0] be_b;

  // Acceso jerárquico a la señal interna opcode
  wire [4:0] mon_opcode = dut.opcode;

  // Instancia del DUT (decoder.vhd)
  decoder #(
    .REG_WIDTH (REG_WIDTH),
    .ALU_DECOD (ALU_DECOD)
  ) dut (
    .clk     (clk),
    .nRst    (nRst),
    .instr   (instr),
    .rs1     (rs1),
    .rs2     (rs2),
    .rd      (rd),
    .imm     (imm),
    .ena_imm (ena_imm),
    .aloup   (aloup),
    .we_b    (we_b),
    .be_b    (be_b)
  );

  // Reloj (aunque el decoder actual es combinacional, tiene puertos clk/nRst)
  initial clk = 1'b0;
  always #5 clk = ~clk;

  // Tarea de verificación automática
  task automatic check_decoder(
    input logic [31:0] instruction_word,
    input string       case_name,
    input logic [4:0]  exp_opcode,
    input logic [4:0]  exp_rd,
    input logic [4:0]  exp_rs1,
    input logic [4:0]  exp_rs2,
    input logic [31:0] exp_imm,
    input logic [3:0]  exp_aloup,
    input logic [3:0]  exp_be_b
  );
    begin
      instr = instruction_word;
      #5;

      $display("------------------------------------------------------------------");
      $display("[%s] Instr: 0x%08H | opcode: %b", case_name, instruction_word, mon_opcode);
      $display("  Salidas -> rd:x%0d, rs1:x%0d, rs2:x%0d, imm:0x%08H, ena_imm:%b, aloup:%b, we_b:%b, be_b:%b",
               rd, rs1, rs2, imm, ena_imm, aloup, we_b, be_b);

      assert (mon_opcode === exp_opcode) else $error("Fallo opcode en %s: esp %b, obt %b", case_name, exp_opcode, mon_opcode);
      assert (rd         === exp_rd)     else $error("Fallo rd en %s: esp %0d, obt %0d", case_name, exp_rd, rd);
      assert (rs1        === exp_rs1)    else $error("Fallo rs1 en %s: esp %0d, obt %0d", case_name, exp_rs1, rs1);
      assert (rs2        === exp_rs2)    else $error("Fallo rs2 en %s: esp %0d, obt %0d", case_name, exp_rs2, rs2);
      assert (imm        === exp_imm)    else $error("Fallo imm en %s: esp 0x%08H, obt 0x%08H", case_name, exp_imm, imm);
      assert (aloup      === exp_aloup)  else $error("Fallo aloup en %s: esp %b, obt %b", case_name, exp_aloup, aloup);
      assert (be_b       === exp_be_b)   else $error("Fallo be_b en %s: esp %b, obt %b", case_name, exp_be_b, be_b);
    end
  endtask

  initial begin
    $display("=== INICIO TESTBENCH DECODER (TODOS LOS CASOS DEL CASE) ===");
    nRst  = 1'b0;
    instr = 32'h0;
    #10;
    nRst  = 1'b1;

    // 1. Case "01100": R-type -> sub x3, x1, x2 (funct7[5]=1, funct3=000 -> aloup=1000)
    check_decoder(32'h4020_81B3, "CASE 01100: R-Type (SUB)",      5'b01100, 5'd3, 5'd1, 5'd2, 32'h0000_0000, 4'b1000, 4'bXXXX);

    // 2. Case "00100": I-type -> addi x5, x1, -4 (bit30=1 por imm negativo, funct3=000 -> aloup=1000)[cite: 4]
    check_decoder(32'hFFC0_8293, "CASE 00100: I-Type (ADDI -4)", 5'b00100, 5'd5, 5'd1, 5'd0, 32'hFFFF_FFFC, 4'b1000, 4'bXXXX);

    // 3. Case "00000": I-type LOAD -> lw x6, 8(x2)[cite: 4]
    check_decoder(32'h0081_2303, "CASE 00000: I-Type LOAD (LW)", 5'b00000, 5'd6, 5'd2, 5'd0, 32'h0000_0008, 4'b0000, 4'bXXXX);

    // 4. Case "01000": S-type STORE -> sb, sh, sw (funct3 = 000, 001, 010)[cite: 4]
    check_decoder(32'h0041_0623, "CASE 01000: S-Type (SB)",        5'b01000, 5'd0, 5'd2, 5'd4, 32'h0000_000C, 4'b0000, 4'b0001);
    check_decoder(32'h0041_1623, "CASE 01000: S-Type (SH)",        5'b01000, 5'd0, 5'd2, 5'd4, 32'h0000_000C, 4'b0000, 4'b0011);
    check_decoder(32'h0041_2623, "CASE 01000: S-Type (SW)",        5'b01000, 5'd0, 5'd2, 5'd4, 32'h0000_000C, 4'b0000, 4'b1111);

    // 5. Case "11000": B-type BRANCH -> beq x1, x2, -8[cite: 4]
    check_decoder(32'hFE20_8CE3, "CASE 11000: B-Type (BEQ)",     5'b11000, 5'd0, 5'd1, 5'd2, 32'hFFFF_FFF8, 4'b0000, 4'b1111);

    // 6. Case "11011": J-type JAL -> jal x1, 16[cite: 4]
    check_decoder(32'h0100_00EF, "CASE 11011: J-Type (JAL)",     5'b11011, 5'd1, 5'd0, 5'd0, 32'h0000_0010, 4'b0000, 4'b1111);

    // 7. Case "11001": I-type JALR -> jalr x1, 0(x5)[cite: 4]
    check_decoder(32'h0002_80E7, "CASE 11001: I-Type (JALR)",    5'b11001, 5'd1, 5'd5, 5'd0, 32'h0000_0000, 4'b0000, 4'b1111);

    // 8. Case "01101": U-type LUI -> lui x7, 0x12345 (Nota: en tu VHDL bit 12 se replica hasta 0)[cite: 4]   REVISAR MAS A FONDO
    check_decoder(32'h1234_53B7, "CASE 01101: U-Type (LUI)",     5'b01101, 5'd7, 5'd0, 5'd0, 32'h1234_5FFF, 4'b0000, 4'b1111);

    // 9. Case "00101": U-type AUIPC -> auipc x8, 0x00002[cite: 4]
    check_decoder(32'h0000_2417, "CASE 00101: U-Type (AUIPC)",   5'b00101, 5'd8, 5'd0, 5'd0, 32'h0000_2000, 4'b0000, 4'b1111);

    // 10. Case WHEN OTHERS: Opcode no contemplado (5'b11111 -> bits[6:0] = 7'b1111111)[cite: 4]
    check_decoder(32'hFFFF_FFFF, "CASE WHEN OTHERS (Invalida)",  5'b11111, 5'd0, 5'd0, 5'd0, 32'h0000_0000, 4'b0000, 4'b1111);

    #10;
    $display("------------------------------------------------------------------");
    $display("=== FIN TESTBENCH DECODER ===");
    $finish;
  end

endmodule
