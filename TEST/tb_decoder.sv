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
  logic                 we;
  logic                 wl;
  logic [ALU_DECOD-1:0] be_l;
  logic                 sign_l;
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
    .we      (we),
    .wl      (wl),
    .be_l    (be_l),
    .sign_l  (sign_l),
    .aloup   (aloup),
    .we_b    (we_b),
    .be_b    (be_b)
  );

  // Reloj (aunque el decoder actual es combinacional, tiene puertos clk/nRst)
  initial clk = 1'b0;
  always #5 clk = ~clk;

  // Tarea de verificación de campos de la instruccion (registros, inmediato y operacion ALU)
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
      $display("  Salidas -> rd:x%0d, rs1:x%0d, rs2:x%0d, imm:0x%08H, aloup:%b, be_b:%b",
               rd, rs1, rs2, imm, aloup, be_b);

      assert (mon_opcode === exp_opcode) else $error("Fallo opcode en %s: esp %b, obt %b", case_name, exp_opcode, mon_opcode);
      assert (rd         === exp_rd)     else $error("Fallo rd en %s: esp %0d, obt %0d", case_name, exp_rd, rd);
      assert (rs1        === exp_rs1)    else $error("Fallo rs1 en %s: esp %0d, obt %0d", case_name, exp_rs1, rs1);
      assert (rs2        === exp_rs2)    else $error("Fallo rs2 en %s: esp %0d, obt %0d", case_name, exp_rs2, rs2);
      assert (imm        === exp_imm)    else $error("Fallo imm en %s: esp 0x%08H, obt 0x%08H", case_name, exp_imm, imm);
      assert (aloup      === exp_aloup)  else $error("Fallo aloup en %s: esp %b, obt %b", case_name, exp_aloup, aloup);
      assert (be_b       === exp_be_b)   else $error("Fallo be_b en %s: esp %b, obt %b", case_name, exp_be_b, be_b);
    end
  endtask

  // Tarea de verificación de las señales de control de la instruccion aplicada en check_decoder
  task automatic check_control(
    input string       case_name,
    input logic        exp_ena_imm,
    input logic        exp_we,
    input logic        exp_wl,
    input logic [3:0]  exp_be_l,
    input logic        exp_sign_l,
    input logic        exp_we_b
  );
    begin
      $display("  Control -> ena_imm:%b, we:%b, wl:%b, be_l:%b, sign_l:%b, we_b:%b",
               ena_imm, we, wl, be_l, sign_l, we_b);

      assert (ena_imm === exp_ena_imm) else $error("Fallo ena_imm en %s: esp %b, obt %b", case_name, exp_ena_imm, ena_imm);
      assert (we      === exp_we)      else $error("Fallo we en %s: esp %b, obt %b", case_name, exp_we, we);
      assert (wl      === exp_wl)      else $error("Fallo wl en %s: esp %b, obt %b", case_name, exp_wl, wl);
      assert (be_l    === exp_be_l)    else $error("Fallo be_l en %s: esp %b, obt %b", case_name, exp_be_l, be_l);
      assert (sign_l  === exp_sign_l)  else $error("Fallo sign_l en %s: esp %b, obt %b", case_name, exp_sign_l, sign_l);
      assert (we_b    === exp_we_b)    else $error("Fallo we_b en %s: esp %b, obt %b", case_name, exp_we_b, we_b);
    end
  endtask

  initial begin
    $display("=== INICIO TESTBENCH DECODER (TODOS LOS CASOS DEL CASE) ===");
    nRst  = 1'b0;
    instr = 32'h0;
    #10;
    nRst  = 1'b1;

    //                                                                     opcode    rd     rs1    rs2    imm            aloup    be_b
    //                                                     ena_imm we    wl    be_l     sign_l we_b

    // 1. Case "01100": R-type -> sub x3, x1, x2 (funct7[5]=1, funct3=000 -> aloup=1000)
    check_decoder(32'h4020_81B3, "CASE 01100: R-Type (SUB)",      5'b01100, 5'd3, 5'd1, 5'd2, 32'h0000_0000, 4'b1000, 4'b0000);
    check_control(               "CASE 01100: R-Type (SUB)",      1'b0, 1'b1, 1'b0, 4'b0000, 1'b0, 1'b0);

    // 2. Case "00100": I-type -> addi x5, x1, -4 (bit30=1 por imm negativo, pero solo cuenta con funct3=101 -> aloup=0000)
    check_decoder(32'hFFC0_8293, "CASE 00100: I-Type (ADDI -4)", 5'b00100, 5'd5, 5'd1, 5'd0, 32'hFFFF_FFFC, 4'b0000, 4'b0000);
    check_control(               "CASE 00100: I-Type (ADDI -4)", 1'b1, 1'b1, 1'b0, 4'b0000, 1'b0, 1'b0);

    //    I-type -> srai x5, x1, 3 (funct3=101 y bit30=1 -> aloup=1101)
    check_decoder(32'h4030_D293, "CASE 00100: I-Type (SRAI 3)",  5'b00100, 5'd5, 5'd1, 5'd0, 32'h0000_0403, 4'b1101, 4'b0000);
    check_control(               "CASE 00100: I-Type (SRAI 3)",  1'b1, 1'b1, 1'b0, 4'b0000, 1'b0, 1'b0);

    // 3. Case "00000": I-type LOAD -> lb, lh, lw, lbu, lhu x6, 8(x2) (funct3 = 000, 001, 010, 100, 101)
    check_decoder(32'h0081_0303, "CASE 00000: I-Type LOAD (LB)",  5'b00000, 5'd6, 5'd2, 5'd0, 32'h0000_0008, 4'b0000, 4'b0000);
    check_control(               "CASE 00000: I-Type LOAD (LB)",  1'b1, 1'b0, 1'b1, 4'b0001, 1'b1, 1'b0);
    check_decoder(32'h0081_1303, "CASE 00000: I-Type LOAD (LH)",  5'b00000, 5'd6, 5'd2, 5'd0, 32'h0000_0008, 4'b0000, 4'b0000);
    check_control(               "CASE 00000: I-Type LOAD (LH)",  1'b1, 1'b0, 1'b1, 4'b0011, 1'b1, 1'b0);
    check_decoder(32'h0081_2303, "CASE 00000: I-Type LOAD (LW)",  5'b00000, 5'd6, 5'd2, 5'd0, 32'h0000_0008, 4'b0000, 4'b0000);
    check_control(               "CASE 00000: I-Type LOAD (LW)",  1'b1, 1'b0, 1'b1, 4'b1111, 1'b1, 1'b0);
    check_decoder(32'h0081_4303, "CASE 00000: I-Type LOAD (LBU)", 5'b00000, 5'd6, 5'd2, 5'd0, 32'h0000_0008, 4'b0000, 4'b0000);
    check_control(               "CASE 00000: I-Type LOAD (LBU)", 1'b1, 1'b0, 1'b1, 4'b0001, 1'b0, 1'b0);
    check_decoder(32'h0081_5303, "CASE 00000: I-Type LOAD (LHU)", 5'b00000, 5'd6, 5'd2, 5'd0, 32'h0000_0008, 4'b0000, 4'b0000);
    check_control(               "CASE 00000: I-Type LOAD (LHU)", 1'b1, 1'b0, 1'b1, 4'b0011, 1'b0, 1'b0);

    // 4. Case "01000": S-type STORE -> sb, sh, sw x4, 12(x2) (funct3 = 000, 001, 010)
    check_decoder(32'h0041_0623, "CASE 01000: S-Type (SB)",        5'b01000, 5'd0, 5'd2, 5'd4, 32'h0000_000C, 4'b0000, 4'b0001);
    check_control(               "CASE 01000: S-Type (SB)",        1'b1, 1'b0, 1'b0, 4'b0000, 1'b0, 1'b1);
    check_decoder(32'h0041_1623, "CASE 01000: S-Type (SH)",        5'b01000, 5'd0, 5'd2, 5'd4, 32'h0000_000C, 4'b0000, 4'b0011);
    check_control(               "CASE 01000: S-Type (SH)",        1'b1, 1'b0, 1'b0, 4'b0000, 1'b0, 1'b1);
    check_decoder(32'h0041_2623, "CASE 01000: S-Type (SW)",        5'b01000, 5'd0, 5'd2, 5'd4, 32'h0000_000C, 4'b0000, 4'b1111);
    check_control(               "CASE 01000: S-Type (SW)",        1'b1, 1'b0, 1'b0, 4'b0000, 1'b0, 1'b1);

    // 5. Case "11000": B-type BRANCH -> beq x1, x2, -8 (compara rs1 con rs2: ena_imm=0)
    check_decoder(32'hFE20_8CE3, "CASE 11000: B-Type (BEQ)",     5'b11000, 5'd0, 5'd1, 5'd2, 32'hFFFF_FFF8, 4'b0000, 4'b0000);
    check_control(               "CASE 11000: B-Type (BEQ)",     1'b0, 1'b0, 1'b0, 4'b0000, 1'b0, 1'b0);

    // 6. Case "11011": J-type JAL -> jal x1, 16
    check_decoder(32'h0100_00EF, "CASE 11011: J-Type (JAL)",     5'b11011, 5'd1, 5'd0, 5'd0, 32'h0000_0010, 4'b0000, 4'b0000);
    check_control(               "CASE 11011: J-Type (JAL)",     1'b0, 1'b1, 1'b0, 4'b0000, 1'b0, 1'b0);

    // 7. Case "11001": I-type JALR -> jalr x1, 0(x5)
    check_decoder(32'h0002_80E7, "CASE 11001: I-Type (JALR)",    5'b11001, 5'd1, 5'd5, 5'd0, 32'h0000_0000, 4'b0000, 4'b0000);
    check_control(               "CASE 11001: I-Type (JALR)",    1'b1, 1'b1, 1'b0, 4'b0000, 1'b0, 1'b0);

    // 8. Case "01101": U-type LUI -> lui x7, 0x12345 (imm = instr[31:12] y 12 ceros)
    check_decoder(32'h1234_53B7, "CASE 01101: U-Type (LUI)",     5'b01101, 5'd7, 5'd0, 5'd0, 32'h1234_5000, 4'b0000, 4'b0000);
    check_control(               "CASE 01101: U-Type (LUI)",     1'b1, 1'b1, 1'b0, 4'b0000, 1'b0, 1'b0);

    // 9. Case "00101": U-type AUIPC -> auipc x8, 0x00002
    check_decoder(32'h0000_2417, "CASE 00101: U-Type (AUIPC)",   5'b00101, 5'd8, 5'd0, 5'd0, 32'h0000_2000, 4'b0000, 4'b0000);
    check_control(               "CASE 00101: U-Type (AUIPC)",   1'b1, 1'b1, 1'b0, 4'b0000, 1'b0, 1'b0);

    // 10. Case WHEN OTHERS: Opcode no contemplado (5'b11111 -> bits[6:0] = 7'b1111111)
    check_decoder(32'hFFFF_FFFF, "CASE WHEN OTHERS (Invalida)",  5'b11111, 5'd0, 5'd0, 5'd0, 32'h0000_0000, 4'b0000, 4'b0000);
    check_control(               "CASE WHEN OTHERS (Invalida)",  1'b0, 1'b0, 1'b0, 4'b0000, 1'b0, 1'b0);

    #10;
    $display("------------------------------------------------------------------");
    $display("=== FIN TESTBENCH DECODER ===");
    $finish;
  end

endmodule
