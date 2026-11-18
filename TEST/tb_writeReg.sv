`timescale 1ns / 1ps

// =============================================================================
// tb_writeReg
// Comprueba la escritura en los 32 registros del banco (U0: regFile) de DecExe.
//
// FASE 1: escritura de x0..x31 con ADDI xi, x0, VAL(i)
// FASE 2: comprobacion del contenido del banco (ninguna escritura pisa otra)
// FASE 3: lectura de los registros por los puertos rdata1 / rdata2
// FASE 4: reescritura de x0..x31 con un patron de 32 bits (LUI + ADDI)
//
// FORCE_WRITE_PATH = 0 -> (por defecto) se prueba el camino real decoder -> alu -> regFile
// FORCE_WRITE_PATH = 1 -> igual que DecExe_tb: se fuerza dut.we y dut.wAlu
//                         (solo prueba el banco de registros)
// =============================================================================
module tb_writeReg;

  localparam int REG_WIDTH  = 32;
  localparam int ADDR_REGS  = 5;
  localparam int ADDR_WIDTH = 17;
  localparam int ALU_DECOD  = 4;
  localparam int NUM_REGS   = 2**ADDR_REGS;

  localparam bit FORCE_WRITE_PATH = 1'b0;

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
  wire [REG_WIDTH-1:0] mon_rdata1 = dut.rdata1;
  wire [REG_WIDTH-1:0] mon_rdata2 = dut.rdata2;
  wire [REG_WIDTH-1:0] mon_wAlu   = dut.wAlu;
  wire                 mon_we     = dut.we;

  // Monitorización de los 32 registros del banco (U0.regs)
  wire [REG_WIDTH-1:0] mon_regs [NUM_REGS];
  for (genvar g = 0; g < NUM_REGS; g++) begin : g_mon_regs
    assign mon_regs[g] = dut.U0.regs[g];
  end

  // Señales auxiliares
  logic [REG_WIDTH-1:0] write_val_static;           // valor forzado en wAlu
  logic [REG_WIDTH-1:0] reg_model [NUM_REGS];       // modelo de referencia del banco
  int                   num_errors;

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

  // Valor de prueba para cada registro (cabe en 12 bits con signo).
  // Los impares son negativos para comprobar la extension de signo del imm.
  function automatic logic [31:0] test_val(input int i);
    return (i % 2 == 0) ? (i * 61 + 7) : -(i * 37 + 3);
  endfunction

  // Tarea que ejecuta una instruccion que escribe en rd el valor exp_val.
  // Actualiza el modelo de referencia (x0 siempre vale 0) y comprueba con assert.
  task automatic write_reg(
    input logic [31:0] instruction_word,
    input logic [4:0]  rd_addr,
    input logic [31:0] exp_val,
    input string       test_name
  );
    begin
      @(negedge clk);
      rdata_a = instruction_word;
      #2; // Propagación combinacional: decoder -> regFile -> alu

      assert (mon_rd === rd_addr)
        else begin num_errors++; $error("Fallo rd en %s: esp x%0d, obt x%0d", test_name, rd_addr, mon_rd); end

      if (FORCE_WRITE_PATH) begin
        write_val_static = exp_val;
        force dut.we   = 1'b1;
        force dut.wAlu = write_val_static;
      end else begin
        assert (mon_wAlu === exp_val)
          else begin num_errors++; $error("Fallo wAlu en %s: esp 0x%08H, obt 0x%08H", test_name, exp_val, mon_wAlu); end
      end

      @(posedge clk);
      #1;
      if (FORCE_WRITE_PATH) begin
        release dut.we;
        release dut.wAlu;
      end

      reg_model[rd_addr] = (rd_addr == 5'd0) ? 32'h0 : exp_val;

      assert (mon_regs[rd_addr] === reg_model[rd_addr])
        else begin num_errors++; $error("Fallo escritura en %s: x%0d esp 0x%08H, obt 0x%08H",
                                        test_name, rd_addr, reg_model[rd_addr], mon_regs[rd_addr]); end

      $display("  [%s] x%0d <= 0x%08H (%0d)", test_name, rd_addr, mon_regs[rd_addr], $signed(mon_regs[rd_addr]));
    end
  endtask

  // Tarea que simula la pseudoinstrucción RISC-V: li rd, imm32
  // - Si imm32 cabe en 12 bits con signo -> addi rd, x0, imm12
  // - Si requiere 32 bits                -> lui rd, hi20 + addi rd, rd, lo12
  task automatic sim_li(
    input logic [4:0]  rd_addr,
    input logic [31:0] imm32
  );
    logic [19:0] imm_hi;
    logic [11:0] imm_lo;
    logic [31:0] lui_val;
    begin
      imm_lo  = imm32[11:0];
      imm_hi  = imm32[31:12] + imm32[11];   // compensa el signo de ADDI
      lui_val = {imm_hi, 12'h000};

      if ($signed(imm32) >= -2048 && $signed(imm32) <= 2047) begin
        // ADDI rd, x0, imm_lo (I-Type, opcode = 7'b0010011)
        write_reg({imm_lo, 5'd0, 3'b000, rd_addr, 7'b0010011}, rd_addr, imm32, "ADDI");
      end else begin
        // LUI rd, imm_hi (U-Type, opcode = 7'b0110111)
        write_reg({imm_hi, rd_addr, 7'b0110111}, rd_addr, lui_val, "LUI ");
        // ADDI rd, rd, imm_lo -> lui_val + sext(imm_lo) == imm32
        // (en x0 el LUI no escribe, por lo que se parte de 0)
        write_reg({imm_lo, rd_addr, 3'b000, rd_addr, 7'b0010011}, rd_addr,
                  reg_model[rd_addr] + {{20{imm_lo[11]}}, imm_lo}, "ADDI");
      end
    end
  endtask

  // Tarea que comprueba el contenido de los 32 registros frente al modelo
  task automatic check_all_regs(input string phase_name);
    begin
      for (int i = 0; i < NUM_REGS; i++) begin
        assert (mon_regs[i] === reg_model[i])
          else begin num_errors++; $error("Fallo %s: x%0d esp 0x%08H, obt 0x%08H",
                                          phase_name, i, reg_model[i], mon_regs[i]); end
      end
    end
  endtask

  // Tarea que lee xi y xj por los puertos del banco con add x0, xi, xj (no escribe)
  task automatic read_regs(
    input logic [4:0] rs1_addr,
    input logic [4:0] rs2_addr
  );
    begin
      @(negedge clk);
      rdata_a = {7'b0000000, rs2_addr, rs1_addr, 3'b000, 5'd0, 7'b0110011};
      #2;

      assert (mon_opcode === 5'b01100)
        else begin num_errors++; $error("Fallo opcode en lectura x%0d, x%0d", rs1_addr, rs2_addr); end
      assert (mon_rs1 === rs1_addr && mon_rs2 === rs2_addr)
        else begin num_errors++; $error("Fallo rs1/rs2: esp x%0d/x%0d, obt x%0d/x%0d",
                                        rs1_addr, rs2_addr, mon_rs1, mon_rs2); end
      assert (mon_rdata1 === reg_model[rs1_addr])
        else begin num_errors++; $error("Fallo rdata1 x%0d: esp 0x%08H, obt 0x%08H",
                                        rs1_addr, reg_model[rs1_addr], mon_rdata1); end
      assert (mon_rdata2 === reg_model[rs2_addr])
        else begin num_errors++; $error("Fallo rdata2 x%0d: esp 0x%08H, obt 0x%08H",
                                        rs2_addr, reg_model[rs2_addr], mon_rdata2); end

      @(posedge clk);
      #1;
      assert (mon_regs[0] === 32'h0)
        else begin num_errors++; $error("Fallo: x0 modificado tras add x0, x%0d, x%0d", rs1_addr, rs2_addr); end
    end
  endtask

  initial begin
    $display("=== INICIO SIMULACION ESCRITURA DE LOS %0d REGISTROS ===", NUM_REGS);
    $display("    FORCE_WRITE_PATH = %0d", FORCE_WRITE_PATH);
    num_errors = 0;
    foreach (reg_model[i]) reg_model[i] = '0;

    // 1. Reset inicial
    nRst    = 1'b0;
    rdata_a = 32'h0000_0013;   // NOP (addi x0, x0, 0)
    rdata_b = 32'h0;
    repeat (2) @(posedge clk);
    #1;
    check_all_regs("Reset");
    nRst = 1'b1;

    // 2. Escritura de los 32 registros con valores de 12 bits
    $display("\n--- FASE 1: ESCRITURA DE x0..x31 (ADDI xi, x0, VAL) ---");
    for (int i = 0; i < NUM_REGS; i++)
      sim_li(i[4:0], test_val(i));

    // 3. Contenido completo del banco
    $display("\n--- FASE 2: COMPROBACION DEL CONTENIDO DEL BANCO ---");
    check_all_regs("Fase 2");
    for (int i = 0; i < NUM_REGS; i++)
      $display("  x%-2d = 0x%08H (%0d)", i, mon_regs[i], $signed(mon_regs[i]));

    // 4. Lectura por los puertos del banco: rs1 = xi, rs2 = x(31-i)
    $display("\n--- FASE 3: LECTURA POR LOS PUERTOS rdata1 / rdata2 ---");
    for (int i = 0; i < NUM_REGS; i++)
      read_regs(i[4:0], 5'(NUM_REGS - 1 - i));

    // 5. Reescritura con valores de 32 bits (LUI + ADDI)
    $display("\n--- FASE 4: REESCRITURA DE x0..x31 CON 32 BITS (LUI + ADDI) ---");
    for (int i = 0; i < NUM_REGS; i++)
      sim_li(i[4:0], 32'hA5A5_0000 ^ (i << 20) ^ (i * 32'h0000_0FED));
    check_all_regs("Fase 4");

    #20;
    $display("------------------------------------------------------------------");
    if (num_errors == 0)
      $display("=== PASS: escritura correcta en los %0d registros ===", NUM_REGS);
    else
      $display("=== FAIL: %0d errores ===", num_errors);
    $finish;
  end

endmodule
