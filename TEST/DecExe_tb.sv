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

  // Tarea que simula la pseudoinstrucción RISC-V: li rd, imm32
  // - Si imm32 cabe en 12 bits con signo [-2048, 2047] -> ejecuta: addi rd, x0, imm12
  // - Si requiere 32 bits -> ejecuta: lui rd, hi20 seguido de addi rd, rd, lo12
  task automatic sim_li(
    input logic [4:0]  rd_addr,
    input logic [31:0] imm32
  );
    logic [19:0] imm_hi;
    logic [11:0] imm_lo;
    logic [31:0] lui_val;
    begin
      imm_lo = imm32[11:0];
      // Compensación del bit de signo de ADDI: si imm32[11] == 1, ADDI restará,
      // por lo que sumamos 1 al campo de 20 bits superiores de LUI.
      imm_hi = imm32[31:12] + imm32[11];

      // Comprobamos si el inmediato cabe en 12 bits con signo
      if ($signed(imm32) >= -2048 && $signed(imm32) <= 2047) begin
        // =====================================================================
        // Caso 12 bits: ADDI rd, x0, imm_lo (I-Type, opcode = 7'b0010011)
        // Formato: {imm[11:0], rs1(5'd0), funct3(3'b000), rd[4:0], 7'b0010011}
        // =====================================================================
        @(negedge clk);
        rdata_a = {imm_lo, 5'd0, 3'b000, rd_addr, 7'b0010011};
        
        // Forzamos la ruta de escritura en DecExe hasta que conectes el mux de imm y we
        preload_val_static = imm32;
        force dut.we   = 1'b1;
        force dut.wAlu = preload_val_static;

        @(posedge clk);
        #1;
        release dut.we;
        release dut.wAlu;
        $display("  [LI - 1 ciclo ] ADDI x%0d, x0, %0d -> x%0d <= 0x%08H (%0d)",
                 rd_addr, $signed(imm_lo), rd_addr, imm32, $signed(imm32));

      end else begin
        // =====================================================================
        // Caso 32 bits (Paso 1/2): LUI rd, imm_hi (U-Type, opcode = 7'b0110111)
        // Formato: {imm[31:12], rd[4:0], 7'b0110111}
        // =====================================================================
        @(negedge clk);
        rdata_a = {imm_hi, rd_addr, 7'b0110111};
        
        lui_val = {imm_hi, 12'h000};
        preload_val_static = lui_val;
        force dut.we   = 1'b1;
        force dut.wAlu = preload_val_static;

        @(posedge clk);
        #1;
        $display("  [LI - Paso 1/2] LUI  x%0d, 0x%05H -> x%0d <= 0x%08H",
                 rd_addr, imm_hi, rd_addr, lui_val);

        // =====================================================================
        // Caso 32 bits (Paso 2/2): ADDI rd, rd, imm_lo (I-Type, opcode = 7'b0010011)
        // Formato: {imm[11:0], rs1(rd), funct3(3'b000), rd[4:0], 7'b0010011}
        // =====================================================================
        @(negedge clk);
        rdata_a = {imm_lo, rd_addr, 3'b000, rd_addr, 7'b0010011};
        
        // Resultado de sumar lui_val + sign_extend(imm_lo) == imm32
        preload_val_static = lui_val + {{20{imm_lo[11]}}, imm_lo};
        force dut.we   = 1'b1;
        force dut.wAlu = preload_val_static;

        @(posedge clk);
        #1;
        release dut.we;
        release dut.wAlu;
        $display("  [LI - Paso 2/2] ADDI x%0d, x%0d, %0d -> x%0d <= 0x%08H (%0d)",
                 rd_addr, rd_addr, $signed(imm_lo), rd_addr, preload_val_static, $signed(preload_val_static));
      end
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
    // Valores esperados en el banco (x0 siempre vale 0)
    logic [REG_WIDTH-1:0] reg_vals [2**ADDR_REGS];
    int                   j;

    $display("=== INICIO SIMULACION DECEXE: ESCRITURA EN LOS %0d REGISTROS ===", 2**ADDR_REGS);

    // 1. Reset inicial
    nRst    = 1'b0;
    rdata_a = 32'h0;
    rdata_b = 32'h0;
    repeat (2) @(posedge clk);
    nRst = 1'b1;

    // Valores de prueba:
    //  - registros pares  -> caben en 12 bits con signo (positivos y negativos) -> ADDI
    //  - registros impares -> necesitan 32 bits -> LUI + ADDI
    for (int i = 0; i < 2**ADDR_REGS; i++) begin
      if (i % 2 == 0) reg_vals[i] = i * 100 - 1500;
      else            reg_vals[i] = 32'h8765_4321 ^ (i << 8);
    end

    // 2. Escritura de cada registro con la pseudoinstrucción LI y comprobación
    //    inmediata con test_instr_with_regs: add x0, xi, x0
    //    -> rdata1 = xi, rdata2 = 0, wAlu = xi (rd = x0, no modifica el banco)
    $display("\n--- FASE 1: ESCRITURA Y COMPROBACION DE x0..x31 ---");
    for (int i = 0; i < 2**ADDR_REGS; i++) begin
      sim_li(i[4:0], reg_vals[i]);
      if (i == 0) reg_vals[0] = 32'h0;  // x0 no se puede escribir: debe seguir valiendo 0

      test_instr_with_regs({7'b0000000, 5'd0, i[4:0], 3'b000, 5'd0, 7'b0110011},
                           $sformatf("Escritura x%0d (add x0, x%0d, x0)", i, i),
                           5'b01100, reg_vals[i], 32'h0, reg_vals[i],
                           (reg_vals[i] == 32'h0), ($signed(reg_vals[i]) < 0));
    end

    // 3. Lectura simultanea por ambos puertos: add x0, xi, x(31-i)
    //    Comprueba que ninguna escritura posterior ha pisado a otro registro
    $display("\n--- FASE 2: LECTURA POR rdata1 Y rdata2 (add x0, xi, x(31-i)) ---");
    for (int i = 0; i < 2**ADDR_REGS; i++) begin
      j = 2**ADDR_REGS - 1 - i;
      test_instr_with_regs({7'b0000000, j[4:0], i[4:0], 3'b000, 5'd0, 7'b0110011},
                           $sformatf("Lectura x%0d y x%0d (add x0, x%0d, x%0d)", i, j, i, j),
                           5'b01100, reg_vals[i], reg_vals[j], reg_vals[i] + reg_vals[j],
                           (reg_vals[i] == reg_vals[j]), ($signed(reg_vals[i]) < $signed(reg_vals[j])));
    end

    #20;
    $display("------------------------------------------------------------------");
    $display("=== SIMULACION COMPLETADA (revisar si hay mensajes ** Error) ===");
    $finish;
  end

endmodule
