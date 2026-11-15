`timescale 1ns / 1ps

module tb_Regfile;

  localparam int REG_WIDTH = 32;
  localparam int ADDR_REGS = 5;

  // Señales de control y puertos[cite: 1]
  logic                 clk;
  logic                 nRst;
  logic [ADDR_REGS-1:0] raddr1;
  logic [REG_WIDTH-1:0] rdata1;
  logic [ADDR_REGS-1:0] raddr2;
  logic [REG_WIDTH-1:0] rdata2;
  logic [ADDR_REGS-1:0] waddr;
  logic [REG_WIDTH-1:0] wreg;
  logic                 we;
  logic                 wb;
  logic [REG_WIDTH-1:0] wdata;

  // Instancia del DUT (regFile.vhd)[cite: 1]
  regFile #(
    .REG_WIDTH (REG_WIDTH),
    .ADDR_REGS (ADDR_REGS)
  ) dut (
    .clk    (clk),
    .nRst   (nRst),
    .raddr1 (raddr1),
    .rdata1 (rdata1),
    .raddr2 (raddr2),
    .rdata2 (rdata2),
    .waddr  (waddr),
    .wreg   (wreg),
    .we     (we),
    .wb     (wb),
    .wdata  (wdata)
  );

  // Reloj (10 ns)
  initial clk = 1'b0;
  always #5 clk = ~clk;

  initial begin
    $display("=== INICIO TESTBENCH REGFILE ===");

    // Inicialización
    nRst   = 1'b0;
    raddr1 = 5'd0;
    raddr2 = 5'd0;
    waddr  = 5'd0;
    wreg   = 32'h0;
    we     = 1'b0;
    wb     = 1'b0;
    wdata  = 32'h0;

    // 1. Prueba de Reset asíncrono[cite: 1]
    repeat (2) @(posedge clk);
    nRst = 1'b1;
    #2;
    assert (rdata1 === 32'h0 && rdata2 === 32'h0) else $error("Fallo tras reset");
    $display("[OK] Reset inicial verificado.");

    // 2. Intentar escribir en registro x0 (debe ignorarse y seguir valiendo 0)[cite: 1]
    @(negedge clk);
    waddr = 5'd0;
    wreg  = 32'hDEAD_BEEF;
    we    = 1'b1;
    @(posedge clk);
    #1;
    we     = 1'b0;
    raddr1 = 5'd0;
    #1;
    assert (rdata1 === 32'h0) else $error("Fallo proteccion x0: se escribio 0x%08H", rdata1);
    $display("[OK] Proteccion de x0 verificada (rdata1 = 0x%08H).", rdata1);

    // 3. Escritura desde ALU (we = 1) en x1 y x2[cite: 1]
    @(negedge clk);
    waddr = 5'd1;
    wreg  = 32'h1111_2222;
    we    = 1'b1;
    @(posedge clk);

    @(negedge clk);
    waddr = 5'd2;
    wreg  = 32'h3333_4444;
    we    = 1'b1;
    @(posedge clk);
    #1;
    we = 1'b0;

    // Lectura asíncrona simultánea de x1 y x2[cite: 1]
    raddr1 = 5'd1;
    raddr2 = 5'd2;
    #2;
    assert (rdata1 === 32'h1111_2222) else $error("Fallo lectura x1: obt 0x%08H", rdata1);
    assert (rdata2 === 32'h3333_4444) else $error("Fallo lectura x2: obt 0x%08H", rdata2);
    $display("[OK] Escritura ALU y lectura dual: x1=0x%08H, x2=0x%08H", rdata1, rdata2);

    // 4. Escritura desde Memoria / Write-Back (wb = 1) en x10[cite: 1]
    @(negedge clk);
    waddr = 5'd10;
    wdata = 32'hABCD_1234;
    wb    = 1'b1;
    @(posedge clk);
    #1;
    wb     = 1'b0;
    raddr1 = 5'd10;
    #2;
    assert (rdata1 === 32'hABCD_1234) else $error("Fallo escritura WB en x10: obt 0x%08H", rdata1);
    $display("[OK] Escritura Write-Back en x10: rdata1=0x%08H", rdata1);

    // 5. Prueba de prioridad si we=1 y wb=1 simultáneamente en el mismo ciclo[cite: 1]
    // (En regFile.vhd, el 'if wb = 1' va después de 'if we = 1', por lo que wb prevalece)[cite: 1]
    // REVISAR ESCRITURA SIMULTANEA: memoria y alu quieren escribir (MEMORIA NECESITA DOS CICLOS DE RELOJO, ALU ES INMEDIATA) -> PRIORIDAD A WB
    @(negedge clk);
    waddr = 5'd15;
    wreg  = 32'hAAAA_AAAA;
    wdata = 32'hBBBB_BBBB;
    we    = 1'b1;
    wb    = 1'b1;
    @(posedge clk);
    #1;
    we     = 1'b0;
    wb     = 1'b0;
    raddr2 = 5'd15;
    #2;
    assert (rdata2 === 32'hBBBB_BBBB) else $error("Fallo prioridad wb sobre we en x15: obt 0x%08H", rdata2);
    $display("[OK] Prioridad wb sobre we verificada en x15: rdata2=0x%08H", rdata2);

    // 6. Reset en mitad de operación limpia los registros[cite: 1]
    nRst = 1'b0;
    #5;
    assert (rdata1 === 32'h0 && rdata2 === 32'h0) else $error("Fallo borrado por reset asincrono");
    $display("[OK] Limpieza de registros tras reset asincrono verificada.");

    #10;
    $display("=== FIN TESTBENCH REGFILE ===");
    $finish;
  end

endmodule
