`timescale 1ns / 1ps

module tb_Regfile;

  localparam int REG_WIDTH = 32;
  localparam int ADDR_REGS = 5;

  // Señales de control y puertos
  logic                 clk;
  logic                 nRst;
  logic [ADDR_REGS-1:0] raddr1;
  logic [REG_WIDTH-1:0] rdata1;
  logic [ADDR_REGS-1:0] raddr2;
  logic [REG_WIDTH-1:0] rdata2;
  logic [ADDR_REGS-1:0] waddr;
  logic [REG_WIDTH-1:0] wreg;
  logic                 we;      // escritura desde la ALU
  logic                 wl;      // escritura desde memoria (LOAD)
  logic [3:0]           be_l;    // bytes validos del dato de memoria
  logic                 sign_l;  // load con signo (lb, lh)
  logic [REG_WIDTH-1:0] wdata;

  // Instancia del DUT (regFile.vhd)
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
    .wl     (wl),
    .be_l   (be_l),
    .sign_l (sign_l),
    .wdata  (wdata)
  );

  // Reloj (10 ns)
  initial clk = 1'b0;
  always #5 clk = ~clk;

  // Tarea que simula la escritura de un LOAD en el banco y comprueba el valor
  task automatic test_load_write(
    input logic [4:0]  reg_addr,
    input logic [31:0] mem_data,
    input logic [3:0]  in_be_l,
    input logic        in_sign_l,
    input logic [31:0] exp_val,
    input string       test_name
  );
    begin
      @(negedge clk);
      waddr  = reg_addr;
      wdata  = mem_data;
      be_l   = in_be_l;
      sign_l = in_sign_l;
      wl     = 1'b1;
      @(posedge clk);
      #1;
      wl     = 1'b0;
      raddr1 = reg_addr;
      #2;
      assert (rdata1 === exp_val) else $error("Fallo %s en x%0d: esp 0x%08H, obt 0x%08H", test_name, reg_addr, exp_val, rdata1);
      $display("[OK?] %s: dato memoria 0x%08H, be_l=%b, sign_l=%b -> x%0d = 0x%08H",
               test_name, mem_data, in_be_l, in_sign_l, reg_addr, rdata1);
    end
  endtask

  initial begin
    $display("=== INICIO TESTBENCH REGFILE ===");

    // Inicialización
    nRst   = 1'b0;
    raddr1 = 5'd0;
    raddr2 = 5'd0;
    waddr  = 5'd0;
    wreg   = 32'h0;
    we     = 1'b0;
    wl     = 1'b0;
    be_l   = 4'b0000;
    sign_l = 1'b0;
    wdata  = 32'h0;

    // 1. Prueba de Reset asíncrono
    repeat (2) @(posedge clk);
    nRst = 1'b1;
    #2;
    assert (rdata1 === 32'h0 && rdata2 === 32'h0) else $error("Fallo tras reset");
    $display("[OK] Reset inicial verificado.");

    // 2. Intentar escribir en registro x0 (debe ignorarse y seguir valiendo 0)
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

    // 3. Escritura desde ALU (we = 1) en x1 y x2
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

    // Lectura asíncrona simultánea de x1 y x2
    raddr1 = 5'd1;
    raddr2 = 5'd2;
    #2;
    assert (rdata1 === 32'h1111_2222) else $error("Fallo lectura x1: obt 0x%08H", rdata1);
    assert (rdata2 === 32'h3333_4444) else $error("Fallo lectura x2: obt 0x%08H", rdata2);
    $display("[OK] Escritura ALU y lectura dual: x1=0x%08H, x2=0x%08H", rdata1, rdata2);

    // 4. Escritura desde Memoria (wl = 1) de una palabra completa (lw) en x10
    test_load_write(5'd10, 32'hABCD_1234, 4'b1111, 1'b1, 32'hABCD_1234, "LW ");

    // 5. Prueba de prioridad si we=1 y wl=1 simultáneamente en el mismo ciclo
    // (En regFile.vhd, el 'if wl = 1' va después de 'if we = 1', por lo que wl prevalece)
    // REVISAR ESCRITURA SIMULTANEA: memoria y alu quieren escribir (MEMORIA NECESITA DOS CICLOS DE RELOJO, ALU ES INMEDIATA) -> PRIORIDAD A WL
    @(negedge clk);
    waddr  = 5'd15;
    wreg   = 32'hAAAA_AAAA;
    wdata  = 32'hBBBB_BBBB;
    be_l   = 4'b1111;
    we     = 1'b1;
    wl     = 1'b1;
    @(posedge clk);
    #1;
    we     = 1'b0;
    wl     = 1'b0;
    raddr2 = 5'd15;
    #2;
    assert (rdata2 === 32'hBBBB_BBBB) else $error("Fallo prioridad wl sobre we en x15: obt 0x%08H", rdata2);
    $display("[OK] Prioridad wl sobre we verificada en x15: rdata2=0x%08H", rdata2);

    // 6. Loads de byte y media palabra: extension de signo (sign_l=1) o con ceros (sign_l=0)
    //    Se escribe sobre registros con valor previo para comprobar que se sobrescriben los 32 bits
    test_load_write(5'd10, 32'hCAFE_F08A, 4'b0001, 1'b1, 32'hFFFF_FF8A, "LB  (negativo)");
    test_load_write(5'd10, 32'hCAFE_F07F, 4'b0001, 1'b1, 32'h0000_007F, "LB  (positivo)");
    test_load_write(5'd15, 32'hCAFE_F08A, 4'b0001, 1'b0, 32'h0000_008A, "LBU");
    test_load_write(5'd1,  32'hCAFE_F08A, 4'b0011, 1'b1, 32'hFFFF_F08A, "LH  (negativo)");
    test_load_write(5'd1,  32'hCAFE_708A, 4'b0011, 1'b1, 32'h0000_708A, "LH  (positivo)");
    test_load_write(5'd2,  32'hCAFE_F08A, 4'b0011, 1'b0, 32'h0000_F08A, "LHU");

    // 7. Reset en mitad de operación limpia los registros
    nRst = 1'b0;
    #5;
    assert (rdata1 === 32'h0 && rdata2 === 32'h0) else $error("Fallo borrado por reset asincrono");
    $display("[OK] Limpieza de registros tras reset asincrono verificada.");

    #10;
    $display("=== FIN TESTBENCH REGFILE ===");
    $finish;
  end

endmodule
