`timescale 1ns / 1ps

// =============================================================================
// tb_riscV_programa
// Test sencillo para simular un programa en el sistema completo
// (core + memory.vhd).
//
// Como usarlo:
//   1. Escribir el programa en la tarea programa(), una instruccion por linea:
//        poner(32'h........);
//      La primera instruccion se coloca en PROG_BASE (0x10000, PC inicial del
//      core) y las siguientes cada 4 bytes. El programa debe acabar en un
//      bucle infinito "jal x0, 0" (0x0000006F), que es donde se detiene el test.
//      (La tarea no puede llamarse "program" porque es una palabra reservada
//      de SystemVerilog.)
//   2. Poner en ESP_X21 ... ESP_X28 el valor final esperado de x21 ... x28.
//   3. Simular. Cada vez que cambia uno de los registros x21 a x28 se muestra
//      su nuevo valor y, al llegar al halt, se comparan con los esperados.
//
// Ahora programa() contiene las 121 instrucciones de instr_test.txt.
//
// Compilar y simular (desde la raiz del proyecto):
//   vlib work
//   vcom -2008 HDL/alu.vhd
//   vcom -2002 HDL/divisor.vhd HDL/regFile.vhd HDL/decoder.vhd HDL/fetch.vhd HDL/DecExe.vhd HDL/core.vhd HDL/memory.vhd
//   vlog -sv TEST/tb_riscV_programa.sv
//   vsim -c work.tb_riscV_programa -do "run -all"
// =============================================================================
module tb_riscV_programa;

  parameter DIV_ARCH = 0;                       // divisor: 0 serie, 1 paralela, 2 segmentada, 3 sin divisor

  localparam [31:0] PROG_BASE  = 32'h0001_0000; // direccion de la primera instruccion
  localparam [31:0] HALT       = 32'h0000_006F; // jal x0, 0
  localparam integer MAX_CICLOS = 2000;

  // Valores finales esperados de x21 ... x28 (programa de instr_test.txt)
  localparam [31:0] ESP_X21 = 32'h12355008;
  localparam [31:0] ESP_X22 = 32'h00000125;
  localparam [31:0] ESP_X23 = 32'h7FFFFF3F;
  localparam [31:0] ESP_X24 = 32'h000000AD;
  localparam [31:0] ESP_X25 = 32'h0000004D;
  localparam [31:0] ESP_X26 = 32'h00000001;
  localparam [31:0] ESP_X27 = 32'h00000002;
  localparam [31:0] ESP_X28 = 32'h123552E9;

  // ---------------------------------------------------------------------------
  // Reloj, reset y conexiones
  // ---------------------------------------------------------------------------
  reg clk  = 0;
  reg nRst = 0;
  always #5 clk = ~clk;

  wire [31:0] rdata_a, rdata_b, wdata_a, wdata_b;
  wire [16:0] addr_a, addr_b;
  wire        we_a, we_b;
  wire [3:0]  be_a, be_b;

  // Carga del programa por el puerto A mientras el core esta en reset
  reg        carga  = 1;
  reg [16:0] c_addr = 0;
  reg [31:0] c_dato = 0;
  reg        c_we   = 0;

  core #(.DIV_ARCH(DIV_ARCH)) dut (
    .clk(clk), .nRst(nRst),
    .rdata_a(rdata_a), .addr_a(addr_a), .wdata_a(wdata_a), .we_a(we_a), .be_a(be_a),
    .rdata_b(rdata_b), .addr_b(addr_b), .wdata_b(wdata_b), .we_b(we_b), .be_b(be_b)
  );

  // La memoria se direcciona por palabras: direccion de byte / 4
  memory u_mem (
    .clk(clk),
    .addr_a (carga ? c_addr  : {2'b00, addr_a[16:2]}),
    .wdata_a(carga ? c_dato  : wdata_a),
    .we_a   (carga ? c_we    : we_a),
    .be_a   (carga ? 4'b1111 : be_a),
    .rdata_a(rdata_a),
    .addr_b ({2'b00, addr_b[16:2]}),
    .wdata_b(wdata_b),
    .we_b   (we_b),
    .be_b   (be_b),
    .rdata_b(rdata_b)
  );

  // ---------------------------------------------------------------------------
  // Programa
  // ---------------------------------------------------------------------------
  reg [31:0] prog [0:1023];
  integer    n = 0;             // numero de instrucciones

  task poner(input [31:0] instr);
    prog[n] = instr;
    n = n + 1;
  endtask

  task automatic programa();
    poner(32'h00001A37); // 0x10000  lui x20, 0x00001
    poner(32'h123455B7); // 0x10004  lui x11, 0x12345
    poner(32'h00000197); // 0x10008  auipc x3, 0x00000
    poner(32'h0035CAB3); // 0x1000C  xor x21, x11, x3
    poner(32'h06400593); // 0x10010  addi x11, x0, 100
    poner(32'h0C85A193); // 0x10014  slti x3, x11, 200
    poner(32'h0325B213); // 0x10018  sltiu x4, x11, 50
    poner(32'h03F5F293); // 0x1001C  andi x5, x11, 63
    poner(32'h00F5E313); // 0x10020  ori x6, x11, 15
    poner(32'h0FF5C393); // 0x10024  xori x7, x11, 255
    poner(32'h00259413); // 0x10028  slli x8, x11, 2
    poner(32'h0015D493); // 0x1002C  srli x9, x11, 1
    poner(32'h4015D513); // 0x10030  srai x10, x11, 1
    poner(32'h0035CB33); // 0x10034  xor x22, x11, x3
    poner(32'h004B4B33); // 0x10038  xor x22, x22, x4
    poner(32'h005B4B33); // 0x1003C  xor x22, x22, x5
    poner(32'h006B4B33); // 0x10040  xor x22, x22, x6
    poner(32'h007B4B33); // 0x10044  xor x22, x22, x7
    poner(32'h008B4B33); // 0x10048  xor x22, x22, x8
    poner(32'h009B4B33); // 0x1004C  xor x22, x22, x9
    poner(32'h00AB4B33); // 0x10050  xor x22, x22, x10
    poner(32'h06400593); // 0x10054  addi x11, x0, 100
    poner(32'hFF900193); // 0x10058  addi x3, x0, -7
    poner(32'h00100213); // 0x1005C  addi x4, x0, 1
    poner(32'h003582B3); // 0x10060  add x5, x11, x3
    poner(32'h40358333); // 0x10064  sub x6, x11, x3
    poner(32'h00B1A3B3); // 0x10068  slt x7, x3, x11
    poner(32'h00B1B433); // 0x1006C  sltu x8, x3, x11
    poner(32'h0035F4B3); // 0x10070  and x9, x11, x3
    poner(32'h0035E533); // 0x10074  or x10, x11, x3
    poner(32'h0035C5B3); // 0x10078  xor x11, x11, x3
    poner(32'h00459633); // 0x1007C  sll x12, x11, x4
    poner(32'h0045D6B3); // 0x10080  srl x13, x11, x4
    poner(32'h4041D733); // 0x10084  sra x14, x3, x4
    poner(32'h0062CBB3); // 0x10088  xor x23, x5, x6
    poner(32'h007BCBB3); // 0x1008C  xor x23, x23, x7
    poner(32'h008BCBB3); // 0x10090  xor x23, x23, x8
    poner(32'h009BCBB3); // 0x10094  xor x23, x23, x9
    poner(32'h00ABCBB3); // 0x10098  xor x23, x23, x10
    poner(32'h00BBCBB3); // 0x1009C  xor x23, x23, x11
    poner(32'h00CBCBB3); // 0x100A0  xor x23, x23, x12
    poner(32'h00DBCBB3); // 0x100A4  xor x23, x23, x13
    poner(32'h00EBCBB3); // 0x100A8  xor x23, x23, x14
    poner(32'h06400593); // 0x100AC  addi x11, x0, 100
    poner(32'h10000A17); // 0x100B0  auipc x20, 0x10000
    poner(32'hF50A0A13); // 0x100B4  addi x20, x20, -176
    poner(32'h00BA2023); // 0x100B8  sw x11, 0(x20)
    poner(32'h000A2183); // 0x100BC  lw x3, 0(x20)
    poner(32'h0AB00213); // 0x100C0  addi x4, x0, 171
    poner(32'h004A1223); // 0x100C4  sh x4, 4(x20)
    poner(32'h004A1283); // 0x100C8  lh x5, 4(x20)
    poner(32'h004A5303); // 0x100CC  lhu x6, 4(x20)
    poner(32'h004A0423); // 0x100D0  sb x4, 8(x20)
    poner(32'h008A0383); // 0x100D4  lb x7, 8(x20)
    poner(32'h008A4403); // 0x100D8  lbu x8, 8(x20)
    poner(32'h03700493); // 0x100DC  addi x9, x0, 55
    poner(32'h009A2623); // 0x100E0  sw x9, 12(x20)
    poner(32'h00CA2503); // 0x100E4  lw x10, 12(x20)
    poner(32'h0051CC33); // 0x100E8  xor x24, x3, x5
    poner(32'h006C4C33); // 0x100EC  xor x24, x24, x6
    poner(32'h007C4C33); // 0x100F0  xor x24, x24, x7
    poner(32'h008C4C33); // 0x100F4  xor x24, x24, x8
    poner(32'h00AC4C33); // 0x100F8  xor x24, x24, x10
    poner(32'h41800C33); // 0x100FC  sub x24, x0, x24
    poner(32'h04D00593); // 0x10100  addi x11, x0, 77
    poner(32'h00BA2823); // 0x10104  sw x11, 16(x20)
    poner(32'h010A2183); // 0x10108  lw x3, 16(x20)
    poner(32'h00018CB3); // 0x1010C  add x25, x3, x0
    poner(32'h00000D13); // 0x10110  addi x26, x0, 0
    poner(32'h06400593); // 0x10114  addi x11, x0, 100
    poner(32'hFF900193); // 0x10118  addi x3, x0, -7
    poner(32'h00100213); // 0x1011C  addi x4, x0, 1
    poner(32'h06400593); // 0x10120  addi x11, x0, 100
    poner(32'h00B58463); // 0x10124  beq x11, x11, 8 (-> 0x1012c)
    poner(32'h001D0D13); // 0x10128  addi x26, x26, 1
    poner(32'h02B59E63); // 0x1012C  bne x11, x11, 60 (-> 0x10168)
    poner(32'h00359463); // 0x10130  bne x11, x3, 8 (-> 0x10138)
    poner(32'h001D0D13); // 0x10134  addi x26, x26, 1
    poner(32'h00B1C463); // 0x10138  blt x3, x11, 8 (-> 0x10140)
    poner(32'h001D0D13); // 0x1013C  addi x26, x26, 1
    poner(32'h0235C463); // 0x10140  blt x11, x3, 40 (-> 0x10168)
    poner(32'h0035D463); // 0x10144  bge x11, x3, 8 (-> 0x1014c)
    poner(32'h001D0D13); // 0x10148  addi x26, x26, 1
    poner(32'h00B1DE63); // 0x1014C  bge x3, x11, 28 (-> 0x10168)
    poner(32'h00B26463); // 0x10150  bltu x4, x11, 8 (-> 0x10158)
    poner(32'h001D0D13); // 0x10154  addi x26, x26, 1
    poner(32'h0045E863); // 0x10158  bltu x11, x4, 16 (-> 0x10168)
    poner(32'h0045F463); // 0x1015C  bgeu x11, x4, 8 (-> 0x10164)
    poner(32'h001D0D13); // 0x10160  addi x26, x26, 1
    poner(32'h00B27263); // 0x10164  bgeu x4, x11, 4 (-> 0x10168)
    poner(32'h001D0D13); // 0x10168  addi x26, x26, 1
    poner(32'h00000D93); // 0x1016C  addi x27, x0, 0
    poner(32'h014000EF); // 0x10170  jal x1, 20 (-> 0x10184)
    poner(32'h001D8D93); // 0x10174  addi x27, x27, 1
    poner(32'h010000EF); // 0x10178  jal x1, 16 (-> 0x10188)
    poner(32'h001D8D93); // 0x1017C  addi x27, x27, 1
    poner(32'h00C0006F); // 0x10180  jal x0, 12 (-> 0x1018c)
    poner(32'h00008067); // 0x10184  jalr x0, 0(x1)
    poner(32'h00008067); // 0x10188  jalr x0, 0(x1)
    poner(32'hFFF00593); // 0x1018C  addi x11, x0, -1
    poner(32'h80000193); // 0x10190  addi x3, x0, -2048
    poner(32'h7FF00213); // 0x10194  addi x4, x0, 2047
    poner(32'hFFF00293); // 0x10198  addi x5, x0, -1
    poner(32'h41F2D293); // 0x1019C  srai x5, x5, 31
    poner(32'hFFF00313); // 0x101A0  addi x6, x0, -1
    poner(32'h01F35313); // 0x101A4  srli x6, x6, 31
    poner(32'h00100393); // 0x101A8  addi x7, x0, 1
    poner(32'h01F39393); // 0x101AC  slli x7, x7, 31
    poner(32'h0035CE33); // 0x101B0  xor x28, x11, x3
    poner(32'h004E4E33); // 0x101B4  xor x28, x28, x4
    poner(32'h005E4E33); // 0x101B8  xor x28, x28, x5
    poner(32'h006E4E33); // 0x101BC  xor x28, x28, x6
    poner(32'h007E4E33); // 0x101C0  xor x28, x28, x7
    poner(32'h417E0E33); // 0x101C4  sub x28, x28, x23
    poner(32'h018E0E33); // 0x101C8  add x28, x28, x24
    poner(32'h019E0E33); // 0x101CC  add x28, x28, x25
    poner(32'h01AE0E33); // 0x101D0  add x28, x28, x26
    poner(32'h01BE0E33); // 0x101D4  add x28, x28, x27
    poner(32'h016E0E33); // 0x101D8  add x28, x28, x22
    poner(32'h015E0E33); // 0x101DC  add x28, x28, x21
    poner(32'h0000006F); // 0x101E0  jal x0, 0 (-> 0x101e0)
  endtask

  // ---------------------------------------------------------------------------
  // Monitorizacion de los registros x21 ... x28
  // ---------------------------------------------------------------------------
  wire [31:0] x21 = dut.U1.U0.regs[21];
  wire [31:0] x22 = dut.U1.U0.regs[22];
  wire [31:0] x23 = dut.U1.U0.regs[23];
  wire [31:0] x24 = dut.U1.U0.regs[24];
  wire [31:0] x25 = dut.U1.U0.regs[25];
  wire [31:0] x26 = dut.U1.U0.regs[26];
  wire [31:0] x27 = dut.U1.U0.regs[27];
  wire [31:0] x28 = dut.U1.U0.regs[28];

  integer ciclo = 0;
  always @(posedge clk)
    if (nRst == 1) ciclo = ciclo + 1;

  always @(x21) if (nRst == 1) $display("  [ciclo %4d] x21 = 0x%08h", ciclo, x21);
  always @(x22) if (nRst == 1) $display("  [ciclo %4d] x22 = 0x%08h", ciclo, x22);
  always @(x23) if (nRst == 1) $display("  [ciclo %4d] x23 = 0x%08h", ciclo, x23);
  always @(x24) if (nRst == 1) $display("  [ciclo %4d] x24 = 0x%08h", ciclo, x24);
  always @(x25) if (nRst == 1) $display("  [ciclo %4d] x25 = 0x%08h", ciclo, x25);
  always @(x26) if (nRst == 1) $display("  [ciclo %4d] x26 = 0x%08h", ciclo, x26);
  always @(x27) if (nRst == 1) $display("  [ciclo %4d] x27 = 0x%08h", ciclo, x27);
  always @(x28) if (nRst == 1) $display("  [ciclo %4d] x28 = 0x%08h", ciclo, x28);

  // ---------------------------------------------------------------------------
  // Comprobacion de un registro
  // ---------------------------------------------------------------------------
  integer errores = 0;

  task comprobar(input string nom, input [31:0] valor, input [31:0] esperado);
    if (valor === esperado)
      $display("  OK     %s = 0x%08h", nom, valor);
    else begin
      $display("  ERROR  %s = 0x%08h (esperado 0x%08h)", nom, valor, esperado);
      errores = errores + 1;
    end
  endtask

  // ---------------------------------------------------------------------------
  // Test
  // ---------------------------------------------------------------------------
  integer i;
  integer halt;

  initial begin
    $display("=== tb_riscV_programa ===");

    // 1. Carga del programa en memoria (core en reset)
    programa();
    for (i = 0; i < n; i = i + 1) begin
      @(negedge clk);
      c_addr = {2'b00, PROG_BASE[16:2]} + i;
      c_dato = prog[i];
      c_we   = 1;
    end
    @(negedge clk);
    c_we  = 0;
    carga = 0;
    nRst  = 1;
    $display("Programa cargado: %0d instrucciones desde 0x%05h", n, PROG_BASE);

    // 2. Ejecucion hasta que la instruccion del decoder es el halt
    halt = 0;
    while (halt == 0 && ciclo < MAX_CICLOS) begin
      @(posedge clk);
      #1;
      if (dut.instr_dec === HALT) halt = 1;
    end
    repeat (3) @(posedge clk);

    // 3. Resultado
    $display("------------------------------------------------");
    if (halt == 1)
      $display("Halt alcanzado en el ciclo %0d", ciclo - 3);
    else begin
      $display("ERROR: no se llega al halt en %0d ciclos", MAX_CICLOS);
      errores = errores + 1;
    end
    comprobar("x21", x21, ESP_X21);
    comprobar("x22", x22, ESP_X22);
    comprobar("x23", x23, ESP_X23);
    comprobar("x24", x24, ESP_X24);
    comprobar("x25", x25, ESP_X25);
    comprobar("x26", x26, ESP_X26);
    comprobar("x27", x27, ESP_X27);
    comprobar("x28", x28, ESP_X28);

    if (errores == 0)
      $display("=== RESULTADO: CORRECTO ===");
    else
      $display("=== RESULTADO: %0d ERRORES ===", errores);
    $finish;
  end

endmodule
