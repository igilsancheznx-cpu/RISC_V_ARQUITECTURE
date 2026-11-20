`timescale 1ns / 1ps

// =============================================================================
// tb_script_programa
// Igual que tb_riscV_programa, pero el programa se lee de un fichero de texto
// externo en lugar de escribirse dentro del testbench.
//
// Formato del fichero: una instruccion por linea en hexadecimal, con o sin
// "0x" delante (por ejemplo 0x00001A37 o 00001A37). Las lineas vacias y las
// que empiezan por "//" o "#" se ignoran. La primera instruccion se coloca en
// PROG_BASE (0x10000, PC inicial del core) y las siguientes cada 4 bytes. El
// programa debe acabar en un bucle infinito "jal x0, 0" (0x0000006F), que es
// donde se detiene el test.
//
// Como usarlo:
//   1. Indicar el fichero en FICHERO (por defecto instr_test.txt, relativo a
//      la carpeta desde la que se lanza el simulador) o en la simulacion con
//      +FICHERO=ruta/al/fichero.txt
//   2. Poner en ESP_X21 ... ESP_X28 el valor final esperado de x21 ... x28.
//   3. Simular. Cada vez que cambia uno de los registros x21 a x28 se muestra
//      su nuevo valor y, al llegar al halt, se comparan con los esperados.
//
// Compilar y simular (desde la raiz del proyecto):
//   vlib work
//   vcom -2008 HDL/alu.vhd
//   vcom -2002 HDL/divisor.vhd HDL/regFile.vhd HDL/decoder.vhd HDL/fetch.vhd HDL/DecExe.vhd HDL/core.vhd HDL/memory.vhd
//   vlog -sv TEST/tb_script_programa.sv
//   vsim -c work.tb_script_programa -do "run -all"
//   vsim -c work.tb_script_programa +FICHERO=otro_programa.txt -do "run -all"
// =============================================================================
module tb_script_programa;

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

  // Lee el programa del fichero de texto, una instruccion por linea
  string FICHERO = "instr_test.txt";

  task leer_programa();
    integer    fichero;
    string     linea;
    reg [31:0] instr;

    if ($value$plusargs("FICHERO=%s", FICHERO))
      $display("Fichero indicado con +FICHERO");

    fichero = $fopen(FICHERO, "r");
    if (fichero == 0) begin
      $display("ERROR: no se puede abrir el fichero %s", FICHERO);
      $finish;
    end

    while (!$feof(fichero)) begin
      linea = "";
      void'($fgets(linea, fichero));
      if ($sscanf(linea, "0x%h", instr) == 1)      // 0x00001A37
        poner(instr);
      else if ($sscanf(linea, "%h", instr) == 1)   // 00001A37
        poner(instr);
      // otras lineas (vacias o comentarios): se ignoran
    end
    $fclose(fichero);
    $display("Leidas %0d instrucciones de %s", n, FICHERO);
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
    $display("=== tb_script_programa ===");

    // 1. Carga del programa en memoria (core en reset)
    leer_programa();
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
