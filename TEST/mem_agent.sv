`timescale 1ns / 1ps

// =============================================================================
// mem_agent
// Agente de verificacion que sustituye a memory.vhd en los testbench.
// Responde al core exactamente igual que la memoria RTL:
//   - Memoria de doble puerto, direccionada por palabras (2**ADDR_WIDTH palabras)
//   - Lectura sincrona: rdata se actualiza en el flanco de subida con el
//     contenido de mem[addr] ANTES de la escritura de ese mismo flanco
//   - Escritura sincrona con byte enable (be[i] habilita el byte i)
//   - Contenido inicial igual que memory.vhd (programa de ejemplo en 0..9)
//
// Ademas del modelo, el agente incluye:
//   - Monitor: muestra las transacciones (fetch en A, lecturas/escrituras en B)
//     segun el nivel VERBOSE (0: nada, 1: escrituras, 2: escrituras + fetch y lecturas)
//   - Comprobaciones de protocolo: direcciones o enables con X/Z, escrituras
//     sin byte enable y escritura simultanea de los dos puertos a la misma palabra
//   - Backdoor: escribir / leer palabras sin pasar por los puertos
//   - Estadisticas: numero de fetch, lecturas y escrituras (report)
// =============================================================================
module mem_agent #(
  parameter int MEM_WIDTH  = 32,
  parameter int NUM_BYTES  = 4,
  parameter int ADDR_WIDTH = 17,
  parameter int VERBOSE    = 1
)(
  input  logic                  clk,
  // Puerto A (instrucciones)
  input  logic [ADDR_WIDTH-1:0] addr_a,
  input  logic [MEM_WIDTH-1:0]  wdata_a,
  output logic [MEM_WIDTH-1:0]  rdata_a,
  input  logic                  we_a,
  input  logic [NUM_BYTES-1:0]  be_a,
  // Puerto B (datos)
  input  logic [ADDR_WIDTH-1:0] addr_b,
  input  logic [MEM_WIDTH-1:0]  wdata_b,
  output logic [MEM_WIDTH-1:0]  rdata_b,
  input  logic                  we_b,
  input  logic [NUM_BYTES-1:0]  be_b
);

  localparam int DEPTH = 2**ADDR_WIDTH;

  // Contenido de la memoria (mismo nombre que en memory.vhd: mem)
  logic [MEM_WIDTH-1:0] mem [0:DEPTH-1];

  // Estadisticas
  int unsigned n_fetch;
  int unsigned n_read_b;
  int unsigned n_write_a;
  int unsigned n_write_b;
  int unsigned n_errors;

  // Ultima direccion leida por B (para no repetir mensajes de lectura)
  logic [ADDR_WIDTH-1:0] last_addr_b;

  // ---------------------------------------------------------------------------
  // Contenido inicial: igual que memory.vhd
  // ---------------------------------------------------------------------------
  initial begin
    foreach (mem[i]) mem[i] = '0;
    mem[0] = 32'h00a00093; // addi x1, x0, 10
    mem[1] = 32'h01400113; // addi x2, x0, 20
    mem[2] = 32'h002081b3; // add  x3, x1, x2
    mem[3] = 32'h00302023; // sw   x3, 0(x0)
    mem[4] = 32'h3e800093;
    mem[5] = 32'h7d008113;
    mem[6] = 32'hc1810193;
    mem[7] = 32'h83018213;
    mem[8] = 32'h3e820293;
    mem[9] = 32'hfec30313;
    n_fetch     = 0;
    n_read_b    = 0;
    n_write_a   = 0;
    n_write_b   = 0;
    n_errors    = 0;
    last_addr_b = '0;
  end

  // ---------------------------------------------------------------------------
  // Funciones auxiliares
  // ---------------------------------------------------------------------------
  // Aplica una escritura con byte enable sobre una palabra
  function automatic logic [MEM_WIDTH-1:0] merge_bytes(
    input logic [MEM_WIDTH-1:0] old_word,
    input logic [MEM_WIDTH-1:0] new_word,
    input logic [NUM_BYTES-1:0] be
  );
    logic [MEM_WIDTH-1:0] res;
    res = old_word;
    for (int i = 0; i < NUM_BYTES; i++)
      if (be[i]) res[8*i +: 8] = new_word[8*i +: 8];
    return res;
  endfunction

  // Lectura de una palabra: devuelve X si la direccion no es valida
  function automatic logic [MEM_WIDTH-1:0] read_word(input logic [ADDR_WIDTH-1:0] addr);
    if ($isunknown(addr)) return 'x;
    return mem[addr];
  endfunction

  // ---------------------------------------------------------------------------
  // Modelo: lectura y escritura sincronas en los dos puertos
  // (las lecturas devuelven el valor anterior a la escritura, como el VHDL)
  // ---------------------------------------------------------------------------
  always @(posedge clk) begin
    // ===================== PUERTO A =====================
    rdata_a <= read_word(addr_a);
    if (we_a === 1'b1 && !$isunknown(addr_a))
      mem[addr_a] <= merge_bytes(mem[addr_a], wdata_a, be_a);

    // ===================== PUERTO B =====================
    rdata_b <= read_word(addr_b);
    if (we_b === 1'b1 && !$isunknown(addr_b))
      mem[addr_b] <= merge_bytes(mem[addr_b], wdata_b, be_b);
  end

  // ---------------------------------------------------------------------------
  // Monitor y comprobaciones de protocolo (se muestrean en el mismo flanco)
  // ---------------------------------------------------------------------------
  always @(posedge clk) begin
    // ---- Puerto A ----
    if ($isunknown(addr_a)) begin
      n_errors++;
      $warning("[MEM_AGENT] %0t ns: direccion del puerto A desconocida (addr_a=%b)", $time, addr_a);
    end else if (we_a === 1'b1) begin
      n_write_a++;
      if (VERBOSE >= 1)
        $display("  [MEM_AGENT] %0t ns: ESCRITURA A mem[0x%05H] (byte 0x%05H) <= 0x%08H be=%b",
                 $time, addr_a, addr_a << 2, wdata_a, be_a);
    end else begin
      n_fetch++;
      if (VERBOSE >= 2)
        $display("  [MEM_AGENT] %0t ns: FETCH     A mem[0x%05H] (PC 0x%05H) -> 0x%08H",
                 $time, addr_a, addr_a << 2, mem[addr_a]);
    end
    if ($isunknown(we_a)) begin
      n_errors++;
      $warning("[MEM_AGENT] %0t ns: we_a desconocido (%b)", $time, we_a);
    end

    // ---- Puerto B ----
    if ($isunknown(addr_b)) begin
      n_errors++;
      $warning("[MEM_AGENT] %0t ns: direccion del puerto B desconocida (addr_b=%b)", $time, addr_b);
    end else if (we_b === 1'b1) begin
      n_write_b++;
      if (VERBOSE >= 1)
        $display("  [MEM_AGENT] %0t ns: ESCRITURA B mem[0x%05H] (byte 0x%05H) <= 0x%08H be=%b | 0x%08H -> 0x%08H",
                 $time, addr_b, addr_b << 2, wdata_b, be_b,
                 mem[addr_b], merge_bytes(mem[addr_b], wdata_b, be_b));
      if (be_b === '0) begin
        n_errors++;
        $warning("[MEM_AGENT] %0t ns: escritura en B sin byte enable (be_b=0000)", $time);
      end
      if ($isunknown(wdata_b) || $isunknown(be_b)) begin
        n_errors++;
        $warning("[MEM_AGENT] %0t ns: escritura en B con X/Z (wdata_b=0x%08H, be_b=%b)", $time, wdata_b, be_b);
      end
    end else if (addr_b !== last_addr_b) begin
      n_read_b++;
      if (VERBOSE >= 2)
        $display("  [MEM_AGENT] %0t ns: LECTURA   B mem[0x%05H] (byte 0x%05H) -> 0x%08H",
                 $time, addr_b, addr_b << 2, mem[addr_b]);
    end
    if ($isunknown(we_b)) begin
      n_errors++;
      $warning("[MEM_AGENT] %0t ns: we_b desconocido (%b)", $time, we_b);
    end
    last_addr_b <= addr_b;

    // ---- Conflicto entre puertos ----
    if (we_a === 1'b1 && we_b === 1'b1 && addr_a === addr_b) begin
      n_errors++;
      $warning("[MEM_AGENT] %0t ns: escritura simultanea de A y B en mem[0x%05H]", $time, addr_a);
    end
  end

  // ---------------------------------------------------------------------------
  // Backdoor: acceso directo al contenido sin consumir ciclos
  // ---------------------------------------------------------------------------
  task automatic backdoor_write(input int unsigned word_addr, input logic [MEM_WIDTH-1:0] data);
    mem[word_addr] = data;
  endtask

  function automatic logic [MEM_WIDTH-1:0] backdoor_read(input int unsigned word_addr);
    return mem[word_addr];
  endfunction

  // Volcado de un rango de memoria (direcciones de palabra)
  task automatic dump(input int unsigned first, input int unsigned last);
    for (int unsigned i = first; i <= last; i++)
      $display("  [MEM_AGENT] mem[0x%05H] (byte 0x%05H) = 0x%08H", i, i << 2, mem[i]);
  endtask

  // Resumen de la actividad del agente
  task automatic report();
    $display("  [MEM_AGENT] Resumen: %0d fetch, %0d lecturas B, %0d escrituras A, %0d escrituras B, %0d avisos de protocolo",
             n_fetch, n_read_b, n_write_a, n_write_b, n_errors);
  endtask

endmodule
