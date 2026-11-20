`timescale 1ns / 1ps

// =============================================================================
// tb_fetchMemory
// Comprobacion de la etapa de busqueda: fetch.vhd conectado al puerto A de la
// memoria de doble puerto (memory.vhd).
//
//   FASE 1: con el fetch en reset, el testbench escribe 20 instrucciones RV32I
//           a partir de PC_RESET (0x10000, palabra 0x4000) por el puerto A y
//           comprueba que han quedado guardadas. Durante el reset PC = PC_RESET
//   FASE 2: se suelta el reset y el contador de programa cuenta 20 veces
//           (PC = 0x10004, 0x10008, ... 0x10050, sin saltos: pc_load = 0)
//           En cada cuenta se comprueba:
//             - pc_out = PC anterior + 4
//             - rdata_a = instruccion de la posicion que se acaba de leer
//   FASE 3: salto y parada
//             - pc_load = 1 : el PC carga pc_target (salto a PC_RESET + 0x08)
//             - pc_hold = 1 : el PC no avanza y la memoria vuelve a leer la
//                             misma instruccion (parada de los loads)
//             - pc_hold = 0 : el PC vuelve a contar
//   FASE 4: resumen (cuentas, instrucciones leidas, errores)
//
// Temporizacion (lectura sincrona de la memoria):
//   flanco n: pc_out pasa de PC_RESET+4(n-1) a PC_RESET+4n y rdata_a <= instruccion n-1
//   -> la instruccion leida va un ciclo por detras del PC.
//
// La memoria se direcciona por palabras y el fetch genera direcciones de byte,
// por lo que el testbench conecta pc_out[ADDR_WIDTH-1:2] al puerto A.
// =============================================================================
module tb_fetchMemory;

  localparam int MEM_WIDTH  = 32;
  localparam int NUM_BYTES  = 4;
  localparam int ADDR_WIDTH = 17;

  localparam int N_INSTR  = 20;        // instrucciones cargadas = cuentas del PC
  localparam int PC_RESET = 'h10000;   // PC tras el reset (generic del fetch)
  localparam int BASE     = PC_RESET / 4;  // palabra de memoria de la primera instruccion

  // Opcodes RV32I (7 bits)
  localparam logic [6:0] OP_R      = 7'b0110011;
  localparam logic [6:0] OP_I      = 7'b0010011;
  localparam logic [6:0] OP_LOAD   = 7'b0000011;
  localparam logic [6:0] OP_STORE  = 7'b0100011;
  localparam logic [6:0] OP_BRANCH = 7'b1100011;
  localparam logic [6:0] OP_JAL    = 7'b1101111;
  localparam logic [6:0] OP_LUI    = 7'b0110111;
  localparam logic [6:0] OP_AUIPC  = 7'b0010111;

  // Reloj y reset
  logic clk;
  logic nRst;

  // Puertos del fetch
  logic                  pc_load;
  logic                  pc_hold;
  logic [ADDR_WIDTH-1:0] pc_target;
  logic [ADDR_WIDTH-1:0] pc_out;

  // Puertos de la memoria (puerto A multiplexado: testbench durante la carga, fetch despues)
  logic                  load_en;
  logic [ADDR_WIDTH-1:0] tb_addr;
  logic [MEM_WIDTH-1:0]  tb_wdata;
  logic                  tb_we;

  logic [ADDR_WIDTH-1:0] mem_addr_a, mem_addr_b;
  logic [MEM_WIDTH-1:0]  mem_wdata_a, mem_wdata_b;
  logic [MEM_WIDTH-1:0]  rdata_a, rdata_b;
  logic                  mem_we_a, mem_we_b;
  logic [NUM_BYTES-1:0]  mem_be_a, mem_be_b;

  assign mem_addr_a  = load_en ? tb_addr  : {2'b00, pc_out[ADDR_WIDTH-1:2]};
  assign mem_wdata_a = load_en ? tb_wdata : '0;
  assign mem_we_a    = load_en ? tb_we    : 1'b0;
  assign mem_be_a    = load_en ? 4'b1111  : 4'b0000;
  // El puerto B no se usa en este test
  assign mem_addr_b  = '0;
  assign mem_wdata_b = '0;
  assign mem_we_b    = 1'b0;
  assign mem_be_b    = '0;

  // Programa y contadores
  logic [31:0] prog [0:N_INSTR-1];
  int errors;
  int pc_counts;
  int instr_ok;

  // ---------------------------------------------------------------------------
  // DUT: fetch + memoria
  // ---------------------------------------------------------------------------
  fetch #(
    .ADDR_WIDTH(ADDR_WIDTH),
    .PC_RESET  (PC_RESET)
  ) dut (
    .clk      (clk),
    .nRst     (nRst),
    .pc_load  (pc_load),
    .pc_target(pc_target),
    .pc_hold  (pc_hold),
    .pc_out   (pc_out)
  );

  memory #(
    .MEM_WIDTH (MEM_WIDTH),
    .NUM_BYTES (NUM_BYTES),
    .ADDR_WIDTH(ADDR_WIDTH)
  ) u_mem (
    .clk    (clk),
    .addr_a (mem_addr_a),
    .wdata_a(mem_wdata_a),
    .rdata_a(rdata_a),
    .we_a   (mem_we_a),
    .be_a   (mem_be_a),
    .addr_b (mem_addr_b),
    .wdata_b(mem_wdata_b),
    .rdata_b(rdata_b),
    .we_b   (mem_we_b),
    .be_b   (mem_be_b)
  );

  // Reloj de 100 MHz
  initial clk = 1'b0;
  always #5 clk = ~clk;

  // ---------------------------------------------------------------------------
  // Codificadores de instrucciones RV32I
  // ---------------------------------------------------------------------------
  function automatic logic [31:0] enc_r(input logic [6:0] f7, input logic [4:0] rs2,
                                        input logic [4:0] rs1, input logic [2:0] f3,
                                        input logic [4:0] rd);
    return {f7, rs2, rs1, f3, rd, OP_R};
  endfunction

  function automatic logic [31:0] enc_i(input logic [31:0] imm, input logic [4:0] rs1,
                                        input logic [2:0] f3, input logic [4:0] rd,
                                        input logic [6:0] op);
    return {imm[11:0], rs1, f3, rd, op};
  endfunction

  function automatic logic [31:0] enc_s(input logic [31:0] imm, input logic [4:0] rs2,
                                        input logic [4:0] rs1, input logic [2:0] f3);
    return {imm[11:5], rs2, rs1, f3, imm[4:0], OP_STORE};
  endfunction

  function automatic logic [31:0] enc_b(input logic [31:0] imm, input logic [4:0] rs2,
                                        input logic [4:0] rs1, input logic [2:0] f3);
    return {imm[12], imm[10:5], rs2, rs1, f3, imm[4:1], imm[11], OP_BRANCH};
  endfunction

  function automatic logic [31:0] enc_u(input logic [19:0] imm20, input logic [4:0] rd,
                                        input logic [6:0] op);
    return {imm20, rd, op};
  endfunction

  function automatic logic [31:0] enc_j(input logic [31:0] imm, input logic [4:0] rd);
    return {imm[20], imm[10:1], imm[11], imm[19:12], rd, OP_JAL};
  endfunction

  // ---------------------------------------------------------------------------
  // Programa de 20 instrucciones (todas distintas para detectar desplazamientos)
  // Solo se leen, no se ejecutan: los saltos no se toman (pc_load = 0)
  // ---------------------------------------------------------------------------
  task automatic build_program();
    prog[0]  = enc_i(32'd10,   5'd0, 3'b000, 5'd1, OP_I);          // 0x00 addi  x1, x0, 10
    prog[1]  = enc_i(32'd20,   5'd0, 3'b000, 5'd2, OP_I);          // 0x04 addi  x2, x0, 20
    prog[2]  = enc_r(7'h00, 5'd2, 5'd1, 3'b000, 5'd3);             // 0x08 add   x3, x1, x2
    prog[3]  = enc_r(7'h20, 5'd1, 5'd2, 3'b000, 5'd4);             // 0x0C sub   x4, x2, x1
    prog[4]  = enc_r(7'h00, 5'd2, 5'd1, 3'b111, 5'd5);             // 0x10 and   x5, x1, x2
    prog[5]  = enc_r(7'h00, 5'd2, 5'd1, 3'b110, 5'd6);             // 0x14 or    x6, x1, x2
    prog[6]  = enc_r(7'h00, 5'd2, 5'd1, 3'b100, 5'd7);             // 0x18 xor   x7, x1, x2
    prog[7]  = enc_r(7'h00, 5'd2, 5'd1, 3'b010, 5'd8);             // 0x1C slt   x8, x1, x2
    prog[8]  = enc_i(32'd3,    5'd1, 3'b001, 5'd9,  OP_I);         // 0x20 slli  x9, x1, 3
    prog[9]  = enc_i(32'd1,    5'd2, 3'b101, 5'd10, OP_I);         // 0x24 srli  x10, x2, 1
    prog[10] = enc_i(32'h0FF,  5'd1, 3'b111, 5'd11, OP_I);         // 0x28 andi  x11, x1, 0xFF
    prog[11] = enc_i(32'h0F0,  5'd2, 3'b110, 5'd12, OP_I);         // 0x2C ori   x12, x2, 0xF0
    prog[12] = enc_u(20'h12345, 5'd13, OP_LUI);                    // 0x30 lui   x13, 0x12345
    prog[13] = enc_u(20'h00001, 5'd14, OP_AUIPC);                  // 0x34 auipc x14, 0x1
    prog[14] = enc_s(32'h400,  5'd3, 5'd0, 3'b010);                // 0x38 sw    x3, 0x400(x0)
    prog[15] = enc_s(32'h404,  5'd4, 5'd0, 3'b001);                // 0x3C sh    x4, 0x404(x0)
    prog[16] = enc_i(32'h400,  5'd0, 3'b010, 5'd15, OP_LOAD);      // 0x40 lw    x15, 0x400(x0)
    prog[17] = enc_i(32'h404,  5'd0, 3'b100, 5'd16, OP_LOAD);      // 0x44 lbu   x16, 0x404(x0)
    prog[18] = enc_b(32'd8,    5'd2, 5'd1, 3'b000);                // 0x48 beq   x1, x2, +8
    prog[19] = enc_j(32'd16,   5'd17);                             // 0x4C jal   x17, +16
  endtask

  // ---------------------------------------------------------------------------
  // FASE 1: carga de las 20 instrucciones por el puerto A (fetch en reset)
  // ---------------------------------------------------------------------------
  task automatic load_memory();
    load_en = 1'b1;
    for (int i = 0; i < N_INSTR; i++) begin
      @(negedge clk);
      tb_addr  = BASE + i;
      tb_wdata = prog[i];
      tb_we    = 1'b1;
    end
    @(negedge clk);
    tb_we    = 1'b0;
    tb_addr  = '0;
    tb_wdata = '0;

    // Comprobacion del contenido escrito (lectura directa del array de la memoria)
    for (int i = 0; i < N_INSTR; i++) begin
      if (u_mem.mem[BASE + i] !== prog[i]) begin
        $error("CARGA: mem[0x%04H] = 0x%08H, esperado 0x%08H", BASE + i, u_mem.mem[BASE + i], prog[i]);
        errors++;
      end
    end
    $display("  Cargadas %0d instrucciones en mem[0x%04H..0x%04H] (PC 0x%05H..0x%05H)",
             N_INSTR, BASE, BASE + N_INSTR - 1, PC_RESET, PC_RESET + 4*(N_INSTR-1));
    load_en = 1'b0;
  endtask

  // ---------------------------------------------------------------------------
  // Comprueba una cuenta del PC y la instruccion leida en ese flanco
  // (se muestrea en el flanco de bajada, con las senales ya estables)
  // ---------------------------------------------------------------------------
  task automatic check_count(input int n);
    logic [ADDR_WIDTH-1:0] exp_pc;
    logic [31:0]           exp_instr;
    exp_pc    = PC_RESET + 4 * n;
    exp_instr = prog[n-1];

    if (pc_out !== exp_pc) begin
      $error("CUENTA %0d: pc_out = 0x%05H, esperado 0x%05H", n, pc_out, exp_pc);
      errors++;
    end
    if (rdata_a !== exp_instr) begin
      $error("CUENTA %0d: rdata_a = 0x%08H, esperado mem[0x%04H] = 0x%08H",
             n, rdata_a, BASE + n - 1, exp_instr);
      errors++;
    end else begin
      instr_ok++;
    end
    $display("  Cuenta %2d: PC = 0x%05H | instruccion leida mem[0x%04H] (PC 0x%05H) = 0x%08H %s",
             n, pc_out, BASE + n - 1, PC_RESET + 4*(n-1), rdata_a,
             (pc_out === exp_pc && rdata_a === exp_instr) ? "OK" : "FALLO");
  endtask

  // ---------------------------------------------------------------------------
  // Un ciclo de la FASE 3: aplica pc_load / pc_hold, da un flanco y comprueba
  // el PC y la instruccion leida (la de la direccion que habia antes del flanco)
  // ---------------------------------------------------------------------------
  task automatic check_step(
    input string                 name,
    input logic                  load,
    input logic                  hold,
    input logic [ADDR_WIDTH-1:0] target,
    input logic [ADDR_WIDTH-1:0] exp_pc,
    input logic [31:0]           exp_instr
  );
    pc_load   = load;
    pc_hold   = hold;
    pc_target = target;
    @(posedge clk);
    @(negedge clk);
    if (pc_out !== exp_pc) begin
      $error("%s: pc_out = 0x%05H, esperado 0x%05H", name, pc_out, exp_pc);
      errors++;
    end
    if (rdata_a !== exp_instr) begin
      $error("%s: rdata_a = 0x%08H, esperado 0x%08H", name, rdata_a, exp_instr);
      errors++;
    end
    $display("  %-28s: PC = 0x%05H | instruccion leida = 0x%08H %s", name, pc_out, rdata_a,
             (pc_out === exp_pc && rdata_a === exp_instr) ? "OK" : "FALLO");
  endtask

  // ---------------------------------------------------------------------------
  // Secuencia principal
  // ---------------------------------------------------------------------------
  initial begin
    $display("=== INICIO SIMULACION FETCH + MEMORIA (%0d cuentas del PC) ===", N_INSTR);
    errors    = 0;
    pc_counts = 0;
    instr_ok  = 0;
    nRst      = 1'b0;
    pc_load   = 1'b0;
    pc_hold   = 1'b0;
    pc_target = '0;
    load_en   = 1'b1;
    tb_addr   = '0;
    tb_wdata  = '0;
    tb_we     = 1'b0;

    build_program();

    // ===================== FASE 1 =====================
    $display("--- FASE 1: carga de la memoria (fetch en reset) ---");
    load_memory();

    // Durante el reset el PC debe valer PC_RESET
    if (pc_out !== PC_RESET[ADDR_WIDTH-1:0]) begin
      $error("RESET: pc_out = 0x%05H, esperado 0x%05H", pc_out, PC_RESET);
      errors++;
    end else
      $display("  Reset: PC = 0x%05H OK", pc_out);

    // ===================== FASE 2 =====================
    $display("--- FASE 2: %0d cuentas del contador de programa ---", N_INSTR);
    @(negedge clk);
    nRst = 1'b1;
    for (int n = 1; n <= N_INSTR; n++) begin
      @(posedge clk);
      pc_counts++;
      @(negedge clk);
      check_count(n);
    end

    // ===================== FASE 3 =====================
    // Al acabar la FASE 2 el PC vale PC_RESET + 0x50 (fuera del programa: mem = 0)
    $display("--- FASE 3: salto (pc_load) y parada (pc_hold) ---");
    check_step("Salto a PC_RESET + 0x08",   1'b1, 1'b0, PC_RESET + 'h08, PC_RESET + 'h08, 32'h0);
    check_step("Parada 1 (pc_hold)",        1'b0, 1'b1, '0,              PC_RESET + 'h08, prog[2]);
    check_step("Parada 2 (pc_hold)",        1'b0, 1'b1, '0,              PC_RESET + 'h08, prog[2]);
    check_step("Fin de parada",             1'b0, 1'b0, '0,              PC_RESET + 'h0C, prog[2]);
    check_step("Cuenta normal",             1'b0, 1'b0, '0,              PC_RESET + 'h10, prog[3]);
    check_step("pc_load prioritario a hold",1'b1, 1'b1, PC_RESET + 'h40, PC_RESET + 'h40, prog[4]);
    check_step("Cuenta tras el salto",      1'b0, 1'b0, '0,              PC_RESET + 'h44, prog[16]);

    // ===================== FASE 4 =====================
    $display("--- FASE 4: resumen ---");
    $display("  Cuentas del PC       : %0d (PC final = 0x%05H)", pc_counts, pc_out);
    $display("  Instrucciones leidas : %0d / %0d correctas", instr_ok, N_INSTR);
    if (errors == 0)
      $display("=== SIMULACION COMPLETADA SIN ERRORES ===");
    else
      $display("=== SIMULACION COMPLETADA CON %0d ERRORES ===", errors);
    $finish;
  end

endmodule
