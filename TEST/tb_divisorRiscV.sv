`timescale 1ns / 1ps

// =============================================================================
// tb_divisorRiscV
// Validacion de la unidad de division RISCV_Div (HDL/divisor.vhd) integrada en
// el procesador, con sus tres arquitecturas: SERIE, PARALELA y SEGMENTADA.
//
// El testbench instancia tres sistemas completos (core + memory.vhd), uno por
// arquitectura (generic DIV_ARCH del core), y ejecuta en los tres el mismo
// programa: divisiones (div, divu, rem, remu) con otras instrucciones
// intercaladas.
//
// PROGRAMA (autocomprobante, como en tb_riscV_validacion)
//   1. Correccion: 18 pares de operandos x 4 instrucciones, con signos
//      combinados, division por cero, desbordamiento (-2^31 / -1), divisor
//      0xFFFFFFFF y 0x80000000, dividendo menor que el divisor, rd = rs1 y
//      rs1 = rs2. Cada resultado se compara con el valor esperado (CHK): un
//      fallo incrementa x29 y todos se acumulan en la firma x31.
//   2. Bloques de temporizacion (A..F): secuencias cuya duracion en ciclos
//      depende de la arquitectura y se compara con la esperada:
//        A  4 divisiones independientes seguidas
//        B  div + rem con los mismos operandos (reutilizacion del resultado)
//        C  division seguida de una instruccion que usa su resultado (RAW)
//        D  divisiones con 5 instrucciones independientes intercaladas
//        E  division que usa el resultado de la division anterior
//        F  instruccion que sobrescribe el destino de una division (WAW)
//   3. Riesgos con otras instrucciones: resultado usado por store, branch y
//      add con dos divisiones en vuelo; load seguido de division; division
//      que atraviesa un salto; division con rd = x0.
//
// ASPECTOS VALIDADOS (enunciado 7.5)
//   - Serie: no se puede iniciar otra division hasta terminar la actual y el
//     procesador esta parado (PC fijo, ninguna instruccion entra al decoder)
//     mientras Ready = 0. Cada division ocupa N + 1 ciclos (1 + 1 si reutiliza
//     el resultado anterior).
//   - Paralela: Ready y Done siempre activos, divisiones consecutivas sin
//     perdida de ciclos y sin riesgos de datos.
//   - Segmentada: Ready siempre activo, varias divisiones en vuelo a la vez,
//     divisiones e instrucciones consecutivas sin perdida de ciclos, y
//     retencion de las instrucciones con riesgo de datos (RAW y WAW).
//
// COMPROBACION AUTOMATICA (ver el resumen final)
//   Para cada arquitectura: firma y contador de errores del programa,
//   32 registros finales y secuencia de instrucciones ejecutadas frente al
//   modelo de referencia (ISS RV32IM), ciclos de cada bloque frente a los
//   esperados y monitores de protocolo de RISCV_Div.
//
// Compilacion (desde la raiz del proyecto, QuestaSim / ModelSim):
//   vlib work
//   vcom -2008 HDL/alu.vhd
//   vcom -2002 HDL/divisor.vhd HDL/regFile.vhd HDL/decoder.vhd HDL/fetch.vhd HDL/DecExe.vhd HDL/core.vhd HDL/memory.vhd
//   vlog -sv TEST/tb_divisorRiscV.sv
//   vsim -c work.tb_divisorRiscV -do "run -all"
// =============================================================================

// -----------------------------------------------------------------------------
// div_sistema: core + memory.vhd con una arquitectura de division y monitores
// -----------------------------------------------------------------------------
module div_sistema #(
  parameter int    DIV_ARCH = 0,
  parameter string NOMBRE   = "serie"
)(
  input logic        clk,
  input logic        nRst,
  // Carga del programa por el puerto A (core en reset)
  input logic        load_en,
  input logic [16:0] tb_addr,
  input logic [31:0] tb_wdata,
  input logic        tb_we
);
  localparam int ADDR_WIDTH = 17;
  localparam int NDIV       = 32;
  localparam logic [31:0] HALT = 32'h0000_006F;

  logic [31:0]           rdata_a, rdata_b, wdata_a, wdata_b;
  logic [ADDR_WIDTH-1:0] addr_a,  addr_b;
  logic                  we_a,    we_b;
  logic [3:0]            be_a,    be_b;
  logic [ADDR_WIDTH-1:0] mem_addr_a, mem_addr_b;
  logic [31:0]           mem_wdata_a, mem_wdata_b;
  logic                  mem_we_a, mem_we_b;
  logic [3:0]            mem_be_a, mem_be_b;

  assign mem_addr_a  = load_en ? tb_addr  : {2'b00, addr_a[ADDR_WIDTH-1:2]};
  assign mem_wdata_a = load_en ? tb_wdata : wdata_a;
  assign mem_we_a    = load_en ? tb_we    : we_a;
  assign mem_be_a    = load_en ? 4'b1111  : be_a;
  assign mem_addr_b  = load_en ? '0       : {2'b00, addr_b[ADDR_WIDTH-1:2]};
  assign mem_wdata_b = load_en ? '0       : wdata_b;
  assign mem_we_b    = load_en ? 1'b0     : we_b;
  assign mem_be_b    = load_en ? '0       : be_b;

  core #(.ADDR_WIDTH(ADDR_WIDTH), .DIV_ARCH(DIV_ARCH)) dut (
    .clk(clk), .nRst(nRst),
    .rdata_a(rdata_a), .addr_a(addr_a), .wdata_a(wdata_a), .we_a(we_a), .be_a(be_a),
    .rdata_b(rdata_b), .addr_b(addr_b), .wdata_b(wdata_b), .we_b(we_b), .be_b(be_b));

  memory #(.ADDR_WIDTH(ADDR_WIDTH)) u_mem (
    .clk(clk),
    .addr_a(mem_addr_a), .wdata_a(mem_wdata_a), .rdata_a(rdata_a), .we_a(mem_we_a), .be_a(mem_be_a),
    .addr_b(mem_addr_b), .wdata_b(mem_wdata_b), .rdata_b(rdata_b), .we_b(mem_we_b), .be_b(mem_be_b));

  function automatic logic [31:0] get_reg(input int r);
    return dut.U1.U0.regs[r];
  endfunction

  function automatic logic [31:0] get_mem(input int idx);
    return u_mem.mem[idx];
  endfunction

  // ---------------------------------------------------------------------------
  // Monitores (se muestrean a mitad de ciclo)
  // ---------------------------------------------------------------------------
  int          cyc;                 // 0 = primer ciclo tras soltar el reset
  int          entry_cyc [int];     // PC -> ciclo en que la instruccion entra al decoder
  logic [31:0] retired [$];         // PCs de las instrucciones ejecutadas, en orden
  bit          halted;
  int          halt_cyc;
  int          n_hold;              // ciclos con una instruccion retenida
  int          n_div;               // divisiones aceptadas por RISCV_Div (Valid y Ready)
  int          n_reuse;             // divisiones que reutilizan el resultado anterior (serie)
  int          n_busy;              // ciclos con Ready = 0
  int          max_inflight;        // maximo de divisiones en vuelo (segmentada)
  int          lat_min, lat_max;    // ciclos desde que se acepta una division hasta que se escribe
  int          n_err;               // errores de protocolo detectados por los monitores
  int          issue_q [$];
  logic [ADDR_WIDTH-1:0] busy_pc;

  always @(posedge clk or negedge nRst)
    if (!nRst) cyc <= 0;
    else       cyc <= cyc + 1;

  task automatic clear();
    entry_cyc.delete();
    retired.delete();
    issue_q.delete();
    halted       = 0;
    halt_cyc     = 0;
    n_hold       = 0;
    n_div        = 0;
    n_reuse      = 0;
    n_busy       = 0;
    max_inflight = 0;
    lat_min      = 1 << 30;
    lat_max      = 0;
    n_err        = 0;
  endtask

  function automatic int entry_of(input logic [31:0] pc);
    return entry_cyc.exists(pc) ? entry_cyc[pc] : -1;
  endfunction

  task automatic mon_error(input string msg);
    n_err++;
    $error("[%s] ciclo %0d: %s", NOMBRE, cyc, msg);
  endtask

  always @(negedge clk) begin
    if (nRst === 1'b1 && !halted) begin
      logic [31:0] pc;
      bit          valid, entry, retire;
      int          lat, inflight;
      pc     = {15'b0, dut.pc_instr};
      valid  = (dut.instr_valid === 1'b1) || (dut.replay === 1'b1);
      entry  = (dut.instr_valid === 1'b1) && (dut.replay !== 1'b1);
      retire = valid && (dut.dec_hold !== 1'b1);

      if (entry && !entry_cyc.exists(pc)) entry_cyc[pc] = cyc;
      if (dut.dec_hold === 1'b1) n_hold++;

      // --- protocolo de RISCV_Div ---
      if (dut.U1.d_ready === 1'b0) begin
        n_busy++;
        if (DIV_ARCH != 0) mon_error("Ready debe estar siempre activo");
        // serie: procesador parado (PC fijo y ninguna instruccion nueva) mientras divide
        if (entry)             mon_error($sformatf("entra la instruccion de PC=0x%05H durante una division", pc));
        if (dut.addr_a !== busy_pc)
          mon_error($sformatf("el PC avanza (0x%05H -> 0x%05H) durante una division", busy_pc, dut.addr_a));
        if (dut.U1.div_valid === 1'b1) mon_error("se solicita una division con Ready = 0");
      end
      busy_pc = dut.addr_a;
      if (DIV_ARCH == 1 && dut.U1.d_done !== 1'b1) mon_error("paralela: Done debe estar siempre activo");

      if (dut.U1.div_valid === 1'b1 && dut.U1.d_ready === 1'b1) begin
        n_div++;
        issue_q.push_back(cyc);
      end
      if (dut.U1.wdiv_en === 1'b1) begin
        if (issue_q.size() == 0) mon_error("se escribe un resultado de division sin division pendiente");
        else begin
          lat = cyc - issue_q.pop_front();
          if (lat < lat_min) lat_min = lat;
          if (lat > lat_max) lat_max = lat;
          if (DIV_ARCH == 0 && lat == 1) n_reuse++;
        end
      end
      inflight = $countones(dut.U1.tag_v);
      if (inflight > max_inflight) max_inflight = inflight;

      if (retire) begin
        retired.push_back(pc);
        if (dut.instr_dec === HALT) begin
          halted   = 1;
          halt_cyc = cyc;
        end
      end
    end
  end
endmodule

// -----------------------------------------------------------------------------
// tb_divisorRiscV
// -----------------------------------------------------------------------------
module tb_divisorRiscV;

  localparam int ADDR_WIDTH = 17;
  localparam int DEPTH      = 2**(ADDR_WIDTH-2);
  localparam int N          = 32;      // iteraciones de la division
  localparam int MAX_CYCLES = 10000;

  localparam logic [31:0] PROG_BASE = 32'h0001_0000;
  localparam logic [31:0] DATA_BASE = 32'h0001_F000;
  localparam logic [31:0] SIG_ADDR  = DATA_BASE + 32'h7F0;
  localparam logic [31:0] ERR_ADDR  = DATA_BASE + 32'h7F4;
  localparam logic [31:0] HALT      = 32'h0000_006F;

  // Firma esperada del programa (x31). Si se modifica el programa, el testbench
  // avisa (error PLAN) con el valor calculado por el modelo de referencia.
  localparam logic [31:0] EXP_SIG = 32'h9F09_034A;

  localparam string ARCH_NAME [0:2] = '{"SERIE", "PARALELA", "SEGMENTADA"};

  localparam logic [6:0] OP_R      = 7'b0110011;
  localparam logic [6:0] OP_I      = 7'b0010011;
  localparam logic [6:0] OP_LOAD   = 7'b0000011;
  localparam logic [6:0] OP_STORE  = 7'b0100011;
  localparam logic [6:0] OP_BRANCH = 7'b1100011;
  localparam logic [6:0] OP_JAL    = 7'b1101111;
  localparam logic [6:0] OP_JALR   = 7'b1100111;
  localparam logic [6:0] OP_LUI    = 7'b0110111;
  localparam logic [6:0] OP_AUIPC  = 7'b0010111;

  localparam int R_BASE = 10, R_EXP = 28, R_ERR = 29, R_TMP = 30, R_SIG = 31;

  // ---------------------------------------------------------------------------
  // Reloj, reset, bus de carga y los tres sistemas
  // ---------------------------------------------------------------------------
  logic        clk, nRst, load_en, tb_we;
  logic [16:0] tb_addr;
  logic [31:0] tb_wdata;

  initial clk = 1'b0;
  always #5 clk = ~clk;

  div_sistema #(.DIV_ARCH(0), .NOMBRE("SERIE"))      u_serie      (.*);
  div_sistema #(.DIV_ARCH(1), .NOMBRE("PARALELA"))   u_paralela   (.*);
  div_sistema #(.DIV_ARCH(2), .NOMBRE("SEGMENTADA")) u_segmentada (.*);

  // Acceso a cada sistema por indice (0 serie, 1 paralela, 2 segmentada)
  function automatic logic [31:0] reg_of(input int a, input int r);
    case (a) 0: return u_serie.get_reg(r); 1: return u_paralela.get_reg(r); default: return u_segmentada.get_reg(r); endcase
  endfunction
  function automatic logic [31:0] mem_of(input int a, input int idx);
    case (a) 0: return u_serie.get_mem(idx); 1: return u_paralela.get_mem(idx); default: return u_segmentada.get_mem(idx); endcase
  endfunction
  function automatic int entry_of(input int a, input logic [31:0] pc);
    case (a) 0: return u_serie.entry_of(pc); 1: return u_paralela.entry_of(pc); default: return u_segmentada.entry_of(pc); endcase
  endfunction
  function automatic bit halted_of(input int a);
    case (a) 0: return u_serie.halted; 1: return u_paralela.halted; default: return u_segmentada.halted; endcase
  endfunction

  // ===========================================================================
  // ENSAMBLADOR
  // ===========================================================================
  logic [31:0] prog [$];

  function automatic int here();
    return prog.size();
  endfunction
  function automatic logic [31:0] pc_of(input int idx);
    return PROG_BASE + 32'(4 * idx);
  endfunction
  task automatic emit(input logic [31:0] w);
    prog.push_back(w);
  endtask

  function automatic logic [31:0] enc_r(input logic [6:0] f7, input int rs2, input int rs1, input logic [2:0] f3, input int rd);
    return {f7, 5'(rs2), 5'(rs1), f3, 5'(rd), OP_R};
  endfunction
  function automatic logic [31:0] enc_i(input logic [31:0] imm, input int rs1, input logic [2:0] f3, input int rd, input logic [6:0] op);
    return {imm[11:0], 5'(rs1), f3, 5'(rd), op};
  endfunction
  function automatic logic [31:0] enc_s(input logic [31:0] imm, input int rs2, input int rs1, input logic [2:0] f3);
    return {imm[11:5], 5'(rs2), 5'(rs1), f3, imm[4:0], OP_STORE};
  endfunction
  function automatic logic [31:0] enc_b(input logic [31:0] imm, input int rs2, input int rs1, input logic [2:0] f3);
    return {imm[12], imm[10:5], 5'(rs2), 5'(rs1), f3, imm[4:1], imm[11], OP_BRANCH};
  endfunction
  function automatic logic [31:0] enc_j(input logic [31:0] imm, input int rd);
    return {imm[20], imm[10:1], imm[11], imm[19:12], 5'(rd), OP_JAL};
  endfunction

  task automatic ADD (input int rd, rs1, rs2); emit(enc_r(7'h00, rs2, rs1, 3'b000, rd)); endtask
  task automatic XOR (input int rd, rs1, rs2); emit(enc_r(7'h00, rs2, rs1, 3'b100, rd)); endtask
  task automatic OR  (input int rd, rs1, rs2); emit(enc_r(7'h00, rs2, rs1, 3'b110, rd)); endtask
  // Extension M: division
  task automatic DIV (input int rd, rs1, rs2); emit(enc_r(7'h01, rs2, rs1, 3'b100, rd)); endtask
  task automatic DIVU(input int rd, rs1, rs2); emit(enc_r(7'h01, rs2, rs1, 3'b101, rd)); endtask
  task automatic REM (input int rd, rs1, rs2); emit(enc_r(7'h01, rs2, rs1, 3'b110, rd)); endtask
  task automatic REMU(input int rd, rs1, rs2); emit(enc_r(7'h01, rs2, rs1, 3'b111, rd)); endtask
  task automatic ADDI(input int rd, rs1, input logic [31:0] imm); emit(enc_i(imm, rs1, 3'b000, rd, OP_I)); endtask
  task automatic SLLI(input int rd, rs1, sh); emit(enc_i(32'(sh & 31), rs1, 3'b001, rd, OP_I)); endtask
  task automatic SRLI(input int rd, rs1, sh); emit(enc_i(32'(sh & 31), rs1, 3'b101, rd, OP_I)); endtask
  task automatic LW  (input int rd, rs1, input logic [31:0] off); emit(enc_i(off, rs1, 3'b010, rd, OP_LOAD)); endtask
  task automatic SW  (input int rs2, rs1, input logic [31:0] off); emit(enc_s(off, rs2, rs1, 3'b010)); endtask
  task automatic LUI (input int rd, input logic [19:0] imm20); emit({imm20, 5'(rd), OP_LUI}); endtask
  task automatic JAL (input int rd, input logic [31:0] off); emit(enc_j(off, rd)); endtask
  task automatic BEQ (input int rs1, rs2, input logic [31:0] off); emit(enc_b(off, rs2, rs1, 3'b000)); endtask

  task automatic LI(input int rd, input logic [31:0] v);
    logic [31:0] lo, hi;
    lo = {{20{v[11]}}, v[11:0]};
    hi = v - lo;
    if (hi != 0) begin
      LUI(rd, hi[31:12]);
      if (lo != 0) ADDI(rd, rd, lo);
    end else
      ADDI(rd, 0, lo);
  endtask

  task automatic ACC(input int r);          // x31 = rotl(x31 ^ r, 1)
    XOR (R_SIG, R_SIG, r);
    SLLI(R_TMP, R_SIG, 1);
    SRLI(R_SIG, R_SIG, 31);
    OR  (R_SIG, R_SIG, R_TMP);
  endtask

  int n_chk;
  task automatic CHK(input int r, input logic [31:0] expected);
    LI (R_EXP, expected);
    BEQ(r, R_EXP, 8);
    ADDI(R_ERR, R_ERR, 1);
    ACC(r);
    n_chk++;
  endtask

  task automatic TRAP();
    ADDI(R_ERR, R_ERR, 1);
  endtask

  task automatic END_PROGRAM();
    LUI(R_BASE, DATA_BASE[31:12]);
    SW (R_SIG, R_BASE, SIG_ADDR - DATA_BASE);
    SW (R_ERR, R_BASE, ERR_ADDR - DATA_BASE);
    JAL(0, 0);
  endtask

  // Prueba de un par de operandos con las cuatro instrucciones de division.
  // div y rem se ejecutan seguidas con los mismos operandos (reutilizacion en
  // la serie) y cada resultado se usa enseguida (riesgo RAW en la segmentada)
  task automatic T4(input logic [31:0] a, b, q, r, qu, ru);
    LI(5, a);
    LI(6, b);
    DIV (7, 5, 6);
    REM (8, 5, 6);
    DIVU(11, 5, 6);
    REMU(12, 5, 6);
    CHK(7, q);
    CHK(8, r);
    CHK(11, qu);
    CHK(12, ru);
  endtask

  // Bloques de temporizacion: [inicio, fin) y ciclos esperados por arquitectura
  typedef struct {
    string name;
    int    i0, i1;
    int    exp [0:2];
  } blk_t;
  blk_t blocks [$];

  task automatic BLOCK(input string name, input int i0, input int serie, paralela, segmentada);
    blk_t b;
    b.name = name;
    b.i0   = i0;
    b.i1   = here();
    b.exp  = '{serie, paralela, segmentada};
    blocks.push_back(b);
  endtask

  // ===========================================================================
  // PROGRAMA
  // ===========================================================================
  task automatic build_program();
    int a;
    prog.delete();
    blocks.delete();
    n_chk = 0;
    LUI(R_BASE, DATA_BASE[31:12]);

    // ---- 1. Correccion: T4(dividendo, divisor, div, rem, divu, remu) ----
    T4(32'd100,       32'd7,         32'd14,        32'd2,         32'd14,        32'd2);
    T4(-32'd100,      32'd7,         -32'd14,       -32'd2,        32'h2492_4916, 32'd2);
    T4(32'd100,       -32'd7,        -32'd14,       32'd2,         32'd0,         32'd100);
    T4(-32'd100,      -32'd7,        32'd14,        -32'd2,        32'd0,         -32'd100);
    T4(-32'd100,      32'd0,         32'hFFFF_FFFF, -32'd100,      32'hFFFF_FFFF, -32'd100);  // / 0
    T4(32'd0,         32'd0,         32'hFFFF_FFFF, 32'd0,         32'hFFFF_FFFF, 32'd0);     // 0 / 0
    T4(32'h8000_0000, -32'd1,        32'h8000_0000, 32'd0,         32'd0,         32'h8000_0000); // desbordamiento
    T4(-32'd1,        -32'd1,        32'd1,         32'd0,         32'd1,         32'd0);
    T4(32'hFFFF_FFFE, 32'hFFFF_FFFF, 32'd2,         32'd0,         32'd0,         32'hFFFF_FFFE);
    T4(32'd3,         32'd10,        32'd0,         32'd3,         32'd0,         32'd3);     // dividendo < divisor
    T4(32'h7FFF_FFFF, 32'd2,         32'h3FFF_FFFF, 32'd1,         32'h3FFF_FFFF, 32'd1);
    T4(32'h8000_0000, 32'd2,         32'hC000_0000, 32'd0,         32'h4000_0000, 32'd0);
    T4(32'd0,         32'd5,         32'd0,         32'd0,         32'd0,         32'd0);
    T4(32'h1234_5678, 32'h1234,      32'h0001_0004, 32'h0000_0DA8, 32'h0001_0004, 32'h0000_0DA8);
    T4(-32'd1,        32'd1,         32'hFFFF_FFFF, 32'd0,         32'hFFFF_FFFF, 32'd0);
    T4(32'd123456789, 32'hFFFF_FFFF, 32'hF8A4_32EB, 32'd0,         32'd0,         32'h075B_CD15); // divisor -1 / 0xFFFFFFFF
    T4(32'hFFFF_FFFF, 32'h8000_0000, 32'd0,         -32'd1,        32'd1,         32'h7FFF_FFFF);
    T4(-32'd1000,     32'd13,        -32'd76,       -32'd12,       32'h13B1_3AC6, 32'd10);
    // rd = rs1 y rs1 = rs2
    LI(5, 32'd1000);  LI(6, 32'd7);
    DIV(5, 5, 6);     CHK(5, 32'd142);
    DIV(7, 6, 6);     CHK(7, 32'd1);

    // ---- 2. Bloques de temporizacion (operandos: x1=-1000, x2=7, x3=1000, x4=13) ----
    LI(1, -32'd1000);
    LI(2, 32'd7);
    LI(3, 32'd1000);
    LI(4, 32'd13);

    a = here();
    DIV (11, 1, 2);       // -142
    DIVU(12, 3, 4);       // 76
    REM (13, 1, 2);       // -6
    REMU(14, 3, 4);       // 12
    BLOCK("A  4 divisiones independientes", a, 4*(N+1), 4, 4);
    CHK(11, -32'd142);  CHK(12, 32'd76);  CHK(13, -32'd6);  CHK(14, 32'd12);

    a = here();
    DIV (15, 1, 2);       // -142
    REM (16, 1, 2);       // -6 (mismos operandos)
    BLOCK("B  div + rem mismos operandos", a, (N+1) + 2, 2, 2);
    CHK(15, -32'd142);  CHK(16, -32'd6);

    a = here();
    DIV (17, 3, 4);       // 76
    ADD (18, 17, 17);     // usa el resultado: 152
    BLOCK("C  division + uso inmediato (RAW)", a, N+2, 2, N+2);
    CHK(18, 32'd152);   CHK(17, 32'd76);

    a = here();
    DIV (19, 1, 2);       // -142
    ADDI(20, 0, 1);
    ADDI(21, 0, 2);
    ADDI(22, 0, 3);
    ADDI(23, 0, 4);
    ADDI(24, 0, 5);
    DIVU(25, 3, 4);       // 76
    BLOCK("D  divisiones + 5 instr. intercaladas", a, 2*(N+1) + 5, 7, 7);
    CHK(19, -32'd142);  CHK(25, 32'd76);
    ADD(20, 20, 24);    CHK(20, 32'd6);

    a = here();
    DIV (26, 1, 2);       // -142
    DIV (27, 26, 2);      // -142 / 7 = -20 (depende de la anterior)
    BLOCK("E  division dependiente de division", a, 2*(N+1), 2, N+2);
    CHK(27, -32'd20);   CHK(26, -32'd142);

    a = here();
    DIV (9, 3, 2);        // 142
    ADDI(9, 0, 5);        // sobrescribe el destino pendiente: debe quedar 5
    BLOCK("F  escritura del destino pendiente (WAW)", a, N+2, 2, N+2);
    CHK(9, 32'd5);

    // ---- 3. Riesgos con otras instrucciones ----
    DIV (20, 3, 2);       SW(20, R_BASE, 64);   LW(21, R_BASE, 64);   CHK(21, 32'd142);  // div -> store
    LW  (5, R_BASE, 64);  DIV(6, 5, 2);         CHK(6, 32'd20);                          // load -> div
    ADDI(23, 0, 76);      DIVU(22, 3, 4);       BEQ(22, 23, 8);       TRAP();            // div -> branch
    DIV (23, 1, 2);       DIVU(24, 3, 4);       ADD(25, 23, 24);      CHK(25, -32'd66);  // 2 en vuelo
    DIV (27, 3, 2);       JAL(0, 8);            TRAP();               ADD(26, 27, 0);    // atraviesa un salto
    CHK(26, 32'd142);
    DIV (0, 3, 2);        ADD(8, 0, 0);         CHK(8, 32'd0);                           // rd = x0

    END_PROGRAM();
  endtask

  // ===========================================================================
  // MODELO DE REFERENCIA (ISS RV32IM, solo las instrucciones del programa)
  // ===========================================================================
  logic [31:0] m_x [0:31];
  logic [31:0] m_mem [0:DEPTH-1];

  function automatic int word_idx(input logic [31:0] byte_addr);
    return int'(byte_addr[ADDR_WIDTH-1:2]);
  endfunction

  function automatic logic [31:0] m_div(input logic [2:0] f3, input logic [31:0] a, b);
    case (f3)
      3'b100:  if (b == 0) return 32'hFFFF_FFFF;                                    // div
               else if (a == 32'h8000_0000 && b == 32'hFFFF_FFFF) return 32'h8000_0000;
               else return 32'($signed(a) / $signed(b));
      3'b101:  if (b == 0) return 32'hFFFF_FFFF;                                    // divu
               else return a / b;
      3'b110:  if (b == 0) return a;                                                // rem
               else if (a == 32'h8000_0000 && b == 32'hFFFF_FFFF) return 32'd0;
               else return 32'($signed(a) % $signed(b));
      default: if (b == 0) return a;                                                // remu
               else return a % b;
    endcase
  endfunction

  // Ejecuta la instruccion de pc; devuelve el PC siguiente
  function automatic logic [31:0] iss_step(input logic [31:0] pc);
    logic [31:0] i, a, b, imm_i, imm_s, imm_b, imm_j, res, addr;
    logic [4:0]  rd;
    bit          wr;
    i     = m_mem[word_idx(pc)];
    a     = (i[19:15] == 0) ? 32'h0 : m_x[i[19:15]];
    b     = (i[24:20] == 0) ? 32'h0 : m_x[i[24:20]];
    rd    = i[11:7];
    imm_i = {{20{i[31]}}, i[31:20]};
    imm_s = {{20{i[31]}}, i[31:25], i[11:7]};
    imm_b = {{19{i[31]}}, i[31], i[7], i[30:25], i[11:8], 1'b0};
    imm_j = {{11{i[31]}}, i[31], i[19:12], i[20], i[30:21], 1'b0};
    wr    = 0;
    iss_step = pc + 4;
    case (i[6:0])
      OP_LUI:   begin res = {i[31:12], 12'b0}; wr = 1; end
      OP_JAL:   begin res = pc + 4; wr = 1; iss_step = pc + imm_j; end
      OP_BRANCH: if (i[14:12] == 3'b000 && a == b) iss_step = pc + imm_b;
      OP_LOAD:  begin res = m_mem[word_idx(a + imm_i)]; wr = 1; end              // solo lw
      OP_STORE: begin addr = a + imm_s; m_mem[word_idx(addr)] = b; end           // solo sw
      OP_I: begin
        wr = 1;
        case (i[14:12])
          3'b000:  res = a + imm_i;
          3'b001:  res = a << i[24:20];
          default: res = a >> i[24:20];                                          // srli
        endcase
      end
      OP_R: begin
        wr = 1;
        if (i[25]) res = m_div(i[14:12], a, b);
        else case (i[14:12])
          3'b000:  res = a + b;
          3'b100:  res = a ^ b;
          default: res = a | b;
        endcase
      end
      default: ;
    endcase
    if (wr && rd != 0) m_x[rd] = res;
  endfunction

  logic [31:0] g_x [0:31];
  logic [31:0] g_pcs [$];
  logic [31:0] g_sig, g_err;
  int          g_div;

  task automatic golden_run();
    logic [31:0] pc, i;
    foreach (m_x[k]) m_x[k] = 32'h0;
    for (int k = 0; k < DEPTH; k++) m_mem[k] = u_serie.get_mem(k);
    pc = PROG_BASE;
    g_pcs.delete();
    g_div = 0;
    forever begin
      i = m_mem[word_idx(pc)];
      g_pcs.push_back(pc);
      if (i[6:0] == OP_R && i[25]) g_div++;
      if (i === HALT || g_pcs.size() > MAX_CYCLES) break;
      pc = iss_step(pc);
    end
    foreach (m_x[k]) g_x[k] = m_x[k];
    g_sig = m_mem[word_idx(SIG_ADDR)];
    g_err = m_mem[word_idx(ERR_ADDR)];
  endtask

  // ===========================================================================
  // COMPROBACIONES
  // ===========================================================================
  int total_errors;
  int arch_errors [0:2];
  int blk_meas [0:2][$];

  task automatic fail(input int a, input string msg);
    total_errors++;
    if (a >= 0) arch_errors[a]++;
    $error("[%s] %s", (a >= 0) ? ARCH_NAME[a] : "PLAN", msg);
  endtask

  task automatic load_program();
    load_en = 1'b1;
    tb_we   = 1'b1;
    foreach (prog[k]) begin
      @(negedge clk);
      tb_addr  = 17'(word_idx(PROG_BASE) + k);
      tb_wdata = prog[k];
    end
    @(negedge clk);
    tb_we   = 1'b0;
    load_en = 1'b0;
  endtask

  task automatic check_arch(input int a);
    logic [31:0] v;
    int          n_ret, t0, t1;
    logic [31:0] ret [$];

    case (a)
      0: ret = u_serie.retired;
      1: ret = u_paralela.retired;
      default: ret = u_segmentada.retired;
    endcase

    if (!halted_of(a)) fail(a, $sformatf("no llega al halt en %0d ciclos", MAX_CYCLES));

    // Resultado del programa (comprobacion principal)
    v = mem_of(a, word_idx(ERR_ADDR));
    assert (v === 32'h0) else fail(a, $sformatf("%0d pruebas CHK fallidas (mem[ERR_ADDR])", v));
    v = mem_of(a, word_idx(SIG_ADDR));
    assert (v === EXP_SIG) else fail(a, $sformatf("firma esp 0x%08H, obt 0x%08H", EXP_SIG, v));

    // Estado final y secuencia de instrucciones frente al modelo
    for (int r = 0; r < 32; r++) begin
      v = reg_of(a, r);
      if (v !== g_x[r]) fail(a, $sformatf("x%0d esp 0x%08H, obt 0x%08H", r, g_x[r], v));
    end
    n_ret = ret.size();
    if (n_ret != g_pcs.size())
      fail(a, $sformatf("ejecuta %0d instrucciones, el modelo %0d", n_ret, g_pcs.size()));
    for (int k = 0; k < n_ret && k < g_pcs.size(); k++)
      if (ret[k] !== g_pcs[k]) begin
        fail(a, $sformatf("instruccion %0d: ejecuta PC=0x%05H, el modelo PC=0x%05H", k, ret[k], g_pcs[k]));
        break;
      end

    // Ciclos de cada bloque
    blk_meas[a].delete();
    foreach (blocks[k]) begin
      t0 = entry_of(a, pc_of(blocks[k].i0));
      t1 = entry_of(a, pc_of(blocks[k].i1));
      blk_meas[a].push_back(t1 - t0);
      if (t0 < 0 || t1 < 0 || (t1 - t0) != blocks[k].exp[a])
        fail(a, $sformatf("bloque %s: %0d ciclos, esperados %0d", blocks[k].name, t1 - t0, blocks[k].exp[a]));
    end

    // Aspectos especificos de cada arquitectura
    case (a)
      0: begin
        assert (u_serie.n_err == 0)    else fail(a, "errores de protocolo (ver mensajes del monitor)");
        assert (u_serie.n_reuse > 0)   else fail(a, "ninguna division reutiliza el resultado anterior");
        assert (u_serie.lat_max == N)  else fail(a, $sformatf("latencia maxima de una division %0d, esperada %0d", u_serie.lat_max, N));
        assert (u_serie.n_busy > 0)    else fail(a, "Ready nunca se desactiva");
      end
      1: begin
        assert (u_paralela.n_err == 0)   else fail(a, "errores de protocolo (ver mensajes del monitor)");
        assert (u_paralela.lat_max == 0) else fail(a, $sformatf("el resultado tarda %0d ciclos (debe ser 0)", u_paralela.lat_max));
        assert (u_paralela.n_hold == 0)  else fail(a, $sformatf("%0d ciclos de retencion (no debe haber riesgos)", u_paralela.n_hold));
      end
      default: begin
        assert (u_segmentada.n_err == 0)          else fail(a, "errores de protocolo (ver mensajes del monitor)");
        assert (u_segmentada.lat_min == N && u_segmentada.lat_max == N)
          else fail(a, $sformatf("latencia %0d..%0d, esperada %0d", u_segmentada.lat_min, u_segmentada.lat_max, N));
        assert (u_segmentada.max_inflight >= 4)   else fail(a, $sformatf("solo %0d divisiones en vuelo a la vez", u_segmentada.max_inflight));
        assert (u_segmentada.n_hold > 0)          else fail(a, "no se detecta ningun riesgo de datos");
      end
    endcase
  endtask

  // ===========================================================================
  // SECUENCIA PRINCIPAL
  // ===========================================================================
  initial begin
    int cyc;
    $display("=== VALIDACION DE LA UNIDAD DE DIVISION (serie, paralela, segmentada) ===");
    nRst         = 1'b0;
    load_en      = 1'b1;
    tb_we        = 1'b0;
    tb_addr      = '0;
    tb_wdata     = '0;
    total_errors = 0;
    arch_errors  = '{0, 0, 0};

    build_program();
    $display("  Programa: %0d instrucciones, %0d pruebas CHK, %0d bloques de temporizacion",
             prog.size(), n_chk, blocks.size());
    load_program();                      // en los tres sistemas a la vez
    golden_run();
    $display("  Modelo de referencia: %0d instrucciones ejecutadas (%0d divisiones), firma 0x%08H",
             g_pcs.size(), g_div, g_sig);
    assert (g_err === 32'h0)
      else fail(-1, $sformatf("el modelo de referencia falla %0d pruebas CHK: revisar los valores esperados", g_err));
    assert (g_sig === EXP_SIG)
      else fail(-1, $sformatf("firma del modelo 0x%08H distinta de EXP_SIG 0x%08H", g_sig, EXP_SIG));

    u_serie.clear();
    u_paralela.clear();
    u_segmentada.clear();
    repeat (3) @(posedge clk);
    @(negedge clk);
    nRst = 1'b1;

    cyc = 0;
    while (!(halted_of(0) && halted_of(1) && halted_of(2)) && cyc < MAX_CYCLES) begin
      @(posedge clk);
      cyc++;
    end
    repeat (4) @(posedge clk);
    #1;

    for (int a = 0; a < 3; a++) check_arch(a);

    // ---- Informe ----
    $display("\n--- CICLOS POR BLOQUE (medidos / esperados) ---");
    $display("  %-42s %13s %13s %13s", "bloque", "SERIE", "PARALELA", "SEGMENTADA");
    foreach (blocks[k])
      $display("  %-42s %6d / %-4d %6d / %-4d %6d / %-4d", blocks[k].name,
               blk_meas[0][k], blocks[k].exp[0], blk_meas[1][k], blocks[k].exp[1], blk_meas[2][k], blocks[k].exp[2]);

    $display("\n--- RESUMEN POR ARQUITECTURA ---");
    $display("  %-11s %7s %7s %5s %10s %9s %10s %8s %10s %12s %9s", "arquitect.", "instr", "ciclos", "CPI",
             "divisiones", "reutiliz.", "latencia", "en vuelo", "retencion", "firma", "resultado");
    $display("  %-11s %7d %7d %5.2f %10d %9d %6d..%-3d %8d %10d   0x%08H %9s", "SERIE",
             u_serie.retired.size(), u_serie.halt_cyc + 1, real'(u_serie.halt_cyc + 1) / real'(u_serie.retired.size()),
             u_serie.n_div, u_serie.n_reuse, u_serie.lat_min, u_serie.lat_max, u_serie.max_inflight, u_serie.n_hold,
             mem_of(0, word_idx(SIG_ADDR)), arch_errors[0] == 0 ? "CORRECTO" : "FALLO");
    $display("  %-11s %7d %7d %5.2f %10d %9s %6d..%-3d %8d %10d   0x%08H %9s", "PARALELA",
             u_paralela.retired.size(), u_paralela.halt_cyc + 1, real'(u_paralela.halt_cyc + 1) / real'(u_paralela.retired.size()),
             u_paralela.n_div, "-", u_paralela.lat_min, u_paralela.lat_max, u_paralela.max_inflight, u_paralela.n_hold,
             mem_of(1, word_idx(SIG_ADDR)), arch_errors[1] == 0 ? "CORRECTO" : "FALLO");
    $display("  %-11s %7d %7d %5.2f %10d %9s %6d..%-3d %8d %10d   0x%08H %9s", "SEGMENTADA",
             u_segmentada.retired.size(), u_segmentada.halt_cyc + 1, real'(u_segmentada.halt_cyc + 1) / real'(u_segmentada.retired.size()),
             u_segmentada.n_div, "-", u_segmentada.lat_min, u_segmentada.lat_max, u_segmentada.max_inflight, u_segmentada.n_hold,
             mem_of(2, word_idx(SIG_ADDR)), arch_errors[2] == 0 ? "CORRECTO" : "FALLO");
    $display("  latencia: ciclos desde que RISCV_Div acepta la division hasta que se escribe el resultado");
    $display("  retencion: ciclos con una instruccion parada en el decoder (division serie o riesgo de datos)");

    if (total_errors == 0)
      $display("\n=== VALIDACION DE LA DIVISION COMPLETADA: SIN ERRORES ===");
    else
      $display("\n=== VALIDACION DE LA DIVISION COMPLETADA: %0d ERRORES ===", total_errors);
    $finish;
  end

endmodule
