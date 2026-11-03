`timescale 1ns / 1ps

module tb_top;

    // Parámetros
    localparam int MEM_WIDTH  = 32;
    localparam int NUM_BYTES  = 4;
    localparam int ADDR_REG   = 5;
    localparam int ADDR_WIDTH = 17;
    localparam int CLK_PERIOD = 10; // Período de reloj (100 MHz)

    // Señales de prueba
    logic clk;
    logic nRst;

    // Instanciación del Top Level (DUT)
    top #(
        .MEM_WIDTH(MEM_WIDTH),
        .NUM_BYTES(NUM_BYTES),
        .ADDR_REG(ADDR_REG),
        .ADDR_WIDTH(ADDR_WIDTH)
    ) dut (
        .clk(clk),
        .nRst(nRst)
    );

    // Generador de Reloj
    initial begin
        clk = 0;
        forever #(CLK_PERIOD / 2) clk = ~clk;
    end

    // PROCESO PRINCIPAL DE SIMULACIÓN Y ASERCIONES

    initial begin

        $display("  INICIANDO TESTBENCH DE RISC-V CON ASERCIONES (AUTOMATED TEST)  ");

        // 1. Fase de Reset
        nRst = 0;
        #(CLK_PERIOD * 2);
        nRst = 1;
        $display("[%0t ns] Reset liberado. Ejecutando programa en memoria...", $time);

        // 2. Ejecución ciclo a ciclo con aserciones inmediatas
        
        // Ciclo 1: Fetch pos 0 (addi x1, x0, 10)
        @(posedge clk);
        
        // Ciclo 2: Exec pos 0 / Fetch pos 1 (addi x2, x0, 20)
        @(posedge clk);
        #1; // Margen para estabilización de señales tras el flanco
        assert (dut.u_core.U1.U0.regs[1] == 32'd10)
            else $error("[%0t ns] ASSERTION FAILED: Error en 'addi x1, x0, 10'. Se esperaba 10 y se obtuvo %0d", 
                        $time, $signed(dut.u_core.U1.U0.regs[1]));

        // Ciclo 3: Exec pos 1 / Fetch pos 2 (add x3, x1, x2)
        @(posedge clk);
        #1;
        assert (dut.u_core.U1.U0.regs[2] == 32'd20)
            else $error("[%0t ns] ASSERTION FAILED: Error en 'addi x2, x0, 20'. Se esperaba 20 y se obtuvo %0d", 
                        $time, $signed(dut.u_core.U1.U0.regs[2]));

        // Ciclo 4: Exec pos 2 / Fetch pos 3 (sw x3, 0(x0))
        @(posedge clk);
        #1;
        assert (dut.u_core.U1.U0.regs[3] == 32'd30)
            else $error("[%0t ns] ASSERTION FAILED: Error en 'add x3, x1, x2'. Se esperaba 30 (10+20) y se obtuvo %0d", 
                        $time, $signed(dut.u_core.U1.U0.regs[3]));

        // Ciclo 5: Escritura en Memoria realizada (sw x3, 0(x0))
        @(posedge clk);
        #1;
        assert (dut.u_memory.mem[0] == 32'd30)
            else $error("[%0t ns] ASSERTION FAILED: Error en 'sw x3, 0(x0)'. La posición de memoria 0 tiene 0x%0h y se esperaba 30", 
                        $time, dut.u_memory.mem[0]);

        // 3. Verificación Final Resumida
        $display("RESUMEN DE PRUEBAS DE ASERCIÓN");
        $display("  -> Registro x1 (10): %0d", $signed(dut.u_core.U1.U0.regs[1]));
        $display("  -> Registro x2 (20): %0d", $signed(dut.u_core.U1.U0.regs[2]));
        $display("  -> Registro x3 (30): %0d", $signed(dut.u_core.U1.U0.regs[3]));
        $display("  -> Memoria [0] (30): %0d", $signed(dut.u_memory.mem[0]));
        $display("[INFO] Si no se han mostrado mensajes de ASSERTION FAILED, todas las pruebas han PASADO con éxito.");

        $finish;
    end

    // ASERCIÓN CONCURRENTE (Propiedad continua durante toda la simulación)

    // Garantiza que el registro x0 NUNCA cambie su valor y se mantenga en 0
    assert_x0_always_zero: assert property (
        @(posedge clk) disable iff (!nRst)
        (dut.u_core.U1.U0.regs[0] == 32'b0)
    ) else $fatal(1, "[CRITICAL ERROR] El registro x0 de RISC-V ha cambiado de valor a 0x%0h!", dut.u_core.U1.U0.regs[0]);

endmodule