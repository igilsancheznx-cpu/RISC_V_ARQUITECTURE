`timescale 1ns / 1ps

module tb_alu;

    // 1. Parámetros
    localparam int ALU_LENGTH = 32;
    localparam int ALU_DECOD  = 5;

    // 2. Señales para conectar a la ALU
    logic [ALU_LENGTH-1:0] op1;
    logic [ALU_LENGTH-1:0] op2;
    logic [ALU_DECOD-1:0]  aloup;
    
    logic                  eq;
    logic                  lt;
    logic                  ltu;
    logic [ALU_LENGTH-1:0] aluOut;

    // 3. Instanciación del módulo VHDL ALU (DUT)
    alu #(
        .ALU_LENGTH(ALU_LENGTH),
        .ALU_DECOD(ALU_DECOD)
    ) dut (
        .op1(op1),
        .op2(op2),
        .aloup(aloup),
        .eq(eq),
        .lt(lt),
        .ltu(ltu),
        .aluOut(aluOut)
    );

    // 4. Bloque de prueba secuencial
    initial begin
        // Monitor para imprimir en la consola del simulador ante cada cambio
        $display("--------------------------------------------------------------------------------------------------");
        $display(" TIME |  OP  |     OP1     |     OP2     |    ALUOUT   | EQ | LT | LTU ");
        $display("--------------------------------------------------------------------------------------------------");
        $monitor("%5t | %5b | %d | %d | %d |  %b |  %b |  %b", 
                 $time, aloup, $signed(op1), $signed(op2), $signed(aluOut), eq, lt, ltu);

        // --- CASO 1: Suma Signed (0000) ---
        op1 = 32'd15;      op2 = 32'd10;      aloup = 5'b00000; #10;
        
        // --- CASO 2: Resta Signed (1000) ---
        op1 = 32'd20;      op2 = 32'd5;       aloup = 5'b01000; #10;
        
        // --- CASO 3: Shift Left Logical (0001) ---
        op1 = 32'h0000000F; op2 = 32'd2;       aloup = 5 me00001; #10; // 15 << 2 = 60
        
        // --- CASO 4: Set Less Than Signed (0010) ---
        op1 = -32'd10;     op2 = 32'd5;       aloup = 5'b00010; #10; // -10 < 5 -> SLT = 1
        
        // --- CASO 5: Set Less Than Unsigned (0011) ---
        op1 = -32'd10;     op2 = 32'd5;       aloup = 5'b00011; #10; // Unsigned: 0xFFFFFFF6 < 5 -> SLTU = 0
        
        // --- CASO 6: XOR (0100) ---
        op1 = 32'hFF00FF00; op2 = 32'hFFFF0000; aloup = 5'b00100; #10;
        
        // --- CASO 7: Shift Right Logical (0101) ---
        op1 = 32'hF0000000; op2 = 32'd4;       aloup = 5'b00101; #10; // Rellena con 0s a la izquierda
        
        // --- CASO 8: Shift Right Arithmetic (1101) ---
        op1 = 32'hF0000000; op2 = 32'd4;       aloup = 5'b01101; #10; // Extiende el bit de signo '1'
        
        // --- CASO 9: OR (0110) ---
        op1 = 32'h12340000; op2 = 32'h00005678; aloup = 5'b00110; #10;
        
        // --- CASO 10: AND (0111) ---
        op1 = 32'hFFFF1234; op2 = 32'hFFFF0000; aloup = 5'b00111; #10;

        // --- CASO 11: Comprobación de Banderas (EQ / LT / LTU) ---
        op1 = 32'd42;      op2 = 32'd42;      aloup = 5'b00000; #10; // Comprobar EQ = 1

        $display("--------------------------------------------------------------------------------------------------");
        $display("Simulación completada con éxito.");
        $finish;
    end

endmodule