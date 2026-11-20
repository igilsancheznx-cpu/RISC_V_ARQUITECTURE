library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity DecExe is
    generic (
        REG_WIDTH  : integer := 32; 
        ADDR_REG   : integer := 5;
        ADDR_WIDTH : integer := 17;
        ALU_DECOD  : integer := 4;
        -- Arquitectura de la unidad de division RISCV_Div (divisor.vhd):
        --   0 = serie, 1 = paralela, 2 = segmentada
        DIV_ARCH   : integer := 0
    );
    port (
        clk       : in   std_logic;
        nRst      : in   std_logic;
        rdata_a   : in   std_logic_vector(REG_WIDTH-1 downto 0);
        rdata_b   : in   std_logic_vector(REG_WIDTH-1 downto 0);
        addr_b    : out  std_logic_vector(ADDR_WIDTH-1 downto 0);
        wdata_b   : out  std_logic_vector(REG_WIDTH-1 downto 0);
        we_b      : out  std_logic;
        be_b      : out  std_logic_vector(ALU_DECOD-1 downto 0);
        -- PC Signals bifurcation
        pc_out    : in   std_logic_vector(ADDR_WIDTH - 1 downto 0);
        pc_load   : out  std_logic;
        pc_target : out  std_logic_vector(ADDR_WIDTH - 1 downto 0);
        -- Load en ejecucion: el core debe parar un ciclo (fetch mantiene el PC y
        -- la instruccion siguiente se anula) mientras el dato llega de memoria
        ld_stall  : out  std_logic;
        -- Depuracion (solo para testbench; se pueden dejar sin conectar)
        dbg_rdata1 : out std_logic_vector(REG_WIDTH-1 downto 0);
        dbg_rdata2 : out std_logic_vector(REG_WIDTH-1 downto 0);
        dbg_wAlu   : out std_logic_vector(REG_WIDTH-1 downto 0);
        dbg_we     : out std_logic;
        dbg_wl     : out std_logic;
        dbg_we_mem : out std_logic;
        dbg_be_mem : out std_logic_vector(ALU_DECOD-1 downto 0);
        dbg_regs   : out std_logic_vector(2**ADDR_REG * REG_WIDTH - 1 downto 0)
    );
end entity DecExe;

architecture rtl of DecExe is
    -- Read port 1
    signal rs1 : std_logic_vector(ADDR_REG - 1 downto 0);
    signal rdata1 : std_logic_vector(REG_WIDTH - 1 downto 0);
    -- Read port 2
    signal rs2 : std_logic_vector(ADDR_REG - 1 downto 0);
    signal rdata2 : std_logic_vector(REG_WIDTH - 1 downto 0);
    -- Write port
    signal wAlu   : std_logic_vector(REG_WIDTH - 1 downto 0); --ALU
    signal we     : std_logic; -- Write enable
    signal wl     : std_logic; -- Write back from memory
    signal be_l   : std_logic_vector(3 downto 0); -- Byte enable for write back
    signal sign_l : std_logic; -- Load con signo (lb, lh)
    signal rd     : std_logic_vector(ADDR_REG - 1 downto 0); -- Dirección del registro destino
    -- Signal decoder instruction
    signal imm    : std_logic_vector(REG_WIDTH - 1 downto 0);
    signal aloup  : std_logic_vector(ALU_DECOD - 1 downto 0);
    signal eq     : std_logic;
    signal lt     : std_logic;
    signal ltu    : std_logic;
    -- Multiplexor para ALU
    signal ena_imm : std_logic;  -- Multiplexor de la ALU para seleccionar entre rs2 o imm
    signal auxOp2 : std_logic_vector(REG_WIDTH - 1 downto 0);  -- Señal auxiliar para multiplexor de la ALU
    -- Signal auxiliar for memory
    signal aux_addr_b  : std_logic_vector(ADDR_WIDTH-1 downto 0);  
    signal aux_wdata_b : std_logic_vector(REG_WIDTH-1 downto 0); 
    signal aux_we_b    : std_logic;  
    signal aux_be_b    : std_logic_vector(ALU_DECOD-1 downto 0);
    -- Signal auxiliar for branches
    signal dec_pc_load : std_logic;  -- pc_load del decoder (jal, jalr y branch sin condición)
    signal br_taken    : std_logic;  -- Condición del branch cumplida (según funct3 y flags de la ALU)
    -- Dato a escribir en rd (ALU, PC + 4 o PC + imm)
    signal pc_ext      : unsigned(REG_WIDTH - 1 downto 0);       -- PC de la instruccion extendido a 32 bits
    signal wreg_data   : std_logic_vector(REG_WIDTH - 1 downto 0);
    -- Acceso a memoria de datos
    signal byte_off    : natural range 0 to 3;                      -- desplazamiento de byte (addr(1:0))
    -- Write-back de los loads (un ciclo despues de presentar la direccion)
    signal ld_wl_r     : std_logic;                                 -- hay que escribir el dato del load
    signal ld_rd_r     : std_logic_vector(ADDR_REG - 1 downto 0);   -- registro destino del load
    signal ld_be_r     : std_logic_vector(3 downto 0);              -- bytes validos (lb/lh/lw)
    signal ld_sign_r   : std_logic;                                 -- extension de signo (lb/lh)
    signal ld_off_r    : natural range 0 to 3;                      -- desplazamiento de byte del load
    signal ld_data     : std_logic_vector(REG_WIDTH - 1 downto 0);  -- dato de memoria alineado al byte 0
    -- Puerto de escritura del banco de registros
    signal rf_waddr    : std_logic_vector(ADDR_REG - 1 downto 0);
    signal rf_we       : std_logic;
    signal wreg_rf     : std_logic_vector(REG_WIDTH - 1 downto 0);  -- dato que se escribe (ALU / PC o division)

    -- =========================================================================
    -- Unidad de division (div, divu, rem, remu)
    -- =========================================================================
    signal is_div      : std_logic;                                 -- instruccion de division en el decoder
    signal div_op      : std_logic_vector(1 downto 0);              -- 00 div, 01 divu, 10 rem, 11 remu
    signal div_signed  : std_logic;                                 -- div / rem (con signo)
    signal div_a_neg   : std_logic;                                 -- dividendo negativo (solo con signo)
    signal div_b_neg   : std_logic;                                 -- divisor negativo (solo con signo)
    signal div_b_zero  : std_logic;                                 -- division por cero
    signal div_a_abs   : std_logic_vector(REG_WIDTH - 1 downto 0);  -- |dividendo| hacia RISCV_Div
    signal div_b_abs   : std_logic_vector(REG_WIDTH - 1 downto 0);  -- |divisor| hacia RISCV_Div
    signal div_qneg    : std_logic;                                 -- hay que cambiar el signo del cociente
    signal div_rneg    : std_logic;                                 -- hay que cambiar el signo del resto
    signal div_valid   : std_logic;
    signal div_ready   : std_logic;
    signal div_done    : std_logic;
    signal div_q       : std_logic_vector(REG_WIDTH - 1 downto 0);  -- cociente (sin signo)
    signal div_r       : std_logic_vector(REG_WIDTH - 1 downto 0);  -- resto (sin signo)
    -- Division en curso (serie / segmentada): datos de la instruccion guardados
    signal div_pend    : std_logic;
    signal div_rd_r    : std_logic_vector(ADDR_REG - 1 downto 0);
    signal div_rem_r   : std_logic;
    signal div_qneg_r  : std_logic;
    signal div_rneg_r  : std_logic;
    -- Seleccion segun la arquitectura (paralela: datos actuales; resto: guardados)
    signal div_rem_s   : std_logic;
    signal div_qneg_s  : std_logic;
    signal div_rneg_s  : std_logic;
    signal div_base    : std_logic_vector(REG_WIDTH - 1 downto 0);
    signal div_neg     : std_logic;
    signal div_res     : std_logic_vector(REG_WIDTH - 1 downto 0);  -- resultado final con signo
    signal div_wr      : std_logic;                                 -- escribir el resultado en el banco
    signal div_waddr   : std_logic_vector(ADDR_REG - 1 downto 0);
    signal div_stall   : std_logic;                                 -- parada del procesador hasta Done

begin

    -- =========================================================================
    -- MEMORIA DE DATOS (puerto B)
    -- La memoria es sincrona: la direccion se presenta en el ciclo de ejecucion
    -- (combinacional) y la memoria la muestrea en el flanco que cierra el ciclo.
    --   store : la memoria escribe en ese mismo flanco
    --   load  : el dato esta en rdata_b durante el ciclo siguiente
    -- Los accesos de byte / media palabra se colocan en su carril segun addr(1:0)
    -- =========================================================================
    byte_off <= to_integer(unsigned(wAlu(1 downto 0))) when not is_x(wAlu(1 downto 0)) else 0;

    addr_b  <= wAlu(ADDR_WIDTH - 1 downto 0);
    wdata_b <= std_logic_vector(shift_left(unsigned(rdata2), 8 * byte_off));
    we_b    <= aux_we_b;
    be_b    <= std_logic_vector(shift_left(unsigned(aux_be_b), byte_off));

    -- Control del write-back de los loads: se registra en el flanco en el que la
    -- memoria muestrea la direccion y se escribe en rd en el flanco siguiente
    process(clk, nRst)
    begin
      if nRst = '0' then
        ld_wl_r   <= '0';
        ld_rd_r   <= (others => '0');
        ld_be_r   <= (others => '0');
        ld_sign_r <= '0';
        ld_off_r  <= 0;
      elsif rising_edge(clk) then
        ld_wl_r   <= wl;
        ld_rd_r   <= rd;
        ld_be_r   <= be_l;
        ld_sign_r <= sign_l;
        ld_off_r  <= byte_off;
      end if;
    end process;

    -- Dato leido alineado al byte 0 (lb/lh/lbu/lhu en direcciones no multiplo de 4)
    ld_data <= std_logic_vector(shift_right(unsigned(rdata_b), 8 * ld_off_r));

    -- Parada: el load ocupa un ciclo mas; la division (serie / segmentada) para
    -- el procesador hasta que la unidad de division activa Done
    ld_stall <= wl or div_stall;

    -- Puerto de escritura del banco de registros:
    --   ciclo de write-back de un load -> rd del load con el dato de memoria
    --   ciclo en que termina una division -> rd de la division con su resultado
    --   resto                          -> rd de la instruccion en ejecucion (ALU / PC)
    -- Durante el write-back del load o de la division el decoder tiene una
    -- burbuja (NOP), cuya escritura se inhibe
    rf_waddr <= ld_rd_r   when ld_wl_r = '1' else
                div_waddr when div_wr  = '1' else
                rd;
    rf_we    <= (we or div_wr) and not ld_wl_r;
    wreg_rf  <= div_res when div_wr = '1' else wreg_data;

    -- =========================================================================
    -- UNIDAD DE DIVISION
    -- RISCV_Div divide numeros sin signo. En div / rem se le pasan los valores
    -- absolutos y despues se corrige el signo:
    --   cociente : negativo si los signos de los operandos son distintos
    --              (division por cero: el cociente es -1 = todo unos, sin cambiar el signo)
    --   resto    : con el signo del dividendo
    -- =========================================================================
    div_signed <= not div_op(0);
    div_a_neg  <= div_signed and rdata1(REG_WIDTH - 1);
    div_b_neg  <= div_signed and rdata2(REG_WIDTH - 1);
    div_b_zero <= '1' when unsigned(rdata2) = 0 else '0';
    div_a_abs  <= std_logic_vector(unsigned(not rdata1) + 1) when div_a_neg = '1' else rdata1;
    div_b_abs  <= std_logic_vector(unsigned(not rdata2) + 1) when div_b_neg = '1' else rdata2;
    div_qneg   <= (div_a_neg xor div_b_neg) and not div_b_zero;
    div_rneg   <= div_a_neg;

    -- Nueva division: la instruccion esta en el decoder y no hay otra en curso
    div_valid  <= is_div and not div_pend;

    -- Division en curso (serie y segmentada): se guardan rd y la correccion de signo
    -- en el flanco en que la unidad acepta la division (Valid y Ready). Done solo se
    -- mira a partir del ciclo siguiente (div_pend = '1'): en el mismo ciclo del Valid
    -- todavia puede tener el valor de la division anterior
    process(clk, nRst)
    begin
      if nRst = '0' then
        div_pend   <= '0';
        div_rd_r   <= (others => '0');
        div_rem_r  <= '0';
        div_qneg_r <= '0';
        div_rneg_r <= '0';
      elsif rising_edge(clk) then
        if DIV_ARCH /= 1 then
          if div_valid = '1' and div_ready = '1' then
            div_pend   <= '1';
            div_rd_r   <= rd;
            div_rem_r  <= div_op(1);
            div_qneg_r <= div_qneg;
            div_rneg_r <= div_rneg;
          elsif div_pend = '1' and div_done = '1' then
            div_pend   <= '0';            -- resultado escrito en este flanco
          end if;
        end if;
      end if;
    end process;

    -- Paralela: el resultado sale en el mismo ciclo y se escribe como una operacion
    -- de la ALU. Serie / segmentada: se escribe en el ciclo en que se activa Done
    div_rem_s  <= div_op(1) when DIV_ARCH = 1 else div_rem_r;
    div_qneg_s <= div_qneg  when DIV_ARCH = 1 else div_qneg_r;
    div_rneg_s <= div_rneg  when DIV_ARCH = 1 else div_rneg_r;
    div_base   <= div_r      when div_rem_s = '1' else div_q;
    div_neg    <= div_rneg_s when div_rem_s = '1' else div_qneg_s;
    div_res    <= std_logic_vector(unsigned(not div_base) + 1) when div_neg = '1' else div_base;

    div_wr     <= is_div when DIV_ARCH = 1 else (div_pend and div_done);
    div_waddr  <= rd     when DIV_ARCH = 1 else div_rd_r;

    -- Parada (serie / segmentada): desde el ciclo de la division hasta que llega Done.
    -- El fetch mantiene el PC y las instrucciones siguientes se anulan (burbujas)
    div_stall  <= '0' when DIV_ARCH = 1 else
                  (is_div and not div_pend) or (div_pend and not div_done);

    gen_div_serie: if DIV_ARCH = 0 generate
      UD: entity work.RISCV_Div(serie)
      port map(Clk => clk, Dividendo => div_a_abs, Divisor => div_b_abs, Valid => div_valid,
               Cociente => div_q, Resto => div_r, Ready => div_ready, Done => div_done);
    end generate;

    gen_div_paralela: if DIV_ARCH = 1 generate
      UD: entity work.RISCV_Div(paralela)
      port map(Clk => clk, Dividendo => div_a_abs, Divisor => div_b_abs, Valid => div_valid,
               Cociente => div_q, Resto => div_r, Ready => div_ready, Done => div_done);
    end generate;

    gen_div_segmentada: if DIV_ARCH = 2 generate
      UD: entity work.RISCV_Div(segmentada)
      port map(Clk => clk, Dividendo => div_a_abs, Divisor => div_b_abs, Valid => div_valid,
               Cociente => div_q, Resto => div_r, Ready => div_ready, Done => div_done);
    end generate;

    -- Salidas de depuracion
    dbg_rdata1 <= rdata1;
    dbg_rdata2 <= rdata2;
    dbg_wAlu   <= wAlu;
    dbg_we     <= we;
    dbg_wl     <= wl;
    dbg_we_mem <= aux_we_b;
    dbg_be_mem <= aux_be_b;

    auxOp2 <= rdata2 when ena_imm = '0' else imm;

    -- Dato que se escribe en rd (pc_out es el PC de la instruccion en decodificacion):
    --   jal / jalr : PC + 4 (direccion de retorno)
    --   auipc      : PC + imm
    --   resto      : salida de la ALU
    pc_ext    <= resize(unsigned(pc_out), REG_WIDTH);
    wreg_data <= std_logic_vector(pc_ext + 4)               when rdata_a(6 downto 2) = "11011" or rdata_a(6 downto 2) = "11001" else
                 std_logic_vector(pc_ext + unsigned(imm))   when rdata_a(6 downto 2) = "00101" else
                 wAlu;

    -- Calcular la dirección de salto (pc_load controla la carga del PC)
    --   jalr       : rs1 + imm (calculado en la ALU = wAlu), con el bit 0 a '0' (especificación RV32I)
    --   jal/branch : PC + imm
    pc_target <= wAlu(ADDR_WIDTH - 1 downto 1) & '0' when rdata_a(6 downto 2) = "11001" else
                 std_logic_vector(unsigned(pc_out) + resize(unsigned(imm), ADDR_WIDTH));

    -- Condición de los branches (funct3 = rdata_a(14 downto 12)). En un branch la ALU
    -- compara rs1 con rs2 (ena_imm = '0') y genera los flags eq, lt y ltu
    with rdata_a(14 downto 12) select
      br_taken <= eq      when "000",  -- beq
                  not eq  when "001",  -- bne
                  lt      when "100",  -- blt
                  not lt  when "101",  -- bge
                  ltu     when "110",  -- bltu
                  not ltu when "111",  -- bgeu
                  '0'     when others;

    -- Branch: solo se salta si se cumple la condición | jal / jalr: salto incondicional (decoder)
    pc_load <= br_taken when rdata_a(6 downto 2) = "11000" else dec_pc_load;

    U0: entity work.regFile(rtl)
    generic map(
        REG_WIDTH => REG_WIDTH,
        ADDR_REG  => ADDR_REG
    )
    port map(
        clk       => clk,
        nRst      => nRst,
        raddr1    => rs1,
        rdata1    => rdata1,
        raddr2    => rs2,
        rdata2    => rdata2,
        waddr     => rf_waddr,
        we        => rf_we,
        wl        => ld_wl_r,      -- el dato del load se escribe un ciclo despues
        be_l      => ld_be_r,
        sign_l    => ld_sign_r,
        wreg      => wreg_rf,
        wdata     => ld_data,
        dbg_regs  => dbg_regs
    );

    -- DECODER
    U1: entity work.decoder(rtl)
    generic map(
        REG_WIDTH => REG_WIDTH,
        ALU_DECOD => ALU_DECOD,
        ADDR_WIDTH => ADDR_WIDTH
    )
    port map(
        clk       => clk,
        nRst      => nRst,
        instr     => rdata_a,
        aloup     => aloup,
        rs1       => rs1,
        rs2       => rs2,
        rd        => rd,
        imm       => imm,
        we        => we,
        wl        => wl,
        be_l      => be_l,
        sign_l    => sign_l,
        ena_imm   => ena_imm,
        we_b      => aux_we_b,
        be_b      => aux_be_b,
        pc_load   => dec_pc_load,
        is_div    => is_div,
        div_op    => div_op
      );

    -- Disenar un multiplexor para elegir imm o rdata2 depende de la aloup (INSTR I)
    
    -- ALU
    U2: entity work.alu(rtl)
    generic map(
      ALU_LENGTH => REG_WIDTH,
      ALU_DECOD  => ALU_DECOD
    )
    port map(
      op1 => rdata1,
      op2 => auxOp2,
      aloup => aloup,
      eq => eq,
      lt => lt,
      ltu => ltu,
      aluOut => wAlu
    );

end architecture rtl;