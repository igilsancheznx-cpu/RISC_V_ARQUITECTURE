library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity DecExe is
    generic (
        REG_WIDTH  : integer := 32; 
        ADDR_REG   : integer := 5;
        ADDR_WIDTH : integer := 17;
        ALU_DECOD  : integer := 4
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
        ld_stall  : out  std_logic
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

    -- Parada: el load ocupa un ciclo mas mientras llega el dato
    ld_stall <= wl;

    -- Puerto de escritura del banco de registros:
    --   ciclo de write-back de un load -> rd del load con el dato de memoria
    --   resto                          -> rd de la instruccion en ejecucion (ALU / PC)
    -- Durante el write-back del load el decoder tiene una burbuja (NOP), cuya
    -- escritura se inhibe
    rf_waddr <= ld_rd_r when ld_wl_r = '1' else rd;
    rf_we    <= we and not ld_wl_r;

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
        wreg      => wreg_data,
        wdata     => ld_data
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
        pc_load   => dec_pc_load
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