library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity core is
generic(
    MEM_WIDTH : integer := 32; 
    NUM_BYTES : integer := 4;
    ADDR_REG  : integer := 5;
    ADDR_WIDTH: integer := 17
);
port(
    clk   : in std_logic;
    nRst  : in std_logic;
    -- VARIABLES FOR PORT A -- INSTRUCTION FETCH
    rdata_a :  in std_logic_vector(MEM_WIDTH - 1 downto 0);
    addr_a  :  out std_logic_vector(ADDR_WIDTH-1 downto 0); 
    wdata_a :  out std_logic_vector(MEM_WIDTH -1 downto 0);
    we_a    :  out std_logic; 
    be_a    :  out std_logic_vector(NUM_BYTES-1 downto 0); 
    -- VARIABLES FOR PORT B -- DATA MEMORY
    rdata_b :  in std_logic_vector(MEM_WIDTH-1 downto 0); 
    addr_b  :  out std_logic_vector(ADDR_WIDTH-1 downto 0); 
    wdata_b :  out std_logic_vector(MEM_WIDTH - 1 downto 0);    
    we_b    :  out std_logic; 
    be_b    :  out std_logic_vector(NUM_BYTES-1 downto 0) 
);
end core;

architecture estructural of core is
    -- Signals for the core module
    signal pc_load   : std_logic;
    signal pc_target : std_logic_vector(ADDR_WIDTH - 1 downto 0);

    -- Auxiliary signals to sincronize the instruction fetch and execution stages
    signal addr_a_sync : std_logic_vector(ADDR_WIDTH-1 downto 0);
    signal rdata_a_sync : std_logic_vector(MEM_WIDTH - 1 downto 0);
    signal addr_b_sync : std_logic_vector(ADDR_WIDTH-1 downto 0);
    signal rdata_b_sync : std_logic_vector(MEM_WIDTH - 1 downto 0);

begin
    --Inicializacion signals
    pc_load   <= '0';                        -- Sin saltos por ahora
    pc_target <= (others => '0');            -- Direccion de salto nula
    wdata_a <= (others => '0');              -- No se escribe en memoria de instrucciones
    we_a    <= '0';                          -- Escritura desactivada
    be_a    <= (others => '0');              -- Byte enable desactivado
    addr_b  <= (others => '0');              -- Direccion por defecto
    wdata_b <= (others => '0');              -- Datos de escritura nulos
    we_b    <= '0';                          -- Escritura desactivada
    be_b    <= (others => '0');              -- Byte enable desactivado


            addr_a <= addr_a_sync;
            addr_b <= addr_b_sync;
    
    U0: entity work.fetch(rtl)
    generic map(
        ADDR_WIDTH  => ADDR_WIDTH
    )
    port map(
        clk       => clk,
        nRst      => nRst,
        pc_load   => pc_load,
        pc_target => pc_target,
        pc_out    => addr_a_sync
    );

    U1: entity work.DecExe(rtl)
    generic map(
       REG_WIDTH  => MEM_WIDTH,
       ADDR_REG   => ADDR_REG,
       ADDR_WIDTH => ADDR_WIDTH,
       ALU_DECOD  => NUM_BYTES
    )
    port map(
       clk       => clk,
       nRst      => nRst,
       rdata_a   => rdata_a,
       rdata_b   => rdata_b_sync,
       addr_b    => addr_b_sync,
       wdata_b   => wdata_b,
       we_b      => we_b,
       be_b      => be_b,
       pc_out    => addr_a_sync,
       pc_load   => pc_load,
       pc_target => pc_target
    );

end estructural;