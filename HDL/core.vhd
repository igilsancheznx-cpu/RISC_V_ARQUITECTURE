library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity core is
generic(
    MEM_WIDTH : integer := 32; 
    NUM_BYTES : integer := 4;
    ADDR_REG  : integer := 5;
    ADDR_WIDTH: integer := 17;
    PC_RESET  : integer := 16#10000#   -- PC inicial tras el reset
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
    be_b    :  out std_logic_vector(NUM_BYTES-1 downto 0);
    -- Depuracion (solo para testbench; se pueden dejar sin conectar)
    dbg_instr_valid : out std_logic;
    dbg_rdata1      : out std_logic_vector(MEM_WIDTH-1 downto 0);
    dbg_rdata2      : out std_logic_vector(MEM_WIDTH-1 downto 0);
    dbg_wAlu        : out std_logic_vector(MEM_WIDTH-1 downto 0);
    dbg_we          : out std_logic;
    dbg_wl          : out std_logic;
    dbg_we_mem      : out std_logic;
    dbg_be_mem      : out std_logic_vector(NUM_BYTES-1 downto 0);
    dbg_pc_load     : out std_logic;
    dbg_pc_target   : out std_logic_vector(ADDR_WIDTH-1 downto 0);
    dbg_regs        : out std_logic_vector(2**ADDR_REG * MEM_WIDTH - 1 downto 0)
);
end core;

architecture estructural of core is
    -- Signals for the core module
    signal pc_load   : std_logic;
    signal pc_target : std_logic_vector(ADDR_WIDTH - 1 downto 0);
    signal ld_stall  : std_logic;   -- load en ejecucion: parada de un ciclo

    -- Auxiliary signals to sincronize the instruction fetch and execution stages
    signal addr_a_sync : std_logic_vector(ADDR_WIDTH-1 downto 0);
    signal addr_b_sync : std_logic_vector(ADDR_WIDTH-1 downto 0);
    signal rdata_b_sync : std_logic_vector(MEM_WIDTH - 1 downto 0);

    -- Etapa de decodificacion. La memoria de instrucciones es sincrona: la
    -- instruccion que llega en rdata_a es la de la direccion que se presento en
    -- addr_a el ciclo anterior, mientras el fetch ya apunta a la siguiente.
    --   pc_instr    : PC de la instruccion que esta en el decoder (addr_a registrado)
    --   instr_valid : '0' en el primer ciclo tras el reset, en el ciclo siguiente a
    --                 un salto tomado (la instruccion leida es la del camino
    --                 equivocado) y en el ciclo siguiente a un load (write-back
    --                 del dato; el fetch mantiene el PC y la instruccion se vuelve
    --                 a leer). Ese ciclo se ejecuta como burbuja (NOP)
    --   instr_dec   : instruccion que se entrega a DecExe
    constant NOP        : std_logic_vector(MEM_WIDTH - 1 downto 0) := x"00000013";  -- addi x0, x0, 0
    signal pc_instr    : std_logic_vector(ADDR_WIDTH - 1 downto 0);
    signal instr_valid : std_logic;
    signal instr_dec   : std_logic_vector(MEM_WIDTH - 1 downto 0);

begin
    -- Puerto A: solo lectura de instrucciones
    wdata_a <= (others => '0');              -- No se escribe en memoria de instrucciones
    we_a    <= '0';                          -- Escritura desactivada
    be_a    <= (others => '0');              -- Byte enable desactivado

    -- pc_load / pc_target los genera DecExe; wdata_b, we_b y be_b salen
    -- directamente de DecExe (port map). Sin asignaciones constantes: cada
    -- senal tiene un unico driver
    addr_a       <= addr_a_sync;
    addr_b       <= addr_b_sync;
    rdata_b_sync <= rdata_b;                 -- Dato leido de memoria hacia DecExe

    -- PC de la instruccion en decodificacion y validez (anulacion tras un salto)
    process(clk, nRst)
    begin
        if nRst = '0' then
            pc_instr    <= std_logic_vector(to_unsigned(PC_RESET, ADDR_WIDTH));
            instr_valid <= '0';               -- primer ciclo: la memoria aun no ha leido
        elsif rising_edge(clk) then
            pc_instr    <= addr_a_sync;       -- direccion que muestrea la memoria en este flanco
            instr_valid <= not (pc_load or ld_stall);  -- salto tomado o load: se anula la siguiente
        end if;
    end process;

    instr_dec <= rdata_a when instr_valid = '1' else NOP;

    -- Salidas de depuracion
    dbg_instr_valid <= instr_valid;
    dbg_pc_load     <= pc_load;
    dbg_pc_target   <= pc_target;

    U0: entity work.fetch(rtl)
    generic map(
        ADDR_WIDTH  => ADDR_WIDTH,
        PC_RESET    => PC_RESET
    )
    port map(
        clk       => clk,
        nRst      => nRst,
        pc_load   => pc_load,
        pc_target => pc_target,
        pc_hold   => ld_stall,
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
       rdata_a   => instr_dec,
       rdata_b   => rdata_b_sync,
       addr_b    => addr_b_sync,
       wdata_b   => wdata_b,
       we_b      => we_b,
       be_b      => be_b,
       pc_out    => pc_instr,                -- PC de la instruccion (no el del fetch)
       pc_load   => pc_load,
       pc_target => pc_target,
       ld_stall  => ld_stall,
       dbg_rdata1 => dbg_rdata1,
       dbg_rdata2 => dbg_rdata2,
       dbg_wAlu   => dbg_wAlu,
       dbg_we     => dbg_we,
       dbg_wl     => dbg_wl,
       dbg_we_mem => dbg_we_mem,
       dbg_be_mem => dbg_be_mem,
       dbg_regs   => dbg_regs
    );

end estructural;