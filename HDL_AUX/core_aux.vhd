-- =============================================================================
-- core_aux: copia auxiliar de HDL/core.vhd para simulacion (tb_coreAux)
-- Cambios respecto al original (marcados con "-- [AUX]"):
--   1. Entidad renombrada a core_aux; instancia DecExe_aux
--   2. Eliminados los dobles drivers:
--        - addr_b, wdata_b, we_b, be_b: tenian una asignacion constante a 0
--          y ademas los manejaba el proceso / DecExe  -> X al escribir
--        - pc_load, pc_target: constante a 0 y salida de DecExe -> 'U'
--        - addr_a_sync, addr_b_sync: los ponia a 0 el reset del proceso y
--          ademas los manejan fetch / DecExe          -> PC = X
--   3. PROBLEMA 1 (store en la direccion del store anterior):
--        DecExe ya registra addr_b, wdata_b, we_b y be_b en el mismo flanco.
--        El original volvia a registrar SOLO addr_b, por lo que la direccion
--        llegaba un ciclo mas tarde que el dato y los enables (el primer sb
--        escribia en mem[0] y machacaba la primera instruccion).
--        Ahora addr_b sale directamente de DecExe, alineado con el resto.
--   4. PROBLEMA 2 (addr_a / addr_b = X durante el reset):
--        el reset ponia a 0 las senales internas (_sync) pero no las salidas.
--        Ahora el reset inicializa addr_a, y addr_b viene de DecExe, que ya
--        lo pone a 0 en reset.
--   5. rdata_b_sync no tenia driver ('U'): se conecta a rdata_b
-- =============================================================================
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity core_aux is
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
end core_aux;

architecture estructural of core_aux is
    -- Signals for the core module
    signal pc_load   : std_logic;
    signal pc_target : std_logic_vector(ADDR_WIDTH - 1 downto 0);

    -- Auxiliary signals to sincronize the instruction fetch and execution stages
    signal addr_a_sync  : std_logic_vector(ADDR_WIDTH-1 downto 0);
    signal rdata_b_sync : std_logic_vector(MEM_WIDTH - 1 downto 0);

begin
    --Inicializacion signals (puerto A solo lectura)
    wdata_a <= (others => '0');              -- No se escribe en memoria de instrucciones
    we_a    <= '0';                          -- Escritura desactivada
    be_a    <= (others => '0');              -- Byte enable desactivado
    -- [AUX] Eliminadas las asignaciones constantes de pc_load, pc_target,
    --       addr_b, wdata_b, we_b y be_b (dobles drivers)

    -- [AUX] Dato leido de memoria hacia DecExe (antes sin driver)
    rdata_b_sync <= rdata_b;

    -- Registro de la direccion de instrucciones (puerto A)
    process(clk, nRst)
    begin
        if nRst = '0' then
            addr_a <= (others => '0');       -- [AUX] se inicializa la salida, no addr_a_sync
        elsif rising_edge(clk) then
            addr_a <= addr_a_sync;
        end if;
    end process;
    -- [AUX] addr_b ya no se vuelve a registrar: sale de DecExe alineado
    --       con wdata_b, we_b y be_b

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

    U1: entity work.DecExe_aux(rtl)
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
       addr_b    => addr_b,               -- [AUX] directo, sin registro extra
       wdata_b   => wdata_b,
       we_b      => we_b,
       be_b      => be_b,
       pc_out    => addr_a_sync,
       pc_load   => pc_load,
       pc_target => pc_target
    );

end estructural;
