library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity regFile is
    generic (
        REG_WIDTH : integer := 32; 
        NUM_REGS  : integer := 5
    );
    -- Dado que una instrucción puede indicar dos registros fuente y un registro 
    -- destino, deberán existir al menos dos interfaces de lectura y uno de escritura. 
    port (
        clk       : in  std_logic;
        nRst      : in  std_logic;
        -- Read port 1
        raddr1    : in  std_logic_vector(NUM_REGS - 1 downto 0);
        rdata1    : out std_logic_vector(REG_WIDTH - 1 downto 0);
        -- Read port 2
        raddr2    : in  std_logic_vector(NUM_REGS - 1 downto 0);
        rdata2    : out std_logic_vector(REG_WIDTH - 1 downto 0);
        -- Write port
        waddr     : in  std_logic_vector(NUM_REGS - 1 downto 0);
        wreg      : in  std_logic_vector(REG_WIDTH - 1 downto 0); -- Write from ALU
        we        : in  std_logic; -- Write enable
        wb        : in  std_logic; -- Write back from memory 
        wdata     : in  std_logic_vector(REG_WIDTH - 1 downto 0)  -- Write from MEMORY
    );
end entity regFile;


architecture rtl of regFile is
    type reg_array is array (0 to 2**NUM_REGS - 1) of std_logic_vector(REG_WIDTH - 1 downto 0);
    signal regs : reg_array := (others => (others => '0'));
begin

    -- Lectura asincrona: El registro 0 SIEMPRE devuelve 0
    rdata1 <= (others => '0') when unsigned(raddr1) = 0 else 
              regs(to_integer(unsigned(raddr1)));
    rdata2 <= (others => '0') when unsigned(raddr2) = 0 else
              regs(to_integer(unsigned(raddr2)));

    -- Escritura sincrona
    process(clk, nRst)
        variable waddr_d : integer;
        variable widx_wb : integer;
    begin
        if nRst = '0' then
            regs <= (others => (others => '0'));
        elsif rising_edge(clk) then
            waddr_d := to_integer(unsigned(waddr));

            -- Escritura desde etapa Execution (evitando escribir en x0)
            if waddr_d /= 0 then
              if we = '1' then
                  regs(waddr_d) <= wreg;
              end if;
  
              -- Escritura independiente desde Write-Back (evitando escribir en x0)
              if wb = '1' then
                  regs(waddr_d) <= wdata;
              end if;
            else
              regs(waddr_d) <= (others => '0');
            end if;
        end if;
    end process;

end architecture rtl;