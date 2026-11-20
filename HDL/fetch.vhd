library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity fetch is
  generic (
    ADDR_WIDTH: integer := 17;
    PC_RESET  : integer := 16#10000#  -- Valor del PC tras el reset (direccion de la primera instruccion)
  );
  port (
    clk       : in  std_logic;
    nRst      : in  std_logic;
    pc_load   : in  std_logic;                                --si (pc_load) pc_mux = pc_target else pc_mux=pc_plus
    pc_target : in  std_logic_vector(ADDR_WIDTH - 1 downto 0);--Direccion para saltos
    pc_hold   : in  std_logic := '0';                         --Parada (load): el PC no avanza
    pc_out    : out std_logic_vector(ADDR_WIDTH - 1 downto 0) --Es la direccion de la instruccion actual.
  );
end fetch;

architecture rtl of fetch is --se encarga del PC
    signal pc_count : unsigned(ADDR_WIDTH - 1 downto 0);   -- elegir si hay salto
    signal pc_reg   : unsigned(ADDR_WIDTH - 1 downto 0);   -- PC siguiente = PC actual + 4

begin

    -- Multiplexor to select the next PC value
    pc_count <= unsigned(pc_target) when pc_load = '1' else
                pc_reg              when pc_hold = '1' else   -- se vuelve a leer la misma instruccion
                (pc_reg + 4);

    -- Flip-flop D to store the current PC value
    process(clk, nRst)
    begin
        if nRst = '0' then
            pc_reg <= to_unsigned(PC_RESET, ADDR_WIDTH);  -- PC inicial (0x10000 por defecto)
        elsif rising_edge(clk) then
            pc_reg <= pc_count;  -- Address of the instruction to be fetched
              
        end if;
    end process;

    pc_out <= std_logic_vector(pc_reg); -- Address of the instruction to be fetched
    
    -- The instruction is handled outside of this module. It must be controlled by Risc-V core. The instruction is fetched from memory and passed to the core for execution.

end architecture rtl;
