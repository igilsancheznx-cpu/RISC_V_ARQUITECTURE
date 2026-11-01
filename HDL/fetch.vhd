library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity fetch is
  generic (
    INSTR_WIDTH : integer := 32; 
    ADDR_WIDTH: integer := 17
  );
  port (
    clk       : in  std_logic;
    nRst      : in  std_logic;
    pc_load   : in  std_logic;--si (pc_load) pc_mux=pc_target else pc_mux=pc_plus
    pc_target : in  std_logic_vector(ADDR_WIDTH - 1 downto 0);--Direccion para saltos
    pc_out    : out std_logic_vector(ADDR_WIDTH - 1 downto 0) --Es la direcci�n de la instrucci�n actual.
  );
end fetch;

architecture rtl of fetch is --se encarga del PC
    signal pc_mux  : std_logic_vector(ADDR_WIDTH - 1 downto 0);--elegir si hay salto
    signal pc_plus : std_logic_vector(ADDR_WIDTH - 1 downto 0);--PC siguiente = PC actual + 4
    signal pc_out_buff : std_logic_vector(ADDR_WIDTH - 1 downto 0);
    constant PLUS_FOUR : integer := 4;
begin

    -- Multiplexor to select the next PC value
    pc_mux <= pc_target when pc_load = '1' else pc_plus;
    
    -- PC adder to increment the PC value by 4
    pc_plus <= std_logic_vector(unsigned(pc_out_buff) + PLUS_FOUR); -- Address of the next instruction to be fetched

    -- Flip-flop D to store the current PC value
    process(clk, nRst)
    begin
        if nRst = '0' then
            pc_out_buff <= (others => '0');
            pc_out <= (others => '0');
        elsif rising_edge(clk) then
            pc_out_buff <= pc_mux; -- Address of the instruction to be fetched
            pc_out <= pc_mux; -- Address of the instruction to be fetched
        end if;
    end process;

    -- The instruction is handled outside of this module. It must be controlled by Risc-V core. The instruction is fetched from memory and passed to the core for execution.

end architecture rtl;
