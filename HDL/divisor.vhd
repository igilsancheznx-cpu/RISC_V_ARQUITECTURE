library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity disvision is
generic (
    INSTR_WIDTH : integer := 32; 
    ADDR_WIDTH: integer := 17
  );
  port (
    dividendo : in     std_logic_vector(INSTR_WIDTH-1 downto 0);  -- Dividendo P
    divisor   : in     std_logic_vector(INSTR_WIDTH-1 downto 0);      -- Divisor B
    cociente  : in     std_logic_vector(INSTR_WIDTH-1 downto 0);    -- Cociente C
    resto     : in     std_logic_vector(ADDR_WIDTH - 1 downto 0);   -- Resto R
    enable    : inout  std_logic;
  );
end division;

architecture rtl of division is 
  signal RC : std_logic_vector(2*INSTR_WIDTH downto 0);
  signal disivor_s : std_logic_vector(INSTR_WIDTH downto 0);
  signal signo : std_logic;

begin

  divisor_s <= resize(signed(divisor), INSTR_WIDTH+1);

  RC <= resize(signed(dividendo), 2*INSTR_WIDTH+1);

  signo <= RC(RC'HIGH);


  
  process(clk, enable)
    variable RC_v : std_logic_vector(2*INSTR_WIDTH downto 0);
  begin
  RELOJ
    RC_v := RC(2*INSTR_WIDTH-1 downto 0) & '0';
    if signo = '1' then -- NEGATIVO
      RC_v := (RC_v(2*INSTR_WIDTH downto INSTR_WIDTH) + divisor);
    else
      RC_v := (RC_v(2*INSTR_WIDTH downto INSTR_WIDTH) - signed(divisor));
    end if;

    RC(RC'LOW) <= RC_v(RC_v'HIGH);
    RC(2*INSTR_WIDTH downto INSTR_WIDTH) <= RC_v;
    
  end process;

  -- CONTADOR DE 32 ITERACIONES


end rtl;