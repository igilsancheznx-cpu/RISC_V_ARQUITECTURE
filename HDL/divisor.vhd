--library ieee;
--use ieee.std_logic_1164.all;
--use ieee.numeric_std.all;
--
--entity division is
--generic (
--    INSTR_WIDTH : integer := 32; 
--    ADDR_WIDTH: integer := 17
--  );
--  port (
--    dividendo : in     std_logic_vector(INSTR_WIDTH-1 downto 0);  -- Dividendo P
--    divisor   : in     std_logic_vector(INSTR_WIDTH-1 downto 0);      -- Divisor B
--    cociente  : in     std_logic_vector(INSTR_WIDTH-1 downto 0);    -- Cociente C
--    resto     : in     std_logic_vector(ADDR_WIDTH - 1 downto 0);   -- Resto R
--    enable    : inout  std_logic
--  );
--end division;
--
--architecture rtl of division is 
--  signal RC : std_logic_vector(2*INSTR_WIDTH downto 0);
--  signal disivor_s : std_logic_vector(INSTR_WIDTH downto 0);
--  signal signo : std_logic;
--
--begin
--
--  divisor_s <= resize(signed(divisor), INSTR_WIDTH+1);
--
--  RC <= resize(signed(dividendo), 2*INSTR_WIDTH+1);
--
--  signo <= RC(RC'HIGH);
--
--
--  
--  process(clk, enable)
--    variable RC_v : std_logic_vector(2*INSTR_WIDTH downto 0);
--  begin
--  RELOJ
--    RC_v := RC(2*INSTR_WIDTH-1 downto 0) & '0';
--    if signo = '1' then -- NEGATIVO
--      RC_v := (RC_v(2*INSTR_WIDTH downto INSTR_WIDTH) + divisor);
--    else
--      RC_v := (RC_v(2*INSTR_WIDTH downto INSTR_WIDTH) - signed(divisor));
--    end if;
--
--    RC(RC'LOW) <= RC_v(RC_v'HIGH);
--    RC(2*INSTR_WIDTH downto INSTR_WIDTH) <= RC_v;
--    
--  end process;

--end rtl;

  -- CONTADOR DE 32 ITERACIONES
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

-------------------------------------------------------------------------------
-- Paquete con el núcleo de la división sin restauración (un paso)
-------------------------------------------------------------------------------
package div_pkg is
  constant N : natural := 32;

  type div_state is record --clase
    R : unsigned(N downto 0);     -- resto parcial, n+1 bits (con signo) 
    C : unsigned(N-1 downto 0);   -- cociente parcial / parte baja del dividendo [Cociente y dividendo]
  end record;

  function div_init(a : unsigned(N-1 downto 0)) return div_state;
  function div_step(s : div_state; B : unsigned(N downto 0)) return div_state;
  function div_fix (s : div_state; B : unsigned(N downto 0)) return unsigned;
end package;

package body div_pkg is

  -- Paso 1: (R,C) = P extendido
  function div_init(a : unsigned(N-1 downto 0)) return div_state is
    variable s : div_state;
  begin
    s.R := (others => '0'); 
    s.C := a; 
    return s;
  end function;

  -- Pasos 2 y 3: desplazar (R,C) y sumar/restar B según el signo de R
  function div_step(s : div_state; B : unsigned(N downto 0)) return div_state is --Conectar con la ALU
    variable Rs, Rn : unsigned(N downto 0);
    variable Cs     : unsigned(N-1 downto 0);
  begin
    Rs := s.R(N-1 downto 0) & s.C(N-1);
    Cs := s.C(N-2 downto 0) & '0';
    if s.R(N) = '0' then
      Rn := Rs - B;               -- R >= 0: restar
    else
      Rn := Rs + B;               -- R <  0: sumar
    end if;
    Cs(0) := not Rn(N);           -- bit de cociente = 1 si el nuevo R es >= 0
    return (R => Rn, C => Cs);
  end function;

  -- Paso 5: corrección final del resto (se suma B, no C)
  function div_fix(s : div_state; B : unsigned(N downto 0)) return unsigned is
  begin
    if s.R(N) = '1' then
      return s.R + B;
    else
      return s.R;
    end if;
  end function;

end package body;

-------------------------------------------------------------------------------
-- Entidad
-------------------------------------------------------------------------------
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use work.div_pkg.all;

entity RISCV_Div is
  port (
    Clk       : in  std_logic;
    Dividendo : in  std_logic_vector(31 downto 0);
    Divisor   : in  std_logic_vector(31 downto 0);
    Valid     : in  std_logic;
    Cociente  : out std_logic_vector(31 downto 0);
    Resto     : out std_logic_vector(31 downto 0);
    Ready     : out std_logic;
    Done      : out std_logic);
end entity;

-------------------------------------------------------------------------------
-- Arquitectura SERIE: un solo núcleo, N ciclos
-------------------------------------------------------------------------------
architecture serie of RISCV_Div is
  signal st      : div_state := (R => (others => '0'), C => (others => '0'));
  signal B       : unsigned(N downto 0) := (others => '0');
  signal cnt     : natural range 0 to N := 0;
  signal busy    : std_logic := '0';
  signal done_i  : std_logic := '0';
  signal haveLast: std_logic := '0';
  signal lastA, lastB : std_logic_vector(31 downto 0) := (others => '0');
begin
  Ready    <= not busy;
  Done     <= done_i;
  Cociente <= std_logic_vector(st.C);
  Resto    <= std_logic_vector(div_fix(st, B)(N-1 downto 0));

  process (Clk)
    variable Bv : unsigned(N downto 0);
  begin
    if rising_edge(Clk) then
      if busy = '1' then
        st  <= div_step(st, B);
        cnt <= cnt + 1;
        if cnt = N-1 then          -- se completa la iteración N
          busy   <= '0';
          done_i <= '1';
        end if;
      elsif Valid = '1' then
        if haveLast = '1' and Dividendo = lastA and Divisor = lastB then
          null;                    -- mismos datos: se reutiliza el resultado, Done sigue activo
        else
          Bv     := '0' & unsigned(Divisor);
          B      <= Bv;
          st     <= div_step(div_init(unsigned(Dividendo)), Bv);  -- 1.ª iteración
          cnt    <= 1;
          busy   <= '1';
          done_i <= '0';
          lastA  <= Dividendo;
          lastB  <= Divisor;
          haveLast <= '1';
        end if;
      end if;
    end if;
  end process;
end architecture;

-------------------------------------------------------------------------------
-- Arquitectura PARALELA: puramente combinacional (sin Clk)
-------------------------------------------------------------------------------
architecture paralela of RISCV_Div is
  type st_array is array (0 to N) of div_state;
  signal s  : st_array;
  signal Bv : unsigned(N downto 0);
begin
  Bv   <= '0' & unsigned(Divisor);
  s(0) <= div_init(unsigned(Dividendo));

  gen : for i in 0 to N-1 generate
    s(i+1) <= div_step(s(i), Bv);
  end generate;

  Cociente <= std_logic_vector(s(N).C);
  Resto    <= std_logic_vector(div_fix(s(N), Bv)(N-1 downto 0));
  Ready    <= '1';
  Done     <= '1';
end architecture;

-------------------------------------------------------------------------------
-- Arquitectura SEGMENTADA: un registro entre cada iteración
-------------------------------------------------------------------------------
architecture segmentada of RISCV_Div is
  type st_array is array (1 to N) of div_state;
  type b_array  is array (1 to N) of unsigned(N downto 0);
  signal s : st_array := (others => (R => (others => '0'), C => (others => '0')));
  signal b : b_array  := (others => (others => '0'));
  signal v : std_logic_vector(1 to N) := (others => '0');
begin
  process (Clk)
    variable Bv : unsigned(N downto 0);
  begin
    if rising_edge(Clk) then
      Bv   := '0' & unsigned(Divisor);
      s(1) <= div_step(div_init(unsigned(Dividendo)), Bv);
      b(1) <= Bv;
      v(1) <= Valid;
      for i in 1 to N-1 loop
        s(i+1) <= div_step(s(i), b(i));
        b(i+1) <= b(i);
        v(i+1) <= v(i);
      end loop;
    end if;
  end process;

  Cociente <= std_logic_vector(s(N).C);
  Resto    <= std_logic_vector(div_fix(s(N), b(N))(N-1 downto 0));
  Ready    <= '1';
  Done     <= v(N);   -- pulso de 1 ciclo cuando sale cada resultado
end architecture;