library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use work.div_pkg.all;          -- constante N = 32

-------------------------------------------------------------------------------
-- Testbench autocomprobado de RISCV_Div (tres arquitecturas en paralelo)
--   serie      : latencia N, Ready bajo mientras ocupa, ignora Valid si ocupado,
--                reutilización del último resultado (div + rem con mismos datos)
--   paralela   : resultado combinacional, Ready = Done = '1'
--   segmentada : una división por ciclo (con burbujas), resultados en orden,
--                latencia N, pulso de Done por resultado
-- Operandos sin signo. Para divisor = 0 se espera cociente = todo unos y
-- resto = dividendo (comportamiento RISC-V).
-------------------------------------------------------------------------------
entity tb_RISCV_Div is
end entity;

architecture sim of tb_RISCV_Div is

  constant T  : time    := 10 ns;
  constant NV : natural := 300;      -- nº de divisiones por arquitectura
  constant NDIR : natural := 16;     -- vectores dirigidos

  subtype word is std_logic_vector(31 downto 0);
  type word_arr is array (0 to NDIR-1) of word;

  constant DIR_A : word_arr := (
    x"00000000", x"00000001", x"00000007", x"00000064",
    x"FFFFFFFF", x"FFFFFFFF", x"80000000", x"00000005",
    x"00000000", x"FFFFFFFF", x"00003039", x"00003038",
    x"FFFFFFFF", x"80000000", x"00000001", x"7FFFFFFF");
  constant DIR_B : word_arr := (
    x"00000001", x"00000001", x"00000002", x"00000007",
    x"00000001", x"FFFFFFFF", x"00000001", x"00000000",
    x"00000000", x"00000000", x"00003039", x"00003039",
    x"00000002", x"7FFFFFFF", x"FFFFFFFF", x"00000002");

  constant HEXCH : string := "0123456789ABCDEF";

  ---------------------------------------------------------------------------
  -- Funciones auxiliares
  ---------------------------------------------------------------------------
  function xs(x : unsigned(31 downto 0)) return unsigned is
    variable v : unsigned(31 downto 0) := x;
  begin
    v := v xor shift_left(v, 13);
    v := v xor shift_right(v, 17);
    v := v xor shift_left(v, 5);
    return v;
  end function;

  function vec_a(k : natural) return word is
    variable x : unsigned(31 downto 0);
  begin
    if k < NDIR then
      return DIR_A(k);
    end if;
    x := to_unsigned(k, 32) xor x"9E3779B9";
    x := xs(xs(x));
    return std_logic_vector(x);
  end function;

  function vec_b(k : natural) return word is
    variable x : unsigned(31 downto 0);
  begin
    if k < NDIR then
      return DIR_B(k);
    end if;
    x := to_unsigned(k, 32) xor x"85EBCA6B";
    x := xs(xs(xs(x)));
    return std_logic_vector(shift_right(x, k mod 32));  -- varía la magnitud del divisor
  end function;

  function exp_q(a, b : word) return word is
    variable r : word;
  begin
    if unsigned(b) = 0 then
      r := (others => '1');
    else
      r := std_logic_vector(unsigned(a) / unsigned(b));
    end if;
    return r;
  end function;

  function exp_r(a, b : word) return word is
    variable r : word;
  begin
    if unsigned(b) = 0 then
      r := a;
    else
      r := std_logic_vector(unsigned(a) rem unsigned(b));
    end if;
    return r;
  end function;

  function hex(v : word) return string is
    variable s : string(1 to 8);
  begin
    for i in 0 to 7 loop
      s(i+1) := HEXCH(to_integer(unsigned(v(31-4*i downto 28-4*i))) + 1);
    end loop;
    return s;
  end function;

  ---------------------------------------------------------------------------
  -- Señales
  ---------------------------------------------------------------------------
  signal Clk : std_logic := '0';

  signal Div_s, Dsr_s, Q_s, R_s : word := (others => '0');
  signal Valid_s, Ready_s, Done_s : std_logic := '0';

  signal Div_p, Dsr_p, Q_p, R_p : word := (others => '0');
  signal Valid_p, Ready_p, Done_p : std_logic := '0';

  signal Div_g, Dsr_g, Q_g, R_g : word := (others => '0');
  signal Valid_g, Ready_g, Done_g : std_logic := '0';

  signal fin_s, fin_p, fin_g : std_logic := '0';
  signal err_s, err_p, err_g : natural := 0;

begin

  ---------------------------------------------------------------------------
  -- Reloj (se detiene cuando terminan los tres tests)
  ---------------------------------------------------------------------------
  clk_gen : process
  begin
    while not (fin_s = '1' and fin_p = '1' and fin_g = '1') loop
      Clk <= '0';  wait for T/2;
      Clk <= '1';  wait for T/2;
    end loop;
    wait;
  end process;

  ---------------------------------------------------------------------------
  -- DUTs
  ---------------------------------------------------------------------------
  dut_serie : entity work.RISCV_Div(serie)
    port map (Clk => Clk, Dividendo => Div_s, Divisor => Dsr_s, Valid => Valid_s,
              Cociente => Q_s, Resto => R_s, Ready => Ready_s, Done => Done_s);

  dut_paralela : entity work.RISCV_Div(paralela)
    port map (Clk => Clk, Dividendo => Div_p, Divisor => Dsr_p, Valid => Valid_p,
              Cociente => Q_p, Resto => R_p, Ready => Ready_p, Done => Done_p);

  dut_segmentada : entity work.RISCV_Div(segmentada)
    port map (Clk => Clk, Dividendo => Div_g, Divisor => Dsr_g, Valid => Valid_g,
              Cociente => Q_g, Resto => R_g, Ready => Ready_g, Done => Done_g);

  ---------------------------------------------------------------------------
  -- Test SERIE
  ---------------------------------------------------------------------------
  test_serie : process
    variable a, b, eq, er : word;
    variable cycles : natural;
    variable errs   : natural := 0;
  begin
    Valid_s <= '0';
    wait for 3*T;

    for k in 0 to NV-1 loop
      a := vec_a(k);  b := vec_b(k);
      eq := exp_q(a, b);  er := exp_r(a, b);

      -- Solicitud de división (Valid de 1 ciclo)
      wait until falling_edge(Clk);
      if Ready_s /= '1' then
        report "SERIE #" & integer'image(k) & ": Ready deberia estar a '1' antes de pedir division" severity error;
        errs := errs + 1;
      end if;
      Div_s <= a;  Dsr_s <= b;  Valid_s <= '1';
      wait until rising_edge(Clk);          -- flanco de aceptacion (Valid y Ready)
      wait until falling_edge(Clk);
      Valid_s <= '0';
      cycles := 1;                          -- flancos transcurridos desde la aceptacion

      -- Espera de Done
      while Done_s = '0' and cycles < N + 10 loop
        if Ready_s /= '0' then
          report "SERIE #" & integer'image(k) & ": Ready deberia estar a '0' mientras divide" severity error;
          errs := errs + 1;
        end if;
        -- Valid espurio con la unidad ocupada: debe ignorarse
        if (k mod 3 = 0) and cycles = 3 then
          Valid_s <= '1';  Div_s <= not a;  Dsr_s <= not b;
        else
          Valid_s <= '0';
        end if;
        wait until falling_edge(Clk);
        cycles := cycles + 1;
      end loop;
      Valid_s <= '0';

      if cycles /= N then
        report "SERIE #" & integer'image(k) & ": latencia " & integer'image(cycles) &
               " ciclos, se esperaban " & integer'image(N) severity error;
        errs := errs + 1;
      end if;
      if Q_s /= eq or R_s /= er then
        report "SERIE #" & integer'image(k) & ": " & hex(a) & " / " & hex(b) &
               " -> Q=" & hex(Q_s) & " R=" & hex(R_s) &
               " (esperado Q=" & hex(eq) & " R=" & hex(er) & ")" severity error;
        errs := errs + 1;
      end if;
      if Ready_s /= '1' then
        report "SERIE #" & integer'image(k) & ": Ready deberia volver a '1' al terminar" severity error;
        errs := errs + 1;
      end if;

      -- Reutilizacion: misma operacion (div seguido de rem) sin recalcular
      if k mod 4 = 0 then
        wait until falling_edge(Clk);
        Div_s <= a;  Dsr_s <= b;  Valid_s <= '1';
        wait until rising_edge(Clk);
        wait for 1 ns;
        if Done_s /= '1' or Ready_s /= '1' or Q_s /= eq or R_s /= er then
          report "SERIE #" & integer'image(k) & ": fallo en la reutilizacion del ultimo resultado" severity error;
          errs := errs + 1;
        end if;
        wait until falling_edge(Clk);
        Valid_s <= '0';
      end if;
    end loop;

    err_s <= errs;
    fin_s <= '1';
    wait;
  end process;

  ---------------------------------------------------------------------------
  -- Test PARALELA (combinacional)
  ---------------------------------------------------------------------------
  test_paralela : process
    variable a, b, eq, er : word;
    variable errs : natural := 0;
  begin
    Valid_p <= '1';
    wait for 2*T;
    for k in 0 to NV-1 loop
      a := vec_a(k);  b := vec_b(k);
      eq := exp_q(a, b);  er := exp_r(a, b);
      Div_p <= a;  Dsr_p <= b;
      wait for 1 ns;                         -- solo propagacion combinacional
      if Ready_p /= '1' or Done_p /= '1' then
        report "PARALELA #" & integer'image(k) & ": Ready y Done deben estar siempre a '1'" severity error;
        errs := errs + 1;
      end if;
      if Q_p /= eq or R_p /= er then
        report "PARALELA #" & integer'image(k) & ": " & hex(a) & " / " & hex(b) &
               " -> Q=" & hex(Q_p) & " R=" & hex(R_p) &
               " (esperado Q=" & hex(eq) & " R=" & hex(er) & ")" severity error;
        errs := errs + 1;
      end if;
      wait for 1 ns;
    end loop;
    err_p <= errs;
    fin_p <= '1';
    wait;
  end process;

  ---------------------------------------------------------------------------
  -- Test SEGMENTADA: divisiones consecutivas con burbujas
  ---------------------------------------------------------------------------
  test_segmentada : process
    type int_arr is array (0 to NV-1) of integer;
    variable issue_c : int_arr;
    variable c, issued, checked : natural := 0;
    variable a, b, eq, er : word;
    variable errs : natural := 0;
    constant LIM : natural := 2*NV + N + 50;
  begin
    Valid_g <= '0';
    wait for 3*T;

    while checked < NV and c < LIM loop
      wait until falling_edge(Clk);

      -- Comprobacion: Done es un pulso por cada resultado, N ciclos tras la solicitud
      if Done_g = '1' then
        if checked >= issued then
          report "SEGMENTADA: Done espurio en el ciclo " & integer'image(c) severity error;
          errs := errs + 1;
        else
          a := vec_a(checked);  b := vec_b(checked);
          eq := exp_q(a, b);  er := exp_r(a, b);
          if c /= issue_c(checked) + N then
            report "SEGMENTADA #" & integer'image(checked) & ": latencia " &
                   integer'image(c - issue_c(checked)) & " ciclos, se esperaban " &
                   integer'image(N) severity error;
            errs := errs + 1;
          end if;
          if Q_g /= eq or R_g /= er then
            report "SEGMENTADA #" & integer'image(checked) & ": " & hex(a) & " / " & hex(b) &
                   " -> Q=" & hex(Q_g) & " R=" & hex(R_g) &
                   " (esperado Q=" & hex(eq) & " R=" & hex(er) & ")" severity error;
            errs := errs + 1;
          end if;
          checked := checked + 1;
        end if;
      end if;

      if Ready_g /= '1' then
        report "SEGMENTADA: Ready debe estar siempre a '1'" severity error;
        errs := errs + 1;
      end if;

      -- Nueva solicitud en este ciclo (salvo burbujas)
      if issued < NV and (c mod 7) /= 3 and (c mod 7) /= 4 then
        Div_g <= vec_a(issued);  Dsr_g <= vec_b(issued);  Valid_g <= '1';
        issue_c(issued) := c;
        issued := issued + 1;
      else
        Valid_g <= '0';        -- en las burbujas se ponen datos basura con Valid = '0'
        Div_g <= std_logic_vector(xs(to_unsigned(c + 1, 32)));
        Dsr_g <= std_logic_vector(xs(xs(to_unsigned(c + 1, 32))));
      end if;

      c := c + 1;
    end loop;

    if checked /= NV then
      report "SEGMENTADA: solo se recibieron " & integer'image(checked) & " de " &
             integer'image(NV) & " resultados" severity error;
      errs := errs + 1;
    end if;

    Valid_g <= '0';
    err_g <= errs;
    fin_g <= '1';
    wait;
  end process;

  ---------------------------------------------------------------------------
  -- Informe final
  ---------------------------------------------------------------------------
  resumen : process
  begin
    wait until fin_s = '1' and fin_p = '1' and fin_g = '1';
    wait for 1 ns;
    report "Errores -> serie: " & integer'image(err_s) &
           ", paralela: " & integer'image(err_p) &
           ", segmentada: " & integer'image(err_g);
    if err_s + err_p + err_g = 0 then
      report "TEST SUPERADO: las tres arquitecturas son correctas";
    else
      report "TEST FALLIDO" severity failure;
    end if;
    wait;
  end process;

end architecture;
