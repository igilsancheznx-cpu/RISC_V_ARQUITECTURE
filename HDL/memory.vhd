library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity memory is
    generic(
        MEM_WIDTH  : integer := 32;
        NUM_BYTES  : integer := 4;
        ADDR_WIDTH : integer := 17
    );
    port(
        clk : in std_logic;

        -- Puerto A
        addr_a  : in  std_logic_vector(ADDR_WIDTH-1 downto 0);
        wdata_a : in  std_logic_vector(MEM_WIDTH-1 downto 0);
        rdata_a : out std_logic_vector(MEM_WIDTH-1 downto 0);
        we_a    : in  std_logic;
        be_a    : in  std_logic_vector(NUM_BYTES-1 downto 0);

        -- Puerto B
        addr_b  : in  std_logic_vector(ADDR_WIDTH-1 downto 0);
        wdata_b : in  std_logic_vector(MEM_WIDTH-1 downto 0);
        rdata_b : out std_logic_vector(MEM_WIDTH-1 downto 0);
        we_b    : in  std_logic;
        be_b    : in  std_logic_vector(NUM_BYTES-1 downto 0)
    );
end memory;


architecture rtl of memory is

    type mem_t is array (
        0 to 2**ADDR_WIDTH - 1
    ) of std_logic_vector(MEM_WIDTH-1 downto 0);

    signal mem : mem_t:= (--Convertir de RISC-V Online Assembler
        0 => X"00a00093", -- Position 0  (PC = 0x00): addi x1, x0, 10
        1 => X"01400113", -- Position 1  (PC = 0x04): addi x2, x0, 20
        2 => X"002081b3", -- Position 2  (PC = 0x08): add  x3, x1, x2
        3 => X"00302023", -- Position 3  (PC = 0x0C): sw   x3, 0(x0)
        4 => X"3e800093", -- Position 4  (PC = 0x0C): sw   x3, 0(x0)
        5 => X"7d008113", -- Position 3  (PC = 0x0C): sw   x3, 0(x0)
        6 => X"c1810193", -- Position 3  (PC = 0x0C): sw   x3, 0(x0)
        7 => X"83018213", -- Position 3  (PC = 0x0C): sw   x3, 0(x0)
        8 => X"3e820293", -- Position 3  (PC = 0x0C): sw   x3, 0(x0)
        9 => X"fec30313", -- Position 3  (PC = 0x0C): sw   x3, 0(x0)
        others => (others => '0') -- El resto de la memoria se llena con ceros
    );

begin

 process(clk)
 begin
  if rising_edge(clk) then

    -- =====================
    -- PUERTO A
    -- =====================

    -- Lectura
    rdata_a <= mem(to_integer(unsigned(addr_a)));

    -- Escritura
    if we_a = '1' then
      for i in 0 to NUM_BYTES-1 loop
        if be_a(i) = '1' then
          mem(to_integer(unsigned(addr_a)))(8*i + 7 downto 8*i) <= wdata_a(8*i + 7 downto 8*i);
        end if;
      end loop;
    end if;

    -- =====================
    -- PUERTO B
    -- =====================

    -- Lectura
    rdata_b <= mem(to_integer(unsigned(addr_b)));

    -- Escritura
    if we_b = '1' then
      for i in 0 to NUM_BYTES-1 loop
        if be_b(i) = '1' then
          mem(to_integer(unsigned(addr_b)))(8*i + 7 downto 8*i) <= wdata_b(8*i + 7 downto 8*i);
        end if;
      end loop;
    end if;

  end if;
 
end process;

end rtl;