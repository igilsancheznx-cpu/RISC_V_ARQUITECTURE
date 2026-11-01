library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;  -- define el tipo signed y unsigned

entity alu is
  generic (
    ALU_LENGTH : integer := 5;
    NUM_REGS  : integer := 5;
    ALU_DECOD : integer := 5
  );
  port(
    op1    : in std_logic_vector(ALU_LENGTH-1 downto 0);
    op2    : in std_logic_vector(ALU_LENGTH-1 downto 0);  -- Puede valer imm o rs2, se gestiona en el decoder
    aloup  : in std_logic_vector(ALU_DECOD-1 downto 0);
    eq     : in std_logic;
    lt     : in std_logic;
    ltu    : in std_logic;
    aluOut : in std_logic_vector(ALU_LENGTH-1 downto 0)
  );
end entity;

architecture rtl of alu is
   signal auxAlu : std_logic_vector(ALU_LENGTH downto 0);
   signal desp: integer range 0 to REG_WIDTH - 1;

begin

   op1_s := signed(op1);
   op2_s := signed(op2);
   op1_u := unsigned(op1);
   op2_u := unsigned(op2);

   desp := to_integer(unsigned(op2(NUM_REGS - 1 downto 0)));

   -- 3 ibit para funct3 y 1 LSB para funct7
   auxAlu <= ('0' & op1) + ('0' & op2)                      when aloup = "0000" else -- add   (R, funct3=0x0, funct7=0x00)
             op1_u - (not op2_u)                            when aloup = "0001" else -- sub   (R, funct3=0x0, funct7=0x20)
             op1 xor op2                                    when aloup = "1000" else -- xor   (R, funct3=0x4, funct7=0x00)
             op1 or  op2                                    when aloup = "1100" else -- or    (R, funct3=0x6, funct7=0x00)
             op1 and op2                                    when aloup = "1110" else -- and   (R, funct3=0x7, funct7=0x00)
             std_logic_vector(shift_left(op1_u, desp))      when alu_op = "0010" else -- sll   (R, funct3=0x1, funct7=0x00)
             std_logic_vector(shift_right(op1_u, desp))     when alu_op = "1010" else -- srl   (R, funct3=0x5, funct7=0x00)
             std_logic_vector(shift_right(op1_s, desp))     when alu_op = "1011" else -- sra   (R, funct3=0x5, funct7=0x20)
             (0 => '1', others => '0')                      when (aloup = "0100" and op1_s < op2_s) else -- slt   (R, funct3=0x2, funct7=0x00)
             (others => '0')                                when aloup = "0100" else
             (0 => '1', others => '0')                      when (aloup = "0110" and op1_u < op2_u) else -- slt   (R, funct3=0x2, funct7=0x00)
             (others => '0')                                when aloup = "0110" else
             (others => '0');

  process(clk, nRst)
  begin
    if nRst = '0' then
      aluOut <= (others => '0');

    elsif clk'event and clk = '1' then
      aluOut <= auxAlu(5 downto 0);

      if auxAlu == 0 then
        eq <= '1';
      else 
        eq <= '0';
      end if;

      if auxAlu < 0 then
        lt <= '1';
      else 
        lt <= '0';
      end if;

      if op1 < op2 then
        ltu <= '1';
      else
        ltu <= '0';
      end if;
    end if;
  end process;

end rtl;
  
