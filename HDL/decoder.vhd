library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity decoder is
    generic (
        REG_WIDTH : integer := 32; 
        NUM_REGS  : integer := 5;
        ALU_DECOD : integer := 8
    );
    port (
        clk       : in  std_logic;
        nRst      : in  std_logic;
        instr     : in  std_logic_vector(REG_WIDTH - 1 downto 0);
        rs1       : out std_logic_vector(NUM_REGS - 1 downto 0);
        rs2       : out std_logic_vector(NUM_REGS - 1 downto 0);
        rd        : out std_logic_vector(NUM_REGS - 1 downto 0);
        imm       : out std_logic_vector(REG_WIDTH - 1 downto 0);
        aloup     : out std_logic_vector(NUM_REGS - 1 downto 0)
    );
end entity decoder;

architecture rtl of decoder is
    signal opcode    : std_logic_vector(4 downto 0);
    
begin

    opcode <= instr(6 downto 2);

   process(opcode, instr)
     variable funct3 : std_logic_vector(2 downto 0);
     variable funct7 : std_logic;
   begin
     if nRst = '0' then
         rs1    <= (others => '0');
         rs2    <= (others => '0');
         rd     <= (others => '0');
         imm    <= (others => '0');
         aloup  <= (others => '0');

      elsif rising_edge(clk) then
        case opcode is
         when "01100" => -- R-type
           funct3 := instr(14 downto 12);
           funct7 := instr(30);
           aloup  <= funct7 & funct3;  -- Cogemos el bit(5) porque es el único que cambia y asi no implemetamos ninguna lógica
           rs1    <= instr(19 downto 15);
           rs2    <= instr(24 downto 20);
           rd     <= instr(11 downto 7);
           imm    <= (others => '0');

         when "00100" => -- I-type
           funct3 := instr(14 downto 12);
           aloup  <= instr(30) & funct3;  -- Cogemos el bit 30 para selecionar funct3 = 0x05 y ambas combinaciones 
           rs1    <= instr(19 downto 15);
           rs2    <= (others => '0');
           rd     <= instr(11 downto 7);
           imm    <= (31 downto 11 => instr(31)) & instr(30 downto 20);

         when "00000" => -- I-type LOAD
           funct3 := instr(14 downto 12);
           rs1    <= instr(19 downto 15);
           rd     <= instr(11 downto 7);
           imm    <= (31 downto 11 => instr(31)) & instr(30 downto 20);
           rs2    <= (others => '0');
           aloup  <= (others => '0');

         when "01000" => -- S-type STORE
           rs1    <= instr(19 downto 15);
           rs2    <= instr(24 downto 20); 
           imm    <= (31 downto 11 => instr(31)) & instr(30 downto 25) & instr(11 downto 7);
           rd     <= (others => '0');
           aloup  <= (others => '0');

         when "11000" => -- B-type BRANCH
           rs1    <= instr(19 downto 15);
           rs2    <= instr(24 downto 20); 
           imm    <= (31 downto 12 => instr(31)) & instr(7) & instr(30 downto 25) & instr(11 downto 8) & '0';
           rd     <= (others => '0');
           aloup  <= (others => '0');

         when "11011" => -- J-type jal
           rd     <= instr(11 downto 7);
           imm    <= (31 downto 20 => instr(31)) & instr(19 downto 12) & instr(20) & instr(30 downto 21) & '0';
           rs1    <= (others => '0');
           rs2    <= (others => '0');
           aloup  <= (others => '0');
           
         when "11001" => -- I-type jalr
           rs1    <= instr(19 downto 15);
           rs2    <= (others => '0');
           rd     <= instr(11 downto 7);
           imm    <= (31 downto 11 => instr(31)) & instr(30 downto 20);
           aloup  <= (others => '0');
           
         when "01101" => -- U-type lui
           rd     <= instr(11 downto 7);
           imm    <= instr(31 downto 13) & (12 downto 0 => instr(12)); 
           rs1    <= (others => '0');
           rs2    <= (others => '0');
           aloup  <= (others => '0');

         when "00101" => -- U-type auipc
           rd     <= instr(11 downto 7);
           imm    <= instr(31 downto 13) & (12 downto 0 => instr(12)); 
           rs1    <= (others => '0');
           rs2    <= (others => '0');
           aloup  <= (others => '0');

         when others =>
           rs1    <= (others => '0');
           rs2    <= (others => '0');
           rd     <= (others => '0');
           imm    <= (others => '0');
           aloup  <= (others => '0');

        end case;
      end if;
    end process;
end architecture rtl;


    