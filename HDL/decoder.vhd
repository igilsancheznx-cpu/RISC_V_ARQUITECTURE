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
    signal funct3    : std_logic_vector(2 downto 0);
    signal funct7    : std_logic_vector(6 downto 0);
    
begin

    op <= instr(6 downto 2);

--    rs1 <= instr(19 downto 15) when op = "01100" or op = "00100" or op = "00000" or op = "01000" else
--           (others => '0');
--    rs2 <= instr(24 downto 20) when instr(6 downto 2) = "01100" or instr(6 downto 2) = "00000" or instr(6 downto 2) = "01000" else
--           (others => '0');
--    rd <= instr(11 downto 7) when instr(6 downto 2) = "01100" or instr(6 downto 2) = "00100" or instr(6 downto 2) = "11000" or instr(6 downto 2) = "11011" or instr(6 downto 2) = "01101" or instr(6 downto 2) = "00101" else
--           --     (others => '0');


   process(clk, nRst)
   begin
     if nRst = '0' then
         rs1    <= (others => '0');
         rs2    <= (others => '0');
         rd     <= (others => '0');
      elsif rising_edge(clk) then
        case opcode is
         when "01100" => -- R-type
           funct3 <= instr(14 downto 12);
           funct7 <= instr(31 downto 25);
           aloup  <= funct3 & funct7(5);  -- Cogemos el bit(5) porque es el único que cambia y asi no implemetamos ninguna lógica
           rs1    <= instr(19 downto 15);
           rs2    <= instr(24 downto 20);
           rd     <= instr(11 downto 7);
           imm    <= (others => '0');

         when "00100" => -- I-type
           funct3 <= instr(14 downto 12);
           aloup  <= funct3 & imm(10);  -- Cogemos el bit 10 porque es el que cambia    REVISAR 
           rs1    <= instr(19 downto 15);
           rs2    <= instr(24 downto 20);
           rd     <= instr(11 downto 7);
           imm    <= instr(31 downto 25);
         when "00000" => -- LOAD
           funct3 <= instr(14 downto 12);
         when "01000" => -- STORE
         when "11000" => -- BRANCH
         when "11011" => -- jal
         when "11001" => -- jalr
         when "01101" => -- lui
         when "00101" => --auipc
         when others =>
          -- rs1
           if op = "01100" or op = "00100" or op = "00000" or op = "01000" then
               rs1 <= instr(19 downto 15);
           else
               rs1 <= (others => '0');
           end if;
            -- rs2
           if op = "01100" or op = "00000" or op = "01000" then
               rs2 <= instr(24 downto 20);
           else
               rs2 <= (others => '0');
            end if;

            -- rd
            if op = "01100" or op = "00100" or op = "11000" or op = "11011" or op = "01101" or op = "00101" then
                rd <= instr(11 downto 7);
            else
                rd <= (others => '0');
            end if;

            -- funct3
            if op = "01100" or op = "00100" or op = "00000" or op = "01000" then
                funct3 <= instr(14 downto 12);
            else
                funct3 <= (others => '0');
            end if;

            -- funct7
            if op = "01100" then
                funct7 <= instr(31 downto 25);
            else
                funct7 <= (others => '0');
            end if;

        end if;
    end process;

    aloup <= "0000" when 
             "0001"
             "1000"
             "1100"
             "1110"
             "0010"
             "1011"
             "1010"
             "0100"
             "0110"
    

end architecture rtl;

--Solo en vivado 2008 o eso dice
--Architecture rtl of decoder is
--begin
--    process(clk, nRst)
  --  begin
    --    if nRst = '0' then
      --      opcode <= (others => '0');
        --    rs1 <= (others => '0');
          --  rs2 <= (others => '0');
            --rd <= (others => '0');
 --           funct3 <= (others => '0');
   --         funct7 <= (others => '0');
     --       imm <= (others => '0');
       -- elsif rising_edge(clk) then
         --   rs1 <= instr(19 downto 15) when instr(6 downto 2) = "01100" or instr(6 downto 2) = "00100" or instr(6 downto 2) = "00000" or instr(6 downto 2) = "01000" else
          --      (others => '0');
            --rs2 <= instr(24 downto 20) when instr(6 downto 2) = "01100" or instr(6 downto 2) = "00000" or instr(6 downto 2) = "01000" else
              --  (others => '0');
           -- rd <= instr(11 downto 7) when instr(6 downto 2) = "01100" or instr(6 downto 2) = "00100" or instr(6 downto 2) = "11000" or instr(6 downto 2) = "11011" or instr(6 downto 2) = "01101" or instr(6 downto 2) = "00101" else
           --     (others => '0');
           -- funct3 <= instr(14 downto 12) when instr(6 downto 2) = "01100" or instr(6 downto 2) = "00100" or instr(6 downto 2) = "00000" or instr(6 downto 2) = "01000" else
           --     (others => '0');
           -- funct7 <= instr(31 downto 25) when instr(6 downto 2) = "01100" else
            --    (others => '0');
       -- end if;
   -- end process;
    
    