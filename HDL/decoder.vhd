library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity decoder is
    generic (
        REG_WIDTH  : integer := 32; 
        ALU_DECOD  : integer := 4;
        ADDR_WIDTH : integer := 17
    );
    port (
        clk       : in  std_logic;
        nRst      : in  std_logic;
        instr     : in  std_logic_vector(REG_WIDTH - 1 downto 0);
        rs1       : out std_logic_vector(ALU_DECOD downto 0);
        rs2       : out std_logic_vector(ALU_DECOD downto 0);
        rd        : out std_logic_vector(ALU_DECOD downto 0);
        imm       : out std_logic_vector(REG_WIDTH - 1 downto 0);
        ena_imm   : out std_logic;                                 -- Multiplexor de la ALU para seleccionar entre rs2 o imm
        we        : out std_logic;                                 -- Habilita la escritura en el banco de registros desde la etapa de Execution (para ALU)
        wl        : out std_logic;                                 -- Habilita la escritura en el banco de registros desde la etapa de Write-Back (para LOAD)
        be_l      : out std_logic_vector(ALU_DECOD - 1 downto 0);  -- Byte enable for write back
        sign_l    : out std_logic;                                 -- Load con signo (lb, lh) -> funct3(2) = '0'
        aloup     : out std_logic_vector(ALU_DECOD - 1 downto 0);
        -- ENABLE FOR MEMORY
        we_b      : out std_logic;
        be_b      : out std_logic_vector(ALU_DECOD - 1 downto 0);
        -- PC Signals bifurcation
        pc_load   : out  std_logic;
        -- Extension M: div, divu, rem, remu (opcode R, funct7 = 0000001, funct3(2) = '1')
        div_op    : out  std_logic
    );
end entity decoder;

architecture rtl of decoder is
    signal opcode    : std_logic_vector(4 downto 0);
    
begin

  opcode <= instr(6 downto 2);

  -- Las instrucciones de la extension M (funct7(0) = instr(25) = '1') no escriben
  -- el resultado de la ALU: la division lo escribe por su propio puerto al terminar
  we <= '1' when ((opcode = "01100" and instr(25) = '0') or opcode = "00100" or opcode = "11001" or opcode = "01101" or opcode = "00101" or opcode = "11011")
            else '0';

  div_op <= '1' when (opcode = "01100" and instr(25) = '1' and instr(14) = '1') else '0';
  
  ena_imm <= '1' when (opcode = "00100" or opcode = "00000" or opcode = "01000" or opcode = "11001" or opcode = "01101" or opcode = "00101") 
            else '0';

  we_b <= '1' when (opcode = "01000") else '0';

  wl      <= '1' when (opcode = "00000") else '0';

  sign_l  <= '1' when (opcode = "00000" and instr(14) = '0') else '0';  -- lb, lh (funct3(2) = '0') | lbu, lhu (funct3(2) = '1')

  -- Activar la señal pc_load para las instrucciones de salto (jal, jalr, branch)
  pc_load <= '1' when (opcode = "11011" or opcode = "11000" or opcode = "11001") else '0';

  process(opcode, instr)
    variable funct3 : std_logic_vector(2 downto 0);
    variable funct7 : std_logic;
  begin
    rs1     <= (others => '0');
    rs2     <= (others => '0');
    rd      <= (others => '0');
    imm     <= (others => '0');
    aloup   <= (others => '0');
    be_b    <= (others => '0');
    be_l    <= (others => '0');

    funct3  := instr(14 downto 12);
    funct7  := instr(30);

    case opcode is
      when "01100" => -- R-type
        aloup   <= funct7 & funct3;  -- Cogemos el bit(5) porque es el unico que cambia y asi no implemetamos ninguna l�gica
        rs1     <= instr(19 downto 15);
        rs2     <= instr(24 downto 20);
        rd      <= instr(11 downto 7);
        imm     <= (others => '0');

      when "00100" => -- I-type

        if funct3 = "101" then  -- Shift right logical o arithmetic
          aloup <= funct7 & funct3;           -- Cogemos el bit 30 para selecionar funct3 = 0x05 y ambas combinaciones 
        else
          aloup <= '0' & funct3;  -- Cogemos el bit(5) porque es el unico que cambia y asi no implemetamos ninguna l�gica
        end if;
        rs1     <= instr(19 downto 15);
        rs2     <= (others => '0');
        rd      <= instr(11 downto 7);
        imm     <= (31 downto 11 => instr(31)) & instr(30 downto 20);

      when "00000" => -- I-type LOAD
        rs1     <= instr(19 downto 15);
        rd      <= instr(11 downto 7);
        imm     <= (31 downto 11 => instr(31)) & instr(30 downto 20);
        rs2     <= (others => '0');      
        aloup   <= (others => '0');  -- La ALU suma rs1 + imm conseguir la direccion

        -- Senales de control de escritura en memoria        
        if    funct3 = "000" or funct3 = "100" then be_l <= "0001";
        elsif funct3 = "001" or funct3 = "101" then be_l <= "0011";
        elsif funct3 = "010"                   then be_l <= "1111";
        else  be_l <= "0000";
        end if;

      when "01000" => -- S-type STORE
        rs1     <= instr(19 downto 15);
        rs2     <= instr(24 downto 20); 
        imm     <= (31 downto 11 => instr(31)) & instr(30 downto 25) & instr(11 downto 7);
        rd      <= (others => '0');
        aloup   <= (others => '0');

        if    funct3 = "000" then be_b <= "0001";
        elsif funct3 = "001" then be_b <= "0011";
        elsif funct3 = "010" then be_b <= "1111";
        else  be_b <= "0000";
        end if;

        
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
        imm    <= instr(31 downto 12) & (11 downto 0 => '0'); 
        rs1    <= (others => '0');
        rs2    <= (others => '0');
        aloup  <= (others => '0');

      when "00101" => -- U-type auipc
        rd     <= instr(11 downto 7);
        imm    <= instr(31 downto 12) & (11 downto 0 => '0'); 
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
  end process;

end architecture rtl;


    