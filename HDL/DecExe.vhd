library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity DecExe is
    generic (
        REG_WIDTH  : integer := 32; 
        ADDR_REGS  : integer := 5;
        ADDR_WIDTH : integer := 17;
        ALU_DECOD  : integer := 4
    );
    port (
        clk       : in   std_logic;
        nRst      : in   std_logic;
        rdata_a   : in   std_logic_vector(REG_WIDTH-1 downto 0);
        rdata_b   : in   std_logic_vector(REG_WIDTH-1 downto 0);
        addr_b    : out  std_logic_vector(ADDR_WIDTH-1 downto 0);
        wdata_b   : out  std_logic_vector(REG_WIDTH-1 downto 0);
        we_b      : out  std_logic;
        be_b      : out  std_logic_vector(ALU_DECOD-1 downto 0)
    );
end entity DecExe;

architecture rtl of DecExe is
    -- Read port 1
    signal rs1 : std_logic_vector(ADDR_REGS - 1 downto 0);
    signal rdata1 : std_logic_vector(REG_WIDTH - 1 downto 0);
    -- Read port 2
    signal rs2 : std_logic_vector(ADDR_REGS - 1 downto 0);
    signal rdata2 : std_logic_vector(REG_WIDTH - 1 downto 0);
    -- Write port
    signal wAlu   : std_logic_vector(REG_WIDTH - 1 downto 0); --ALU
    signal we     : std_logic; -- Write enable
    signal wb     : std_logic; -- Write back from memory
    signal rd     : std_logic_vector(ADDR_REGS - 1 downto 0); -- Register to write back  ELIMINAR RD o waddr Unificar !!!
    -- Signal decoder instruction
    signal imm    : std_logic_vector(REG_WIDTH - 1 downto 0);
    signal aloup  : std_logic_vector(ALU_DECOD - 1 downto 0);
    signal eq     : std_logic;
    signal lt     : std_logic;
    signal ltu    : std_logic;
    -- Signal auxiliar for memory
    signal aux_addr_b  : std_logic_vector(ADDR_WIDTH-1 downto 0);  
    signal aux_wdata_b : std_logic_vector(REG_WIDTH-1 downto 0); 
    signal aux_rdata_b : std_logic_vector(REG_WIDTH-1 downto 0);
    signal aux_we_b    : std_logic;  
    signal aux_be_b    : std_logic_vector(ALU_DECOD-1 downto 0);

begin

  process(clk, nRst)
  begin
    if nRst = '0' then
      addr_b  <= (others => '0');   
      wdata_b <= (others => '0');    
      we_b    <= '0';  
      be_b    <= (others => '0');

    elsif clk'event and clk = '1' then
      if aux_we_b = '1' then
        addr_b  <= wAlu;
        wdata_b <= rdata2;
        we_b    <= aux_we_b;
        be_b    <= aux_be_b;
      else
        addr_b  <= (others => '0');   
        wdata_b <= (others => '0');    
        we_b    <= '0';  
        be_b    <= (others => '0');

      end if;
    end if;
  end process;


    U0: entity work.regFile(rtl)
    generic map(
        REG_WIDTH => REG_WIDTH,
        ADDR_REGS => ADDR_REGS
    )
    port map(
        clk       => clk,
        nRst      => nRst,
        raddr1    => rs1,
        rdata1    => rdata1,
        raddr2    => rs2,
        rdata2    => rdata2,
        waddr     => rd,
        we        => we,
        wb        => wb,
        wreg      => wAlu,
        wdata     => aux_rdata_b
    );

    -- DECODER
    U1: entity work.decoder(rtl)
    generic map(
        REG_WIDTH => REG_WIDTH,
        ALU_DECOD => ALU_DECOD
    )
    port map(
        clk       => clk,
        nRst      => nRst,
        instr     => rdata_a,
        aloup     => aloup,
        rs1       => rs1,
        rs2       => rs2,
        rd        => rd,
        imm       => imm       -- Determinar tamano, si hay que extenderlo y por cuantos multiplexores tiene que pasar, y se utiliza para los saltos PC_Target
    );

    -- Disenar un multiplexor para elegir imm o rdata2 depende de la aloup (INSTR I)
    
    -- ALU
    U2: entity work.alu(rtl)
    generic map(
      ALU_LENGTH => REG_WIDTH,
      ALU_DECOD  => ALU_DECOD
    )
    port map(
      op1 => rdata1,
      op2 => rdata2,
      aloup => aloup,
      eq => eq,
      lt => lt,
      ltu => ltu,
      aluOut => wAlu
    );

end architecture rtl;