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
    signal wl     : std_logic; -- Write back from memory
    signal be_l   : std_logic_vector(3 downto 0); -- Byte enable for write back
    signal sign_l : std_logic; -- Load con signo (lb, lh)
    signal rd     : std_logic_vector(ADDR_REGS - 1 downto 0); -- Dirección del registro destino
    -- Signal decoder instruction
    signal imm    : std_logic_vector(REG_WIDTH - 1 downto 0);
    signal aloup  : std_logic_vector(ALU_DECOD - 1 downto 0);
    signal eq     : std_logic;
    signal lt     : std_logic;
    signal ltu    : std_logic;
    -- Multiplexor para ALU
    signal ena_imm : std_logic;  -- Multiplexor de la ALU para seleccionar entre rs2 o imm
    signal auxOp2 : std_logic_vector(REG_WIDTH - 1 downto 0);  -- Señal auxiliar para multiplexor de la ALU
    -- Signal auxiliar for memory
    signal aux_addr_b  : std_logic_vector(ADDR_WIDTH-1 downto 0);  
    signal aux_wdata_b : std_logic_vector(REG_WIDTH-1 downto 0); 
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
        addr_b  <= wAlu(ADDR_WIDTH - 1 downto 0);
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

    auxOp2 <= rdata2 when ena_imm = '0' else imm;

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
        wl        => wl,
        be_l      => be_l,
        sign_l    => sign_l,
        wreg      => wAlu,
        wdata     => rdata_b
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
        imm       => imm,
        we        => we,
        wl        => wl,
        be_l      => be_l,
        sign_l    => sign_l,
        ena_imm   => ena_imm,
        we_b      => aux_we_b,
        be_b      => aux_be_b
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
      op2 => auxOp2,
      aloup => aloup,
      eq => eq,
      lt => lt,
      ltu => ltu,
      aluOut => wAlu
    );

end architecture rtl;