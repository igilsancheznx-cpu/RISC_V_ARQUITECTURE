library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity DecExe is
    generic (
        REG_WIDTH  : integer := 32; 
        ADDR_REG   : integer := 5;
        ADDR_WIDTH : integer := 17;
        ALU_DECOD  : integer := 4;
        -- Arquitectura de la unidad de division RISCV_Div:
        --   0 = serie, 1 = paralela, 2 = segmentada
        DIV_ARCH   : integer := 0
    );
    port (
        clk       : in   std_logic;
        nRst      : in   std_logic;
        rdata_a   : in   std_logic_vector(REG_WIDTH-1 downto 0);
        rdata_b   : in   std_logic_vector(REG_WIDTH-1 downto 0);
        addr_b    : out  std_logic_vector(ADDR_WIDTH-1 downto 0);
        wdata_b   : out  std_logic_vector(REG_WIDTH-1 downto 0);
        we_b      : out  std_logic;
        be_b      : out  std_logic_vector(ALU_DECOD-1 downto 0);
        -- PC Signals bifurcation
        pc_out    : in   std_logic_vector(ADDR_WIDTH - 1 downto 0);
        pc_load   : out  std_logic;
        pc_target : out  std_logic_vector(ADDR_WIDTH - 1 downto 0);
        -- Load en ejecucion: el core debe parar un ciclo (fetch mantiene el PC y
        -- la instruccion siguiente se anula) mientras el dato llega de memoria
        ld_stall  : out  std_logic;
        -- Retencion: la instruccion del decoder NO se ejecuta en este ciclo y el
        -- core debe repetirla en el siguiente (division serie en curso o riesgo
        -- de datos con una division pendiente en la segmentada)
        dec_hold  : out  std_logic
    );
end entity DecExe;

architecture rtl of DecExe is
    -- Read port 1
    signal rs1 : std_logic_vector(ADDR_REG - 1 downto 0);
    signal rdata1 : std_logic_vector(REG_WIDTH - 1 downto 0);
    -- Read port 2
    signal rs2 : std_logic_vector(ADDR_REG - 1 downto 0);
    signal rdata2 : std_logic_vector(REG_WIDTH - 1 downto 0);
    -- Write port
    signal wAlu   : std_logic_vector(REG_WIDTH - 1 downto 0); --ALU
    signal we     : std_logic; -- Write enable
    signal wl     : std_logic; -- Write back from memory
    signal be_l   : std_logic_vector(3 downto 0); -- Byte enable for write back
    signal sign_l : std_logic; -- Load con signo (lb, lh)
    signal rd     : std_logic_vector(ADDR_REG - 1 downto 0); -- Dirección del registro destino
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
    -- Signal auxiliar for branches
    signal dec_pc_load : std_logic;  -- pc_load del decoder (jal, jalr y branch sin condición)
    signal br_taken    : std_logic;  -- Condición del branch cumplida (según funct3 y flags de la ALU)
    -- Dato a escribir en rd (ALU, PC + 4 o PC + imm)
    signal pc_ext      : unsigned(REG_WIDTH - 1 downto 0);       -- PC de la instruccion extendido a 32 bits
    signal wreg_data   : std_logic_vector(REG_WIDTH - 1 downto 0);
    -- Acceso a memoria de datos
    signal byte_off    : natural range 0 to 3;                      -- desplazamiento de byte (addr(1:0))
    -- Write-back de los loads (un ciclo despues de presentar la direccion)
    signal ld_wl_r     : std_logic;                                 -- hay que escribir el dato del load
    signal ld_rd_r     : std_logic_vector(ADDR_REG - 1 downto 0);   -- registro destino del load
    signal ld_be_r     : std_logic_vector(3 downto 0);              -- bytes validos (lb/lh/lw)
    signal ld_sign_r   : std_logic;                                 -- extension de signo (lb/lh)
    signal ld_off_r    : natural range 0 to 3;                      -- desplazamiento de byte del load
    signal ld_data     : std_logic_vector(REG_WIDTH - 1 downto 0);  -- dato de memoria alineado al byte 0
    -- Puerto de escritura del banco de registros
    signal rf_waddr    : std_logic_vector(ADDR_REG - 1 downto 0);
    signal rf_we       : std_logic;

    -- =========================================================================
    -- Unidad de division
    -- =========================================================================
    constant DIV_SERIE      : integer := 0;
    constant DIV_PARALELA   : integer := 1;
    constant DIV_SEGMENTADA : integer := 2;
    constant DIV_NINGUNA    : integer := 3;   -- sistema sin unidad de division
    constant NDIV           : integer := 32;   -- iteraciones (etapas de la segmentada)

    signal div_op     : std_logic;   -- instruccion div/divu/rem/remu en el decoder
    signal div_signed : std_logic;   -- div / rem (con signo)
    signal div_rem    : std_logic;   -- rem / remu (se escribe el resto)
    signal d_sa, d_sb : std_logic;   -- signo de dividendo y divisor (solo div / rem)
    signal d_bzero    : std_logic;   -- divisor = 0
    signal d_negq     : std_logic;   -- hay que cambiar el signo del cociente
    signal d_negr     : std_logic;   -- hay que cambiar el signo del resto (= signo del dividendo)
    signal d_a, d_b   : std_logic_vector(REG_WIDTH - 1 downto 0);  -- operandos (valor absoluto en div / rem)
    signal div_valid  : std_logic;
    signal d_ready    : std_logic;
    signal d_done     : std_logic;
    signal d_q, d_r   : std_logic_vector(REG_WIDTH - 1 downto 0);
    signal hold       : std_logic;   -- retencion de la instruccion del decoder
    -- Escritura del resultado (segundo puerto del banco de registros)
    signal wdiv_en    : std_logic;
    signal wdiv_addr  : std_logic_vector(ADDR_REG - 1 downto 0);
    signal wdiv_data  : std_logic_vector(REG_WIDTH - 1 downto 0);
    -- Serie: division lanzada, a la espera de Done
    signal s_issued   : std_logic;
    signal s_rd       : std_logic_vector(ADDR_REG - 1 downto 0);
    signal s_rem, s_negq, s_negr : std_logic;
    -- Segmentada: etiquetas que avanzan junto al pipeline del divisor
    type rd_array is array (1 to NDIV) of std_logic_vector(ADDR_REG - 1 downto 0);
    signal tag_v      : std_logic_vector(1 to NDIV);   -- etapa ocupada por una division
    signal tag_rd     : rd_array;                      -- registro destino
    signal tag_rem, tag_negq, tag_negr : std_logic_vector(1 to NDIV);
    signal pend       : std_logic_vector(2**ADDR_REG - 1 downto 0);  -- registros pendientes de una division
    signal hazard     : std_logic;   -- la instruccion del decoder usa un registro pendiente

    -- Resultado final: cociente o resto con el signo corregido
    function div_result(q, r : std_logic_vector; is_rem, neg_q, neg_r : std_logic)
      return std_logic_vector is
      variable v : unsigned(q'length - 1 downto 0);
    begin
      if is_rem = '1' then
        v := unsigned(r);
        if neg_r = '1' then v := 0 - v; end if;
      else
        v := unsigned(q);
        if neg_q = '1' then v := 0 - v; end if;
      end if;
      return std_logic_vector(v);
    end function;

begin

    -- =========================================================================
    -- MEMORIA DE DATOS (puerto B)
    -- La memoria es sincrona: la direccion se presenta en el ciclo de ejecucion
    -- (combinacional) y la memoria la muestrea en el flanco que cierra el ciclo.
    --   store : la memoria escribe en ese mismo flanco
    --   load  : el dato esta en rdata_b durante el ciclo siguiente
    -- Los accesos de byte / media palabra se colocan en su carril segun addr(1:0)
    -- =========================================================================
    byte_off <= to_integer(unsigned(wAlu(1 downto 0))) when not is_x(wAlu(1 downto 0)) else 0;

    addr_b  <= wAlu(ADDR_WIDTH - 1 downto 0);
    wdata_b <= std_logic_vector(shift_left(unsigned(rdata2), 8 * byte_off));
    we_b    <= aux_we_b and not hold;      -- una instruccion retenida no escribe en memoria
    be_b    <= std_logic_vector(shift_left(unsigned(aux_be_b), byte_off));

    -- Control del write-back de los loads: se registra en el flanco en el que la
    -- memoria muestrea la direccion y se escribe en rd en el flanco siguiente
    process(clk, nRst)
    begin
      if nRst = '0' then
        ld_wl_r   <= '0';
        ld_rd_r   <= (others => '0');
        ld_be_r   <= (others => '0');
        ld_sign_r <= '0';
        ld_off_r  <= 0;
      elsif rising_edge(clk) then
        ld_wl_r   <= wl and not hold;
        ld_rd_r   <= rd;
        ld_be_r   <= be_l;
        ld_sign_r <= sign_l;
        ld_off_r  <= byte_off;
      end if;
    end process;

    -- Dato leido alineado al byte 0 (lb/lh/lbu/lhu en direcciones no multiplo de 4)
    ld_data <= std_logic_vector(shift_right(unsigned(rdata_b), 8 * ld_off_r));

    -- Parada: el load ocupa un ciclo mas mientras llega el dato
    ld_stall <= wl and not hold;

    -- Puerto de escritura del banco de registros:
    --   ciclo de write-back de un load -> rd del load con el dato de memoria
    --   resto                          -> rd de la instruccion en ejecucion (ALU / PC)
    -- Durante el write-back del load el decoder tiene una burbuja (NOP), cuya
    -- escritura se inhibe
    rf_waddr <= ld_rd_r when ld_wl_r = '1' else rd;
    rf_we    <= we and not ld_wl_r and not hold;

    auxOp2 <= rdata2 when ena_imm = '0' else imm;

    -- Dato que se escribe en rd (pc_out es el PC de la instruccion en decodificacion):
    --   jal / jalr : PC + 4 (direccion de retorno)
    --   auipc      : PC + imm
    --   resto      : salida de la ALU
    pc_ext    <= resize(unsigned(pc_out), REG_WIDTH);
    wreg_data <= std_logic_vector(pc_ext + 4)               when rdata_a(6 downto 2) = "11011" or rdata_a(6 downto 2) = "11001" else
                 std_logic_vector(pc_ext + unsigned(imm))   when rdata_a(6 downto 2) = "00101" else
                 wAlu;

    -- Calcular la dirección de salto (pc_load controla la carga del PC)
    --   jalr       : rs1 + imm (calculado en la ALU = wAlu), con el bit 0 a '0' (especificación RV32I)
    --   jal/branch : PC + imm
    pc_target <= wAlu(ADDR_WIDTH - 1 downto 1) & '0' when rdata_a(6 downto 2) = "11001" else
                 std_logic_vector(unsigned(pc_out) + resize(unsigned(imm), ADDR_WIDTH));

    -- Condición de los branches (funct3 = rdata_a(14 downto 12)). En un branch la ALU
    -- compara rs1 con rs2 (ena_imm = '0') y genera los flags eq, lt y ltu
    with rdata_a(14 downto 12) select
      br_taken <= eq      when "000",  -- beq
                  not eq  when "001",  -- bne
                  lt      when "100",  -- blt
                  not lt  when "101",  -- bge
                  ltu     when "110",  -- bltu
                  not ltu when "111",  -- bgeu
                  '0'     when others;

    -- Branch: solo se salta si se cumple la condición | jal / jalr: salto incondicional (decoder)
    -- (una instruccion retenida no salta)
    pc_load <= (br_taken and not hold) when rdata_a(6 downto 2) = "11000" else (dec_pc_load and not hold);

    dec_hold <= hold;

    -- =========================================================================
    -- UNIDAD DE DIVISION (div, divu, rem, remu)
    -- div / rem: se pasan a RISCV_Div los valores absolutos y despues se corrige
    -- el signo: cociente negativo si los signos difieren (salvo division por 0,
    -- que da -1), resto con el signo del dividendo. Casos especiales RV32M:
    --   x / 0          -> cociente 0xFFFFFFFF, resto x
    --   -2^31 / -1     -> cociente -2^31, resto 0 (|-2^31| = 0x80000000 sin signo)
    -- =========================================================================
    div_signed <= not rdata_a(12);                 -- funct3 = 100 (div) / 110 (rem)
    div_rem    <= rdata_a(13);                     -- funct3 = 11x
    d_sa       <= div_signed and rdata1(REG_WIDTH - 1);
    d_sb       <= div_signed and rdata2(REG_WIDTH - 1);
    d_bzero    <= '1' when unsigned(rdata2) = 0 else '0';
    d_negq     <= (d_sa xor d_sb) and not d_bzero;
    d_negr     <= d_sa;
    d_a        <= std_logic_vector(0 - unsigned(rdata1)) when d_sa = '1' else rdata1;
    d_b        <= std_logic_vector(0 - unsigned(rdata2)) when d_sb = '1' else rdata2;

    -- -------------------------------------------------------------------------
    -- SERIE: la division se lanza (Valid) y la instruccion queda retenida en el
    -- decoder, con el procesador parado, hasta que se activa Done (N ciclos, o
    -- 1 si RISCV_Div reutiliza el resultado de la division anterior)
    -- -------------------------------------------------------------------------
    gen_serie: if DIV_ARCH = DIV_SERIE generate
      u_div: entity work.RISCV_Div(serie)
        port map (Clk => clk, Dividendo => d_a, Divisor => d_b, Valid => div_valid,
                  Cociente => d_q, Resto => d_r, Ready => d_ready, Done => d_done);

      div_valid <= div_op and not s_issued;

      process(clk, nRst)
      begin
        if nRst = '0' then
          s_issued <= '0';
          s_rd     <= (others => '0');
          s_rem    <= '0';
          s_negq   <= '0';
          s_negr   <= '0';
        elsif rising_edge(clk) then
          if div_op = '1' and s_issued = '0' and d_ready = '1' then
            s_issued <= '1';                       -- RISCV_Div acepta la division
            s_rd     <= rd;
            s_rem    <= div_rem;
            s_negq   <= d_negq;
            s_negr   <= d_negr;
          elsif s_issued = '1' and d_done = '1' then
            s_issued <= '0';                       -- resultado escrito
          end if;
        end if;
      end process;

      wdiv_en   <= s_issued and d_done;
      wdiv_addr <= s_rd;
      wdiv_data <= div_result(d_q, d_r, s_rem, s_negq, s_negr);
      hold      <= div_op and not (s_issued and d_done);
      hazard    <= '0';
      tag_v     <= (others => '0');
    end generate;

    -- -------------------------------------------------------------------------
    -- PARALELA: combinacional, el resultado se escribe en el mismo ciclo
    -- -------------------------------------------------------------------------
    gen_paralela: if DIV_ARCH = DIV_PARALELA generate
      u_div: entity work.RISCV_Div(paralela)
        port map (Clk => clk, Dividendo => d_a, Divisor => d_b, Valid => div_valid,
                  Cociente => d_q, Resto => d_r, Ready => d_ready, Done => d_done);

      div_valid <= div_op;
      wdiv_en   <= div_op and d_ready and d_done;
      wdiv_addr <= rd;
      wdiv_data <= div_result(d_q, d_r, div_rem, d_negq, d_negr);
      hold      <= '0';
      hazard    <= '0';
      tag_v     <= (others => '0');
    end generate;

    -- -------------------------------------------------------------------------
    -- SEGMENTADA: se lanza una division por ciclo sin parar el procesador. Las
    -- etiquetas (rd, tipo y signos) avanzan junto al pipeline del divisor y
    -- marcan los registros pendientes. Una instruccion que lee un registro
    -- pendiente (RAW) o lo sobrescribe (WAW, salvo otra division, que termina en
    -- orden) queda retenida hasta que la division escribe el resultado.
    -- -------------------------------------------------------------------------
    gen_segmentada: if DIV_ARCH = DIV_SEGMENTADA generate
      u_div: entity work.RISCV_Div(segmentada)
        port map (Clk => clk, Dividendo => d_a, Divisor => d_b, Valid => div_valid,
                  Cociente => d_q, Resto => d_r, Ready => d_ready, Done => d_done);

      div_valid <= div_op and d_ready and not hazard;

      process(clk, nRst)
      begin
        if nRst = '0' then
          tag_v    <= (others => '0');
          tag_rd   <= (others => (others => '0'));
          tag_rem  <= (others => '0');
          tag_negq <= (others => '0');
          tag_negr <= (others => '0');
        elsif rising_edge(clk) then
          tag_v(1)    <= div_valid;
          tag_rd(1)   <= rd;
          tag_rem(1)  <= div_rem;
          tag_negq(1) <= d_negq;
          tag_negr(1) <= d_negr;
          for i in 2 to NDIV loop
            tag_v(i)    <= tag_v(i-1);
            tag_rd(i)   <= tag_rd(i-1);
            tag_rem(i)  <= tag_rem(i-1);
            tag_negq(i) <= tag_negq(i-1);
            tag_negr(i) <= tag_negr(i-1);
          end loop;
        end if;
      end process;

      -- Registros pendientes de actualizacion por una division
      process(tag_v, tag_rd)
        variable p : std_logic_vector(2**ADDR_REG - 1 downto 0);
      begin
        p := (others => '0');
        for i in 1 to NDIV loop
          if tag_v(i) = '1' and not is_x(tag_rd(i)) then
            p(to_integer(unsigned(tag_rd(i)))) := '1';
          end if;
        end loop;
        p(0) := '0';
        pend <= p;
      end process;

      hazard <= '1' when not is_x(rs1) and not is_x(rs2) and not is_x(rd) and
                         (pend(to_integer(unsigned(rs1))) = '1' or
                          pend(to_integer(unsigned(rs2))) = '1' or
                          ((we = '1' or wl = '1') and div_op = '0' and pend(to_integer(unsigned(rd))) = '1'))
                else '0';

      wdiv_en   <= d_done and tag_v(NDIV);
      wdiv_addr <= tag_rd(NDIV);
      wdiv_data <= div_result(d_q, d_r, tag_rem(NDIV), tag_negq(NDIV), tag_negr(NDIV));
      hold      <= hazard;
    end generate;

    -- -------------------------------------------------------------------------
    -- SIN DIVISOR (DIV_ARCH = 3): no se instancia RISCV_Div. Las instrucciones
    -- div/divu/rem/remu no escriben ningun registro (se ejecutan como un NOP)
    -- -------------------------------------------------------------------------
    gen_sin_div: if DIV_ARCH = DIV_NINGUNA generate
      div_valid <= '0';
      d_ready   <= '1';
      d_done    <= '0';
      d_q       <= (others => '0');
      d_r       <= (others => '0');
      wdiv_en   <= '0';
      wdiv_addr <= (others => '0');
      wdiv_data <= (others => '0');
      hold      <= '0';
      hazard    <= '0';
      tag_v     <= (others => '0');
    end generate;

    U0: entity work.regFile(rtl)
    generic map(
        REG_WIDTH => REG_WIDTH,
        ADDR_REG  => ADDR_REG
    )
    port map(
        clk       => clk,
        nRst      => nRst,
        raddr1    => rs1,
        rdata1    => rdata1,
        raddr2    => rs2,
        rdata2    => rdata2,
        waddr     => rf_waddr,
        we        => rf_we,
        wl        => ld_wl_r,      -- el dato del load se escribe un ciclo despues
        be_l      => ld_be_r,
        sign_l    => ld_sign_r,
        wreg      => wreg_data,
        wdata     => ld_data,
        we2       => wdiv_en,      -- resultado de la division
        waddr2    => wdiv_addr,
        wdata2    => wdiv_data
    );

    -- DECODER
    U1: entity work.decoder(rtl)
    generic map(
        REG_WIDTH => REG_WIDTH,
        ALU_DECOD => ALU_DECOD,
        ADDR_WIDTH => ADDR_WIDTH
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
        be_b      => aux_be_b,
        pc_load   => dec_pc_load,
        div_op    => div_op
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