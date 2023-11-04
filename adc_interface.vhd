----------------------------------------------------------------------------------
-- Company: 
-- Engineer: 
-- 
-- Create Date:    20:00:12 11/14/2020 
-- Design Name: 
-- Module Name:    adc_interface - Behavioral 
-- Project Name: 
-- Target Devices: 
-- Tool versions: 
-- Description: 
--
-- Dependencies: 
--
-- Revision: 
-- Revision 0.01 - File Created
-- Additional Comments: 
--
----------------------------------------------------------------------------------
library IEEE;
use IEEE.STD_LOGIC_1164.ALL;

-- Uncomment the following library declaration if using
-- arithmetic functions with Signed or Unsigned values
use IEEE.NUMERIC_STD.ALL;

-- Uncomment the following library declaration if instantiating
-- any Xilinx primitives in this code.
--library UNISIM;
--use UNISIM.VComponents.all;

entity adc_interface is
    Port ( clk_i : in  STD_LOGIC;
	        reset_i : in STD_LOGIC;
			  miso_i : in STD_LOGIC;
			  mosi_o : out STD_LOGIC;
			  sclk_o : out STD_LOGIC;
			  ncs_o : out STD_LOGIC;
			  data_o : out STD_LOGIC_VECTOR(15 downto 0));
end adc_interface;

architecture Behavioral of adc_interface is

	COMPONENT spi_master_v1
	GENERIC ( 
		width : integer range 0 to 64 := 16; -- data width
		cpha : STD_LOGIC := '1';  -- 1st or 2nd clk edge MISO reading
		cpol : STD_LOGIC := '1'); -- low or high default CLOCK level
			  
	PORT(
		clk_i : IN std_logic;
		clk_en_i : IN std_logic;
		reset_i : IN std_logic;
		data_i : IN std_logic_vector(15 downto 0);
		miso_i : IN std_logic;          
		mosi_o : OUT std_logic;
		sclk_o : OUT std_logic;
		ncs_o : OUT std_logic;
		data_o : OUT std_logic_vector(15 downto 0);
		rdy_o : OUT std_logic
		);
	END COMPONENT;
	
type spi_state_t is (RESET, CONVERSION);

signal sigSPI_St : spi_state_t := RESET;
signal sigSPI_Data_q : STD_LOGIC_VECTOR(15 downto 0) := (others => '0');
signal sigSPI_Data_d : STD_LOGIC_VECTOR(15 downto 0) := (others => '0');
signal sigSPI_Rdy : STD_LOGIC := '0';
signal sigSPI_RST : STD_LOGIC := '0';
signal sEn44kHz : STD_LOGIC := '1';

--constant cDivider : integer := 90; //16MHz/90 = ~44kHz
constant cDivider : integer := 0;

begin
	

en44kHz : process(clk_i, reset_i)

variable vCount : integer range 0 to 127 := 0;

begin
	if reset_i = '1' then
		vCount := 0;
		sEn44kHz <= '0';
	elsif falling_edge(clk_i) then
		if vCount = cDivider then
			sEn44kHz <= '1';
			vCount := 0;
		else
			sEn44kHz <= '0';
			vCount := vCount + 1;
		end if;
	end if;
end process; 

	control_spi : process(clk_i, reset_i)
	begin
		if reset_i = '1' then
			sigSPI_St <= RESET;
			sigSPI_Data_q <= (others => '0');
		elsif rising_edge(clk_i) then
			if sigSPI_St = RESET then
				sigSPI_RST <= '0';
				sigSPI_St <= CONVERSION;
			elsif sigSPI_St = CONVERSION and sigSPI_Rdy = '1' and sEn44kHz = '1' then
				sigSPI_Data_q <= std_logic_vector(shift_right(unsigned(sigSPI_Data_d), 1));
				sigSPI_RST <= '1';
				sigSPI_St <= RESET;
			end if;
		end if;
	end process;
	  
	Inst_spi_master_v1_2: spi_master_v1
		GENERIC MAP(
		width => 16,
		cpha => '1',
		cpol => '1'
		)
		PORT MAP(
		clk_i => clk_i,
		clk_en_i => '1',
		reset_i => sigSPI_RST,
		data_i => (others => '0'),
		miso_i => miso_i,
		mosi_o => mosi_o,
		sclk_o => sclk_o,
		ncs_o => ncs_o,
		data_o => sigSPI_Data_d,
		rdy_o => sigSPI_Rdy
	);
	data_o <= std_logic_vector(X"0000" + unsigned(sigSPI_Data_q(10 downto 0)));

end Behavioral;

