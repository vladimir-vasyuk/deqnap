#**************************************************************
# Time Information
#**************************************************************

set_time_format -unit ns -decimal_places 3

#**************************************************************
# Create Clock
#**************************************************************

derive_clock_uncertainty
create_clock -name {clk50} -period 20 [get_ports {clk50}] -add
#create_clock -name {e_rxc125} -period 8 [get_ports {e_rxc}]
create_clock -name {e_rxc25} -period 40 [get_ports {e_rxc}] -add
#create_clock -name {e_rxc2.5} -period 400 [get_ports {e_rxc}] -add
create_clock -name {e_txc25} -period 40 [get_ports {e_txc}]
#create_clock -name {e_txc2.5} -period 400 [get_ports {e_txc}] -add

#**************************************************************
# Create Generated Clock
#**************************************************************

#derive_pll_clocks

create_generated_clock -name mdc_clk -source [get_pins {pll1|altpll_component|auto_generated|pll1|clk[0]}] -divide_by 22 [get_registers {topboard22:kernel|deqnap:deqnam|mdc_clk:mclk|mdcclk}]
#create_generated_clock -name mdc_clk -source [get_ports {clk50}] -divide_by 22 [get_registers {topboard22:kernel|deqnap:deqnam|mdc_clk:mclk|mdcclk}]
create_generated_clock -name ddout25 -source [get_ports {e_txc}] -master_clock e_txc25 -divide_by 2 [get_registers {topboard22:kernel|deqnap:deqnam|ether:etherm|ddout:dd_out|txclk_reg}]
#create_generated_clock -name ddout2.5 -source [get_ports {e_txc}] -master_clock e_txc2.5 -divide_by 2 [get_registers {topboard22:kernel|deqnap:deqnam|ether:etherm|ddout:dd_out|txclk_reg}] -add
create_generated_clock -name ddin25 -source [get_ports {e_rxc}] -master_clock e_rxc25 -divide_by 2 [get_registers {topboard22:kernel|deqnap:deqnam|ether:etherm|ddin:dd_in|rxclk_reg}]
#create_generated_clock -name ddin2.5 -source [get_ports {e_rxc}] -master_clock e_rxc2.5 -divide_by 2 [get_registers {topboard22:kernel|deqnap:deqnam|ether:etherm|ddin:dd_in|rxclk_reg}] -add

#**************************************************************
# Set Clock Latency
#**************************************************************



#**************************************************************
# Set Clock Uncertainty
#**************************************************************

set_clock_uncertainty -rise_from [get_clocks {e_rxc25}] -rise_to [get_clocks {e_rxc25}] 0.020
set_clock_uncertainty -rise_from [get_clocks {e_rxc25}] -fall_to [get_clocks {e_rxc25}] 0.020

set_clock_uncertainty -rise_from [get_clocks {e_txc25}] -rise_to [get_clocks {e_txc25}] 0.020
set_clock_uncertainty -rise_from [get_clocks {e_txc25}] -fall_to [get_clocks {e_txc25}] 0.020

set_clock_uncertainty -rise_from [get_clocks {e_rxc25}] -rise_to [get_clocks {ddin25}] 0.030
set_clock_uncertainty -rise_from [get_clocks {e_rxc25}] -fall_to [get_clocks {ddin25}] 0.030
set_clock_uncertainty -fall_from [get_clocks {e_rxc25}] -rise_to [get_clocks {ddin25}] 0.030
set_clock_uncertainty -fall_from [get_clocks {e_rxc25}] -fall_to [get_clocks {ddin25}] 0.030

set_clock_uncertainty -rise_from [get_clocks {e_txc25}] -rise_to [get_clocks {ddout25}] 0.030
set_clock_uncertainty -rise_from [get_clocks {e_txc25}] -fall_to [get_clocks {ddout25}] 0.030
set_clock_uncertainty -fall_from [get_clocks {e_txc25}] -rise_to [get_clocks {ddout25}] 0.030
set_clock_uncertainty -fall_from [get_clocks {e_txc25}] -fall_to [get_clocks {ddout25}] 0.030

set_clock_uncertainty -rise_from [get_clocks {ddin25}] -rise_to [get_clocks {ddin25}] 0.040
set_clock_uncertainty -rise_from [get_clocks {ddin25}] -fall_to [get_clocks {ddin25}] 0.040
set_clock_uncertainty -rise_from [get_clocks {ddout25}] -rise_to [get_clocks {ddout25}] 0.040
set_clock_uncertainty -rise_from [get_clocks {ddout25}] -fall_to [get_clocks {ddout25}] 0.040

#set_clock_uncertainty -rise_from [get_clocks {ddin2.5}] -rise_to [get_clocks {ddin2.5}] 0.200
#set_clock_uncertainty -rise_from [get_clocks {ddin2.5}] -fall_to [get_clocks {ddin2.5}] 0.200

#set_clock_uncertainty -rise_from [get_clocks {ddout2.5}] -rise_to [get_clocks {ddout2.5}] 0.200
#set_clock_uncertainty -rise_from [get_clocks {ddout2.5}] -fall_to [get_clocks {ddout2.5}] 0.200

#set_clock_uncertainty -rise_from [get_clocks {e_rxc25}] -rise_to [get_clocks {ddin2.5}] 0.200
#set_clock_uncertainty -rise_from [get_clocks {e_rxc25}] -fall_to [get_clocks {ddin2.5}] 0.200
#set_clock_uncertainty -fall_from [get_clocks {e_rxc25}] -rise_to [get_clocks {ddin2.5}] 0.200
#set_clock_uncertainty -fall_from [get_clocks {e_rxc25}] -fall_to [get_clocks {ddin2.5}] 0.200

#set_clock_uncertainty -rise_from [get_clocks {e_txc25}] -rise_to [get_clocks {ddout2.5}] 0.200
#set_clock_uncertainty -rise_from [get_clocks {e_txc25}] -fall_to [get_clocks {ddout2.5}] 0.200
#set_clock_uncertainty -fall_from [get_clocks {e_txc25}] -rise_to [get_clocks {ddout2.5}] 0.200
#set_clock_uncertainty -fall_from [get_clocks {e_txc25}] -fall_to [get_clocks {ddout2.5}] 0.200

#**************************************************************
# Set Input Delay
#**************************************************************



#**************************************************************
# Set Output Delay
#**************************************************************



#**************************************************************
# Set Clock Groups
#**************************************************************

#set_clock_groups -asynchronous -group [get_clocks {altera_reserved_tck}]
#set_clock_groups -physically_exclusive -group [get_clocks {pll1|altpll_component|auto_generated|pll1|clk[0]}] \
	-group [get_clocks {ddin2.5 ddin25 ddout2.5 ddout25 e_rxc2.5 e_rxc25 e_rxc125 e_txc2.5 e_txc25}]
set_clock_groups -physically_exclusive -group [get_clocks {pll1|altpll_component|auto_generated|pll1|clk[0]}] \
	-group [get_clocks {ddin25 ddout25 e_rxc25 e_txc25}]

#set_clock_groups -exclusive -group [get_clocks {e_rxc2.5 e_rxc25 e_rxc125 ddin2.5 ddin25}] \
	-group [get_clocks {e_txc2.5 e_txc25 ddout2.5 ddout25}]
set_clock_groups -exclusive -group [get_clocks {e_rxc25 ddin25}] \
	-group [get_clocks {e_txc25 ddout25}]

#set_clock_groups -exclusive -group e_rxc125 -group {e_rxc25 e_rxc2.5}
#set_clock_groups -exclusive -group e_rxc125 -group {ddin2.5 ddin25}
#set_clock_groups -exclusive -group e_rxc25 -group {ddin2.5 e_rxc2.5}
#set_clock_groups -exclusive -group e_rxc2.5 -group ddin25
#set_clock_groups -exclusive -group ddin25 -group ddin2.5
#set_clock_groups -exclusive -group ddout25 -group ddout2.5 
#set_clock_groups -exclusive -group e_txc25 -group {e_txc2.5 ddout2.5}
#set_clock_groups -exclusive -group e_txc2.5 -group ddout25
#set_clock_groups -exclusive -group [get_clocks {e_txc2.5}] -group [get_clocks {e_txc25}]
#set_clock_groups -physically_exclusive -group mdc_clk -group {e_rxc125 e_rxc25 e_rxc2.5}
set_clock_groups -physically_exclusive -group {mdc_clk} -group {e_rxc25}
set_clock_groups -physically_exclusive -group {mdc_clk} -group {ddin25}

#**************************************************************
# Set False Path
#**************************************************************



#**************************************************************
# Set Multicycle Path
#**************************************************************



#**************************************************************
# Set Maximum Delay
#**************************************************************



#**************************************************************
# Set Minimum Delay
#**************************************************************



#**************************************************************
# Set Input Transition
#**************************************************************

