#**************************************************************
# Time Information
#**************************************************************
set_time_format -unit ns -decimal_places 3

#**************************************************************
# Create Clock
#**************************************************************
create_clock -name {clk50}   -period 20.000 [get_ports {clk50}]
create_clock -name {e_rxc25} -period 40.000 [get_ports {e_rxc}] -add
create_clock -name {e_txc25} -period 40.000 [get_ports {e_txc}]

#**************************************************************
# Create Generated Clock
#**************************************************************
create_generated_clock -name mdc_clk \
    -source    [get_pins {pll1|altpll_component|auto_generated|pll1|clk[0]}] \
    -divide_by 22 \
    [get_registers {topboard22:kernel|deqnap:deqnam|mdc:mdclk|mdcclk}]

create_generated_clock -name ddout25 \
    -source       [get_ports {e_txc}] \
    -master_clock e_txc25 \
    -divide_by    2 \
    [get_registers {topboard22:kernel|deqnap:deqnam|ether:etherm|ddout:dd_out|txclk_reg}]

create_generated_clock -name ddin25 \
    -source       [get_ports {e_rxc}] \
    -master_clock e_rxc25 \
    -divide_by    2 \
    [get_registers {topboard22:kernel|deqnap:deqnam|ether:etherm|ddin:dd_in|rxclk_reg}]

#**************************************************************
# Set Clock Uncertainty
#**************************************************************

# PLL-based clocks — low jitter
set_clock_uncertainty -rise_from [get_clocks {clk50}] \
                      -rise_to   [get_clocks {clk50}] 0.100
set_clock_uncertainty -rise_from [get_clocks {pll1|altpll_component|auto_generated|pll1|clk[0]}] \
                      -rise_to   [get_clocks {pll1|altpll_component|auto_generated|pll1|clk[0]}] 0.050

# External clocks on non-GCLK I/O pin — conservative values
set_clock_uncertainty -rise_from [get_clocks {e_rxc25}] -rise_to [get_clocks {e_rxc25}] 0.200
set_clock_uncertainty -rise_from [get_clocks {e_rxc25}] -fall_to [get_clocks {e_rxc25}] 0.200
set_clock_uncertainty -rise_from [get_clocks {e_txc25}] -rise_to [get_clocks {e_txc25}] 0.200
set_clock_uncertainty -rise_from [get_clocks {e_txc25}] -fall_to [get_clocks {e_txc25}] 0.200

# Generated clocks from I/O pin — higher uncertainty due to fabric routing
set_clock_uncertainty -rise_from [get_clocks {e_rxc25}] -rise_to [get_clocks {ddin25}]  0.300
set_clock_uncertainty -rise_from [get_clocks {e_rxc25}] -fall_to [get_clocks {ddin25}]  0.300
set_clock_uncertainty -fall_from [get_clocks {e_rxc25}] -rise_to [get_clocks {ddin25}]  0.300
set_clock_uncertainty -fall_from [get_clocks {e_rxc25}] -fall_to [get_clocks {ddin25}]  0.300
set_clock_uncertainty -rise_from [get_clocks {e_txc25}] -rise_to [get_clocks {ddout25}] 0.300
set_clock_uncertainty -rise_from [get_clocks {e_txc25}] -fall_to [get_clocks {ddout25}] 0.300
set_clock_uncertainty -fall_from [get_clocks {e_txc25}] -rise_to [get_clocks {ddout25}] 0.300
set_clock_uncertainty -fall_from [get_clocks {e_txc25}] -fall_to [get_clocks {ddout25}] 0.300
set_clock_uncertainty -rise_from [get_clocks {ddin25}]  -rise_to [get_clocks {ddin25}]  0.300
set_clock_uncertainty -rise_from [get_clocks {ddin25}]  -fall_to [get_clocks {ddin25}]  0.300
set_clock_uncertainty -rise_from [get_clocks {ddout25}] -rise_to [get_clocks {ddout25}] 0.300
set_clock_uncertainty -rise_from [get_clocks {ddout25}] -fall_to [get_clocks {ddout25}] 0.300

#**************************************************************
# Set Clock Groups
#**************************************************************

# PLL domain vs Ethernet domain — asynchronous
set_clock_groups -asynchronous \
    -group [get_clocks {pll1|altpll_component|auto_generated|pll1|clk[0]}] \
    -group [get_clocks {e_rxc25 ddin25 e_txc25 ddout25}]

# RX domain vs TX domain — asynchronous (full duplex)
set_clock_groups -asynchronous \
    -group [get_clocks {e_rxc25}] \
    -group [get_clocks {ddin25}]

set_clock_groups -asynchronous \
    -group [get_clocks {e_rxc25 ddin25}] \
    -group [get_clocks {e_txc25 ddout25}]

# MDC vs Ethernet domains — asynchronous
set_clock_groups -asynchronous \
    -group [get_clocks {mdc_clk}] \
    -group [get_clocks {e_rxc25 ddin25 e_txc25 ddout25}]

#**************************************************************
# Set False Path — CDC synchronizer crossings only
#**************************************************************

set_false_path -from [get_registers {*wptr_gray*}] \
               -to   [get_registers {*g_wptr_sync1*}]
set_false_path -from [get_registers {*rptr_gray*}] \
               -to   [get_registers {*g_rptr_sync1*}]

#**************************************************************
# Set Input Delay  (adjust to RTL8211EG datasheet values)
#**************************************************************

set_input_delay -clock e_rxc25 -max 2.000 [get_ports {e_rxd[*] e_rxdv e_rxer}]
set_input_delay -clock e_rxc25 -min 0.000 [get_ports {e_rxd[*] e_rxdv e_rxer}]

#**************************************************************
# Set Output Delay  (adjust to RTL8211EG datasheet values)
#**************************************************************

set_output_delay -clock e_txc25 -max 2.000 [get_ports {e_txd[*] e_txen e_txer}]
set_output_delay -clock e_txc25 -min 0.000 [get_ports {e_txd[*] e_txen e_txer}]
