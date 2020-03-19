set_global_assignment -name VERILOG_MACRO "CORENAME=\"JTBTIGER\""
set_global_assignment -name VERILOG_MACRO "GAMETOP=jtbtiger_game"
set_global_assignment -name VERILOG_MACRO "JT12=1"
set_global_assignment -name VERILOG_MACRO "AVATARS=1"
set_global_assignment -name VERILOG_MACRO "VIDEO_WIDTH=256"
set_global_assignment -name VERILOG_MACRO "VIDEO_HEIGHT=224"
# 6 MHz used in MCU to avoid timing errors
set_global_assignment -name VERILOG_MACRO "JTFRAME_CLK6"

# OSD options
set_global_assignment -name VERILOG_MACRO "HAS_TESTMODE=1"
set_global_assignment -name VERILOG_MACRO "JOIN_JOYSTICKS=1"

# Verilog version of Z80 IP
set_global_assignment -name VERILOG_MACRO "TV80S=1"

set_global_assignment -name VERILOG_FILE ../../../modules/jt12/jt49/hdl/filter/jt49_dcrm2.v
set_global_assignment -name VERILOG_FILE ../../../modules/jt12/hdl/mixer/jt12_mixer.v
 
