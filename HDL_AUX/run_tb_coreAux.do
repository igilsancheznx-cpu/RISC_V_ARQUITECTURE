# Compila y simula tb_coreAux (ejecutar desde la raiz del proyecto: do HDL_AUX/run_tb_coreAux.do)
vlib work
vcom -2008 -explicit HDL/alu.vhd
vcom -2002 -explicit HDL/regFile.vhd HDL/decoder.vhd HDL/fetch.vhd HDL/memory.vhd
vcom -2002 -explicit HDL_AUX/DecExe_aux.vhd HDL_AUX/core_aux.vhd
vlog -sv TEST/mem_agent.sv TEST/tb_coreAux.sv
vsim -voptargs=+acc work.tb_coreAux
run -all
