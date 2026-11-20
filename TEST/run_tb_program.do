# Compila y simula tb_program (ejecutar desde la raiz del proyecto: do TEST/run_tb_program.do)
# Para usar las copias auxiliares (HDL_AUX/core_aux.vhd) anadir +define+USE_CORE_AUX al vlog
vlib work
vcom -2008 -explicit HDL/alu.vhd
vcom -2002 -explicit HDL/divisor.vhd HDL/regFile.vhd HDL/decoder.vhd HDL/fetch.vhd HDL/DecExe.vhd HDL/core.vhd
vlog -sv TEST/mem_agent.sv TEST/tb_program.sv
vsim -voptargs=+acc work.tb_program
run -all
