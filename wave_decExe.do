onerror {resume}
quietly WaveActivateNextPane {} 0
add wave -noupdate -expand -group REG_FILE /tb_DecExe/dut/U0/raddr1
add wave -noupdate -expand -group REG_FILE /tb_DecExe/dut/U0/rdata1
add wave -noupdate -expand -group REG_FILE /tb_DecExe/dut/U0/raddr2
add wave -noupdate -expand -group REG_FILE /tb_DecExe/dut/U0/rdata2
add wave -noupdate -expand -group REG_FILE /tb_DecExe/dut/U0/waddr
add wave -noupdate -expand -group REG_FILE /tb_DecExe/dut/U0/wreg
add wave -noupdate -expand -group REG_FILE /tb_DecExe/dut/U0/we
add wave -noupdate -expand -group REG_FILE /tb_DecExe/dut/U0/wb
add wave -noupdate -expand -group REG_FILE /tb_DecExe/dut/U0/wdata
add wave -noupdate -expand -group REG_FILE /tb_DecExe/dut/U0/regs
add wave -noupdate -expand -group DECODER /tb_DecExe/dut/U1/instr
add wave -noupdate -expand -group DECODER /tb_DecExe/dut/U1/rs1
add wave -noupdate -expand -group DECODER /tb_DecExe/dut/U1/rs2
add wave -noupdate -expand -group DECODER /tb_DecExe/dut/U1/rd
add wave -noupdate -expand -group DECODER /tb_DecExe/dut/U1/imm
add wave -noupdate -expand -group DECODER /tb_DecExe/dut/U1/ena_imm
add wave -noupdate -expand -group DECODER /tb_DecExe/dut/U1/aloup
add wave -noupdate -expand -group DECODER /tb_DecExe/dut/U1/we_b
add wave -noupdate -expand -group DECODER /tb_DecExe/dut/U1/be_b
add wave -noupdate -expand -group DECODER /tb_DecExe/dut/U1/opcode
add wave -noupdate -expand -group ALU /tb_DecExe/dut/U2/op1
add wave -noupdate -expand -group ALU /tb_DecExe/dut/U2/op2
add wave -noupdate -expand -group ALU /tb_DecExe/dut/U2/aloup
add wave -noupdate -expand -group ALU /tb_DecExe/dut/U2/eq
add wave -noupdate -expand -group ALU /tb_DecExe/dut/U2/lt
add wave -noupdate -expand -group ALU /tb_DecExe/dut/U2/ltu
add wave -noupdate -expand -group ALU /tb_DecExe/dut/U2/aluOut
add wave -noupdate -group DEC_EXE_IO /tb_DecExe/dut/clk
add wave -noupdate -group DEC_EXE_IO /tb_DecExe/dut/nRst
add wave -noupdate -group DEC_EXE_IO /tb_DecExe/dut/rdata_a
add wave -noupdate -group DEC_EXE_IO /tb_DecExe/dut/rdata_b
add wave -noupdate -group DEC_EXE_IO /tb_DecExe/dut/addr_b
add wave -noupdate -group DEC_EXE_IO /tb_DecExe/dut/wdata_b
add wave -noupdate -group DEC_EXE_IO /tb_DecExe/dut/we_b
add wave -noupdate -group DEC_EXE_IO /tb_DecExe/dut/be_b
add wave -noupdate -group DEC_EXE_IO /tb_DecExe/dut/rs1
add wave -noupdate -group DEC_EXE_IO /tb_DecExe/dut/rdata1
add wave -noupdate -group DEC_EXE_IO /tb_DecExe/dut/rs2
add wave -noupdate -group DEC_EXE_IO /tb_DecExe/dut/rdata2
add wave -noupdate -group DEC_EXE_IO /tb_DecExe/dut/wAlu
add wave -noupdate -group DEC_EXE_IO /tb_DecExe/dut/we
add wave -noupdate -group DEC_EXE_IO /tb_DecExe/dut/wb
add wave -noupdate -group DEC_EXE_IO /tb_DecExe/dut/rd
add wave -noupdate -group DEC_EXE_IO /tb_DecExe/dut/imm
add wave -noupdate -group DEC_EXE_IO /tb_DecExe/dut/aloup
add wave -noupdate -group DEC_EXE_IO /tb_DecExe/dut/eq
add wave -noupdate -group DEC_EXE_IO /tb_DecExe/dut/lt
add wave -noupdate -group DEC_EXE_IO /tb_DecExe/dut/ltu
add wave -noupdate -group DEC_EXE_IO /tb_DecExe/dut/aux_addr_b
add wave -noupdate -group DEC_EXE_IO /tb_DecExe/dut/aux_wdata_b
add wave -noupdate -group DEC_EXE_IO /tb_DecExe/dut/aux_rdata_b
add wave -noupdate -group DEC_EXE_IO /tb_DecExe/dut/aux_we_b
add wave -noupdate -group DEC_EXE_IO /tb_DecExe/dut/aux_be_b
TreeUpdate [SetDefaultTree]
WaveRestoreCursors {{Cursor 1} {50408 ps} 0}
quietly wave cursor active 1
configure wave -namecolwidth 150
configure wave -valuecolwidth 100
configure wave -justifyvalue left
configure wave -signalnamewidth 1
configure wave -snapdistance 10
configure wave -datasetprefix 0
configure wave -rowmargin 4
configure wave -childrowmargin 2
configure wave -gridoffset 0
configure wave -gridperiod 1
configure wave -griddelta 40
configure wave -timeline 0
configure wave -timelineunits ps
update
WaveRestoreZoom {0 ps} {132300 ps}
