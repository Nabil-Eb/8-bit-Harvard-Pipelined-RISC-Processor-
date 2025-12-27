vlib work
vlog -f src_files.list
vsim -voptargs=+acc work.processor_tb
add wave -position insertpoint  \
sim:/processor_tb/clk \
sim:/processor_tb/reset \
sim:/processor_tb/interrupt \
sim:/processor_tb/in_port \
sim:/processor_tb/out_port
add wave -position insertpoint  \
sim:/processor_tb/uut/imem_inst/memory
add wave -position insertpoint  \
sim:/processor_tb/uut/regfile_inst/registers
add wave -position insertpoint  \
sim:/processor_tb/uut/ccr_out
add wave -position insertpoint  \
sim:/processor_tb/uut/memory_data
add wave -position insertpoint  \
sim:/processor_tb/uut/alu_result
add wave -position insertpoint  \
sim:/processor_tb/uut/MEMWB_data \
sim:/processor_tb/uut/pc_out
run -all
wave zoom full