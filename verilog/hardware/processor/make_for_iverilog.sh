#!/usr/bin/env bash

set -e

cd "source/$1/"
make
make install

cd ../..
cp programs/data.hex verilog/
cp programs/program.hex verilog/

cd sail-core/verilog
echo `pwd`

verilator -DSIMULATION --lint-only --clk clk -cc clk_gate.v adder.v cpu.v mux2to1.v alu_control.v pipeline_registers.v alu.v program_counter.v branch_decide.v forwarding_unit.v branch_predictor.v imm_gen.v control_unit.v instruction_mem.v data_mem.v register_file.v CSR.v dataMem_mask_gen.v ../../toplevel.v --exe main.cpp
iverilog -DSIMULATION -DIVERILOG -o ../../iv.out clk_gate.v SB_HFOCSC.v adder.v cpu.v mux2to1.v alu_control.v pipeline_registers.v alu.v program_counter.v branch_decide.v forwarding_unit.v branch_predictor.v imm_gen.v control_unit.v instruction_mem.v data_mem.v register_file.v CSR.v dataMem_mask_gen.v ../../toplevel.v

echo "Output iv.out created"
