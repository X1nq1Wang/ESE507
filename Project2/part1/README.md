# ESE 507 Project 2 - Part 1:  4x4 Matrix-Vector Multiplier 

#### Files included in this directory:

  - mac_unit.sv : this is our MAC unit modified from Project 1
  - mvm4_part1.sv : this is our top module
  - synrepo1.txt : this is our synthesis report

#### The commands we used:
```sh
$ vlog memory.sv
$ vlog mac_unit.sv
$ vlog mvm4_part1.sv
$ vlog part1_random_tb.sv
$ vsim -sv_seed #(some number) tbench1 -c -do "run -all"
$ dc_shell -f runsynth.tcl | tee synrepo1.txt
```
