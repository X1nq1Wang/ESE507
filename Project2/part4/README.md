# ESE 507 Project 2 - Part 4:  Delay-Optimized System 

#### Files included in this directory:

  - mac_unit_part4.sv : this is our MAC unit modified from Project 1 and using a 3-stage pipelined multiplier
  - mvm8_part4.sv : this is our top module
  - synrepo4.txt : this is our synthesis report

#### The commands we used:
```sh
$ vlog memory.sv
$ vlog mac_unit_part4.sv
$ vlog mvm8_part4.sv
$ vlog part3_random_tb.sv
$ vsim -sv_seed (some number) tbench3 -c -do "run -all"
$ dc_shell -f runsynth.tcl | tee synrepo4.txt
```
