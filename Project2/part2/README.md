# ESE 507 Project 2 - Part 2:  Using One Matrix Multiple Times 

#### Files included in this directory:

  - mac_unit.sv : this is our MAC unit modified from Project 1
  - mvm4_part2.sv : this is our top module
  - synrepo2.txt : this is our synthesis report

#### The commands we used:
```sh
$ vlog memory.sv
$ vlog mac_unit.sv
$ vlog mvm4_part2.sv
$ vlog part2_random_tb.sv
$ vsim -sv_seed (some number) tbench2 -c -do "run -all"
$ dc_shell -f runsynth.tcl | tee synrepo2.txt
```
