# ESE 507 Project 2 - Part 3:  Larger Matrix and Vector Size

#### Files included in this directory:

  - mac_unit.sv : this is our MAC unit modified from Project 1
  - mvm8_part3.sv : this is our top module
  - synrepo3.txt : this is our synthesis report

#### The commands we used:
```sh
$ vlog memory.sv
$ vlog mac_unit.sv
$ vlog mvm8_part3.sv
$ vlog part3_random_tb.sv
$ vsim -sv_seed (some number) tbench3 -c -do "run -all"
$ dc_shell -f runsynth.tcl | tee synrepo3.txt
```
