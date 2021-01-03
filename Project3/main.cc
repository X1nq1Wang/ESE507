// ESE 507 Project 3 Handout Code
// Fall 2020
// You may not redistribute this code

/*
ESE 507 Project3
by Jinkyu Lee, Xinqi Wang
modified on 11/20/20
*/

#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <cstdlib>
#include <cstring>
#include <assert.h>
#include <math.h>
using namespace std;

void printUsage();
void genFCLayer(int M, int N, int T, int R, int P, vector<int>& constvector, string modName, ofstream &os);
void genNetwork(int N, int M1, int M2, int M3, int T, int R, int B, vector<int>& constVector, string modName, ofstream &os);
void readConstants(ifstream &constStream, vector<int>& constvector);
void genROM(vector<int>& constVector, int bits, string modName, ofstream &os);
void genMem(ofstream& os);
void genMAC(ofstream& os, int T);

int main(int argc, char* argv[]) {

   // If the user runs the program without enough parameters, print a helpful message
   // and quit.
   if (argc < 7) {
      printUsage();
      return 1;
   }

   int mode = atoi(argv[1]);

   ifstream const_file; //ifstream: Stream class to read from files.  
   ofstream os;//ofstream: Stream class to write on files
   vector<int> constVector;

   //----------------------------------------------------------------------
   // Look here for Part 1 and 2
   if (((mode == 1) && (argc == 7)) || ((mode == 2) && (argc == 8))) {//check the mode and argc

      // Mode 1/2: Generate one layer with given dimensions and one testbench

      // --------------- read parameters, etc. ---------------
      int M = atoi(argv[2]);//atoi is to convert string to integer
      int N = atoi(argv[3]);//get MNTR parameters
      int T = atoi(argv[4]);
      int R = atoi(argv[5]);
 
      int P;//initialize P

      if (mode == 1) {//if mode is 1
         P=1;//then P is 1
         const_file.open(argv[6]);//pass argv 6         
      }
      else {
         P = atoi(argv[6]);//otherwise P is argv 6
         const_file.open(argv[7]);//pass argv 7
      }

      if (const_file.is_open() != true) {//if const_file isn't open then let the user know.
		  cout << "ERROR reading constant file " << argv[6] << endl;
         return 1;
      }

      // Read the constants out of the provided file and place them in the constVector vector
      readConstants(const_file, constVector);

      string out_file = "fc_" + to_string(M) + "_" + to_string(N) + "_" + to_string(T) + "_" + to_string(R) + "_" + to_string(P) + ".sv";

      os.open(out_file);
      if (os.is_open() != true) {
		  cout << "ERROR opening " << out_file << " for write." << endl;
         return 1;
      }
      // -------------------------------------------------------------

      // call the genFCLayer function you will write to generate this layer
      string modName = "fc_" + to_string(M) + "_" + to_string(N) + "_" + to_string(T) + "_" + to_string(R) + "_" + to_string(P);
      genFCLayer(M, N, T, R, P, constVector, modName, os);
      genMAC(os, T);
      genMem(os);

   }
   //--------------------------------------------------------------------


   // ----------------------------------------------------------------
   // Look here for Part 3
   else if ((mode == 3) && (argc == 10)) {
      // Mode 3: Generate three layers interconnected 

      // --------------- read parameters, etc. ---------------
      int N  = atoi(argv[2]);
      int M1 = atoi(argv[3]);
      int M2 = atoi(argv[4]);
      int M3 = atoi(argv[5]);
      int T  = atoi(argv[6]);
      int R  = atoi(argv[7]);
      int B  = atoi(argv[8]);

      const_file.open(argv[9]);
      if (const_file.is_open() != true) {
		  cout << "ERROR reading constant file " << argv[8] << endl;
         return 1;
      }
      readConstants(const_file, constVector);

      string out_file = "net_" + to_string(N) + "_" + to_string(M1) + "_" + to_string(M2) + "_" + to_string(M3) + "_" + to_string(T) + "_" + to_string(R) + "_" + to_string(B)+ ".sv";


      os.open(out_file);
      if (os.is_open() != true) {
		  cout << "ERROR opening " << out_file << " for write." << endl;
         return 1;
      }
      // -------------------------------------------------------------

      string mod_name = "net_" + to_string(N) + "_" + to_string(M1) + "_" + to_string(M2) + "_" + to_string(M3) + "_" + to_string(T) + "_" + to_string(R) + "_" + to_string(B);

      // generate the design
      genNetwork(N, M1, M2, M3, T, R, B, constVector, mod_name, os);

   }
   //-------------------------------------------------------

   else {
      printUsage();
      return 1;
   }

   // close the output stream
   os.close();

}

// Read values from the constant file into the vector
void readConstants(ifstream &constStream, vector<int>& constvector) {
   string constLineString;
   while(getline(constStream, constLineString)) {
      int val = atoi(constLineString.c_str());
      constvector.push_back(val);
   }
}

// Generate a ROM based on values constVector.
// Values should each be "bits" number of bits.
void genROM(vector<int>& constVector, int bits, string modName, ofstream &os) {

      int numWords = constVector.size();
      int addrBits = ceil(log2(numWords));

	  os << "module " << modName << "(clk, addr, z);" << endl;
	  os << "   input clk;" << endl;
	  os << "   input [" << addrBits - 1 << ":0] addr;" << endl;
	  os << "   output logic signed [" << bits - 1 << ":0] z;" << endl;
	  os << "   always_ff @(posedge clk) begin" << endl;
	  os << "      case(addr)" << endl;
      int i=0;
      for (vector<int>::iterator it = constVector.begin(); it < constVector.end(); it++, i++) {
         if (*it < 0)
			 os << "        " << i << ": z <= -" << bits << "'d" << abs(*it) << ";" << endl;
         else
			 os << "        " << i << ": z <= " << bits << "'d" << *it << ";" << endl;
      }
	  os << "      endcase" << endl << "   end" << endl << "endmodule" << endl << endl;
}

void genMem(ofstream& os) {
    /* Memory */
    os << "module memory(clk, data_in, data_out, addr, wr_en);" << endl;
    os << "" << endl;;
    os << "    parameter                   WIDTH=16, SIZE=64;" << endl;
    os << "    localparam                  LOGSIZE=$clog2(SIZE);" << endl;
    os << "    input [WIDTH-1:0]           data_in;" << endl;
    os << "    output logic [WIDTH-1:0]    data_out;" << endl;
    os << "    input [LOGSIZE-1:0]         addr;" << endl;
    os << "    input                       clk, wr_en;\n" << endl;
    os << "    logic [SIZE-1:0][WIDTH-1:0] mem;\n" << endl;
    os << "    always_ff @(posedge clk) begin" << endl;
    os << "        data_out <= mem[addr];" << endl;
    os << "        if (wr_en)" << endl;
    os << "            mem[addr] <= data_in;" << endl;
    os << "    end" << endl;
    os << "endmodule\n" << endl;
}

void genMAC(ofstream& os, int T) {

    /* MAC unit*/
    long int MIN = -pow(2, (T - 1));
    long int MAX = pow(2, (T - 1)) - 1;
    os << "module mac(clk, reset, a, b, valid_in, f, valid_out);" << endl;
    os << "    parameter                           WIDTH = " << T << ";" << endl;
    os << "    localparam                          MAX = " << MAX << ";" << endl;
    os << "    localparam                          MIN = " << MIN << ";" << endl;
    os << "    input                               clk, reset, valid_in;" << endl;
    os << "    input signed [WIDTH - 1:0]          a, b;" << endl;
    os << "    output logic signed [WIDTH - 1:0]   f;" << endl;
    os << "    output                              valid_out;\n" << endl;
    os << "    logic signed [WIDTH - 1:0]          a_r, b_r;" << endl;
    os << "    logic signed [WIDTH*2 - 1:0]        product;" << endl;
    os << "    logic signed [WIDTH-1:0]            acc;\n" << endl;
    os << "    logic [2:0]                         shift_delay;" << endl;
    os << "    logic signed [WIDTH-1:0]            product_sat;\n" << endl;
    os << "    always_ff @(posedge clk) begin" << endl;
    os << "          if(reset)" << endl;
    os << "            shift_delay <= 0;" << endl;
    os << "          else" << endl;
    os << "            shift_delay <= {shift_delay[2:0], valid_in};" << endl;
    os << "    end" << endl;
    os << "    assign valid_out = shift_delay[2];\n" << endl;
    os << "    always_comb begin" << endl;
    os << "        if(shift_delay[1]==1)" << endl;
    os << "            acc = f + product_sat;" << endl;
    os << "        else" << endl;
    os << "            acc = 0;" << endl;
    os << "    end" << endl;
    os << "    always_comb begin" << endl;
    os << "        product = a_r*b_r;" << endl;
    os << "    end\n" << endl;
    os << "    always_ff @(posedge clk) begin" << endl;
    os << "        if (product > MAX)" << endl;
    os << "            product_sat <= MAX;" << endl;
    os << "        else if (product < MIN)" << endl;
    os << "            product_sat <= MIN;" << endl;
    os << "        else" << endl;
    os << "            product_sat <= product[WIDTH-1:0];" << endl;
    os << "    end\n" << endl;
    os << "    always_ff @(posedge clk)begin" << endl;
    os << "        if(reset) begin" << endl;
    os << "            a_r <= 0;" << endl;
    os << "            b_r <= 0;" << endl;
    os << "        end" << endl;
    os << "        else if(valid_in)begin" << endl;
    os << "            a_r <= a;" << endl;
    os << "            b_r <= b;" << endl;
    os << "        end" << endl;
    os << "        else begin" << endl;
    os << "            a_r <= a_r;" << endl;
    os << "            b_r <= b_r;" << endl;
    os << "        end" << endl;
    os << "    end\n" << endl;
    os << "    always_ff @ (posedge clk) begin" << endl;
    os << "        if(reset)" << endl;
    os << "            f <= 0;" << endl;
    os << "		else if(shift_delay[1]==1)begin" << endl;
    os << "		      if(f[WIDTH-1]==0 & product_sat[WIDTH-1]==0 & acc[WIDTH-1]==1)   //overflow" << endl;
    os << "	              f <= MAX;" << endl;
    os << "		      else if (f[WIDTH-1]==1 & product_sat[WIDTH-1]==1 & acc[WIDTH-1]==0) //underflow" << endl;
    os << "			      f <= MIN;" << endl;
    os << "		      else" << endl;
    os << "		   	      f <= acc;" << endl;
    os << "        end" << endl;
    os << "        else" << endl;
    os << "            f <= f;" << endl;
    os << "    end" << endl;
    os << "endmodule\n" << endl;
}

// Parts 1 and 2
// Here is where you add your code to produce a neural network layer.
// 11/17 edits: P value are set to 1 for now and should be modified later in Part 2
void genFCLayer(int M, int N, int T, int R, int P, vector<int>& constVector, string modName, ofstream &os) {

	if (P < 1) {
		os << "P value cannot be less than 1. Please provide a resonable integer value bigger than 0" << endl;
		assert(false);
	}
	if (M%P != 0) {
		os << "Only P values such that M divided by P is an integer are allowed." << endl;
		assert(false);
	}


   /* Top Module*/
   os << "module " << modName << "(clk, reset, input_valid, input_ready, input_data, output_valid, output_ready, output_data);" << endl;
   os << "    localparam              WIDTH = " << T << ";" << endl;
   os << "    localparam              W_M = " << M << ";" << endl;
   os << "    localparam              W_N = " << N << ";" << endl;
   os << "    localparam              P = " << P << ";" << endl;
   os << "    localparam              R = " << R << ";" << endl;
   os << "    localparam              LOGSIZE_X = $clog2(W_N);" << endl;
   os << "    localparam              LOGSIZE_W = $clog2(W_M/P * W_N);" << endl;
   os << "    localparam              NUM_OUTPUT = $clog2(P);\n" << endl;
   os << "    input                           clk, reset, input_valid, output_ready;" << endl;
   os << "    input  signed [WIDTH-1:0]       input_data;" << endl;
   os << "    output signed [WIDTH-1:0]       output_data;" << endl;
   os << "    output                          output_valid, input_ready;\n" << endl;
   os << "    logic                           clk, reset, wr_en_x, clear_acc, en_acc;" << endl;
   os << "    logic [LOGSIZE_X-1:0]           addr_x;" << endl;
   os << "    logic [LOGSIZE_W-1:0]           addr_w;" << endl;   
   os << "    logic [NUM_OUTPUT:0]            out_addr;" << endl;
   os << "    logic [1:0]                     delay_ctrl;\n" << endl;
   os << "    " << modName << "_datapath #(WIDTH, W_M, W_N, P, R) dp( " << endl;
   os << "                            .in_data(input_data), .clk(clk), .reset(reset), .clear_acc(clear_acc), .en_acc(en_acc), " << endl;
   os << "                            .addr_x(addr_x), .wr_en_x(wr_en_x), .addr_w(addr_w)," << endl;
   os << "                            .out_addr(out_addr), .delay_ctrl(delay_ctrl)," << endl;
   os << "                            .out_data(output_data));\n" << endl;
   os << "    " << modName << "_control #(WIDTH, W_M, W_N, P) ctrl(" << endl;
   os << "                            .clk(clk), .reset(reset), .in_valid(input_valid), .out_ready(output_ready)," << endl;
   os << "                            .addr_x(addr_x), .wr_en_x(wr_en_x), .addr_w(addr_w)," << endl;
   os << "                            .clear_acc(clear_acc), .en_acc(en_acc), .in_ready(input_ready)," << endl;
   os << "                            .out_valid(output_valid), .out_addr(out_addr)," << endl;
   os << "                            .delay_ctrl(delay_ctrl));" << endl;
   os << "endmodule\n" << endl;

   /* Datapath */
   os << "module " << modName << "_datapath(in_data, clk, reset, " << endl;
   os << "                addr_x, wr_en_x, addr_w," << endl;
   os << "                clear_acc, en_acc, out_data, out_addr, delay_ctrl);" << endl;
   os << "    parameter                           WIDTH = 20, W_M = 5, W_N = 4, P = 1, R = 1;" << endl;
   os << "    localparam                          X_SIZE = W_N;" << endl;
   os << "    localparam                          LOGSIZE_X=$clog2(W_N);" << endl;
   os << "    localparam                          LOGSIZE_W=$clog2(W_M/P * W_N);" << endl;
   os << "    localparam                          NUM_PER_ROM = W_M/P * W_N;" << endl;
   os << "    localparam                          NUM_OUTPUT = $clog2(P);\n" << endl;
   os << "    input                               clk, reset, clear_acc, en_acc;" << endl;
   os << "    input signed    [WIDTH-1:0]         in_data;" << endl;
   os << "    input                               wr_en_x;" << endl;
   os << "    input           [LOGSIZE_X-1:0]     addr_x;" << endl;
   os << "    input           [LOGSIZE_W-1:0]     addr_w;" << endl;
   os << "    input           [NUM_OUTPUT:0]      out_addr;" << endl;
   os << "    input           [1:0]               delay_ctrl;\n" << endl;
   os << "    output signed   [WIDTH-1:0]         out_data;\n" << endl;
   os << "    logic                               acc_clr;" << endl;
   if (P > 1) {
       for (int i = 0; i < P; i++)
           os << "    logic signed    [WIDTH-1:0]         w_in" << i << ";" << endl; // changed
   }
   else {
       os << "    logic signed    [WIDTH-1:0]         w_in;" << endl;// changed
   }
   os << "    logic signed    [WIDTH-1:0]         x_in;" << endl;
   if (P > 1) {
	   for (int i = 0 ; i < P; i++)
		   os << "    logic signed    [WIDTH-1:0]         mac_out" << i << ";" << endl;// changed
   }
   else
       os << "    logic signed    [WIDTH-1:0]         mac_out;" << endl;// changed
   if (P > 1) {
	   for (int i = 0 ; i < P; i++)
		   os << "    logic                               mac_out_valid" << i << ";" << endl;
   }
   else
       os << "    logic                               mac_out_valid;" << endl;// changed
   os << "" << endl;
   os << "    logic signed    [P:0][WIDTH-1:0]     out_v;" << endl;
   os << "    logic signed    [WIDTH-1:0]          out;\n" << endl;

   os << "    memory #(WIDTH, X_SIZE) " << modName << "_vector(.clk(clk), .data_in(in_data), .addr(addr_x), .wr_en(wr_en_x), .data_out(x_in));" << endl;
   
   if (P > 1) {
	   for (int i = 0 ; i < P; i++)
		   os << "    " << modName << "_W_rom" << i << " " << modName << "_w" << i << "(.clk(clk), .addr(addr_w), .z(w_in" << i << "));" << endl; //changed
   }
   else
       os << "    " << modName << "_W_rom " << modName << "_w(.clk(clk), .addr(addr_w), .z(w_in));" << endl; //changed
   if (P > 1)
	   for (int i = 0 ; i < P; i++) {
		   os << "    mac #(WIDTH) " << modName << "_mc" << i << "(.clk(clk), .reset(acc_clr), .a(w_in" << i << "), .b(x_in), .valid_in(en_acc), .f(mac_out" << i << "), .valid_out(mac_out_valid" << i << "));" << endl;//changed
   }
   else
       os << "    mac #(WIDTH) " << modName << "_mc(.clk(clk), .reset(acc_clr), .a(w_in), .b(x_in), .valid_in(en_acc), .f(mac_out), .valid_out(mac_out_valid));" << endl; // changed   
  
   os << "\n    always_comb begin" << endl;
   os << "        acc_clr = reset || clear_acc;" << endl;
   os << "    end" << endl;
   os << "    always_ff @(posedge clk) begin" << endl;
   os << "        if(delay_ctrl == 2)begin" << endl;
   os << "            if (R==0)begin" << endl; //No ReLu
   if (P > 1) {
	   for (int i = 0 ; i < P; i++)
		   os << "                out_v[" << i << "] <= mac_out" << i << ";" << endl;  // changed
   }
   else
       os << "                out_v[0] <= mac_out;" << endl;  // changed
   os << "            end" << endl;
   os << "            else begin" << endl;
   if (P > 1) {
	   for (int i = 0 ; i < P; i++) {
           os << "                if (mac_out" << i << " > 0)" << endl;  // changed
           os << "                    out_v[" << i << "] <= mac_out" << i << ";" << endl;
           os << "                else" << endl;
           os << "                    out_v[" << i << "] <= 0;" << endl;
	   }
   }
   else
       os << "                out_v[0] <= (mac_out > 0)? mac_out : 0;" << endl;
   os << "            end" << endl;
   os << "        end" << endl;
   os << "        else" << endl;
   os << "            out_v <= out_v;" << endl;
   os << "        end" << endl;
   if (P > 1) {
       os << "        always_comb begin" << endl;
       os << "            case(out_addr)" << endl;               // changed
	   for (int i = 0 ; i < P; i++) {
           os << "                " << i << ": out = out_v[" << i << "];" << endl;
	   }
       os << "                default: out = 0;" << endl;
       os << "            endcase" << endl;
       os << "        end" << endl;
   }
   else {
       os << "        always_comb begin" << endl;
       os << "            case(out_addr)" << endl;               // changed
       os << "                0: out = out_v[0];" << endl;
       os << "                default: out = 0;" << endl;
       os << "            endcase" << endl;
       os << "        end" << endl;
   }
   os << "        assign out_data = out;" << endl;
   os << "endmodule\n" << endl;

   /* Control Module */
   os << "module " << modName << "_control(clk, reset, in_valid, out_ready," << endl;
   os << "               addr_x, wr_en_x, addr_w, " << endl;
   os << "               clear_acc, en_acc, in_ready, out_valid, out_addr, delay_ctrl);" << endl;
   os << "    parameter                     WIDTH = 20, W_M = 5, W_N = 4, P = 1;" << endl;
   os << "    localparam                    X_SIZE = W_N;" << endl;
   os << "    localparam                    LOGSIZE_X=$clog2(W_N);" << endl;
   os << "    localparam                    LOGSIZE_W=$clog2(W_M/P * W_N);" << endl;
   os << "    localparam                    NUM_ACC=W_N;" << endl;
   os << "    localparam                    LOG_NUM_ACC = $clog2(W_N);" << endl;
   os << "    localparam                    NUM_OUTPUT = W_M;" << endl;
   os << "    localparam                    LOG_NUM_OUTPUT = $clog2(W_M);" << endl;
   os << "    localparam                    NUM_ITER = P;" << endl;
   os << "    localparam                    LOG_NUM_ITER = $clog2(P);\n" << endl;
   os << "    input                                   clk, reset, in_valid, out_ready;\n" << endl;
   os << "    output                                  wr_en_x;" << endl;
   os << "    output                                  clear_acc;" << endl;
   os << "    output logic                            en_acc;" << endl;
   os << "    output logic [LOGSIZE_X-1:0]            addr_x;" << endl;
   os << "    output logic [LOGSIZE_W-1:0]            addr_w;" << endl;   // changed
   os << "    output                                  in_ready, out_valid;" << endl;
   os << "    output       [LOG_NUM_ITER:0]           out_addr;" << endl;
   os << "    output logic [1:0]                      delay_ctrl;\n" << endl;
   os << "    logic        [2:0]                      state, next_state;" << endl;
   os << "    logic        [LOG_NUM_ACC:0]            acc_count;" << endl;
   os << "    logic        [LOG_NUM_OUTPUT:0]         output_count;" << endl;
   os << "    logic        [LOGSIZE_W+1:0]            addr_count_w; " << endl;
   os << "    logic        [LOGSIZE_X+1:0]            addr_count_x;" << endl;
   os << "    logic                                   last_valid;" << endl;
   os << "    logic        [LOG_NUM_ITER+1:0]         iter_count;\n" << endl;

   os << "    always_ff @(posedge clk) begin" << endl;
   os << "        if(reset)" << endl;
   os << "            state <= 0;" << endl;
   os << "        else" << endl;
   os << "            state <= next_state;\n" << endl;
   os << "    end" << endl;

   os << "    always_comb begin" << endl;
   os << "        next_state = state;" << endl;
   os << "        /* 0: Reset State*/" << endl;
   os << "        if(state == 0) begin" << endl;
   os << "            if(in_valid == 1)begin" << endl;
   os << "                next_state = 2;" << endl;
   os << "            end " << endl;
   os << "            else" << endl;
   os << "                next_state = 0;" << endl;
   os << "        end" << endl;
   os << "        else if(state == 2) begin" << endl;
   os << "            if (addr_count_x < X_SIZE)" << endl;
   os << "                next_state = 2;" << endl;
   os << "            else if (delay_ctrl == 2 && output_count < W_M)" << endl;
   os << "                next_state = 2;" << endl;
   os << "            else" << endl;
   os << "                next_state = 3;" << endl;
   os << "        end" << endl;
   os << "        /* 3: Compute*/" << endl;
   os << "        else if (state == 3) begin" << endl;
   os << "            if(last_valid == 0) " << endl;
   os << "                next_state = 3;" << endl;
   os << "            else if (output_count < W_M-P)" << endl;
   os << "                next_state = 4;" << endl;
   os << "            else" << endl;
   os << "                next_state = 0;" << endl;
   os << "        end" << endl;
   os << "        /* 4: Load output*/" << endl;
   os << "        else if (state == 4) begin" << endl;
   os << "            if(delay_ctrl == 2 && iter_count < P)" << endl;
   os << "               next_state = 4;" << endl;
   os << "            else" << endl;
   os << "               next_state = 3;" << endl;
   os << "        end" << endl;
   os << "    end\n" << endl;

   os << "    assign clear_acc = ((state == 2 && output_count >= W_M) ||(state == 4 && iter_count >= P) || reset == 1)? 1 : 0;" << endl;
   os << "    assign in_ready = (state == 2 && addr_count_x < X_SIZE)? 1 : 0; " << endl;
   os << "    assign wr_en_x = (state == 2) && (in_valid == 1) && (addr_count_x < X_SIZE)? 1 : 0;" << endl;
   os << "    assign out_valid = ( last_valid == 1 && (((state == 2 || state == 0) && output_count < W_M) || (state == 4 && iter_count < P)))? 1 : 0;" << endl;
   os << "    assign out_addr = iter_count;\n" << endl;

   os << "    always_ff @(posedge clk)begin" << endl;
   os << "        if(state == 0) begin" << endl;
   os << "            addr_x <= 0;" << endl;
   os << "            addr_count_x <= 0;" << endl;
   os << "        end " << endl;
   os << "        else if (state == 2 && addr_count_x >= X_SIZE && output_count >= W_M)begin" << endl;
   os << "                addr_x <= 0;" << endl;
   os << "                addr_count_x <= 0;" << endl;
   os << "        end" << endl;
   os << "        else if (state == 2 && delay_ctrl < 2 && addr_count_x >= X_SIZE)begin" << endl;
   os << "                addr_x <= 0;" << endl;
   os << "                addr_count_x <= 0;" << endl;
   os << "        end" << endl;
   os << "        else if (state == 4 && iter_count >= P)begin" << endl;
   os << "                addr_x <= 0;" << endl;
   os << "                addr_count_x <= 0;" << endl;
   os << "        end" << endl;
   os << "        else if ((state == 2 && in_valid == 1 && addr_count_x < X_SIZE)|| (state == 3 && acc_count < NUM_ACC)) begin" << endl;
   os << "            addr_x <= addr_x + 1;" << endl;
   os << "            addr_count_x <= addr_count_x + 1;" << endl;
   os << "        end else begin" << endl;
   os << "            addr_x <= addr_x;" << endl;
   os << "            addr_count_x <= addr_count_x;" << endl;
   os << "        end" << endl;
   os << "    end\n" << endl;

   os << "    always_ff @(posedge clk)begin" << endl;
   os << "        if(state == 0 || state == 2) begin" << endl;
   os << "            addr_w <= 0;" << endl;   // changed
   os << "            addr_count_w <= 0;" << endl;
   os << "        end " << endl;
   os << "        else if (state == 3 && acc_count < NUM_ACC) begin" << endl;
   os << "            addr_w <= addr_w + 1;" << endl;   // changed
   os << "            addr_count_w <= addr_count_w + 1;" << endl;
   os << "        end" << endl;
   os << "        else begin" << endl;
   os << "            addr_w <= addr_w;" << endl;   // changed
   os << "            addr_count_w <= addr_count_w;" << endl;
   os << "        end" << endl;
   os << "    end\n" << endl;

   os << "    /* Accumulation Counter*/" << endl;
   os << "    always_ff @(posedge clk)begin" << endl;
   os << "        if(state == 0 | state == 4)" << endl;
   os << "            acc_count <= 0;" << endl;
   os << "        else if(state == 3 && acc_count < NUM_ACC)" << endl;
   os << "            acc_count <= acc_count + 1;" << endl;
   os << "        else" << endl;
   os << "            acc_count <= acc_count;" << endl;
   os << "    end\n" << endl;

   os << "    always_ff @(posedge clk)begin" << endl;
   os << "        if(reset == 1 || (state == 2 && addr_count_x >= X_SIZE && output_count >= W_M))" << endl;
   os << "            output_count <= 0;" << endl;
   os << "        else if(out_ready == 1 && out_valid == 1 && (((state == 2 || state == 0) && output_count < W_M) || (state == 4 && iter_count < P)))" << endl;
   os << "            output_count <= output_count + 1;" << endl;
   os << "        else" << endl;
   os << "            output_count <= output_count;" << endl;
   os << "    end\n" << endl;

   os << "    always_ff @(posedge clk) begin" << endl;
   os << "        if(state == 3 && acc_count < NUM_ACC )" << endl;
   os << "            en_acc <= 1;" << endl;
   os << "        else" << endl;
   os << "            en_acc <= 0;" << endl;
   os << "    end\n" << endl;

   os << "    always_ff @(posedge clk)begin" << endl;
   os << "       if(reset == 1 | state == 3)" << endl;
   os << "         iter_count <= 0;" << endl;
   os << "       else if ((state == 4 || state == 2 || state == 0) && out_ready == 1 && out_valid == 1 && iter_count < P)" << endl;
   os << "         iter_count <= iter_count + 1;" << endl;
   os << "       else" << endl;
   os << "         iter_count <= iter_count;" << endl;
   os << "    end\n" << endl;

   os << "    always_ff @(posedge clk) begin" << endl;
   os << "        if(reset == 1 || en_acc == 1 || ((state == 4 || state ==3) && iter_count >= P))" << endl;
   os << "            delay_ctrl <= 0;" << endl;
   os << "        else if (state == 3 && delay_ctrl < 2)" << endl;
   os << "            delay_ctrl <= delay_ctrl + 1;" << endl;
   os << "        else" << endl;
   os << "            delay_ctrl <= delay_ctrl;" << endl;
   os << "    end\n" << endl;

   os << "    always_ff @(posedge clk) begin" << endl;
   os << "        if(delay_ctrl == 2 && (((state == 4||state == 3) && iter_count < P) || ((state == 2 || state == 0) && output_count < W_M))) " << endl;
   os << "            last_valid <= 1;" << endl;
   os << "        else" << endl;
   os << "            last_valid <= 0;" << endl;
   os << "    end" << endl;
   os << "endmodule\n" << endl;

   // At some point you will want to generate a ROM with values from the pre-stored constant values.
   // Here is code that demonstrates how to do this for the simple case where you want to put all of
   // the matrix values W in one ROM. This is probably what you will need for P=1, but you will want 
   // to change this for P>1. Please also see some examples of splitting these vectors in the Part 3
   // code.

   // Check there are enough values in the constant file.
   if (M*N != constVector.size()) {
	   cout << "ERROR: constVector does not contain correct amount of data for the requested design" << endl;
	   cout << "The design p arameters requested require " << M * N << " numbers, but the provided data only have " << constVector.size() << " constants" << endl;
      assert(false);
   }

   if (P > 1) {
       int rom_size = M / P * N;
       vector<int> roms[P];
       int r_count = 0;
       int i_count = 0;
       for (int i = 0; i < constVector.size(); i+=N) {
           i_count = r_count % P;
           for (int j = 0; j < N; j++)
               roms[i_count].push_back(constVector[i+j]);
           r_count += 1;
       }
       for (int m = 0; m < P; m++) {
           string romModName = modName + "_W_rom" + to_string(m);
           genROM(roms[m], T, romModName, os);
       }
   }
   else {
       // Generate a ROM (for W) with constants 0 through M*N-1, with T bits
       string romModName = modName + "_W_rom";
       genROM(constVector, T, romModName, os);
   }

}

int max_M_P(int M_P[], bool checked[]) {
    int max = 0;
    int max_index = 0;

    for (int i = 0; i < 3; i++) {
        if (M_P[i] > max && !checked[i]) {
            max_index = i;
            max = M_P[i];
        }
    }
    return max_index;
}

int new_P(int num_mac, int M, int P) {
    int new_P = P + 1;
    while (new_P <= num_mac && new_P <= M) {
        if (M % new_P == 0)
            return new_P;
        new_P += 1;
    }
    return P;
}

bool checked_all(bool checked[]) {
    for (int i = 0; i < 3; i++) {
        if (!checked[i])
            return false;
    }
    return true;
}

// Part 3: Generate a hardware system with three layers interconnected.
// Layer 1: Input length: N, output length: M1
// Layer 2: Input length: M1, output length: M2
// Layer 3: Input length: M2, output length: M3
// B is the number of multipliers your overall design may use.
// Your goal is to build the fastest design that uses B or fewer multipliers
// constVector holds all the constants for your system (all three layers, in order)
void genNetwork(int N, int M1, int M2, int M3, int T, int R, int B, vector<int>& constVector, string modName, ofstream &os) {
    
   // Here you will write code to figure out the best values to use for P1, P2, and P3, given
   // B. 
   int P1 = 1; // replace this with your optimized value
   int P2 = 2; // replace this with your optimized value
   int P3 = 3; // replace this with your optimized value
   bool finished = false;
   bool changed = false;
   bool checked[3];
   int M_P[3];
   int index;
   int new_P1 = P1;
   int new_P2 = P2;
   int new_P3 = P3;

   while (!finished) {
       for (int i = 0; i < 3; i++)
           checked[i] = false;
       changed = false;

       M_P[0] = M1 / P1; M_P[1] = M2 / P2; M_P[2] = M3 / P3;

       while (!checked_all(checked) && !changed) {
           index = max_M_P(M_P, checked);

           switch (index)
           {
           case 0:
               new_P1 = new_P(B - P2 - P3, M1, P1);
               break;
           case 1:
               new_P2 = new_P(B - P1 - P3, M2, P2);
               break;
           case 2:
               new_P3 = new_P(B - P1 - P2, M3, P3);
               break;
           default:
               printf("Wrong case.");
               break;
           }

           if (new_P1 != P1) {
               P1 = new_P1;
               changed = true;
           }
           else if (new_P2 != P2) {
               P2 = new_P2;
               changed = true;
           }
           else if (new_P3 != P3) {
               P3 = new_P3;
               changed = true;
           }
           else
               checked[index] = true;
       }

       if (P1 + P2 + P3 >= B || checked_all(checked))
           finished = true;
   }

   // Module names
   string subModName1 = "l1_fc_" + to_string(M1) + "_" + to_string(N) + "_" + to_string(T) + "_" + to_string(R) + "_" + to_string(P1);
   string subModName2 = "l2_fc_" + to_string(M2) + "_" + to_string(M1) + "_" + to_string(T) + "_" + to_string(R) + "_" + to_string(P2);
   string subModName3 = "l3_fc_" + to_string(M3) + "_" + to_string(M2) + "_" + to_string(T) + "_" + to_string(R) + "_" + to_string(P3);

   // output top-level module
   os << "module " << modName << "(clk, reset, input_valid, input_ready, input_data, output_valid, output_ready, output_data);" << endl;

   os << "   localparam N = " << N << ";" << endl;
   os << "   localparam M1 = " << M1 << ";" << endl;
   os << "   localparam M2 = " << M2 << ";" << endl;
   os << "   localparam M3 = " << M3 << ";" << endl;
   os << "   localparam T = " << T << ";" << endl;
   os << "   localparam R = " << R << ";" << endl;
   os << "   localparam B = " << B << ";\n" << endl;
   os << "   input                    clk, reset, input_valid, output_ready;" << endl;
   os << "   input  signed [T-1:0]    input_data;" << endl;
   os << "   output                   output_valid, input_ready;" << endl;
   os << "   output signed [T-1:0]    output_data;\n" << endl;
   os << "   logic                    output_valid_l1;" << endl;
   os << "   logic  signed [T-1:0]    output_data_l1;" << endl;
   os << "   logic                    input_ready_l2, output_valid_l2, output_ready_l2;" << endl;
   os << "   logic  signed [T-1:0]    output_data_l2;" << endl;
   os << "   logic                    input_ready_l3;\n" << endl;

   os << "   " << subModName1 << " l1(.clk(clk), .reset(reset), .input_valid(input_valid), .input_ready(input_ready), .input_data(input_data),\n"
      << "                       .output_valid(output_valid_l1), .output_ready(input_ready_l2), .output_data(output_data_l1));" << endl;
   os << "   " << subModName2 << " l2(.clk(clk), .reset(reset), .input_valid(output_valid_l1), .input_ready(input_ready_l2), .input_data(output_data_l1),\n"
      << "                       .output_valid(output_valid_l2), .output_ready(input_ready_l3), .output_data(output_data_l2));" << endl;

   os << "   " << subModName3 << " l3(.clk(clk), .reset(reset), .input_valid(output_valid_l2), .input_ready(input_ready_l3), .input_data(output_data_l2),\n"
       << "                      .output_valid(output_valid), .output_ready(output_ready), .output_data(output_data));\n" << endl;

   os << "endmodule\n" << endl;
   
   // -------------------------------------------------------------------------
   // Split up constVector for the three layers
   // layer 1's W matrix is M1 x N
   int start = 0;
   int stop = M1*N;
   vector<int> constVector1(&constVector[start], &constVector[stop]);

   // layer 2's W matrix is M2 x M1
   start = stop;
   stop = start+M2*M1;
   vector<int> constVector2(&constVector[start], &constVector[stop]);

   // layer 3's W matrix is M3 x M2
   start = stop;
   stop = start+M3*M2;
   vector<int> constVector3(&constVector[start], &constVector[stop]);

   if (stop > constVector.size()) {
	   os << "ERROR: constVector does not contain enough data for the requested design" << endl;
	   os << "The design parameters requested require " << stop << " numbers, but the provided data only have " << constVector.size() << " constants" << endl;
      assert(false);
   }
   // --------------------------------------------------------------------------

   // generate the three layer modules
   genFCLayer(M1, N, T, R, P1, constVector1, subModName1, os);

   genFCLayer(M2, M1, T, R, P2, constVector2, subModName2, os);

   genFCLayer(M3, M2, T, R, P3, constVector3, subModName3, os);

   // generate memory module, only need one
   genMem(os);
   genMAC(os, T);
}


void printUsage() {
	cout << "Usage: ./gen MODE ARGS" << endl << endl;

	cout << "   Mode 1 (Part 1): Produce one neural network layer (unparallelized)" << endl;
	cout << "      ./gen 1 M N T R const_file" << endl << endl;

	cout << "   Mode 2 (Part 2): Produce one neural network layer (parallelized)" << endl;
	cout << "      ./gen 2 M N T R P const_file" << endl << endl;

	cout << "   Mode 3 (Part 3): Produce a system with three interconnected layers" << endl;
	cout << "      ./gen 3 N M1 M2 M3 T R B const_file" << endl;
}
