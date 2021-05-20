module tb_fc_16_8_16_1_16();

   parameter T = 16;
   parameter NUMINPUTVALS = 10000;
   parameter NUMOUTPUTVALS = 20000;
   parameter INFILENAME = "tb_fc_16_8_16_1_16.in";
   parameter EXPFILENAME = "tb_fc_16_8_16_1_16.exp";

   logic clk, input_valid, input_ready, output_valid, output_ready, reset;
   logic  [T-1:0] input_data;
   logic signed [T-1:0] output_data;

   logic signed [T-1:0] inValues [NUMINPUTVALS-1:0];
   logic signed [T-1:0] expValues [NUMOUTPUTVALS-1:0];
   logic s;

   initial clk=0;
   always #5 clk = ~clk;
   
   fc_16_8_16_1_16 dut(clk, reset, input_valid, input_ready, input_data, output_valid, output_ready, output_data);

   logic rb, rb2;
   always begin
      @(posedge clk);
      #1;
      s=std::randomize(rb, rb2);
   end

   logic [31:0] j;

   always @* begin
      if (input_valid == 1)
         input_data = inValues[j];
      else
         input_data = 'x;
   end

   always @* begin
      if ((j>=0) && (j<NUMINPUTVALS) && (rb==1'b1))
         input_valid=1;
      else
         input_valid=0;
   end

   always @(posedge clk) begin
      if (input_valid && input_ready)
         j <= #1 j+1;
   end
  
   logic [31:0] i;
   always @* begin
      if ((i>=0) && (i<NUMOUTPUTVALS) && (rb2==1'b1))
         output_ready = 1;
      else
         output_ready = 0;
   end

   integer errors = 0;

   always @(posedge clk) begin
      if (output_ready && output_valid) begin
         if (output_data !== expValues[i]) begin
            $display($time,,"ERROR: y[%d] = %x; expected value = %x", i, output_data, expValues[i]);
            errors = errors+1;
         end
         i=i+1; 
      end 
   end

   ////////////////////////////////////////////////////////////////////////////////

   initial begin
     $readmemb(INFILENAME, inValues);
     $readmemb(EXPFILENAME, expValues);
     
      j=0; i=0;

      // Before first clock edge, initialize
      output_ready = 0; 
      reset = 0;
   
      // reset
      @(posedge clk); #1; reset = 1; 
      @(posedge clk); #1; reset = 0; 

      wait(i==NUMOUTPUTVALS);
      $display("Simulated %d outputs. Found %d errors.", NUMOUTPUTVALS, errors);
      $finish;
   end


endmodule
