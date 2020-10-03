// Peter Milder

// This is a very small testbench for you to check that you have the right
// idea for the input/output timing.

// This should not be your only test -- it's simply a basic way to make
// sure you have the right idea.

module tb_part2_mac();

   logic clk, reset, valid_in, valid_out;
   logic signed [11:0] a, b;
   logic signed [23:0] f;

   part2_mac dut(.clk(clk), .reset(reset), .a(a), .b(b), .valid_in(valid_in), .f(f), .valid_out(valid_out));

   initial clk = 0;
   always #5 clk = ~clk;

   initial begin

      // Before first clock edge, initialize
      reset = 1;
      {a, b} = {12'b0,12'b0};
      valid_in = 0;

      @(posedge clk);
      #1; // After 1 posedge
      reset = 0; a = 1; b = 1; valid_in = 0;
      @(posedge clk);
      #1; // After 2 posedges
      a = 2; b = 2; valid_in = 1;
      @(posedge clk);
      #1; // After 3 posedges
      a = 3; b = 3; valid_in = 1;
      @(posedge clk);
      #1; // After 4 posedges
      a = 4; b = 4; valid_in = 0;
      @(posedge clk);
      #1; // After 5 posedges
      a = 5; b = 5; valid_in = 0;
      @(posedge clk);
      #1; // After 6 posedges
      a = 6; b = 6; valid_in = 1;   

      //*****Added test cases - Reset*****
      @(posedge clk);
      #1; // After 7 posedges
      reset = 1;
      a = 7; b = 7; valid_in = 1; 
      @(posedge clk);
      #1; // After 8 posedges
      reset = 0;
      a = 8; b = 8; valid_in = 0;
      @(posedge clk);
      #1; // After 9 posedges
      a = 9; b = 9; valid_in = 1;   
      @(posedge clk);
      #1; // After 10 posedges
      a = 10; b = 10; valid_in = 1;   

      //*****Added test cases - Negative values*****
      @(posedge clk);
      #1; // After 11 posedges
      a = -1; b = 1; valid_in = 1; 
      @(posedge clk);
      #1; // After 12 posedges
      a = 2; b = -5; valid_in = 0;
      @(posedge clk);
      #1; // After 13 posedges
      a = -10; b = 10; valid_in = 1;   
      @(posedge clk);
      #1; // After 14 posedges
      a = -10; b = -10; valid_in = 1;         
      
      //*****Added test cases - Over/Underflow*****
      @(posedge clk);
      #1; // After 15 posedges
      a = 1023; b = 1023; valid_in = 1; //1023*1023 + 180 = 1046709
      @(posedge clk);
      #1; // After 16 posedges
      a = 1023; b = 1023; valid_in = 1; //1023*1023 + 1046709 = 2093238
      @(posedge clk);
      #1; // After 17 posedges
      a = 1023; b = 1023; valid_in = 1; //1023*1023 + 2093238 = 3139767
      @(posedge clk);
      #1; // After 18 posedges
      a = 1023; b = 1023; valid_in = 1; //1023*1023 + 3139767 = 4186296
      @(posedge clk);
      #1; // After 19 posedges
      a = 1023; b = 1023; valid_in = 1; //1023*1023 + 4186296 = 5232825
      @(posedge clk);
      #1; // After 20 posedges
      a = 1023; b = 1023; valid_in = 1; //1023*1023 + 5232825 = 6279354
      @(posedge clk);
      #1; // After 21 posedges
      a = 1023; b = 1023; valid_in = 1; //1023*1023 + 6279354 = 7325883
      @(posedge clk);
      #1; // After 22 posedges
      a = 1023; b = 1023; valid_in = 1; //1023*1023 + 7325883 = 8372412
      @(posedge clk);
      #1; // After 23 posedges
      a = 1023; b = 1023; valid_in = 1; //1023*1023 + 8372412 = 9418941, overflow to -7358275 
      @(posedge clk);
      #1; // After 24 posedges
      a = -1024; b = 1023; valid_in = 1;//-1024*1023  + (-7358275) = -8405827, underflow to 8371389
      @(posedge clk);
      #1; // After 25 posedges
      a = 2; b = 5; valid_in = 1;


   end // initial begin

   initial begin
      @(posedge clk);
      #1; // After 1 posedge
      $display("valid_out = %b. Expected value is 0.", valid_out);
      $display("f = %d. Expected value is 0.", f);
 
      @(posedge clk);
      #1; // After 2 posedges
      $display("valid_out = %b. Expected value is 0.", valid_out);
      $display("f = %d. Expected value is 0.", f);

      @(posedge clk);
      #1; // After 3 posedges
      $display("valid_out = %b. Expected value is 0.", valid_out);
      $display("f = %d. Expected value is 0.", f);

      @(posedge clk);
      #1; // After 4 posedges
      $display("valid_out = %b. Expected value is 1.", valid_out);
      $display("f = %d. Expected value is 4.", f);

      @(posedge clk);
      #1; // After 5 posedges
      $display("valid_out = %b. Expected value is 1.", valid_out);
      $display("f = %d. Expected value is 13.", f);

      @(posedge clk);
      #1; // After 6 posedges
      $display("valid_out = %b. Expected value is 0.", valid_out);
      $display("f = %d. Expected value is don't care (probably will be 13 in your design).", f);

      @(posedge clk);
      #1; // After 7 posedges
      $display("valid_out = %b. Expected value is 0.", valid_out);
      $display("f = %d. Expected value is is don't care (probably will be 13 in your design).", f);

      @(posedge clk);
      #1; // After 8 posedges
      $display("valid_out = %b. Expected value is 1.", valid_out);
      $display("f = %d. Expected value is 49 (output should be valid but reset is asserted).", f);    // not sure if outputs reset immediately

      //*****Added tests - Reset*****
      @(posedge clk);
      #1; // After 9 posedges
      $display("valid_out = %b. Expected value is 0.", valid_out);
      $display("f = %d. Expected value is 0.", f);

      @(posedge clk);
      #1; // After 10 posedges
      $display("valid_out = %b. Expected value is 0.", valid_out);
      $display("f = %d. Expected value is 0.", f);

      @(posedge clk);
      #1; // After 11 posedges
      $display("valid_out = %b. Expected value is 1.", valid_out);
      $display("f = %d. Expected value is 81.", f);

      @(posedge clk);
      #1; // After 12 posedges
      $display("valid_out = %b. Expected value is 1.", valid_out);
      $display("f = %d. Expected value is 181.", f);
      
      //*****Added tests - Negative values*****
      @(posedge clk);
      #1; // After 13 posedges
      $display("valid_out = %b. Expected value is 1.", valid_out);
      $display("f = %d. Expected value is 180.", f);

      @(posedge clk);
      #1; // After 14 posedges
      $display("valid_out = %b. Expected value is 0.", valid_out);
      $display("f = %d. Expected value is 180.", f);

      @(posedge clk);
      #1; // After 15 posedges
      $display("valid_out = %b. Expected value is 1.", valid_out);
      $display("f = %d. Expected value is 80.", f);

      @(posedge clk);
      #1; // After 16 posedges
      $display("valid_out = %b. Expected value is 1.", valid_out);
      $display("f = %d. Expected value is 180.", f);      
      
      //*****Added test cases - Over/Underflow*****
      @(posedge clk);
      #1; // After 17 posedges
      $display("valid_out = %b. Expected value is 1.", valid_out);
      $display("f = %d. Expected value is 1046709.", f);
      @(posedge clk);
      #1; // After 18 posedges
      $display("valid_out = %b. Expected value is 1.", valid_out);
      $display("f = %d. Expected value is 2093238.", f);         
      @(posedge clk);
      #1; // After 19 posedges
      $display("valid_out = %b. Expected value is 1.", valid_out);
      $display("f = %d. Expected value is 3139767.", f);
      @(posedge clk);
      #1; // After 20 posedges
      $display("valid_out = %b. Expected value is 1.", valid_out);
      $display("f = %d. Expected value is 4186296.", f); 
      @(posedge clk);
      #1; // After 21 posedges
      $display("valid_out = %b. Expected value is 1.", valid_out);
      $display("f = %d. Expected value is 5232825.", f);
      @(posedge clk);
      #1; // After 22 posedges
      $display("valid_out = %b. Expected value is 1.", valid_out);
      $display("f = %d. Expected value is 6279354.", f);       
      @(posedge clk);
      #1; // After 23 posedges
      $display("valid_out = %b. Expected value is 1.", valid_out);
      $display("f = %d. Expected value is 7325883.", f);
      @(posedge clk);
      #1; // After 24 posedges
      $display("valid_out = %b. Expected value is 1.", valid_out);
      $display("f = %d. Expected value is 8372412.", f);           
      @(posedge clk);
      #1; // After 25 posedges
      $display("valid_out = %b. Expected value is 1.", valid_out);
      $display("f = %d. Expected value is -7358275.", f);       
      @(posedge clk);
      #1; // After 26 posedges
      $display("valid_out = %b. Expected value is 1.", valid_out);
      $display("f = %d. Expected value is 8371389.", f);
      @(posedge clk);
      #1; // After 27 posedges
      $display("valid_out = %b. Expected value is 1.", valid_out);
      $display("f = %d. Expected value is 8371399.", f);          
      
      #20;
      $finish;
   end

endmodule // tb_part2_mac
