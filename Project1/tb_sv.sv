/*
ESE507 Projct 1 Part2 Testbench
by Xinqi Wang, Jinkyu Lee
last modified: 10/1/2020
*/

module part2_tb_c();

   logic clk, reset, valid_in, valid_out;
   logic signed [11:0] a, b;
   logic signed [23:0] f;

   initial clk=0;
   always #5 clk=~clk;
   
   //part2_mac dut(.clk(clk), .reset(reset), .a(a), .b(b), .valid_in(valid_in), .f(f), .valid_out(valid_out));

   logic [31:0] testData[299:0];
   initial $readmemh("inputData", testData);

   integer i;
   initial begin

      reset = 1;
      {a, b} = {12'b0,12'b0};
      valid_in = 0;
      @(posedge clk);
      #1; // After 1 posedge
      reset = 0;

      //*****Random Tests*****
      for (i=0; i<100; i=i+1) begin
          @(posedge clk);
          #1; 
          a = testData[3*i][11:0]; 
          b = testData[3*i+1][11:0]; 
          valid_in = testData[3*i+2][3:0];
      end
      
      //*****Added test cases - Reset*****
      @(posedge clk);
      #1; 
      reset = 1;
      a = 7; b = 7; valid_in = 1; 
      @(posedge clk);
      #1; 
      reset = 0;
      a = 8; b = 8; valid_in = 0;
      @(posedge clk);
      #1;
      a = 4; b = 20; valid_in = 1;   
      @(posedge clk);
      #1;
      a = 10; b = 10; valid_in = 1; 
      
      //*****Added test cases - Over/Underflow*****
      @(posedge clk);
      #1;  
      a = 1023; b = 1023; valid_in = 1; //1023*1023 + 180 = 1046709
      @(posedge clk);
      #1;  
      a = 1023; b = 1023; valid_in = 1; //1023*1023 + 1046709 = 2093238
      @(posedge clk);
      #1; 
      a = 1023; b = 1023; valid_in = 1; //1023*1023 + 2093238 = 3139767
      @(posedge clk);
      #1;  
      a = 1023; b = 1023; valid_in = 1; //1023*1023 + 3139767 = 4186296
      @(posedge clk);
      #1; 
      a = 1023; b = 1023; valid_in = 1; //1023*1023 + 4186296 = 5232825
      @(posedge clk);
      #1;
      a = 1023; b = 1023; valid_in = 1; //1023*1023 + 5232825 = 6279354
      @(posedge clk);
      #1; 
      a = 1023; b = 1023; valid_in = 1; //1023*1023 + 6279354 = 7325883
      @(posedge clk);
      #1; 
      a = 1023; b = 1023; valid_in = 1; //1023*1023 + 7325883 = 8372412
      @(posedge clk);
      #1;
      a = 1023; b = 1023; valid_in = 1; //1023*1023 + 8372412 = 9418941, overflow to -7358275 
      @(posedge clk);
      #1;
      a = -1024; b = 1023; valid_in = 1;//-1024*1023  + (-7358275) = -8405827, underflow to 8371389
      @(posedge clk);
      #1; 
      a = 2; b = 5; valid_in = 1;
      
      @(posedge clk);
      @(posedge clk);
      @(posedge clk);
      
      $fclose(filehandle);
      $finish;
   end   
   
   integer filehandle=$fopen("outValues");
   always @(posedge clk) 
       $fdisplay(filehandle, "%d\n%d", f, valid_out);

endmodule


