/*
ESE 507 Modified MAC from Project 1 plus pipelined multiplier
by Xinqi Wang
Last modified: 10/28/2020
*/

module mac(clk, reset, a, b, valid_in, f, valid_out);
    input clk, reset, valid_in;
    input signed [11:0] a, b;
    output logic signed [23:0] f;
    output valid_out;

    logic signed [11:0] a_r, b_r;
    logic signed [23:0] product, acc;

    logic [3:0] shift_delay;
    

    DW02_mult_3_stage #(12, 12) mult(a_r, b_r, 1'b1, clk, product);

    
    always_ff @(posedge clk) begin
          if(reset)
            shift_delay <= 0;
          else
            shift_delay <= {shift_delay[2:0], valid_in};
    end
    assign valid_out = shift_delay[3];
    
    always_comb begin
        if(shift_delay[2]==1)
            acc = f + product;
    end
    
    always_ff @(posedge clk)begin
        if(reset) begin
            a_r <= 0;
            b_r <= 0;
        end
        else if(valid_in)begin
            a_r <= a;
            b_r <= b;
        end
        else begin
            a_r <= a_r;
            b_r <= b_r;
        end
    end

    always_ff @ (posedge clk) begin
        if(reset)
            f <= 0;
		    else if(shift_delay[2]==1)begin      
			      if(f[23]==0 & product[23]==0 & acc[23]==1)   //overflow
	            f <= 8388607; 
			      else if (f[23]==1 & product[23]==1 & acc[23]==0) //underflow
				      f <= -8388608;
			      else 
		   	      f <= acc;			
        end
        else
            f <= f;
    end
   
endmodule

