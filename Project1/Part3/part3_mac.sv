/*
ESE 507 Project 1 Part 3 - Multiply and Accumulate
by Xinqi Wang, Jinkyu Lee
Last modified: 10/5/2020
*/

module part3_mac(clk, reset, a, b, valid_in, f, valid_out);
    input clk, reset, valid_in;
    input signed [11:0] a, b;
    output logic signed [23:0] f;
    output logic valid_out;

    // Internal regs
    logic en_reg;
	logic signed [23:0] product;
	logic signed [23:0] testf;
	
    always_ff @ (posedge clk) begin
        if(reset)begin
            en_reg <= 0; 
            product <= 0;       
        end
        else if(valid_in) begin
            en_reg <= 1;
            product <= a * b;
        end
        else begin
            en_reg <= 0;
        end
    end

    always_ff @ (posedge clk)begin
        if (reset)begin
            testf <= 0;
        end
        else if (valid_in)begin
            if(f[23]==0 & product[23]==0 & testf[23]==1) begin   //overflow
	            testf <= 8388607 + a*b; 
		    	  end
			      else if (f[23]==1 & product[23]==1 & testf[23]==0) begin //underflow
				      testf <= -8388608 + a*b;
			      end
			      else begin 
		   	      testf <= testf + a*b;			
  		      end
        end
    end

    always_ff @ (posedge clk) begin
        if(reset) begin
            f <= 0;
        end
		    else if (en_reg) begin      
			    if(f[23]==0 & product[23]==0 & testf[23]==1) begin   //overflow
	          f <= 8388607; 
		    	end
			    else if (f[23]==1 & product[23]==1 & testf[23]==0) begin //underflow
				    f <= -8388608;
			    end
			    else begin 
		   	    f <= testf;			
  		    end
		    end
    end  

    always_ff @ (posedge clk) begin
        if(reset) begin
            valid_out <= 0;
        end
        else if (en_reg) begin
            valid_out <= 1;
        end
        else begin
            valid_out <= 0;
        end
    end
	
endmodule