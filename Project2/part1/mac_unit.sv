/*
ESE 507 Modified MAC from Project 1
by Xinqi Wang
Last modified: 10/28/2020
*/

module mac(clk, reset, a, b, valid_in, f, valid_out);
    input clk, reset, valid_in;
    input signed [11:0] a, b;
    output logic signed [23:0] f;
    output logic valid_out;

    logic en_r;
    logic signed [11:0] a_r, b_r;
    logic signed [23:0] product, acc;

    always_comb begin
        acc = f + a_r*b_r;
        product = a_r*b_r;
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
		else if(en_r == 1)begin      
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

    always_ff @ (posedge clk) begin
        if(reset)begin
            en_r <= 0;      
        end
        else if(valid_in) begin
            en_r <= 1;
        end
        else begin
            en_r <= 0;
        end
    end
    
    always_ff @ (posedge clk) begin
        if(reset) begin
            valid_out <= 0;
        end
        else if (en_r) begin
            valid_out <= 1;
        end
        else begin
            valid_out <= 0;
        end
    end
   
endmodule

