/*
ESE 507 Project 1 Part 2 - Multiply and Accumulate
by Xinqi Wang, Jinkyu Lee
Last modified: 9/28/2020
*/

module part2_mac(clk, reset, a, b, valid_in, f, valid_out);
    input clk, reset, valid_in;
    input signed [11:0] a, b;
    output logic signed [23:0] f;
    output logic valid_out;

    // Internal regs
    logic signed [11:0] a_reg, b_reg;
    logic en_reg;

    always_ff @ (posedge clk) begin
        if(reset)begin
            a_reg <= 0;
            b_reg <= 0;
            en_reg <= 0;        
        end
        else if(valid_in) begin
            a_reg <= a;
            b_reg <= b;
            en_reg <= 1;
        end
        else begin
            a_reg <= 0;
            b_reg <= 0;
            en_reg <= 0;
        end
    end

    always_ff @ (posedge clk) begin
        if(reset) begin
            f <= 0;
        end
        else if (en_reg) begin
            f <= f + a_reg * b_reg;
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