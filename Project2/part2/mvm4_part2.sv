/*
ESE 507 Project 2 Part 2
by Xinqi Wang, Jinkyu Lee
modified on: 10/21/20
*/

module mvm4_part2(clk, reset, input_valid, input_ready, input_data, new_matrix, output_valid, output_ready, output_data);
    input                   clk, reset, input_valid, new_matrix, output_ready;
    input  signed [11:0]    input_data;
    output signed [23:0]    output_data;
    output                  output_valid, input_ready;

    // Internal connections
    logic                       clk, reset, wr_en_x, wr_en_w, clear_acc, en_acc;
    logic [1:0]                 addr_x;
    logic [3:0]                 addr_w;
    logic                       last_valid;

    datapath #(12, 4, 16) dp( 
                            //Inputs
                            .in_data(input_data), .clk(clk), .reset(reset), .addr_x(addr_x), .wr_en_x(wr_en_x), 
                            .addr_w(addr_w), .wr_en_w(wr_en_w), .clear_acc(clear_acc), .en_acc(en_acc),
                            //Outputs
                            .out_data(output_data), .last_valid(last_valid));

    control #(12, 4, 16) ctrl(
                            //Inputs
                            .clk(clk), .reset(reset), .in_valid(input_valid), .out_ready(output_ready), .new_matrix(new_matrix),
                            //Outputs
                            .addr_x(addr_x), .wr_en_x(wr_en_x),    //vector
                            .addr_w(addr_w), .wr_en_w(wr_en_w),    //matrix
                            .clear_acc(clear_acc), .en_acc(en_acc), 
                            .in_ready(input_ready), .out_valid(output_valid), .last_valid(last_valid));
endmodule

/*DataPath*/
module datapath(in_data, clk, reset, addr_x, wr_en_x, addr_w, wr_en_w, clear_acc, en_acc, out_data, last_valid);
    parameter                           WIDTH = 12, X_SIZE = 4, W_SIZE = 16; 
    localparam                          LOGSIZE_X=$clog2(X_SIZE);
    localparam                          LOGSIZE_W=$clog2(W_SIZE);
    input                               clk, reset, wr_en_x, wr_en_w, clear_acc, en_acc;
    input signed [WIDTH-1:0]            in_data;
    input [LOGSIZE_X-1:0]               addr_x;
    input [LOGSIZE_W-1:0]               addr_w;
    output logic signed [WIDTH*2-1:0]   out_data;
    output logic                        last_valid;

    logic                           acc_clr;           // Clear MAC
    logic signed [WIDTH-1:0]        w_in;              // Matrix input
    logic signed [WIDTH-1:0]        x_in;              // Vector input
    logic signed [WIDTH*2-1:0]      mac_out;           // MAC output
    logic                           mac_out_valid;     // MAC valid output

    logic [2:0]           mac_delay_count;   // our MAC unit has a 2 cycle delay


    memory #(WIDTH, X_SIZE) vector(.clk(clk), .data_in(in_data), .addr(addr_x), .wr_en(wr_en_x), .data_out(x_in));

    memory #(WIDTH, W_SIZE) matrix(.clk(clk), .data_in(in_data), .addr(addr_w),  .wr_en(wr_en_w), .data_out(w_in));

    mac mc(.clk(clk), .reset(acc_clr), .a(w_in), .b(x_in), .valid_in(en_acc), .f(mac_out), .valid_out(mac_out_valid));   
    
    always_comb begin
        acc_clr = reset || clear_acc;
    end

    always_ff @(posedge clk) begin
        if(mac_out_valid && mac_delay_count == 1)
            out_data <= mac_out;
        else
            out_data <= out_data;
    end

    always_ff @(posedge clk) begin
        // This is the final output we want
        if(mac_out_valid && mac_delay_count == 1)  
        // New iteration, deassert last_valid
            last_valid <= 1;
        else if (en_acc == 1)                      
            last_valid <= 0;    
        else 
            last_valid <= last_valid;
    end

    always_ff @(posedge clk) begin
        // New iteration, clear daley counter
        if(reset == 1 || en_acc == 1)                  
            mac_delay_count <= 0;
        // OW increment
        else if (mac_delay_count < 2) 
            mac_delay_count <= mac_delay_count + 1;
        // Stop incrementing once it reaches 2 and wait for new iteration
        else
            mac_delay_count <= mac_delay_count;
    end
    
endmodule

/*ControlPath*/
module control(clk, reset, in_valid, out_ready, addr_x, wr_en_x, addr_w, wr_en_w, clear_acc, en_acc, in_ready, out_valid, last_valid, new_matrix);
    parameter                     WIDTH = 12, X_SIZE = 4, W_SIZE = 16; 
    localparam                    LOGSIZE_X=$clog2(X_SIZE);
    localparam                    LOGSIZE_W=$clog2(W_SIZE);
    localparam                    NUM_ACC=X_SIZE - 1;
    localparam                    NUM_OUTPUT = W_SIZE / X_SIZE - 1;
    input                         clk, reset, in_valid, out_ready, new_matrix;
    output                        wr_en_x, wr_en_w;
    output                        clear_acc;
    output logic                  en_acc; 
    output logic [LOGSIZE_X-1:0]  addr_x;
    output logic [LOGSIZE_W-1:0]  addr_w;
    output                        in_ready, out_valid;
    input                         last_valid; 

    logic [2:0]             state, next_state;
    logic [3:0]             acc_count;          // Keep track of number of accumulations
    logic [3:0]             output_count;       // Keep track of number of outputs generated
    logic [LOGSIZE_W:0]     addr_count_w;       // Keep track of Matrix address
    logic [LOGSIZE_X:0]     addr_count_x;       // Keep track of Vector address

    // FSM States 
    always_ff @(posedge clk) begin
        if(reset)
            state <= 0;
        else
            state <= next_state;
    end
    
    always_comb begin
        next_state = state;
        
        /* 0: Reset State*/
        if(state == 0) begin
            if(in_valid == 1)begin 
                if(new_matrix == 1)  // If new W needs to be loaded, we go to state 1
                    next_state = 1;
                else 
                    next_state = 2;  // OW we go to state 2 and load the new X
            end 
            else
                next_state = 0;
        end
        /* 1: Load Matrix*/
        else if (state == 1) begin
            if (addr_count_w < W_SIZE)
                next_state = 1;
            else
                next_state = 2;
        end
        /* 2: Load Vecotr*/
        else if(state == 2) begin
            if (addr_count_x < X_SIZE)
                next_state = 2;
            else
                next_state = 3;
        end
        /* 3: Compute*/
        else if (state == 3) begin
            if(acc_count < NUM_ACC)
                next_state = 3;
            else
                next_state = 4;
        end
        /* 4: Output*/
        else if (state == 4) begin
            if (out_ready == 1 && last_valid == 1) begin
                if(output_count < NUM_OUTPUT)
                  next_state = 3;
                else
                  next_state = 0;
            end else
              next_state = 4;
        end
    end

    /* Clear accumulator states 0 and 4 (when final output has been generated and saved*/
    assign clear_acc = (state == 0) || (state == 4 && last_valid == 1)? 1 : 0;
    /* Take input in states 1 and 2 when address is in range(so we don't lose any input value)*/
    assign in_ready = (state == 1 && addr_count_w < W_SIZE) || (state == 2 && addr_count_x < X_SIZE)? 1 : 0; 
    /* Enable write signal for Matrix when there is valid input and address is in range */
    assign wr_en_w = (state == 1 && in_valid == 1 && addr_count_w < W_SIZE)? 1 : 0;
    /* Enable write signal for Vector when there is valid input and address is in range */
    assign wr_en_x = (state == 2) && (in_valid == 1) && (addr_count_x < X_SIZE)? 1 : 0;
    /* Assert out_valid in output state and only when the final output has been generated and saved */
    assign out_valid = (state == 4 && last_valid == 1)? 1 : 0;
    
    /* Vector address*/
    always_ff @(posedge clk)begin
        // Clear address in reset state, after loading entire vector in state 2 (compute state takes value from begnning)
        // and state 4 (one iteraton has completed)
        if(state == 0 || (state == 2 && addr_count_x == X_SIZE) || state == 4) begin
            addr_x <= 2'b00;
            addr_count_x <= 4'b0000;
        // Increment in state 2 (only when there is a valid input on that clk edge) and 3
        end else if ((state == 2 && in_valid ==1)|| state == 3) begin
            addr_x <= addr_x + 1;
            addr_count_x <= addr_count_x + 1;
        end else begin
            addr_x <= addr_x; 
            addr_count_x <= addr_count_x;
        end
    end

    /* Matrix address*/
    always_ff @(posedge clk)begin
        // Clear address in reset state and state 2 (compute state takes value from begnning)
        if(state == 0 || state == 2) begin
            addr_w <= 4'b0000;
            addr_count_w <= 5'b00000;
        // Increment in state 1 (only when there is a valid input on that clk edge) and 3
        end else if((state==1 && in_valid==1) || state == 3) begin
            addr_w <= addr_w + 1;
            addr_count_w <= addr_count_w + 1;
        end else begin
            addr_w <= addr_w;
            addr_count_w <= addr_count_w;
        end
    end

    /* Accumulation counter*/
    always_ff @(posedge clk)begin
        // Clear counter in reset state and output state
        if(state == 0 || state == 4)
            acc_count <= 4'b0000;
        // Increment in compute state
        else if(state == 3)
            acc_count <= acc_count + 1;
        else
            acc_count <= acc_count;
    end

    /* Output counter*/
    always_ff @(posedge clk)begin
        // Clear counter in reset state
        if(state == 0)
            output_count <= 4'b0000;
        // Increment in state 4 only when the final value has been generated and the output port is ready to accept output
        else if(state == 4 && out_ready == 1 && last_valid == 1)
            output_count <= output_count + 1;
        else
            output_count <= output_count;
    end

    /* Enable MAC*/
    always_ff @(posedge clk) begin
        // Enable MAC unit in state 3
        if(state == 3)
            en_acc <= 1;
        else
            en_acc <= 0;
    end

endmodule          
