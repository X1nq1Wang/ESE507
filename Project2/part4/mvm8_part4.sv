/*
ESE 507 Project 2 Part 4
by Xinqi Wang, Jinkyu Lee
modified on: 10/31/20
*/

module mvm8_part4(clk, reset, input_valid, input_ready, input_data, new_matrix, output_valid, output_ready, output_data);
    input                   clk, reset, input_valid, new_matrix, output_ready;
    input  signed [11:0]    input_data;
    output signed [23:0]    output_data;
    output                  output_valid, input_ready;

    // Internal connections
    logic                       clk, reset, wr_en_x, wr_en_w0, wr_en_w1, wr_en_w2, wr_en_w3, wr_en_w4, wr_en_w5,                                        wr_en_w6, wr_en_w7, clear_acc, en_acc;
    logic [2:0]                 addr_x;
    logic [2:0]                 addr_w0, addr_w1, addr_w2, addr_w3, addr_w4, addr_w5, addr_w6, addr_w7;
    logic [4:0]                 out_addr;
    logic [2:0]                 delay_ctrl;

    /* Instantiate Datapath */
    datapath #(12, 8, 64) dp( 
                            //Inputs
                            .in_data(input_data), .clk(clk), .reset(reset), .clear_acc(clear_acc), .en_acc(en_acc), 
                            .addr_x(addr_x), .wr_en_x(wr_en_x),    //vector
                            .addr_w0(addr_w0), .addr_w1(addr_w1), .addr_w2(addr_w2), .addr_w3(addr_w3), //matrix
                            .addr_w4(addr_w4), .addr_w5(addr_w5), .addr_w6(addr_w6), .addr_w7(addr_w7), 
                            .wr_en_w0(wr_en_w0), .wr_en_w1(wr_en_w1), .wr_en_w2(wr_en_w2), .wr_en_w3(wr_en_w3), 
                            .wr_en_w4(wr_en_w4), .wr_en_w5(wr_en_w5), .wr_en_w6(wr_en_w6), .wr_en_w7(wr_en_w7),
                            .out_addr(out_addr), .delay_ctrl(delay_ctrl),
                            //Outputs
                            .out_data(output_data));

    /* Instantiate Control */
    control #(12, 8, 64) ctrl(
                            //Inputs
                            .clk(clk), .reset(reset), .in_valid(input_valid), .out_ready(output_ready), .new_matrix(new_matrix),
                            //Outputs
                            .addr_x(addr_x), .wr_en_x(wr_en_x),    //vector
                            .addr_w0(addr_w0), .addr_w1(addr_w1), .addr_w2(addr_w2), .addr_w3(addr_w3), //matrix
                            .addr_w4(addr_w4), .addr_w5(addr_w5), .addr_w6(addr_w6), .addr_w7(addr_w7), 
                            .wr_en_w0(wr_en_w0), .wr_en_w1(wr_en_w1), .wr_en_w2(wr_en_w2), .wr_en_w3(wr_en_w3), 
                            .wr_en_w4(wr_en_w4), .wr_en_w5(wr_en_w5), .wr_en_w6(wr_en_w6), .wr_en_w7(wr_en_w7), 
                            .clear_acc(clear_acc), .en_acc(en_acc), .in_ready(input_ready), 
                            .out_valid(output_valid), .out_addr(out_addr),
                            .delay_ctrl(delay_ctrl));
endmodule

/*DataPath*/
module datapath(in_data, clk, reset, 
                addr_x, wr_en_x, 
                addr_w0, addr_w1, addr_w2, addr_w3, addr_w4, addr_w5, addr_w6, addr_w7, 
                wr_en_w0, wr_en_w1, wr_en_w2, wr_en_w3, wr_en_w4, wr_en_w5, wr_en_w6, wr_en_w7, 
                clear_acc, en_acc, out_data, out_addr, delay_ctrl);
    parameter                           WIDTH = 12, X_SIZE = 4, W_SIZE = 16; 
    localparam                          LOGSIZE_X=$clog2(X_SIZE);
    localparam                          LOGSIZE_W=$clog2(W_SIZE);

    input                               clk, reset, wr_en_x, wr_en_w0, wr_en_w1, wr_en_w2, wr_en_w3, wr_en_w4, wr_en_w5, wr_en_w6, wr_en_w7, clear_acc, en_acc;
    input signed    [WIDTH-1:0]         in_data;
    input           [LOGSIZE_X-1:0]     addr_x;
                                        //Since we store each row as a separate memory, W addr bits = X addr bits
    input           [LOGSIZE_X-1:0]     addr_w0, addr_w1, addr_w2, addr_w3, addr_w4, addr_w5, addr_w6, addr_w7;
    input           [4:0]               out_addr;
    input           [2:0]               delay_ctrl;

    output signed   [WIDTH*2-1:0]       out_data;

    logic                               acc_clr;                                                // Clear MAC
    logic signed    [WIDTH-1:0]         w_in0, w_in1, w_in2, w_in3, w_in4, w_in5, w_in6, w_in7; // Matrix input
    logic signed    [WIDTH-1:0]         x_in;                                                   // Vector input
    logic signed    [WIDTH*2-1:0]       mac_out0, mac_out1, mac_out2, mac_out3, mac_out4, mac_out5, mac_out6, mac_out7;           // MAC output
    logic                               mac_out_valid0, mac_out_valid1, mac_out_valid2, mac_out_valid3, mac_out_valid4,                                         mac_out_valid5, mac_out_valid6, mac_out_valid7;     // MAC valid output

    logic signed    [X_SIZE-1:0][WIDTH*2-1:0]     out_v;

    memory #(WIDTH, X_SIZE) vector(.clk(clk), .data_in(in_data), .addr(addr_x), .wr_en(wr_en_x), .data_out(x_in));

    memory #(WIDTH, X_SIZE) matrix0(.clk(clk), .data_in(in_data), .addr(addr_w0),  .wr_en(wr_en_w0), .data_out(w_in0));
    memory #(WIDTH, X_SIZE) matrix1(.clk(clk), .data_in(in_data), .addr(addr_w1),  .wr_en(wr_en_w1), .data_out(w_in1));
    memory #(WIDTH, X_SIZE) matrix2(.clk(clk), .data_in(in_data), .addr(addr_w2),  .wr_en(wr_en_w2), .data_out(w_in2));
    memory #(WIDTH, X_SIZE) matrix3(.clk(clk), .data_in(in_data), .addr(addr_w3),  .wr_en(wr_en_w3), .data_out(w_in3));
    memory #(WIDTH, X_SIZE) matrix4(.clk(clk), .data_in(in_data), .addr(addr_w4),  .wr_en(wr_en_w4), .data_out(w_in4));
    memory #(WIDTH, X_SIZE) matrix5(.clk(clk), .data_in(in_data), .addr(addr_w5),  .wr_en(wr_en_w5), .data_out(w_in5));
    memory #(WIDTH, X_SIZE) matrix6(.clk(clk), .data_in(in_data), .addr(addr_w6),  .wr_en(wr_en_w6), .data_out(w_in6));
    memory #(WIDTH, X_SIZE) matrix7(.clk(clk), .data_in(in_data), .addr(addr_w7),  .wr_en(wr_en_w7), .data_out(w_in7));

    mac mc0(.clk(clk), .reset(acc_clr), .a(w_in0), .b(x_in), .valid_in(en_acc), .f(mac_out0), .valid_out(mac_out_valid0));
    mac mc1(.clk(clk), .reset(acc_clr), .a(w_in1), .b(x_in), .valid_in(en_acc), .f(mac_out1), .valid_out(mac_out_valid1));
    mac mc2(.clk(clk), .reset(acc_clr), .a(w_in2), .b(x_in), .valid_in(en_acc), .f(mac_out2), .valid_out(mac_out_valid2));
    mac mc3(.clk(clk), .reset(acc_clr), .a(w_in3), .b(x_in), .valid_in(en_acc), .f(mac_out3), .valid_out(mac_out_valid3));
    mac mc4(.clk(clk), .reset(acc_clr), .a(w_in4), .b(x_in), .valid_in(en_acc), .f(mac_out4), .valid_out(mac_out_valid4));
    mac mc5(.clk(clk), .reset(acc_clr), .a(w_in5), .b(x_in), .valid_in(en_acc), .f(mac_out5), .valid_out(mac_out_valid5));
    mac mc6(.clk(clk), .reset(acc_clr), .a(w_in6), .b(x_in), .valid_in(en_acc), .f(mac_out6), .valid_out(mac_out_valid6));
    mac mc7(.clk(clk), .reset(acc_clr), .a(w_in7), .b(x_in), .valid_in(en_acc), .f(mac_out7), .valid_out(mac_out_valid7));


    always_comb begin
        acc_clr = reset || clear_acc;
    end

    always_ff @(posedge clk) begin
        // When delay counts to 1, load all valid mac outputs to the output vector
        if(delay_ctrl == 3)begin  
            out_v[0] <= mac_out0;
            out_v[1] <= mac_out1;
            out_v[2] <= mac_out2;
            out_v[3] <= mac_out3;
            out_v[4] <= mac_out4;
            out_v[5] <= mac_out5;
            out_v[6] <= mac_out6;
            out_v[7] <= mac_out7;
        end  
        else 
            out_v <= out_v;
    end

    //Assign out_data to next output in the vector
    assign out_data = (out_addr==0)? out_v[0] : (
                      (out_addr==1)? out_v[1] : (
                      (out_addr==2)? out_v[2] : (
                      (out_addr==3)? out_v[3] : (
                      (out_addr==4)? out_v[4] : (
                      (out_addr==5)? out_v[5] : (
                      (out_addr==6)? out_v[6] : (
                      (out_addr==7)? out_v[7] : 0)))))));    
endmodule

/*ControlPath*/
module control(clk, reset, in_valid, out_ready, 
               addr_x, wr_en_x, 
               addr_w0, addr_w1, addr_w2, addr_w3, addr_w4, addr_w5, addr_w6, addr_w7, 
               wr_en_w0, wr_en_w1, wr_en_w2, wr_en_w3, wr_en_w4, wr_en_w5, wr_en_w6, wr_en_w7, 
               clear_acc, en_acc, in_ready, out_valid, new_matrix, out_addr, delay_ctrl);
    parameter                     WIDTH = 12, X_SIZE = 4, W_SIZE = 16; 
    localparam                    LOGSIZE_X=$clog2(X_SIZE);
    localparam                    LOGSIZE_W=$clog2(W_SIZE);
    localparam                    NUM_ACC=X_SIZE;
    localparam                    NUM_OUTPUT = W_SIZE / X_SIZE;

    input                         clk, reset, in_valid, out_ready, new_matrix;
    
    output                        wr_en_x;
    output                        wr_en_w0, wr_en_w1, wr_en_w2, wr_en_w3, wr_en_w4, wr_en_w5, wr_en_w6, wr_en_w7;
    output                        clear_acc;
    output logic                  en_acc; 
    output logic [LOGSIZE_X-1:0]  addr_x;
    output logic [LOGSIZE_X-1:0]  addr_w0, addr_w1, addr_w2, addr_w3, addr_w4, addr_w5, addr_w6, addr_w7;
    output                        in_ready, out_valid;
    output       [4:0]            out_addr;
    output logic [2:0]            delay_ctrl;

    logic        [1:0]             state, next_state;
    logic        [3:0]             acc_count;          // Keep track of number of accumulations
    logic        [4:0]             output_count;       // Keep track of number of outputs generated
    logic        [LOGSIZE_W+1:0]   addr_count_w;       // Keep track of Matrix address
    logic        [LOGSIZE_X+1:0]   addr_count_x;       // Keep track of Vector address
    logic                          last_valid;

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
                if(new_matrix == 1) // New matrix
                    next_state = 1;
                else 
                    next_state = 2;
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
            if (addr_count_x < X_SIZE)  // Vector not loaded
                next_state = 2;
            else if (delay_ctrl == 4 && output_count < 8) // Outputing previous results not done
                next_state = 2;
            else
                next_state = 3;
        end
        /* 3: Compute*/
        else if (state == 3) begin
            if(last_valid == 0) // Output not loaded to output vector yet
                next_state = 3;
            else
                next_state = 0;
        end
    end

    /* Clear accumulator in states 0*/
    assign clear_acc = (state == 0)? 1 : 0;
    /* Take input in states 1 and 2 when address is in range(so we don't lose any input value)*/
    assign in_ready = (state == 1 && addr_count_w < W_SIZE) || (state == 2 && addr_count_x < X_SIZE)? 1 : 0; 
    /* Enable write signal for corresponding Matrix memory when there is valid input and address is in range */
    assign wr_en_w0 = (state == 1 && in_valid == 1 && addr_count_w >= 0 && addr_count_w < 8)? 1 : 0;
    assign wr_en_w1 = (state == 1 && in_valid == 1 && addr_count_w >= 8 && addr_count_w < 16)? 1 : 0;
    assign wr_en_w2 = (state == 1 && in_valid == 1 && addr_count_w >= 16 && addr_count_w < 24)? 1 : 0;
    assign wr_en_w3 = (state == 1 && in_valid == 1 && addr_count_w >= 24 && addr_count_w < 32)? 1 : 0;
    assign wr_en_w4 = (state == 1 && in_valid == 1 && addr_count_w >= 32 && addr_count_w < 40)? 1 : 0;
    assign wr_en_w5 = (state == 1 && in_valid == 1 && addr_count_w >= 40 && addr_count_w < 48)? 1 : 0;
    assign wr_en_w6 = (state == 1 && in_valid == 1 && addr_count_w >= 48 && addr_count_w < 56)? 1 : 0;
    assign wr_en_w7 = (state == 1 && in_valid == 1 && addr_count_w >= 56 && addr_count_w < 64)? 1 : 0;
    /* Enable write signal for Vector when there is valid input and address is in range */
    assign wr_en_x = (state == 2) && (in_valid == 1) && (addr_count_x < X_SIZE)? 1 : 0;
    /* Assert out_valid while last_valid is asserted and not all outputs have been accepted */
    assign out_valid = (last_valid == 1 && output_count < NUM_OUTPUT )? 1 : 0;
    assign out_addr = output_count;
    // assign done = (output_count >= NUM_OUTPUT)? 1:0;
    
    /* Vector address*/
    always_ff @(posedge clk)begin
        // Clear address in reset state
        if(state == 0) begin
            addr_x <= 2'b00;
            addr_count_x <= 5'b00000;
        end 
        // Clear address in state 2 when new vector has been loaded and all outputs from previous iteration have been accepted
        else if (state == 2 && delay_ctrl == 4 && addr_count_x >= X_SIZE && output_count >= 8)begin
                addr_x <= 2'b00;
                addr_count_x <= 5'b00000;
        end
        // Clear address in state 2 if delay never counts to 2 (no previous outputs generated) and a new vector has been loaded
        // NOTE: This should happen in the very first computation
        else if (state == 2 && delay_ctrl < 4 && addr_count_x >= X_SIZE)begin
                addr_x <= 2'b00;
                addr_count_x <= 5'b00000;
        end 
        // Increment when loading new vector or when accessing element in compute state
        else if ((state == 2 && in_valid ==1 && addr_count_x < X_SIZE)|| state == 3) begin
            addr_x <= addr_x + 1;
            addr_count_x <= addr_count_x + 1;
        end else begin
            addr_x <= addr_x; 
            addr_count_x <= addr_count_x; 
        end
    end

    /* Matrix addressor*/
    always_ff @(posedge clk)begin
        // Clear address in reset state and state 2 (compute state takes value from begnning)
        if(state == 0 || state == 2) begin
            addr_w0 <= 4'b0000;
            addr_w1 <= 4'b0000;
            addr_w2 <= 4'b0000;
            addr_w3 <= 4'b0000;
            addr_w4 <= 4'b0000;
            addr_w5 <= 4'b0000;
            addr_w6 <= 4'b0000;
            addr_w7 <= 4'b0000;
            addr_count_w <= 6'b000000;
        end 
        // Increment corresponding W addresses in state 1
        else if(state==1 && in_valid==1) begin
            if (addr_count_w >= 0 && addr_count_w < 8)
                addr_w0 <= addr_w0 + 1;
            if (addr_count_w >= 8 && addr_count_w < 16)
                addr_w1 <= addr_w1 + 1;
            if (addr_count_w >= 16 && addr_count_w < 24)
                addr_w2 <= addr_w2 + 1;
            if (addr_count_w >= 24 && addr_count_w < 32)
                addr_w3 <= addr_w3 + 1;
            if (addr_count_w >= 32 && addr_count_w < 40)
                addr_w4 <= addr_w4 + 1;
            if (addr_count_w >= 40 && addr_count_w < 48)
                addr_w5 <= addr_w5 + 1;
            if (addr_count_w >= 48 && addr_count_w < 56)
                addr_w6 <= addr_w6 + 1;
            if (addr_count_w >= 56 && addr_count_w < 64)
                addr_w7 <= addr_w7 + 1;
            addr_count_w <= addr_count_w + 1;
        end 
        // Increment all addresses concurrently in compute state
        else if (state == 3) begin
            addr_w0 <= addr_w0 + 1;
            addr_w1 <= addr_w1 + 1;
            addr_w2 <= addr_w2 + 1;
            addr_w3 <= addr_w3 + 1;
            addr_w4 <= addr_w4 + 1;
            addr_w5 <= addr_w5 + 1;
            addr_w6 <= addr_w6 + 1;
            addr_w7 <= addr_w7 + 1;
            addr_count_w <= addr_count_w + 1;
        end 
        else begin
            addr_w0 <= addr_w0;
            addr_w1 <= addr_w1;
            addr_w2 <= addr_w2;
            addr_w3 <= addr_w3;
            addr_w4 <= addr_w4;
            addr_w5 <= addr_w5;
            addr_w6 <= addr_w6;
            addr_w7 <= addr_w7;
            addr_count_w <= addr_count_w;
        end
    end

    /* Accumulation counter*/
    always_ff @(posedge clk)begin
        // Clear counter in reset state and output state
        if(state == 0)
            acc_count <= 4'b0000;
        // Increment in compute state
        else if(state == 3)
            acc_count <= acc_count + 1;
        else
            acc_count <= acc_count;
    end

    /* Output counter*/
    always_ff @(posedge clk)begin
        // Clear counter in reset state and compute state when output is not ready
        if(reset == 1 || (state == 3 && delay_ctrl < 4))
            output_count <= 5'b00000;
        // Increment when the final results have been loaded and the output port is ready to accept output
        else if(out_ready == 1 && last_valid == 1)
            output_count <= output_count + 1;
        else
            output_count <= output_count;
    end

    /* Enable MAC*/
    always_ff @(posedge clk) begin
        // Enable MAC unit in state 3
        if(state == 3 && acc_count < NUM_ACC )
            en_acc <= 1;
        else
            en_acc <= 0;
    end

    /* Delay Counter*/
    always_ff @(posedge clk) begin
        // Clear delay counter on reset and in compute state when loading inputs to MAC
        if(reset == 1 || en_acc == 1)                  
            delay_ctrl <= 0;
        // Once done feeding inputs to MAC units, start counting (MAC unit from proj1 has a 2 cycle delay)
        else if (state == 3 && delay_ctrl < 4) 
            delay_ctrl <= delay_ctrl + 1;
        else
            delay_ctrl <= delay_ctrl;
    end    

    /* Final result*/
    always_ff @(posedge clk) begin
        // When delay counts to 1, output vecotr is loaded with outputs for this iteration
        // and will be accessible on the next edge (when delay counts to 2)
        // so, if not all outputs have been accepted by the port, we assert last_valid
        if(delay_ctrl == 4 && output_count < NUM_OUTPUT)  
            last_valid <= 1;
        else 
            last_valid <= 0;
    end    

endmodule
