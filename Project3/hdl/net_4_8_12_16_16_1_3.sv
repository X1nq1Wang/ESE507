module net_4_8_12_16_16_1_3(clk, reset, input_valid, input_ready, input_data, output_valid, output_ready, output_data);
   localparam N = 4;
   localparam M1 = 8;
   localparam M2 = 12;
   localparam M3 = 16;
   localparam T = 16;
   localparam R = 1;
   localparam B = 3;

   input                    clk, reset, input_valid, output_ready;
   input  signed [T-1:0]    input_data;
   output                   output_valid, input_ready;
   output signed [T-1:0]    output_data;

   logic                    output_valid_l1;
   logic  signed [T-1:0]    output_data_l1;
   logic                    input_ready_l2, output_valid_l2, output_ready_l2;
   logic  signed [T-1:0]    output_data_l2;
   logic                    input_ready_l3;

   l1_fc_8_4_16_1_1 l1(.clk(clk), .reset(reset), .input_valid(input_valid), .input_ready(input_ready), .input_data(input_data),
                       .output_valid(output_valid_l1), .output_ready(input_ready_l2), .output_data(output_data_l1));
   l2_fc_12_8_16_1_1 l2(.clk(clk), .reset(reset), .input_valid(output_valid_l1), .input_ready(input_ready_l2), .input_data(output_data_l1),
                       .output_valid(output_valid_l2), .output_ready(input_ready_l3), .output_data(output_data_l2));
   l3_fc_16_12_16_1_1 l3(.clk(clk), .reset(reset), .input_valid(output_valid_l2), .input_ready(input_ready_l3), .input_data(output_data_l2),
                      .output_valid(output_valid), .output_ready(output_ready), .output_data(output_data));

endmodule

module l1_fc_8_4_16_1_1(clk, reset, input_valid, input_ready, input_data, output_valid, output_ready, output_data);
    localparam              WIDTH = 16;
    localparam              W_M = 8;
    localparam              W_N = 4;
    localparam              P = 1;
    localparam              R = 1;
    localparam              LOGSIZE_X = $clog2(W_N);
    localparam              LOGSIZE_W = $clog2(W_M/P * W_N);
    localparam              NUM_OUTPUT = $clog2(P);

    input                           clk, reset, input_valid, output_ready;
    input  signed [WIDTH-1:0]       input_data;
    output signed [WIDTH-1:0]       output_data;
    output                          output_valid, input_ready;

    logic                           clk, reset, wr_en_x, clear_acc, en_acc;
    logic [LOGSIZE_X-1:0]           addr_x;
    logic [LOGSIZE_W-1:0]           addr_w;
    logic [NUM_OUTPUT:0]            out_addr;
    logic [1:0]                     delay_ctrl;

    l1_fc_8_4_16_1_1_datapath #(WIDTH, W_M, W_N, P, R) dp( 
                            .in_data(input_data), .clk(clk), .reset(reset), .clear_acc(clear_acc), .en_acc(en_acc), 
                            .addr_x(addr_x), .wr_en_x(wr_en_x), .addr_w(addr_w),
                            .out_addr(out_addr), .delay_ctrl(delay_ctrl),
                            .out_data(output_data));

    l1_fc_8_4_16_1_1_control #(WIDTH, W_M, W_N, P) ctrl(
                            .clk(clk), .reset(reset), .in_valid(input_valid), .out_ready(output_ready),
                            .addr_x(addr_x), .wr_en_x(wr_en_x), .addr_w(addr_w),
                            .clear_acc(clear_acc), .en_acc(en_acc), .in_ready(input_ready),
                            .out_valid(output_valid), .out_addr(out_addr),
                            .delay_ctrl(delay_ctrl));
endmodule

module l1_fc_8_4_16_1_1_datapath(in_data, clk, reset, 
                addr_x, wr_en_x, addr_w,
                clear_acc, en_acc, out_data, out_addr, delay_ctrl);
    parameter                           WIDTH = 20, W_M = 5, W_N = 4, P = 1, R = 1;
    localparam                          X_SIZE = W_N;
    localparam                          LOGSIZE_X=$clog2(W_N);
    localparam                          LOGSIZE_W=$clog2(W_M/P * W_N);
    localparam                          NUM_PER_ROM = W_M/P * W_N;
    localparam                          NUM_OUTPUT = $clog2(P);

    input                               clk, reset, clear_acc, en_acc;
    input signed    [WIDTH-1:0]         in_data;
    input                               wr_en_x;
    input           [LOGSIZE_X-1:0]     addr_x;
    input           [LOGSIZE_W-1:0]     addr_w;
    input           [NUM_OUTPUT:0]      out_addr;
    input           [1:0]               delay_ctrl;

    output signed   [WIDTH-1:0]         out_data;

    logic                               acc_clr;
    logic signed    [WIDTH-1:0]         w_in;
    logic signed    [WIDTH-1:0]         x_in;
    logic signed    [WIDTH-1:0]         mac_out;
    logic                               mac_out_valid;

    logic signed    [P:0][WIDTH-1:0]     out_v;
    logic signed    [WIDTH-1:0]          out;

    memory #(WIDTH, X_SIZE) l1_fc_8_4_16_1_1_vector(.clk(clk), .data_in(in_data), .addr(addr_x), .wr_en(wr_en_x), .data_out(x_in));
    l1_fc_8_4_16_1_1_W_rom l1_fc_8_4_16_1_1_w(.clk(clk), .addr(addr_w), .z(w_in));
    mac #(WIDTH) l1_fc_8_4_16_1_1_mc(.clk(clk), .reset(acc_clr), .a(w_in), .b(x_in), .valid_in(en_acc), .f(mac_out), .valid_out(mac_out_valid));

    always_comb begin
        acc_clr = reset || clear_acc;
    end
    always_ff @(posedge clk) begin
        if(delay_ctrl == 2)begin
            if (R==0)begin
                out_v[0] <= mac_out;
            end
            else begin
                out_v[0] <= (mac_out > 0)? mac_out : 0;
            end
        end
        else
            out_v <= out_v;
        end
        always_comb begin
            case(out_addr)
                0: out = out_v[0];
                default: out = 0;
            endcase
        end
        assign out_data = out;
endmodule

module l1_fc_8_4_16_1_1_control(clk, reset, in_valid, out_ready,
               addr_x, wr_en_x, addr_w, 
               clear_acc, en_acc, in_ready, out_valid, out_addr, delay_ctrl);
    parameter                     WIDTH = 20, W_M = 5, W_N = 4, P = 1;
    localparam                    X_SIZE = W_N;
    localparam                    LOGSIZE_X=$clog2(W_N);
    localparam                    LOGSIZE_W=$clog2(W_M/P * W_N);
    localparam                    NUM_ACC=W_N;
    localparam                    LOG_NUM_ACC = $clog2(W_N);
    localparam                    NUM_OUTPUT = W_M;
    localparam                    LOG_NUM_OUTPUT = $clog2(W_M);
    localparam                    NUM_ITER = P;
    localparam                    LOG_NUM_ITER = $clog2(P);

    input                                   clk, reset, in_valid, out_ready;

    output                                  wr_en_x;
    output                                  clear_acc;
    output logic                            en_acc;
    output logic [LOGSIZE_X-1:0]            addr_x;
    output logic [LOGSIZE_W-1:0]            addr_w;
    output                                  in_ready, out_valid;
    output       [LOG_NUM_ITER:0]           out_addr;
    output logic [1:0]                      delay_ctrl;

    logic        [2:0]                      state, next_state;
    logic        [LOG_NUM_ACC:0]            acc_count;
    logic        [LOG_NUM_OUTPUT:0]         output_count;
    logic        [LOGSIZE_W+1:0]            addr_count_w; 
    logic        [LOGSIZE_X+1:0]            addr_count_x;
    logic                                   last_valid;
    logic        [LOG_NUM_ITER+1:0]         iter_count;

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
                next_state = 2;
            end 
            else
                next_state = 0;
        end
        else if(state == 2) begin
            if (addr_count_x < X_SIZE)
                next_state = 2;
            else if (delay_ctrl == 2 && output_count < W_M)
                next_state = 2;
            else
                next_state = 3;
        end
        /* 3: Compute*/
        else if (state == 3) begin
            if(last_valid == 0) 
                next_state = 3;
            else if (output_count < W_M-P)
                next_state = 4;
            else
                next_state = 0;
        end
        /* 4: Load output*/
        else if (state == 4) begin
            if(delay_ctrl == 2 && iter_count < P)
               next_state = 4;
            else
               next_state = 3;
        end
    end

    assign clear_acc = ((state == 2 && output_count >= W_M) ||(state == 4 && iter_count >= P) || reset == 1)? 1 : 0;
    assign in_ready = (state == 2 && addr_count_x < X_SIZE)? 1 : 0; 
    assign wr_en_x = (state == 2) && (in_valid == 1) && (addr_count_x < X_SIZE)? 1 : 0;
    assign out_valid = ( last_valid == 1 && (((state == 2 || state == 0) && output_count < W_M) || (state == 4 && iter_count < P)))? 1 : 0;
    assign out_addr = iter_count;

    always_ff @(posedge clk)begin
        if(state == 0) begin
            addr_x <= 0;
            addr_count_x <= 0;
        end 
        else if (state == 2 && addr_count_x >= X_SIZE && output_count >= W_M)begin
                addr_x <= 0;
                addr_count_x <= 0;
        end
        else if (state == 2 && delay_ctrl < 2 && addr_count_x >= X_SIZE)begin
                addr_x <= 0;
                addr_count_x <= 0;
        end
        else if (state == 4 && iter_count >= P)begin
                addr_x <= 0;
                addr_count_x <= 0;
        end
        else if ((state == 2 && in_valid == 1 && addr_count_x < X_SIZE)|| (state == 3 && acc_count < NUM_ACC)) begin
            addr_x <= addr_x + 1;
            addr_count_x <= addr_count_x + 1;
        end else begin
            addr_x <= addr_x;
            addr_count_x <= addr_count_x;
        end
    end

    always_ff @(posedge clk)begin
        if(state == 0 || state == 2) begin
            addr_w <= 0;
            addr_count_w <= 0;
        end 
        else if (state == 3 && acc_count < NUM_ACC) begin
            addr_w <= addr_w + 1;
            addr_count_w <= addr_count_w + 1;
        end
        else begin
            addr_w <= addr_w;
            addr_count_w <= addr_count_w;
        end
    end

    /* Accumulation Counter*/
    always_ff @(posedge clk)begin
        if(state == 0 | state == 4)
            acc_count <= 0;
        else if(state == 3 && acc_count < NUM_ACC)
            acc_count <= acc_count + 1;
        else
            acc_count <= acc_count;
    end

    always_ff @(posedge clk)begin
        if(reset == 1 || (state == 2 && addr_count_x >= X_SIZE && output_count >= W_M))
            output_count <= 0;
        else if(out_ready == 1 && out_valid == 1 && (((state == 2 || state == 0) && output_count < W_M) || (state == 4 && iter_count < P)))
            output_count <= output_count + 1;
        else
            output_count <= output_count;
    end

    always_ff @(posedge clk) begin
        if(state == 3 && acc_count < NUM_ACC )
            en_acc <= 1;
        else
            en_acc <= 0;
    end

    always_ff @(posedge clk)begin
       if(reset == 1 | state == 3)
         iter_count <= 0;
       else if ((state == 4 || state == 2 || state == 0) && out_ready == 1 && out_valid == 1 && iter_count < P)
         iter_count <= iter_count + 1;
       else
         iter_count <= iter_count;
    end

    always_ff @(posedge clk) begin
        if(reset == 1 || en_acc == 1 || ((state == 4 || state ==3) && iter_count >= P))
            delay_ctrl <= 0;
        else if (state == 3 && delay_ctrl < 2)
            delay_ctrl <= delay_ctrl + 1;
        else
            delay_ctrl <= delay_ctrl;
    end

    always_ff @(posedge clk) begin
        if(delay_ctrl == 2 && (((state == 4||state == 3) && iter_count < P) || ((state == 2 || state == 0) && output_count < W_M))) 
            last_valid <= 1;
        else
            last_valid <= 0;
    end
endmodule

module l1_fc_8_4_16_1_1_W_rom(clk, addr, z);
   input clk;
   input [4:0] addr;
   output logic signed [15:0] z;
   always_ff @(posedge clk) begin
      case(addr)
        0: z <= 16'd0;
        1: z <= -16'd4;
        2: z <= 16'd2;
        3: z <= 16'd2;
        4: z <= 16'd7;
        5: z <= -16'd7;
        6: z <= -16'd4;
        7: z <= 16'd4;
        8: z <= -16'd4;
        9: z <= 16'd1;
        10: z <= -16'd1;
        11: z <= -16'd6;
        12: z <= -16'd1;
        13: z <= -16'd7;
        14: z <= -16'd6;
        15: z <= 16'd7;
        16: z <= 16'd0;
        17: z <= 16'd2;
        18: z <= 16'd6;
        19: z <= -16'd7;
        20: z <= -16'd7;
        21: z <= 16'd3;
        22: z <= -16'd2;
        23: z <= 16'd5;
        24: z <= 16'd2;
        25: z <= 16'd6;
        26: z <= 16'd5;
        27: z <= 16'd2;
        28: z <= -16'd3;
        29: z <= 16'd6;
        30: z <= -16'd2;
        31: z <= 16'd5;
      endcase
   end
endmodule

module l2_fc_12_8_16_1_1(clk, reset, input_valid, input_ready, input_data, output_valid, output_ready, output_data);
    localparam              WIDTH = 16;
    localparam              W_M = 12;
    localparam              W_N = 8;
    localparam              P = 1;
    localparam              R = 1;
    localparam              LOGSIZE_X = $clog2(W_N);
    localparam              LOGSIZE_W = $clog2(W_M/P * W_N);
    localparam              NUM_OUTPUT = $clog2(P);

    input                           clk, reset, input_valid, output_ready;
    input  signed [WIDTH-1:0]       input_data;
    output signed [WIDTH-1:0]       output_data;
    output                          output_valid, input_ready;

    logic                           clk, reset, wr_en_x, clear_acc, en_acc;
    logic [LOGSIZE_X-1:0]           addr_x;
    logic [LOGSIZE_W-1:0]           addr_w;
    logic [NUM_OUTPUT:0]            out_addr;
    logic [1:0]                     delay_ctrl;

    l2_fc_12_8_16_1_1_datapath #(WIDTH, W_M, W_N, P, R) dp( 
                            .in_data(input_data), .clk(clk), .reset(reset), .clear_acc(clear_acc), .en_acc(en_acc), 
                            .addr_x(addr_x), .wr_en_x(wr_en_x), .addr_w(addr_w),
                            .out_addr(out_addr), .delay_ctrl(delay_ctrl),
                            .out_data(output_data));

    l2_fc_12_8_16_1_1_control #(WIDTH, W_M, W_N, P) ctrl(
                            .clk(clk), .reset(reset), .in_valid(input_valid), .out_ready(output_ready),
                            .addr_x(addr_x), .wr_en_x(wr_en_x), .addr_w(addr_w),
                            .clear_acc(clear_acc), .en_acc(en_acc), .in_ready(input_ready),
                            .out_valid(output_valid), .out_addr(out_addr),
                            .delay_ctrl(delay_ctrl));
endmodule

module l2_fc_12_8_16_1_1_datapath(in_data, clk, reset, 
                addr_x, wr_en_x, addr_w,
                clear_acc, en_acc, out_data, out_addr, delay_ctrl);
    parameter                           WIDTH = 20, W_M = 5, W_N = 4, P = 1, R = 1;
    localparam                          X_SIZE = W_N;
    localparam                          LOGSIZE_X=$clog2(W_N);
    localparam                          LOGSIZE_W=$clog2(W_M/P * W_N);
    localparam                          NUM_PER_ROM = W_M/P * W_N;
    localparam                          NUM_OUTPUT = $clog2(P);

    input                               clk, reset, clear_acc, en_acc;
    input signed    [WIDTH-1:0]         in_data;
    input                               wr_en_x;
    input           [LOGSIZE_X-1:0]     addr_x;
    input           [LOGSIZE_W-1:0]     addr_w;
    input           [NUM_OUTPUT:0]      out_addr;
    input           [1:0]               delay_ctrl;

    output signed   [WIDTH-1:0]         out_data;

    logic                               acc_clr;
    logic signed    [WIDTH-1:0]         w_in;
    logic signed    [WIDTH-1:0]         x_in;
    logic signed    [WIDTH-1:0]         mac_out;
    logic                               mac_out_valid;

    logic signed    [P:0][WIDTH-1:0]     out_v;
    logic signed    [WIDTH-1:0]          out;

    memory #(WIDTH, X_SIZE) l2_fc_12_8_16_1_1_vector(.clk(clk), .data_in(in_data), .addr(addr_x), .wr_en(wr_en_x), .data_out(x_in));
    l2_fc_12_8_16_1_1_W_rom l2_fc_12_8_16_1_1_w(.clk(clk), .addr(addr_w), .z(w_in));
    mac #(WIDTH) l2_fc_12_8_16_1_1_mc(.clk(clk), .reset(acc_clr), .a(w_in), .b(x_in), .valid_in(en_acc), .f(mac_out), .valid_out(mac_out_valid));

    always_comb begin
        acc_clr = reset || clear_acc;
    end
    always_ff @(posedge clk) begin
        if(delay_ctrl == 2)begin
            if (R==0)begin
                out_v[0] <= mac_out;
            end
            else begin
                out_v[0] <= (mac_out > 0)? mac_out : 0;
            end
        end
        else
            out_v <= out_v;
        end
        always_comb begin
            case(out_addr)
                0: out = out_v[0];
                default: out = 0;
            endcase
        end
        assign out_data = out;
endmodule

module l2_fc_12_8_16_1_1_control(clk, reset, in_valid, out_ready,
               addr_x, wr_en_x, addr_w, 
               clear_acc, en_acc, in_ready, out_valid, out_addr, delay_ctrl);
    parameter                     WIDTH = 20, W_M = 5, W_N = 4, P = 1;
    localparam                    X_SIZE = W_N;
    localparam                    LOGSIZE_X=$clog2(W_N);
    localparam                    LOGSIZE_W=$clog2(W_M/P * W_N);
    localparam                    NUM_ACC=W_N;
    localparam                    LOG_NUM_ACC = $clog2(W_N);
    localparam                    NUM_OUTPUT = W_M;
    localparam                    LOG_NUM_OUTPUT = $clog2(W_M);
    localparam                    NUM_ITER = P;
    localparam                    LOG_NUM_ITER = $clog2(P);

    input                                   clk, reset, in_valid, out_ready;

    output                                  wr_en_x;
    output                                  clear_acc;
    output logic                            en_acc;
    output logic [LOGSIZE_X-1:0]            addr_x;
    output logic [LOGSIZE_W-1:0]            addr_w;
    output                                  in_ready, out_valid;
    output       [LOG_NUM_ITER:0]           out_addr;
    output logic [1:0]                      delay_ctrl;

    logic        [2:0]                      state, next_state;
    logic        [LOG_NUM_ACC:0]            acc_count;
    logic        [LOG_NUM_OUTPUT:0]         output_count;
    logic        [LOGSIZE_W+1:0]            addr_count_w; 
    logic        [LOGSIZE_X+1:0]            addr_count_x;
    logic                                   last_valid;
    logic        [LOG_NUM_ITER+1:0]         iter_count;

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
                next_state = 2;
            end 
            else
                next_state = 0;
        end
        else if(state == 2) begin
            if (addr_count_x < X_SIZE)
                next_state = 2;
            else if (delay_ctrl == 2 && output_count < W_M)
                next_state = 2;
            else
                next_state = 3;
        end
        /* 3: Compute*/
        else if (state == 3) begin
            if(last_valid == 0) 
                next_state = 3;
            else if (output_count < W_M-P)
                next_state = 4;
            else
                next_state = 0;
        end
        /* 4: Load output*/
        else if (state == 4) begin
            if(delay_ctrl == 2 && iter_count < P)
               next_state = 4;
            else
               next_state = 3;
        end
    end

    assign clear_acc = ((state == 2 && output_count >= W_M) ||(state == 4 && iter_count >= P) || reset == 1)? 1 : 0;
    assign in_ready = (state == 2 && addr_count_x < X_SIZE)? 1 : 0; 
    assign wr_en_x = (state == 2) && (in_valid == 1) && (addr_count_x < X_SIZE)? 1 : 0;
    assign out_valid = ( last_valid == 1 && (((state == 2 || state == 0) && output_count < W_M) || (state == 4 && iter_count < P)))? 1 : 0;
    assign out_addr = iter_count;

    always_ff @(posedge clk)begin
        if(state == 0) begin
            addr_x <= 0;
            addr_count_x <= 0;
        end 
        else if (state == 2 && addr_count_x >= X_SIZE && output_count >= W_M)begin
                addr_x <= 0;
                addr_count_x <= 0;
        end
        else if (state == 2 && delay_ctrl < 2 && addr_count_x >= X_SIZE)begin
                addr_x <= 0;
                addr_count_x <= 0;
        end
        else if (state == 4 && iter_count >= P)begin
                addr_x <= 0;
                addr_count_x <= 0;
        end
        else if ((state == 2 && in_valid == 1 && addr_count_x < X_SIZE)|| (state == 3 && acc_count < NUM_ACC)) begin
            addr_x <= addr_x + 1;
            addr_count_x <= addr_count_x + 1;
        end else begin
            addr_x <= addr_x;
            addr_count_x <= addr_count_x;
        end
    end

    always_ff @(posedge clk)begin
        if(state == 0 || state == 2) begin
            addr_w <= 0;
            addr_count_w <= 0;
        end 
        else if (state == 3 && acc_count < NUM_ACC) begin
            addr_w <= addr_w + 1;
            addr_count_w <= addr_count_w + 1;
        end
        else begin
            addr_w <= addr_w;
            addr_count_w <= addr_count_w;
        end
    end

    /* Accumulation Counter*/
    always_ff @(posedge clk)begin
        if(state == 0 | state == 4)
            acc_count <= 0;
        else if(state == 3 && acc_count < NUM_ACC)
            acc_count <= acc_count + 1;
        else
            acc_count <= acc_count;
    end

    always_ff @(posedge clk)begin
        if(reset == 1 || (state == 2 && addr_count_x >= X_SIZE && output_count >= W_M))
            output_count <= 0;
        else if(out_ready == 1 && out_valid == 1 && (((state == 2 || state == 0) && output_count < W_M) || (state == 4 && iter_count < P)))
            output_count <= output_count + 1;
        else
            output_count <= output_count;
    end

    always_ff @(posedge clk) begin
        if(state == 3 && acc_count < NUM_ACC )
            en_acc <= 1;
        else
            en_acc <= 0;
    end

    always_ff @(posedge clk)begin
       if(reset == 1 | state == 3)
         iter_count <= 0;
       else if ((state == 4 || state == 2 || state == 0) && out_ready == 1 && out_valid == 1 && iter_count < P)
         iter_count <= iter_count + 1;
       else
         iter_count <= iter_count;
    end

    always_ff @(posedge clk) begin
        if(reset == 1 || en_acc == 1 || ((state == 4 || state ==3) && iter_count >= P))
            delay_ctrl <= 0;
        else if (state == 3 && delay_ctrl < 2)
            delay_ctrl <= delay_ctrl + 1;
        else
            delay_ctrl <= delay_ctrl;
    end

    always_ff @(posedge clk) begin
        if(delay_ctrl == 2 && (((state == 4||state == 3) && iter_count < P) || ((state == 2 || state == 0) && output_count < W_M))) 
            last_valid <= 1;
        else
            last_valid <= 0;
    end
endmodule

module l2_fc_12_8_16_1_1_W_rom(clk, addr, z);
   input clk;
   input [6:0] addr;
   output logic signed [15:0] z;
   always_ff @(posedge clk) begin
      case(addr)
        0: z <= -16'd5;
        1: z <= -16'd8;
        2: z <= 16'd0;
        3: z <= -16'd6;
        4: z <= -16'd6;
        5: z <= 16'd4;
        6: z <= 16'd6;
        7: z <= -16'd2;
        8: z <= -16'd3;
        9: z <= -16'd3;
        10: z <= 16'd0;
        11: z <= 16'd4;
        12: z <= -16'd2;
        13: z <= 16'd3;
        14: z <= 16'd3;
        15: z <= 16'd6;
        16: z <= -16'd3;
        17: z <= 16'd1;
        18: z <= 16'd7;
        19: z <= -16'd2;
        20: z <= -16'd4;
        21: z <= -16'd2;
        22: z <= -16'd5;
        23: z <= 16'd6;
        24: z <= -16'd4;
        25: z <= -16'd8;
        26: z <= 16'd1;
        27: z <= 16'd2;
        28: z <= 16'd7;
        29: z <= 16'd7;
        30: z <= -16'd1;
        31: z <= -16'd6;
        32: z <= 16'd7;
        33: z <= 16'd7;
        34: z <= -16'd4;
        35: z <= -16'd7;
        36: z <= 16'd3;
        37: z <= -16'd6;
        38: z <= 16'd0;
        39: z <= -16'd8;
        40: z <= -16'd1;
        41: z <= -16'd8;
        42: z <= 16'd4;
        43: z <= 16'd5;
        44: z <= 16'd3;
        45: z <= -16'd1;
        46: z <= 16'd3;
        47: z <= -16'd8;
        48: z <= -16'd8;
        49: z <= 16'd3;
        50: z <= -16'd2;
        51: z <= -16'd3;
        52: z <= -16'd7;
        53: z <= 16'd2;
        54: z <= -16'd5;
        55: z <= -16'd3;
        56: z <= 16'd2;
        57: z <= 16'd4;
        58: z <= 16'd7;
        59: z <= 16'd1;
        60: z <= 16'd3;
        61: z <= -16'd1;
        62: z <= 16'd3;
        63: z <= 16'd3;
        64: z <= -16'd2;
        65: z <= 16'd7;
        66: z <= 16'd4;
        67: z <= -16'd6;
        68: z <= -16'd7;
        69: z <= -16'd4;
        70: z <= -16'd6;
        71: z <= 16'd0;
        72: z <= -16'd3;
        73: z <= 16'd7;
        74: z <= -16'd3;
        75: z <= -16'd8;
        76: z <= -16'd2;
        77: z <= -16'd7;
        78: z <= -16'd7;
        79: z <= -16'd1;
        80: z <= 16'd4;
        81: z <= -16'd1;
        82: z <= 16'd4;
        83: z <= 16'd5;
        84: z <= -16'd7;
        85: z <= 16'd7;
        86: z <= -16'd6;
        87: z <= 16'd4;
        88: z <= 16'd4;
        89: z <= -16'd6;
        90: z <= -16'd3;
        91: z <= -16'd1;
        92: z <= 16'd1;
        93: z <= -16'd7;
        94: z <= -16'd6;
        95: z <= 16'd7;
      endcase
   end
endmodule

module l3_fc_16_12_16_1_1(clk, reset, input_valid, input_ready, input_data, output_valid, output_ready, output_data);
    localparam              WIDTH = 16;
    localparam              W_M = 16;
    localparam              W_N = 12;
    localparam              P = 1;
    localparam              R = 1;
    localparam              LOGSIZE_X = $clog2(W_N);
    localparam              LOGSIZE_W = $clog2(W_M/P * W_N);
    localparam              NUM_OUTPUT = $clog2(P);

    input                           clk, reset, input_valid, output_ready;
    input  signed [WIDTH-1:0]       input_data;
    output signed [WIDTH-1:0]       output_data;
    output                          output_valid, input_ready;

    logic                           clk, reset, wr_en_x, clear_acc, en_acc;
    logic [LOGSIZE_X-1:0]           addr_x;
    logic [LOGSIZE_W-1:0]           addr_w;
    logic [NUM_OUTPUT:0]            out_addr;
    logic [1:0]                     delay_ctrl;

    l3_fc_16_12_16_1_1_datapath #(WIDTH, W_M, W_N, P, R) dp( 
                            .in_data(input_data), .clk(clk), .reset(reset), .clear_acc(clear_acc), .en_acc(en_acc), 
                            .addr_x(addr_x), .wr_en_x(wr_en_x), .addr_w(addr_w),
                            .out_addr(out_addr), .delay_ctrl(delay_ctrl),
                            .out_data(output_data));

    l3_fc_16_12_16_1_1_control #(WIDTH, W_M, W_N, P) ctrl(
                            .clk(clk), .reset(reset), .in_valid(input_valid), .out_ready(output_ready),
                            .addr_x(addr_x), .wr_en_x(wr_en_x), .addr_w(addr_w),
                            .clear_acc(clear_acc), .en_acc(en_acc), .in_ready(input_ready),
                            .out_valid(output_valid), .out_addr(out_addr),
                            .delay_ctrl(delay_ctrl));
endmodule

module l3_fc_16_12_16_1_1_datapath(in_data, clk, reset, 
                addr_x, wr_en_x, addr_w,
                clear_acc, en_acc, out_data, out_addr, delay_ctrl);
    parameter                           WIDTH = 20, W_M = 5, W_N = 4, P = 1, R = 1;
    localparam                          X_SIZE = W_N;
    localparam                          LOGSIZE_X=$clog2(W_N);
    localparam                          LOGSIZE_W=$clog2(W_M/P * W_N);
    localparam                          NUM_PER_ROM = W_M/P * W_N;
    localparam                          NUM_OUTPUT = $clog2(P);

    input                               clk, reset, clear_acc, en_acc;
    input signed    [WIDTH-1:0]         in_data;
    input                               wr_en_x;
    input           [LOGSIZE_X-1:0]     addr_x;
    input           [LOGSIZE_W-1:0]     addr_w;
    input           [NUM_OUTPUT:0]      out_addr;
    input           [1:0]               delay_ctrl;

    output signed   [WIDTH-1:0]         out_data;

    logic                               acc_clr;
    logic signed    [WIDTH-1:0]         w_in;
    logic signed    [WIDTH-1:0]         x_in;
    logic signed    [WIDTH-1:0]         mac_out;
    logic                               mac_out_valid;

    logic signed    [P:0][WIDTH-1:0]     out_v;
    logic signed    [WIDTH-1:0]          out;

    memory #(WIDTH, X_SIZE) l3_fc_16_12_16_1_1_vector(.clk(clk), .data_in(in_data), .addr(addr_x), .wr_en(wr_en_x), .data_out(x_in));
    l3_fc_16_12_16_1_1_W_rom l3_fc_16_12_16_1_1_w(.clk(clk), .addr(addr_w), .z(w_in));
    mac #(WIDTH) l3_fc_16_12_16_1_1_mc(.clk(clk), .reset(acc_clr), .a(w_in), .b(x_in), .valid_in(en_acc), .f(mac_out), .valid_out(mac_out_valid));

    always_comb begin
        acc_clr = reset || clear_acc;
    end
    always_ff @(posedge clk) begin
        if(delay_ctrl == 2)begin
            if (R==0)begin
                out_v[0] <= mac_out;
            end
            else begin
                out_v[0] <= (mac_out > 0)? mac_out : 0;
            end
        end
        else
            out_v <= out_v;
        end
        always_comb begin
            case(out_addr)
                0: out = out_v[0];
                default: out = 0;
            endcase
        end
        assign out_data = out;
endmodule

module l3_fc_16_12_16_1_1_control(clk, reset, in_valid, out_ready,
               addr_x, wr_en_x, addr_w, 
               clear_acc, en_acc, in_ready, out_valid, out_addr, delay_ctrl);
    parameter                     WIDTH = 20, W_M = 5, W_N = 4, P = 1;
    localparam                    X_SIZE = W_N;
    localparam                    LOGSIZE_X=$clog2(W_N);
    localparam                    LOGSIZE_W=$clog2(W_M/P * W_N);
    localparam                    NUM_ACC=W_N;
    localparam                    LOG_NUM_ACC = $clog2(W_N);
    localparam                    NUM_OUTPUT = W_M;
    localparam                    LOG_NUM_OUTPUT = $clog2(W_M);
    localparam                    NUM_ITER = P;
    localparam                    LOG_NUM_ITER = $clog2(P);

    input                                   clk, reset, in_valid, out_ready;

    output                                  wr_en_x;
    output                                  clear_acc;
    output logic                            en_acc;
    output logic [LOGSIZE_X-1:0]            addr_x;
    output logic [LOGSIZE_W-1:0]            addr_w;
    output                                  in_ready, out_valid;
    output       [LOG_NUM_ITER:0]           out_addr;
    output logic [1:0]                      delay_ctrl;

    logic        [2:0]                      state, next_state;
    logic        [LOG_NUM_ACC:0]            acc_count;
    logic        [LOG_NUM_OUTPUT:0]         output_count;
    logic        [LOGSIZE_W+1:0]            addr_count_w; 
    logic        [LOGSIZE_X+1:0]            addr_count_x;
    logic                                   last_valid;
    logic        [LOG_NUM_ITER+1:0]         iter_count;

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
                next_state = 2;
            end 
            else
                next_state = 0;
        end
        else if(state == 2) begin
            if (addr_count_x < X_SIZE)
                next_state = 2;
            else if (delay_ctrl == 2 && output_count < W_M)
                next_state = 2;
            else
                next_state = 3;
        end
        /* 3: Compute*/
        else if (state == 3) begin
            if(last_valid == 0) 
                next_state = 3;
            else if (output_count < W_M-P)
                next_state = 4;
            else
                next_state = 0;
        end
        /* 4: Load output*/
        else if (state == 4) begin
            if(delay_ctrl == 2 && iter_count < P)
               next_state = 4;
            else
               next_state = 3;
        end
    end

    assign clear_acc = ((state == 2 && output_count >= W_M) ||(state == 4 && iter_count >= P) || reset == 1)? 1 : 0;
    assign in_ready = (state == 2 && addr_count_x < X_SIZE)? 1 : 0; 
    assign wr_en_x = (state == 2) && (in_valid == 1) && (addr_count_x < X_SIZE)? 1 : 0;
    assign out_valid = ( last_valid == 1 && (((state == 2 || state == 0) && output_count < W_M) || (state == 4 && iter_count < P)))? 1 : 0;
    assign out_addr = iter_count;

    always_ff @(posedge clk)begin
        if(state == 0) begin
            addr_x <= 0;
            addr_count_x <= 0;
        end 
        else if (state == 2 && addr_count_x >= X_SIZE && output_count >= W_M)begin
                addr_x <= 0;
                addr_count_x <= 0;
        end
        else if (state == 2 && delay_ctrl < 2 && addr_count_x >= X_SIZE)begin
                addr_x <= 0;
                addr_count_x <= 0;
        end
        else if (state == 4 && iter_count >= P)begin
                addr_x <= 0;
                addr_count_x <= 0;
        end
        else if ((state == 2 && in_valid == 1 && addr_count_x < X_SIZE)|| (state == 3 && acc_count < NUM_ACC)) begin
            addr_x <= addr_x + 1;
            addr_count_x <= addr_count_x + 1;
        end else begin
            addr_x <= addr_x;
            addr_count_x <= addr_count_x;
        end
    end

    always_ff @(posedge clk)begin
        if(state == 0 || state == 2) begin
            addr_w <= 0;
            addr_count_w <= 0;
        end 
        else if (state == 3 && acc_count < NUM_ACC) begin
            addr_w <= addr_w + 1;
            addr_count_w <= addr_count_w + 1;
        end
        else begin
            addr_w <= addr_w;
            addr_count_w <= addr_count_w;
        end
    end

    /* Accumulation Counter*/
    always_ff @(posedge clk)begin
        if(state == 0 | state == 4)
            acc_count <= 0;
        else if(state == 3 && acc_count < NUM_ACC)
            acc_count <= acc_count + 1;
        else
            acc_count <= acc_count;
    end

    always_ff @(posedge clk)begin
        if(reset == 1 || (state == 2 && addr_count_x >= X_SIZE && output_count >= W_M))
            output_count <= 0;
        else if(out_ready == 1 && out_valid == 1 && (((state == 2 || state == 0) && output_count < W_M) || (state == 4 && iter_count < P)))
            output_count <= output_count + 1;
        else
            output_count <= output_count;
    end

    always_ff @(posedge clk) begin
        if(state == 3 && acc_count < NUM_ACC )
            en_acc <= 1;
        else
            en_acc <= 0;
    end

    always_ff @(posedge clk)begin
       if(reset == 1 | state == 3)
         iter_count <= 0;
       else if ((state == 4 || state == 2 || state == 0) && out_ready == 1 && out_valid == 1 && iter_count < P)
         iter_count <= iter_count + 1;
       else
         iter_count <= iter_count;
    end

    always_ff @(posedge clk) begin
        if(reset == 1 || en_acc == 1 || ((state == 4 || state ==3) && iter_count >= P))
            delay_ctrl <= 0;
        else if (state == 3 && delay_ctrl < 2)
            delay_ctrl <= delay_ctrl + 1;
        else
            delay_ctrl <= delay_ctrl;
    end

    always_ff @(posedge clk) begin
        if(delay_ctrl == 2 && (((state == 4||state == 3) && iter_count < P) || ((state == 2 || state == 0) && output_count < W_M))) 
            last_valid <= 1;
        else
            last_valid <= 0;
    end
endmodule

module l3_fc_16_12_16_1_1_W_rom(clk, addr, z);
   input clk;
   input [7:0] addr;
   output logic signed [15:0] z;
   always_ff @(posedge clk) begin
      case(addr)
        0: z <= -16'd8;
        1: z <= 16'd7;
        2: z <= -16'd7;
        3: z <= -16'd6;
        4: z <= -16'd5;
        5: z <= -16'd4;
        6: z <= 16'd2;
        7: z <= 16'd0;
        8: z <= -16'd5;
        9: z <= -16'd8;
        10: z <= 16'd1;
        11: z <= 16'd1;
        12: z <= -16'd7;
        13: z <= 16'd2;
        14: z <= -16'd8;
        15: z <= 16'd5;
        16: z <= -16'd7;
        17: z <= 16'd4;
        18: z <= 16'd2;
        19: z <= -16'd5;
        20: z <= 16'd4;
        21: z <= 16'd4;
        22: z <= 16'd7;
        23: z <= 16'd0;
        24: z <= 16'd6;
        25: z <= -16'd4;
        26: z <= 16'd7;
        27: z <= -16'd1;
        28: z <= -16'd3;
        29: z <= -16'd6;
        30: z <= -16'd1;
        31: z <= -16'd2;
        32: z <= -16'd7;
        33: z <= 16'd0;
        34: z <= 16'd0;
        35: z <= -16'd4;
        36: z <= 16'd4;
        37: z <= -16'd6;
        38: z <= 16'd5;
        39: z <= 16'd7;
        40: z <= -16'd6;
        41: z <= -16'd2;
        42: z <= 16'd1;
        43: z <= -16'd5;
        44: z <= -16'd8;
        45: z <= 16'd1;
        46: z <= -16'd8;
        47: z <= -16'd7;
        48: z <= -16'd2;
        49: z <= 16'd2;
        50: z <= -16'd4;
        51: z <= -16'd6;
        52: z <= -16'd1;
        53: z <= -16'd5;
        54: z <= 16'd2;
        55: z <= -16'd3;
        56: z <= 16'd0;
        57: z <= 16'd1;
        58: z <= 16'd5;
        59: z <= 16'd5;
        60: z <= 16'd3;
        61: z <= -16'd4;
        62: z <= -16'd5;
        63: z <= 16'd4;
        64: z <= 16'd4;
        65: z <= 16'd3;
        66: z <= -16'd7;
        67: z <= 16'd1;
        68: z <= 16'd6;
        69: z <= 16'd6;
        70: z <= 16'd0;
        71: z <= -16'd8;
        72: z <= -16'd4;
        73: z <= -16'd7;
        74: z <= -16'd4;
        75: z <= -16'd4;
        76: z <= 16'd3;
        77: z <= -16'd4;
        78: z <= -16'd3;
        79: z <= -16'd7;
        80: z <= 16'd7;
        81: z <= 16'd2;
        82: z <= -16'd5;
        83: z <= -16'd2;
        84: z <= 16'd5;
        85: z <= 16'd5;
        86: z <= 16'd3;
        87: z <= -16'd3;
        88: z <= -16'd2;
        89: z <= 16'd0;
        90: z <= -16'd5;
        91: z <= -16'd6;
        92: z <= 16'd4;
        93: z <= -16'd2;
        94: z <= 16'd6;
        95: z <= 16'd1;
        96: z <= -16'd6;
        97: z <= 16'd7;
        98: z <= -16'd6;
        99: z <= -16'd8;
        100: z <= 16'd5;
        101: z <= 16'd2;
        102: z <= -16'd8;
        103: z <= -16'd7;
        104: z <= 16'd4;
        105: z <= -16'd4;
        106: z <= -16'd3;
        107: z <= -16'd1;
        108: z <= 16'd1;
        109: z <= 16'd3;
        110: z <= 16'd0;
        111: z <= 16'd0;
        112: z <= -16'd3;
        113: z <= 16'd3;
        114: z <= 16'd6;
        115: z <= -16'd6;
        116: z <= 16'd0;
        117: z <= 16'd1;
        118: z <= 16'd0;
        119: z <= 16'd6;
        120: z <= -16'd6;
        121: z <= 16'd3;
        122: z <= -16'd8;
        123: z <= 16'd6;
        124: z <= -16'd7;
        125: z <= 16'd7;
        126: z <= -16'd1;
        127: z <= -16'd5;
        128: z <= 16'd6;
        129: z <= 16'd1;
        130: z <= -16'd5;
        131: z <= 16'd4;
        132: z <= -16'd4;
        133: z <= -16'd4;
        134: z <= 16'd5;
        135: z <= -16'd8;
        136: z <= 16'd0;
        137: z <= -16'd5;
        138: z <= -16'd1;
        139: z <= -16'd7;
        140: z <= 16'd6;
        141: z <= 16'd7;
        142: z <= 16'd1;
        143: z <= -16'd5;
        144: z <= 16'd2;
        145: z <= -16'd1;
        146: z <= -16'd3;
        147: z <= -16'd6;
        148: z <= -16'd7;
        149: z <= 16'd5;
        150: z <= -16'd8;
        151: z <= -16'd5;
        152: z <= 16'd0;
        153: z <= -16'd7;
        154: z <= -16'd7;
        155: z <= 16'd2;
        156: z <= -16'd8;
        157: z <= 16'd1;
        158: z <= 16'd5;
        159: z <= 16'd6;
        160: z <= -16'd6;
        161: z <= -16'd7;
        162: z <= 16'd2;
        163: z <= -16'd2;
        164: z <= -16'd3;
        165: z <= 16'd0;
        166: z <= -16'd2;
        167: z <= 16'd5;
        168: z <= 16'd3;
        169: z <= 16'd5;
        170: z <= 16'd7;
        171: z <= 16'd1;
        172: z <= 16'd4;
        173: z <= 16'd0;
        174: z <= 16'd4;
        175: z <= -16'd2;
        176: z <= -16'd8;
        177: z <= -16'd7;
        178: z <= 16'd0;
        179: z <= -16'd7;
        180: z <= 16'd7;
        181: z <= 16'd1;
        182: z <= -16'd4;
        183: z <= -16'd1;
        184: z <= 16'd2;
        185: z <= -16'd3;
        186: z <= -16'd7;
        187: z <= 16'd2;
        188: z <= 16'd6;
        189: z <= 16'd7;
        190: z <= 16'd0;
        191: z <= -16'd7;
      endcase
   end
endmodule

module memory(clk, data_in, data_out, addr, wr_en);

    parameter                   WIDTH=16, SIZE=64;
    localparam                  LOGSIZE=$clog2(SIZE);
    input [WIDTH-1:0]           data_in;
    output logic [WIDTH-1:0]    data_out;
    input [LOGSIZE-1:0]         addr;
    input                       clk, wr_en;

    logic [SIZE-1:0][WIDTH-1:0] mem;

    always_ff @(posedge clk) begin
        data_out <= mem[addr];
        if (wr_en)
            mem[addr] <= data_in;
    end
endmodule

module mac(clk, reset, a, b, valid_in, f, valid_out);
    parameter                           WIDTH = 16;
    localparam                          MAX = 32767;
    localparam                          MIN = -32768;
    input                               clk, reset, valid_in;
    input signed [WIDTH - 1:0]          a, b;
    output logic signed [WIDTH - 1:0]   f;
    output                              valid_out;

    logic signed [WIDTH - 1:0]          a_r, b_r;
    logic signed [WIDTH*2 - 1:0]        product;
    logic signed [WIDTH-1:0]            acc;

    logic [2:0]                         shift_delay;
    logic signed [WIDTH-1:0]            product_sat;

    always_ff @(posedge clk) begin
          if(reset)
            shift_delay <= 0;
          else
            shift_delay <= {shift_delay[2:0], valid_in};
    end
    assign valid_out = shift_delay[2];

    always_comb begin
        if(shift_delay[1]==1)
            acc = f + product_sat;
        else
            acc = 0;
    end
    always_comb begin
        product = a_r*b_r;
    end

    always_ff @(posedge clk) begin
        if (product > MAX)
            product_sat <= MAX;
        else if (product < MIN)
            product_sat <= MIN;
        else
            product_sat <= product[WIDTH-1:0];
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
		else if(shift_delay[1]==1)begin
		      if(f[WIDTH-1]==0 & product_sat[WIDTH-1]==0 & acc[WIDTH-1]==1)   //overflow
	              f <= MAX;
		      else if (f[WIDTH-1]==1 & product_sat[WIDTH-1]==1 & acc[WIDTH-1]==0) //underflow
			      f <= MIN;
		      else
		   	      f <= acc;
        end
        else
            f <= f;
    end
endmodule

