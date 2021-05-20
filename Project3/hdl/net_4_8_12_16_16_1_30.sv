module net_4_8_12_16_16_1_30(clk, reset, input_valid, input_ready, input_data, output_valid, output_ready, output_data);
   localparam N = 4;
   localparam M1 = 8;
   localparam M2 = 12;
   localparam M3 = 16;
   localparam T = 16;
   localparam R = 1;
   localparam B = 30;

   input                    clk, reset, input_valid, output_ready;
   input  signed [T-1:0]    input_data;
   output                   output_valid, input_ready;
   output signed [T-1:0]    output_data;

   logic                    output_valid_l1;
   logic  signed [T-1:0]    output_data_l1;
   logic                    input_ready_l2, output_valid_l2, output_ready_l2;
   logic  signed [T-1:0]    output_data_l2;
   logic                    input_ready_l3;

   l1_fc_8_4_16_1_8 l1(.clk(clk), .reset(reset), .input_valid(input_valid), .input_ready(input_ready), .input_data(input_data),
                       .output_valid(output_valid_l1), .output_ready(input_ready_l2), .output_data(output_data_l1));
   l2_fc_12_8_16_1_12 l2(.clk(clk), .reset(reset), .input_valid(output_valid_l1), .input_ready(input_ready_l2), .input_data(output_data_l1),
                       .output_valid(output_valid_l2), .output_ready(input_ready_l3), .output_data(output_data_l2));
   l3_fc_16_12_16_1_8 l3(.clk(clk), .reset(reset), .input_valid(output_valid_l2), .input_ready(input_ready_l3), .input_data(output_data_l2),
                      .output_valid(output_valid), .output_ready(output_ready), .output_data(output_data));

endmodule

module l1_fc_8_4_16_1_8(clk, reset, input_valid, input_ready, input_data, output_valid, output_ready, output_data);
    localparam              WIDTH = 16;
    localparam              W_M = 8;
    localparam              W_N = 4;
    localparam              P = 8;
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

    l1_fc_8_4_16_1_8_datapath #(WIDTH, W_M, W_N, P, R) dp( 
                            .in_data(input_data), .clk(clk), .reset(reset), .clear_acc(clear_acc), .en_acc(en_acc), 
                            .addr_x(addr_x), .wr_en_x(wr_en_x), .addr_w(addr_w),
                            .out_addr(out_addr), .delay_ctrl(delay_ctrl),
                            .out_data(output_data));

    l1_fc_8_4_16_1_8_control #(WIDTH, W_M, W_N, P) ctrl(
                            .clk(clk), .reset(reset), .in_valid(input_valid), .out_ready(output_ready),
                            .addr_x(addr_x), .wr_en_x(wr_en_x), .addr_w(addr_w),
                            .clear_acc(clear_acc), .en_acc(en_acc), .in_ready(input_ready),
                            .out_valid(output_valid), .out_addr(out_addr),
                            .delay_ctrl(delay_ctrl));
endmodule

module l1_fc_8_4_16_1_8_datapath(in_data, clk, reset, 
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
    logic signed    [WIDTH-1:0]         w_in0;
    logic signed    [WIDTH-1:0]         w_in1;
    logic signed    [WIDTH-1:0]         w_in2;
    logic signed    [WIDTH-1:0]         w_in3;
    logic signed    [WIDTH-1:0]         w_in4;
    logic signed    [WIDTH-1:0]         w_in5;
    logic signed    [WIDTH-1:0]         w_in6;
    logic signed    [WIDTH-1:0]         w_in7;
    logic signed    [WIDTH-1:0]         x_in;
    logic signed    [WIDTH-1:0]         mac_out0;
    logic signed    [WIDTH-1:0]         mac_out1;
    logic signed    [WIDTH-1:0]         mac_out2;
    logic signed    [WIDTH-1:0]         mac_out3;
    logic signed    [WIDTH-1:0]         mac_out4;
    logic signed    [WIDTH-1:0]         mac_out5;
    logic signed    [WIDTH-1:0]         mac_out6;
    logic signed    [WIDTH-1:0]         mac_out7;
    logic                               mac_out_valid0;
    logic                               mac_out_valid1;
    logic                               mac_out_valid2;
    logic                               mac_out_valid3;
    logic                               mac_out_valid4;
    logic                               mac_out_valid5;
    logic                               mac_out_valid6;
    logic                               mac_out_valid7;

    logic signed    [P:0][WIDTH-1:0]     out_v;
    logic signed    [WIDTH-1:0]          out;

    memory #(WIDTH, X_SIZE) l1_fc_8_4_16_1_8_vector(.clk(clk), .data_in(in_data), .addr(addr_x), .wr_en(wr_en_x), .data_out(x_in));
    l1_fc_8_4_16_1_8_W_rom0 l1_fc_8_4_16_1_8_w0(.clk(clk), .addr(addr_w), .z(w_in0));
    l1_fc_8_4_16_1_8_W_rom1 l1_fc_8_4_16_1_8_w1(.clk(clk), .addr(addr_w), .z(w_in1));
    l1_fc_8_4_16_1_8_W_rom2 l1_fc_8_4_16_1_8_w2(.clk(clk), .addr(addr_w), .z(w_in2));
    l1_fc_8_4_16_1_8_W_rom3 l1_fc_8_4_16_1_8_w3(.clk(clk), .addr(addr_w), .z(w_in3));
    l1_fc_8_4_16_1_8_W_rom4 l1_fc_8_4_16_1_8_w4(.clk(clk), .addr(addr_w), .z(w_in4));
    l1_fc_8_4_16_1_8_W_rom5 l1_fc_8_4_16_1_8_w5(.clk(clk), .addr(addr_w), .z(w_in5));
    l1_fc_8_4_16_1_8_W_rom6 l1_fc_8_4_16_1_8_w6(.clk(clk), .addr(addr_w), .z(w_in6));
    l1_fc_8_4_16_1_8_W_rom7 l1_fc_8_4_16_1_8_w7(.clk(clk), .addr(addr_w), .z(w_in7));
    mac #(WIDTH) l1_fc_8_4_16_1_8_mc0(.clk(clk), .reset(acc_clr), .a(w_in0), .b(x_in), .valid_in(en_acc), .f(mac_out0), .valid_out(mac_out_valid0));
    mac #(WIDTH) l1_fc_8_4_16_1_8_mc1(.clk(clk), .reset(acc_clr), .a(w_in1), .b(x_in), .valid_in(en_acc), .f(mac_out1), .valid_out(mac_out_valid1));
    mac #(WIDTH) l1_fc_8_4_16_1_8_mc2(.clk(clk), .reset(acc_clr), .a(w_in2), .b(x_in), .valid_in(en_acc), .f(mac_out2), .valid_out(mac_out_valid2));
    mac #(WIDTH) l1_fc_8_4_16_1_8_mc3(.clk(clk), .reset(acc_clr), .a(w_in3), .b(x_in), .valid_in(en_acc), .f(mac_out3), .valid_out(mac_out_valid3));
    mac #(WIDTH) l1_fc_8_4_16_1_8_mc4(.clk(clk), .reset(acc_clr), .a(w_in4), .b(x_in), .valid_in(en_acc), .f(mac_out4), .valid_out(mac_out_valid4));
    mac #(WIDTH) l1_fc_8_4_16_1_8_mc5(.clk(clk), .reset(acc_clr), .a(w_in5), .b(x_in), .valid_in(en_acc), .f(mac_out5), .valid_out(mac_out_valid5));
    mac #(WIDTH) l1_fc_8_4_16_1_8_mc6(.clk(clk), .reset(acc_clr), .a(w_in6), .b(x_in), .valid_in(en_acc), .f(mac_out6), .valid_out(mac_out_valid6));
    mac #(WIDTH) l1_fc_8_4_16_1_8_mc7(.clk(clk), .reset(acc_clr), .a(w_in7), .b(x_in), .valid_in(en_acc), .f(mac_out7), .valid_out(mac_out_valid7));

    always_comb begin
        acc_clr = reset || clear_acc;
    end
    always_ff @(posedge clk) begin
        if(delay_ctrl == 2)begin
            if (R==0)begin
                out_v[0] <= mac_out0;
                out_v[1] <= mac_out1;
                out_v[2] <= mac_out2;
                out_v[3] <= mac_out3;
                out_v[4] <= mac_out4;
                out_v[5] <= mac_out5;
                out_v[6] <= mac_out6;
                out_v[7] <= mac_out7;
            end
            else begin
                if (mac_out0 > 0)
                    out_v[0] <= mac_out0;
                else
                    out_v[0] <= 0;
                if (mac_out1 > 0)
                    out_v[1] <= mac_out1;
                else
                    out_v[1] <= 0;
                if (mac_out2 > 0)
                    out_v[2] <= mac_out2;
                else
                    out_v[2] <= 0;
                if (mac_out3 > 0)
                    out_v[3] <= mac_out3;
                else
                    out_v[3] <= 0;
                if (mac_out4 > 0)
                    out_v[4] <= mac_out4;
                else
                    out_v[4] <= 0;
                if (mac_out5 > 0)
                    out_v[5] <= mac_out5;
                else
                    out_v[5] <= 0;
                if (mac_out6 > 0)
                    out_v[6] <= mac_out6;
                else
                    out_v[6] <= 0;
                if (mac_out7 > 0)
                    out_v[7] <= mac_out7;
                else
                    out_v[7] <= 0;
            end
        end
        else
            out_v <= out_v;
        end
        always_comb begin
            case(out_addr)
                0: out = out_v[0];
                1: out = out_v[1];
                2: out = out_v[2];
                3: out = out_v[3];
                4: out = out_v[4];
                5: out = out_v[5];
                6: out = out_v[6];
                7: out = out_v[7];
                default: out = 0;
            endcase
        end
        assign out_data = out;
endmodule

module l1_fc_8_4_16_1_8_control(clk, reset, in_valid, out_ready,
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

module l1_fc_8_4_16_1_8_W_rom0(clk, addr, z);
   input clk;
   input [1:0] addr;
   output logic signed [15:0] z;
   always_ff @(posedge clk) begin
      case(addr)
        0: z <= 16'd0;
        1: z <= 16'd1;
        2: z <= -16'd8;
        3: z <= -16'd3;
      endcase
   end
endmodule

module l1_fc_8_4_16_1_8_W_rom1(clk, addr, z);
   input clk;
   input [1:0] addr;
   output logic signed [15:0] z;
   always_ff @(posedge clk) begin
      case(addr)
        0: z <= -16'd8;
        1: z <= 16'd1;
        2: z <= 16'd3;
        3: z <= -16'd5;
      endcase
   end
endmodule

module l1_fc_8_4_16_1_8_W_rom2(clk, addr, z);
   input clk;
   input [1:0] addr;
   output logic signed [15:0] z;
   always_ff @(posedge clk) begin
      case(addr)
        0: z <= 16'd5;
        1: z <= -16'd2;
        2: z <= -16'd3;
        3: z <= 16'd3;
      endcase
   end
endmodule

module l1_fc_8_4_16_1_8_W_rom3(clk, addr, z);
   input clk;
   input [1:0] addr;
   output logic signed [15:0] z;
   always_ff @(posedge clk) begin
      case(addr)
        0: z <= -16'd3;
        1: z <= 16'd2;
        2: z <= 16'd5;
        3: z <= 16'd3;
      endcase
   end
endmodule

module l1_fc_8_4_16_1_8_W_rom4(clk, addr, z);
   input clk;
   input [1:0] addr;
   output logic signed [15:0] z;
   always_ff @(posedge clk) begin
      case(addr)
        0: z <= -16'd8;
        1: z <= 16'd4;
        2: z <= 16'd3;
        3: z <= -16'd2;
      endcase
   end
endmodule

module l1_fc_8_4_16_1_8_W_rom5(clk, addr, z);
   input clk;
   input [1:0] addr;
   output logic signed [15:0] z;
   always_ff @(posedge clk) begin
      case(addr)
        0: z <= -16'd3;
        1: z <= 16'd1;
        2: z <= 16'd6;
        3: z <= -16'd3;
      endcase
   end
endmodule

module l1_fc_8_4_16_1_8_W_rom6(clk, addr, z);
   input clk;
   input [1:0] addr;
   output logic signed [15:0] z;
   always_ff @(posedge clk) begin
      case(addr)
        0: z <= 16'd4;
        1: z <= 16'd1;
        2: z <= 16'd4;
        3: z <= -16'd2;
      endcase
   end
endmodule

module l1_fc_8_4_16_1_8_W_rom7(clk, addr, z);
   input clk;
   input [1:0] addr;
   output logic signed [15:0] z;
   always_ff @(posedge clk) begin
      case(addr)
        0: z <= 16'd2;
        1: z <= -16'd5;
        2: z <= 16'd4;
        3: z <= -16'd5;
      endcase
   end
endmodule

module l2_fc_12_8_16_1_12(clk, reset, input_valid, input_ready, input_data, output_valid, output_ready, output_data);
    localparam              WIDTH = 16;
    localparam              W_M = 12;
    localparam              W_N = 8;
    localparam              P = 12;
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

    l2_fc_12_8_16_1_12_datapath #(WIDTH, W_M, W_N, P, R) dp( 
                            .in_data(input_data), .clk(clk), .reset(reset), .clear_acc(clear_acc), .en_acc(en_acc), 
                            .addr_x(addr_x), .wr_en_x(wr_en_x), .addr_w(addr_w),
                            .out_addr(out_addr), .delay_ctrl(delay_ctrl),
                            .out_data(output_data));

    l2_fc_12_8_16_1_12_control #(WIDTH, W_M, W_N, P) ctrl(
                            .clk(clk), .reset(reset), .in_valid(input_valid), .out_ready(output_ready),
                            .addr_x(addr_x), .wr_en_x(wr_en_x), .addr_w(addr_w),
                            .clear_acc(clear_acc), .en_acc(en_acc), .in_ready(input_ready),
                            .out_valid(output_valid), .out_addr(out_addr),
                            .delay_ctrl(delay_ctrl));
endmodule

module l2_fc_12_8_16_1_12_datapath(in_data, clk, reset, 
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
    logic signed    [WIDTH-1:0]         w_in0;
    logic signed    [WIDTH-1:0]         w_in1;
    logic signed    [WIDTH-1:0]         w_in2;
    logic signed    [WIDTH-1:0]         w_in3;
    logic signed    [WIDTH-1:0]         w_in4;
    logic signed    [WIDTH-1:0]         w_in5;
    logic signed    [WIDTH-1:0]         w_in6;
    logic signed    [WIDTH-1:0]         w_in7;
    logic signed    [WIDTH-1:0]         w_in8;
    logic signed    [WIDTH-1:0]         w_in9;
    logic signed    [WIDTH-1:0]         w_in10;
    logic signed    [WIDTH-1:0]         w_in11;
    logic signed    [WIDTH-1:0]         x_in;
    logic signed    [WIDTH-1:0]         mac_out0;
    logic signed    [WIDTH-1:0]         mac_out1;
    logic signed    [WIDTH-1:0]         mac_out2;
    logic signed    [WIDTH-1:0]         mac_out3;
    logic signed    [WIDTH-1:0]         mac_out4;
    logic signed    [WIDTH-1:0]         mac_out5;
    logic signed    [WIDTH-1:0]         mac_out6;
    logic signed    [WIDTH-1:0]         mac_out7;
    logic signed    [WIDTH-1:0]         mac_out8;
    logic signed    [WIDTH-1:0]         mac_out9;
    logic signed    [WIDTH-1:0]         mac_out10;
    logic signed    [WIDTH-1:0]         mac_out11;
    logic                               mac_out_valid0;
    logic                               mac_out_valid1;
    logic                               mac_out_valid2;
    logic                               mac_out_valid3;
    logic                               mac_out_valid4;
    logic                               mac_out_valid5;
    logic                               mac_out_valid6;
    logic                               mac_out_valid7;
    logic                               mac_out_valid8;
    logic                               mac_out_valid9;
    logic                               mac_out_valid10;
    logic                               mac_out_valid11;

    logic signed    [P:0][WIDTH-1:0]     out_v;
    logic signed    [WIDTH-1:0]          out;

    memory #(WIDTH, X_SIZE) l2_fc_12_8_16_1_12_vector(.clk(clk), .data_in(in_data), .addr(addr_x), .wr_en(wr_en_x), .data_out(x_in));
    l2_fc_12_8_16_1_12_W_rom0 l2_fc_12_8_16_1_12_w0(.clk(clk), .addr(addr_w), .z(w_in0));
    l2_fc_12_8_16_1_12_W_rom1 l2_fc_12_8_16_1_12_w1(.clk(clk), .addr(addr_w), .z(w_in1));
    l2_fc_12_8_16_1_12_W_rom2 l2_fc_12_8_16_1_12_w2(.clk(clk), .addr(addr_w), .z(w_in2));
    l2_fc_12_8_16_1_12_W_rom3 l2_fc_12_8_16_1_12_w3(.clk(clk), .addr(addr_w), .z(w_in3));
    l2_fc_12_8_16_1_12_W_rom4 l2_fc_12_8_16_1_12_w4(.clk(clk), .addr(addr_w), .z(w_in4));
    l2_fc_12_8_16_1_12_W_rom5 l2_fc_12_8_16_1_12_w5(.clk(clk), .addr(addr_w), .z(w_in5));
    l2_fc_12_8_16_1_12_W_rom6 l2_fc_12_8_16_1_12_w6(.clk(clk), .addr(addr_w), .z(w_in6));
    l2_fc_12_8_16_1_12_W_rom7 l2_fc_12_8_16_1_12_w7(.clk(clk), .addr(addr_w), .z(w_in7));
    l2_fc_12_8_16_1_12_W_rom8 l2_fc_12_8_16_1_12_w8(.clk(clk), .addr(addr_w), .z(w_in8));
    l2_fc_12_8_16_1_12_W_rom9 l2_fc_12_8_16_1_12_w9(.clk(clk), .addr(addr_w), .z(w_in9));
    l2_fc_12_8_16_1_12_W_rom10 l2_fc_12_8_16_1_12_w10(.clk(clk), .addr(addr_w), .z(w_in10));
    l2_fc_12_8_16_1_12_W_rom11 l2_fc_12_8_16_1_12_w11(.clk(clk), .addr(addr_w), .z(w_in11));
    mac #(WIDTH) l2_fc_12_8_16_1_12_mc0(.clk(clk), .reset(acc_clr), .a(w_in0), .b(x_in), .valid_in(en_acc), .f(mac_out0), .valid_out(mac_out_valid0));
    mac #(WIDTH) l2_fc_12_8_16_1_12_mc1(.clk(clk), .reset(acc_clr), .a(w_in1), .b(x_in), .valid_in(en_acc), .f(mac_out1), .valid_out(mac_out_valid1));
    mac #(WIDTH) l2_fc_12_8_16_1_12_mc2(.clk(clk), .reset(acc_clr), .a(w_in2), .b(x_in), .valid_in(en_acc), .f(mac_out2), .valid_out(mac_out_valid2));
    mac #(WIDTH) l2_fc_12_8_16_1_12_mc3(.clk(clk), .reset(acc_clr), .a(w_in3), .b(x_in), .valid_in(en_acc), .f(mac_out3), .valid_out(mac_out_valid3));
    mac #(WIDTH) l2_fc_12_8_16_1_12_mc4(.clk(clk), .reset(acc_clr), .a(w_in4), .b(x_in), .valid_in(en_acc), .f(mac_out4), .valid_out(mac_out_valid4));
    mac #(WIDTH) l2_fc_12_8_16_1_12_mc5(.clk(clk), .reset(acc_clr), .a(w_in5), .b(x_in), .valid_in(en_acc), .f(mac_out5), .valid_out(mac_out_valid5));
    mac #(WIDTH) l2_fc_12_8_16_1_12_mc6(.clk(clk), .reset(acc_clr), .a(w_in6), .b(x_in), .valid_in(en_acc), .f(mac_out6), .valid_out(mac_out_valid6));
    mac #(WIDTH) l2_fc_12_8_16_1_12_mc7(.clk(clk), .reset(acc_clr), .a(w_in7), .b(x_in), .valid_in(en_acc), .f(mac_out7), .valid_out(mac_out_valid7));
    mac #(WIDTH) l2_fc_12_8_16_1_12_mc8(.clk(clk), .reset(acc_clr), .a(w_in8), .b(x_in), .valid_in(en_acc), .f(mac_out8), .valid_out(mac_out_valid8));
    mac #(WIDTH) l2_fc_12_8_16_1_12_mc9(.clk(clk), .reset(acc_clr), .a(w_in9), .b(x_in), .valid_in(en_acc), .f(mac_out9), .valid_out(mac_out_valid9));
    mac #(WIDTH) l2_fc_12_8_16_1_12_mc10(.clk(clk), .reset(acc_clr), .a(w_in10), .b(x_in), .valid_in(en_acc), .f(mac_out10), .valid_out(mac_out_valid10));
    mac #(WIDTH) l2_fc_12_8_16_1_12_mc11(.clk(clk), .reset(acc_clr), .a(w_in11), .b(x_in), .valid_in(en_acc), .f(mac_out11), .valid_out(mac_out_valid11));

    always_comb begin
        acc_clr = reset || clear_acc;
    end
    always_ff @(posedge clk) begin
        if(delay_ctrl == 2)begin
            if (R==0)begin
                out_v[0] <= mac_out0;
                out_v[1] <= mac_out1;
                out_v[2] <= mac_out2;
                out_v[3] <= mac_out3;
                out_v[4] <= mac_out4;
                out_v[5] <= mac_out5;
                out_v[6] <= mac_out6;
                out_v[7] <= mac_out7;
                out_v[8] <= mac_out8;
                out_v[9] <= mac_out9;
                out_v[10] <= mac_out10;
                out_v[11] <= mac_out11;
            end
            else begin
                if (mac_out0 > 0)
                    out_v[0] <= mac_out0;
                else
                    out_v[0] <= 0;
                if (mac_out1 > 0)
                    out_v[1] <= mac_out1;
                else
                    out_v[1] <= 0;
                if (mac_out2 > 0)
                    out_v[2] <= mac_out2;
                else
                    out_v[2] <= 0;
                if (mac_out3 > 0)
                    out_v[3] <= mac_out3;
                else
                    out_v[3] <= 0;
                if (mac_out4 > 0)
                    out_v[4] <= mac_out4;
                else
                    out_v[4] <= 0;
                if (mac_out5 > 0)
                    out_v[5] <= mac_out5;
                else
                    out_v[5] <= 0;
                if (mac_out6 > 0)
                    out_v[6] <= mac_out6;
                else
                    out_v[6] <= 0;
                if (mac_out7 > 0)
                    out_v[7] <= mac_out7;
                else
                    out_v[7] <= 0;
                if (mac_out8 > 0)
                    out_v[8] <= mac_out8;
                else
                    out_v[8] <= 0;
                if (mac_out9 > 0)
                    out_v[9] <= mac_out9;
                else
                    out_v[9] <= 0;
                if (mac_out10 > 0)
                    out_v[10] <= mac_out10;
                else
                    out_v[10] <= 0;
                if (mac_out11 > 0)
                    out_v[11] <= mac_out11;
                else
                    out_v[11] <= 0;
            end
        end
        else
            out_v <= out_v;
        end
        always_comb begin
            case(out_addr)
                0: out = out_v[0];
                1: out = out_v[1];
                2: out = out_v[2];
                3: out = out_v[3];
                4: out = out_v[4];
                5: out = out_v[5];
                6: out = out_v[6];
                7: out = out_v[7];
                8: out = out_v[8];
                9: out = out_v[9];
                10: out = out_v[10];
                11: out = out_v[11];
                default: out = 0;
            endcase
        end
        assign out_data = out;
endmodule

module l2_fc_12_8_16_1_12_control(clk, reset, in_valid, out_ready,
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

module l2_fc_12_8_16_1_12_W_rom0(clk, addr, z);
   input clk;
   input [2:0] addr;
   output logic signed [15:0] z;
   always_ff @(posedge clk) begin
      case(addr)
        0: z <= 16'd4;
        1: z <= 16'd4;
        2: z <= 16'd0;
        3: z <= 16'd5;
        4: z <= -16'd3;
        5: z <= -16'd4;
        6: z <= -16'd8;
        7: z <= -16'd6;
      endcase
   end
endmodule

module l2_fc_12_8_16_1_12_W_rom1(clk, addr, z);
   input clk;
   input [2:0] addr;
   output logic signed [15:0] z;
   always_ff @(posedge clk) begin
      case(addr)
        0: z <= 16'd2;
        1: z <= -16'd3;
        2: z <= 16'd5;
        3: z <= 16'd7;
        4: z <= 16'd7;
        5: z <= 16'd3;
        6: z <= 16'd3;
        7: z <= -16'd8;
      endcase
   end
endmodule

module l2_fc_12_8_16_1_12_W_rom2(clk, addr, z);
   input clk;
   input [2:0] addr;
   output logic signed [15:0] z;
   always_ff @(posedge clk) begin
      case(addr)
        0: z <= -16'd1;
        1: z <= -16'd2;
        2: z <= -16'd2;
        3: z <= 16'd4;
        4: z <= -16'd8;
        5: z <= -16'd4;
        6: z <= -16'd6;
        7: z <= 16'd4;
      endcase
   end
endmodule

module l2_fc_12_8_16_1_12_W_rom3(clk, addr, z);
   input clk;
   input [2:0] addr;
   output logic signed [15:0] z;
   always_ff @(posedge clk) begin
      case(addr)
        0: z <= 16'd6;
        1: z <= 16'd6;
        2: z <= -16'd5;
        3: z <= 16'd0;
        4: z <= -16'd6;
        5: z <= 16'd7;
        6: z <= 16'd3;
        7: z <= 16'd6;
      endcase
   end
endmodule

module l2_fc_12_8_16_1_12_W_rom4(clk, addr, z);
   input clk;
   input [2:0] addr;
   output logic signed [15:0] z;
   always_ff @(posedge clk) begin
      case(addr)
        0: z <= 16'd3;
        1: z <= -16'd4;
        2: z <= 16'd3;
        3: z <= -16'd8;
        4: z <= 16'd0;
        5: z <= 16'd3;
        6: z <= -16'd6;
        7: z <= -16'd6;
      endcase
   end
endmodule

module l2_fc_12_8_16_1_12_W_rom5(clk, addr, z);
   input clk;
   input [2:0] addr;
   output logic signed [15:0] z;
   always_ff @(posedge clk) begin
      case(addr)
        0: z <= -16'd8;
        1: z <= -16'd8;
        2: z <= -16'd6;
        3: z <= -16'd8;
        4: z <= 16'd3;
        5: z <= 16'd5;
        6: z <= -16'd8;
        7: z <= -16'd6;
      endcase
   end
endmodule

module l2_fc_12_8_16_1_12_W_rom6(clk, addr, z);
   input clk;
   input [2:0] addr;
   output logic signed [15:0] z;
   always_ff @(posedge clk) begin
      case(addr)
        0: z <= -16'd5;
        1: z <= -16'd2;
        2: z <= 16'd7;
        3: z <= -16'd5;
        4: z <= 16'd3;
        5: z <= -16'd7;
        6: z <= -16'd8;
        7: z <= 16'd1;
      endcase
   end
endmodule

module l2_fc_12_8_16_1_12_W_rom7(clk, addr, z);
   input clk;
   input [2:0] addr;
   output logic signed [15:0] z;
   always_ff @(posedge clk) begin
      case(addr)
        0: z <= 16'd7;
        1: z <= -16'd5;
        2: z <= -16'd7;
        3: z <= -16'd7;
        4: z <= -16'd6;
        5: z <= 16'd5;
        6: z <= -16'd8;
        7: z <= 16'd5;
      endcase
   end
endmodule

module l2_fc_12_8_16_1_12_W_rom8(clk, addr, z);
   input clk;
   input [2:0] addr;
   output logic signed [15:0] z;
   always_ff @(posedge clk) begin
      case(addr)
        0: z <= -16'd7;
        1: z <= 16'd3;
        2: z <= 16'd5;
        3: z <= 16'd1;
        4: z <= -16'd1;
        5: z <= 16'd7;
        6: z <= 16'd3;
        7: z <= -16'd1;
      endcase
   end
endmodule

module l2_fc_12_8_16_1_12_W_rom9(clk, addr, z);
   input clk;
   input [2:0] addr;
   output logic signed [15:0] z;
   always_ff @(posedge clk) begin
      case(addr)
        0: z <= 16'd7;
        1: z <= 16'd5;
        2: z <= -16'd1;
        3: z <= 16'd2;
        4: z <= 16'd2;
        5: z <= -16'd1;
        6: z <= 16'd5;
        7: z <= 16'd6;
      endcase
   end
endmodule

module l2_fc_12_8_16_1_12_W_rom10(clk, addr, z);
   input clk;
   input [2:0] addr;
   output logic signed [15:0] z;
   always_ff @(posedge clk) begin
      case(addr)
        0: z <= 16'd6;
        1: z <= 16'd4;
        2: z <= -16'd7;
        3: z <= 16'd1;
        4: z <= 16'd5;
        5: z <= -16'd7;
        6: z <= -16'd6;
        7: z <= 16'd4;
      endcase
   end
endmodule

module l2_fc_12_8_16_1_12_W_rom11(clk, addr, z);
   input clk;
   input [2:0] addr;
   output logic signed [15:0] z;
   always_ff @(posedge clk) begin
      case(addr)
        0: z <= -16'd4;
        1: z <= -16'd5;
        2: z <= 16'd6;
        3: z <= -16'd2;
        4: z <= -16'd8;
        5: z <= 16'd6;
        6: z <= -16'd5;
        7: z <= -16'd7;
      endcase
   end
endmodule

module l3_fc_16_12_16_1_8(clk, reset, input_valid, input_ready, input_data, output_valid, output_ready, output_data);
    localparam              WIDTH = 16;
    localparam              W_M = 16;
    localparam              W_N = 12;
    localparam              P = 8;
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

    l3_fc_16_12_16_1_8_datapath #(WIDTH, W_M, W_N, P, R) dp( 
                            .in_data(input_data), .clk(clk), .reset(reset), .clear_acc(clear_acc), .en_acc(en_acc), 
                            .addr_x(addr_x), .wr_en_x(wr_en_x), .addr_w(addr_w),
                            .out_addr(out_addr), .delay_ctrl(delay_ctrl),
                            .out_data(output_data));

    l3_fc_16_12_16_1_8_control #(WIDTH, W_M, W_N, P) ctrl(
                            .clk(clk), .reset(reset), .in_valid(input_valid), .out_ready(output_ready),
                            .addr_x(addr_x), .wr_en_x(wr_en_x), .addr_w(addr_w),
                            .clear_acc(clear_acc), .en_acc(en_acc), .in_ready(input_ready),
                            .out_valid(output_valid), .out_addr(out_addr),
                            .delay_ctrl(delay_ctrl));
endmodule

module l3_fc_16_12_16_1_8_datapath(in_data, clk, reset, 
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
    logic signed    [WIDTH-1:0]         w_in0;
    logic signed    [WIDTH-1:0]         w_in1;
    logic signed    [WIDTH-1:0]         w_in2;
    logic signed    [WIDTH-1:0]         w_in3;
    logic signed    [WIDTH-1:0]         w_in4;
    logic signed    [WIDTH-1:0]         w_in5;
    logic signed    [WIDTH-1:0]         w_in6;
    logic signed    [WIDTH-1:0]         w_in7;
    logic signed    [WIDTH-1:0]         x_in;
    logic signed    [WIDTH-1:0]         mac_out0;
    logic signed    [WIDTH-1:0]         mac_out1;
    logic signed    [WIDTH-1:0]         mac_out2;
    logic signed    [WIDTH-1:0]         mac_out3;
    logic signed    [WIDTH-1:0]         mac_out4;
    logic signed    [WIDTH-1:0]         mac_out5;
    logic signed    [WIDTH-1:0]         mac_out6;
    logic signed    [WIDTH-1:0]         mac_out7;
    logic                               mac_out_valid0;
    logic                               mac_out_valid1;
    logic                               mac_out_valid2;
    logic                               mac_out_valid3;
    logic                               mac_out_valid4;
    logic                               mac_out_valid5;
    logic                               mac_out_valid6;
    logic                               mac_out_valid7;

    logic signed    [P:0][WIDTH-1:0]     out_v;
    logic signed    [WIDTH-1:0]          out;

    memory #(WIDTH, X_SIZE) l3_fc_16_12_16_1_8_vector(.clk(clk), .data_in(in_data), .addr(addr_x), .wr_en(wr_en_x), .data_out(x_in));
    l3_fc_16_12_16_1_8_W_rom0 l3_fc_16_12_16_1_8_w0(.clk(clk), .addr(addr_w), .z(w_in0));
    l3_fc_16_12_16_1_8_W_rom1 l3_fc_16_12_16_1_8_w1(.clk(clk), .addr(addr_w), .z(w_in1));
    l3_fc_16_12_16_1_8_W_rom2 l3_fc_16_12_16_1_8_w2(.clk(clk), .addr(addr_w), .z(w_in2));
    l3_fc_16_12_16_1_8_W_rom3 l3_fc_16_12_16_1_8_w3(.clk(clk), .addr(addr_w), .z(w_in3));
    l3_fc_16_12_16_1_8_W_rom4 l3_fc_16_12_16_1_8_w4(.clk(clk), .addr(addr_w), .z(w_in4));
    l3_fc_16_12_16_1_8_W_rom5 l3_fc_16_12_16_1_8_w5(.clk(clk), .addr(addr_w), .z(w_in5));
    l3_fc_16_12_16_1_8_W_rom6 l3_fc_16_12_16_1_8_w6(.clk(clk), .addr(addr_w), .z(w_in6));
    l3_fc_16_12_16_1_8_W_rom7 l3_fc_16_12_16_1_8_w7(.clk(clk), .addr(addr_w), .z(w_in7));
    mac #(WIDTH) l3_fc_16_12_16_1_8_mc0(.clk(clk), .reset(acc_clr), .a(w_in0), .b(x_in), .valid_in(en_acc), .f(mac_out0), .valid_out(mac_out_valid0));
    mac #(WIDTH) l3_fc_16_12_16_1_8_mc1(.clk(clk), .reset(acc_clr), .a(w_in1), .b(x_in), .valid_in(en_acc), .f(mac_out1), .valid_out(mac_out_valid1));
    mac #(WIDTH) l3_fc_16_12_16_1_8_mc2(.clk(clk), .reset(acc_clr), .a(w_in2), .b(x_in), .valid_in(en_acc), .f(mac_out2), .valid_out(mac_out_valid2));
    mac #(WIDTH) l3_fc_16_12_16_1_8_mc3(.clk(clk), .reset(acc_clr), .a(w_in3), .b(x_in), .valid_in(en_acc), .f(mac_out3), .valid_out(mac_out_valid3));
    mac #(WIDTH) l3_fc_16_12_16_1_8_mc4(.clk(clk), .reset(acc_clr), .a(w_in4), .b(x_in), .valid_in(en_acc), .f(mac_out4), .valid_out(mac_out_valid4));
    mac #(WIDTH) l3_fc_16_12_16_1_8_mc5(.clk(clk), .reset(acc_clr), .a(w_in5), .b(x_in), .valid_in(en_acc), .f(mac_out5), .valid_out(mac_out_valid5));
    mac #(WIDTH) l3_fc_16_12_16_1_8_mc6(.clk(clk), .reset(acc_clr), .a(w_in6), .b(x_in), .valid_in(en_acc), .f(mac_out6), .valid_out(mac_out_valid6));
    mac #(WIDTH) l3_fc_16_12_16_1_8_mc7(.clk(clk), .reset(acc_clr), .a(w_in7), .b(x_in), .valid_in(en_acc), .f(mac_out7), .valid_out(mac_out_valid7));

    always_comb begin
        acc_clr = reset || clear_acc;
    end
    always_ff @(posedge clk) begin
        if(delay_ctrl == 2)begin
            if (R==0)begin
                out_v[0] <= mac_out0;
                out_v[1] <= mac_out1;
                out_v[2] <= mac_out2;
                out_v[3] <= mac_out3;
                out_v[4] <= mac_out4;
                out_v[5] <= mac_out5;
                out_v[6] <= mac_out6;
                out_v[7] <= mac_out7;
            end
            else begin
                if (mac_out0 > 0)
                    out_v[0] <= mac_out0;
                else
                    out_v[0] <= 0;
                if (mac_out1 > 0)
                    out_v[1] <= mac_out1;
                else
                    out_v[1] <= 0;
                if (mac_out2 > 0)
                    out_v[2] <= mac_out2;
                else
                    out_v[2] <= 0;
                if (mac_out3 > 0)
                    out_v[3] <= mac_out3;
                else
                    out_v[3] <= 0;
                if (mac_out4 > 0)
                    out_v[4] <= mac_out4;
                else
                    out_v[4] <= 0;
                if (mac_out5 > 0)
                    out_v[5] <= mac_out5;
                else
                    out_v[5] <= 0;
                if (mac_out6 > 0)
                    out_v[6] <= mac_out6;
                else
                    out_v[6] <= 0;
                if (mac_out7 > 0)
                    out_v[7] <= mac_out7;
                else
                    out_v[7] <= 0;
            end
        end
        else
            out_v <= out_v;
        end
        always_comb begin
            case(out_addr)
                0: out = out_v[0];
                1: out = out_v[1];
                2: out = out_v[2];
                3: out = out_v[3];
                4: out = out_v[4];
                5: out = out_v[5];
                6: out = out_v[6];
                7: out = out_v[7];
                default: out = 0;
            endcase
        end
        assign out_data = out;
endmodule

module l3_fc_16_12_16_1_8_control(clk, reset, in_valid, out_ready,
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

module l3_fc_16_12_16_1_8_W_rom0(clk, addr, z);
   input clk;
   input [4:0] addr;
   output logic signed [15:0] z;
   always_ff @(posedge clk) begin
      case(addr)
        0: z <= 16'd1;
        1: z <= -16'd8;
        2: z <= 16'd2;
        3: z <= -16'd8;
        4: z <= -16'd8;
        5: z <= -16'd2;
        6: z <= 16'd0;
        7: z <= 16'd7;
        8: z <= -16'd5;
        9: z <= 16'd7;
        10: z <= 16'd2;
        11: z <= 16'd6;
        12: z <= 16'd5;
        13: z <= 16'd0;
        14: z <= -16'd3;
        15: z <= 16'd5;
        16: z <= 16'd5;
        17: z <= -16'd3;
        18: z <= -16'd1;
        19: z <= 16'd1;
        20: z <= 16'd5;
        21: z <= -16'd6;
        22: z <= -16'd5;
        23: z <= 16'd1;
      endcase
   end
endmodule

module l3_fc_16_12_16_1_8_W_rom1(clk, addr, z);
   input clk;
   input [4:0] addr;
   output logic signed [15:0] z;
   always_ff @(posedge clk) begin
      case(addr)
        0: z <= -16'd1;
        1: z <= -16'd1;
        2: z <= 16'd4;
        3: z <= -16'd3;
        4: z <= -16'd5;
        5: z <= 16'd5;
        6: z <= 16'd6;
        7: z <= -16'd8;
        8: z <= 16'd7;
        9: z <= -16'd8;
        10: z <= 16'd4;
        11: z <= -16'd5;
        12: z <= -16'd5;
        13: z <= 16'd7;
        14: z <= -16'd3;
        15: z <= 16'd0;
        16: z <= 16'd5;
        17: z <= -16'd6;
        18: z <= -16'd7;
        19: z <= 16'd0;
        20: z <= -16'd6;
        21: z <= -16'd6;
        22: z <= 16'd4;
        23: z <= 16'd7;
      endcase
   end
endmodule

module l3_fc_16_12_16_1_8_W_rom2(clk, addr, z);
   input clk;
   input [4:0] addr;
   output logic signed [15:0] z;
   always_ff @(posedge clk) begin
      case(addr)
        0: z <= -16'd5;
        1: z <= 16'd2;
        2: z <= 16'd2;
        3: z <= -16'd4;
        4: z <= 16'd0;
        5: z <= 16'd5;
        6: z <= -16'd3;
        7: z <= -16'd6;
        8: z <= 16'd6;
        9: z <= -16'd8;
        10: z <= -16'd6;
        11: z <= 16'd6;
        12: z <= 16'd6;
        13: z <= -16'd7;
        14: z <= 16'd0;
        15: z <= 16'd3;
        16: z <= 16'd1;
        17: z <= 16'd1;
        18: z <= 16'd1;
        19: z <= -16'd1;
        20: z <= -16'd6;
        21: z <= 16'd6;
        22: z <= -16'd4;
        23: z <= 16'd7;
      endcase
   end
endmodule

module l3_fc_16_12_16_1_8_W_rom3(clk, addr, z);
   input clk;
   input [4:0] addr;
   output logic signed [15:0] z;
   always_ff @(posedge clk) begin
      case(addr)
        0: z <= -16'd2;
        1: z <= 16'd2;
        2: z <= 16'd5;
        3: z <= 16'd1;
        4: z <= 16'd2;
        5: z <= -16'd1;
        6: z <= -16'd1;
        7: z <= -16'd7;
        8: z <= 16'd6;
        9: z <= -16'd5;
        10: z <= -16'd2;
        11: z <= -16'd7;
        12: z <= -16'd5;
        13: z <= 16'd4;
        14: z <= 16'd0;
        15: z <= -16'd7;
        16: z <= 16'd6;
        17: z <= 16'd3;
        18: z <= 16'd2;
        19: z <= -16'd6;
        20: z <= 16'd3;
        21: z <= 16'd7;
        22: z <= 16'd2;
        23: z <= 16'd0;
      endcase
   end
endmodule

module l3_fc_16_12_16_1_8_W_rom4(clk, addr, z);
   input clk;
   input [4:0] addr;
   output logic signed [15:0] z;
   always_ff @(posedge clk) begin
      case(addr)
        0: z <= -16'd7;
        1: z <= -16'd4;
        2: z <= -16'd7;
        3: z <= -16'd8;
        4: z <= -16'd4;
        5: z <= 16'd6;
        6: z <= -16'd5;
        7: z <= -16'd1;
        8: z <= 16'd0;
        9: z <= 16'd5;
        10: z <= 16'd3;
        11: z <= -16'd7;
        12: z <= -16'd7;
        13: z <= 16'd4;
        14: z <= -16'd8;
        15: z <= -16'd5;
        16: z <= 16'd6;
        17: z <= 16'd5;
        18: z <= -16'd5;
        19: z <= 16'd4;
        20: z <= 16'd6;
        21: z <= 16'd3;
        22: z <= -16'd1;
        23: z <= 16'd0;
      endcase
   end
endmodule

module l3_fc_16_12_16_1_8_W_rom5(clk, addr, z);
   input clk;
   input [4:0] addr;
   output logic signed [15:0] z;
   always_ff @(posedge clk) begin
      case(addr)
        0: z <= 16'd3;
        1: z <= -16'd7;
        2: z <= -16'd5;
        3: z <= 16'd1;
        4: z <= -16'd7;
        5: z <= -16'd3;
        6: z <= -16'd1;
        7: z <= -16'd1;
        8: z <= -16'd8;
        9: z <= -16'd4;
        10: z <= -16'd8;
        11: z <= 16'd2;
        12: z <= -16'd4;
        13: z <= -16'd8;
        14: z <= 16'd7;
        15: z <= -16'd2;
        16: z <= 16'd6;
        17: z <= -16'd5;
        18: z <= -16'd3;
        19: z <= -16'd7;
        20: z <= 16'd7;
        21: z <= 16'd5;
        22: z <= -16'd6;
        23: z <= 16'd6;
      endcase
   end
endmodule

module l3_fc_16_12_16_1_8_W_rom6(clk, addr, z);
   input clk;
   input [4:0] addr;
   output logic signed [15:0] z;
   always_ff @(posedge clk) begin
      case(addr)
        0: z <= 16'd4;
        1: z <= 16'd0;
        2: z <= 16'd3;
        3: z <= 16'd2;
        4: z <= 16'd3;
        5: z <= -16'd7;
        6: z <= 16'd4;
        7: z <= 16'd4;
        8: z <= -16'd3;
        9: z <= 16'd5;
        10: z <= 16'd4;
        11: z <= 16'd1;
        12: z <= 16'd1;
        13: z <= 16'd4;
        14: z <= -16'd8;
        15: z <= -16'd4;
        16: z <= 16'd4;
        17: z <= 16'd2;
        18: z <= 16'd4;
        19: z <= 16'd5;
        20: z <= -16'd2;
        21: z <= 16'd4;
        22: z <= -16'd7;
        23: z <= -16'd4;
      endcase
   end
endmodule

module l3_fc_16_12_16_1_8_W_rom7(clk, addr, z);
   input clk;
   input [4:0] addr;
   output logic signed [15:0] z;
   always_ff @(posedge clk) begin
      case(addr)
        0: z <= 16'd3;
        1: z <= -16'd8;
        2: z <= -16'd8;
        3: z <= -16'd4;
        4: z <= 16'd5;
        5: z <= 16'd4;
        6: z <= -16'd3;
        7: z <= 16'd0;
        8: z <= 16'd5;
        9: z <= 16'd0;
        10: z <= -16'd7;
        11: z <= 16'd6;
        12: z <= 16'd1;
        13: z <= -16'd4;
        14: z <= -16'd8;
        15: z <= 16'd0;
        16: z <= 16'd7;
        17: z <= -16'd1;
        18: z <= -16'd8;
        19: z <= -16'd5;
        20: z <= -16'd1;
        21: z <= 16'd7;
        22: z <= 16'd2;
        23: z <= -16'd3;
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

