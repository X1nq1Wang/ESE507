module fc_16_8_16_1_16(clk, reset, input_valid, input_ready, input_data, output_valid, output_ready, output_data);
    localparam              WIDTH = 16;
    localparam              W_M = 16;
    localparam              W_N = 8;
    localparam              P = 16;
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

    fc_16_8_16_1_16_datapath #(WIDTH, W_M, W_N, P, R) dp( 
                            .in_data(input_data), .clk(clk), .reset(reset), .clear_acc(clear_acc), .en_acc(en_acc), 
                            .addr_x(addr_x), .wr_en_x(wr_en_x), .addr_w(addr_w),
                            .out_addr(out_addr), .delay_ctrl(delay_ctrl),
                            .out_data(output_data));

    fc_16_8_16_1_16_control #(WIDTH, W_M, W_N, P) ctrl(
                            .clk(clk), .reset(reset), .in_valid(input_valid), .out_ready(output_ready),
                            .addr_x(addr_x), .wr_en_x(wr_en_x), .addr_w(addr_w),
                            .clear_acc(clear_acc), .en_acc(en_acc), .in_ready(input_ready),
                            .out_valid(output_valid), .out_addr(out_addr),
                            .delay_ctrl(delay_ctrl));
endmodule

module fc_16_8_16_1_16_datapath(in_data, clk, reset, 
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
    logic signed    [WIDTH-1:0]         w_in12;
    logic signed    [WIDTH-1:0]         w_in13;
    logic signed    [WIDTH-1:0]         w_in14;
    logic signed    [WIDTH-1:0]         w_in15;
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
    logic signed    [WIDTH-1:0]         mac_out12;
    logic signed    [WIDTH-1:0]         mac_out13;
    logic signed    [WIDTH-1:0]         mac_out14;
    logic signed    [WIDTH-1:0]         mac_out15;
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
    logic                               mac_out_valid12;
    logic                               mac_out_valid13;
    logic                               mac_out_valid14;
    logic                               mac_out_valid15;

    logic signed    [P:0][WIDTH-1:0]     out_v;
    logic signed    [WIDTH-1:0]          out;

    memory #(WIDTH, X_SIZE) fc_16_8_16_1_16_vector(.clk(clk), .data_in(in_data), .addr(addr_x), .wr_en(wr_en_x), .data_out(x_in));
    fc_16_8_16_1_16_W_rom0 fc_16_8_16_1_16_w0(.clk(clk), .addr(addr_w), .z(w_in0));
    fc_16_8_16_1_16_W_rom1 fc_16_8_16_1_16_w1(.clk(clk), .addr(addr_w), .z(w_in1));
    fc_16_8_16_1_16_W_rom2 fc_16_8_16_1_16_w2(.clk(clk), .addr(addr_w), .z(w_in2));
    fc_16_8_16_1_16_W_rom3 fc_16_8_16_1_16_w3(.clk(clk), .addr(addr_w), .z(w_in3));
    fc_16_8_16_1_16_W_rom4 fc_16_8_16_1_16_w4(.clk(clk), .addr(addr_w), .z(w_in4));
    fc_16_8_16_1_16_W_rom5 fc_16_8_16_1_16_w5(.clk(clk), .addr(addr_w), .z(w_in5));
    fc_16_8_16_1_16_W_rom6 fc_16_8_16_1_16_w6(.clk(clk), .addr(addr_w), .z(w_in6));
    fc_16_8_16_1_16_W_rom7 fc_16_8_16_1_16_w7(.clk(clk), .addr(addr_w), .z(w_in7));
    fc_16_8_16_1_16_W_rom8 fc_16_8_16_1_16_w8(.clk(clk), .addr(addr_w), .z(w_in8));
    fc_16_8_16_1_16_W_rom9 fc_16_8_16_1_16_w9(.clk(clk), .addr(addr_w), .z(w_in9));
    fc_16_8_16_1_16_W_rom10 fc_16_8_16_1_16_w10(.clk(clk), .addr(addr_w), .z(w_in10));
    fc_16_8_16_1_16_W_rom11 fc_16_8_16_1_16_w11(.clk(clk), .addr(addr_w), .z(w_in11));
    fc_16_8_16_1_16_W_rom12 fc_16_8_16_1_16_w12(.clk(clk), .addr(addr_w), .z(w_in12));
    fc_16_8_16_1_16_W_rom13 fc_16_8_16_1_16_w13(.clk(clk), .addr(addr_w), .z(w_in13));
    fc_16_8_16_1_16_W_rom14 fc_16_8_16_1_16_w14(.clk(clk), .addr(addr_w), .z(w_in14));
    fc_16_8_16_1_16_W_rom15 fc_16_8_16_1_16_w15(.clk(clk), .addr(addr_w), .z(w_in15));
    mac #(WIDTH) fc_16_8_16_1_16_mc0(.clk(clk), .reset(acc_clr), .a(w_in0), .b(x_in), .valid_in(en_acc), .f(mac_out0), .valid_out(mac_out_valid0));
    mac #(WIDTH) fc_16_8_16_1_16_mc1(.clk(clk), .reset(acc_clr), .a(w_in1), .b(x_in), .valid_in(en_acc), .f(mac_out1), .valid_out(mac_out_valid1));
    mac #(WIDTH) fc_16_8_16_1_16_mc2(.clk(clk), .reset(acc_clr), .a(w_in2), .b(x_in), .valid_in(en_acc), .f(mac_out2), .valid_out(mac_out_valid2));
    mac #(WIDTH) fc_16_8_16_1_16_mc3(.clk(clk), .reset(acc_clr), .a(w_in3), .b(x_in), .valid_in(en_acc), .f(mac_out3), .valid_out(mac_out_valid3));
    mac #(WIDTH) fc_16_8_16_1_16_mc4(.clk(clk), .reset(acc_clr), .a(w_in4), .b(x_in), .valid_in(en_acc), .f(mac_out4), .valid_out(mac_out_valid4));
    mac #(WIDTH) fc_16_8_16_1_16_mc5(.clk(clk), .reset(acc_clr), .a(w_in5), .b(x_in), .valid_in(en_acc), .f(mac_out5), .valid_out(mac_out_valid5));
    mac #(WIDTH) fc_16_8_16_1_16_mc6(.clk(clk), .reset(acc_clr), .a(w_in6), .b(x_in), .valid_in(en_acc), .f(mac_out6), .valid_out(mac_out_valid6));
    mac #(WIDTH) fc_16_8_16_1_16_mc7(.clk(clk), .reset(acc_clr), .a(w_in7), .b(x_in), .valid_in(en_acc), .f(mac_out7), .valid_out(mac_out_valid7));
    mac #(WIDTH) fc_16_8_16_1_16_mc8(.clk(clk), .reset(acc_clr), .a(w_in8), .b(x_in), .valid_in(en_acc), .f(mac_out8), .valid_out(mac_out_valid8));
    mac #(WIDTH) fc_16_8_16_1_16_mc9(.clk(clk), .reset(acc_clr), .a(w_in9), .b(x_in), .valid_in(en_acc), .f(mac_out9), .valid_out(mac_out_valid9));
    mac #(WIDTH) fc_16_8_16_1_16_mc10(.clk(clk), .reset(acc_clr), .a(w_in10), .b(x_in), .valid_in(en_acc), .f(mac_out10), .valid_out(mac_out_valid10));
    mac #(WIDTH) fc_16_8_16_1_16_mc11(.clk(clk), .reset(acc_clr), .a(w_in11), .b(x_in), .valid_in(en_acc), .f(mac_out11), .valid_out(mac_out_valid11));
    mac #(WIDTH) fc_16_8_16_1_16_mc12(.clk(clk), .reset(acc_clr), .a(w_in12), .b(x_in), .valid_in(en_acc), .f(mac_out12), .valid_out(mac_out_valid12));
    mac #(WIDTH) fc_16_8_16_1_16_mc13(.clk(clk), .reset(acc_clr), .a(w_in13), .b(x_in), .valid_in(en_acc), .f(mac_out13), .valid_out(mac_out_valid13));
    mac #(WIDTH) fc_16_8_16_1_16_mc14(.clk(clk), .reset(acc_clr), .a(w_in14), .b(x_in), .valid_in(en_acc), .f(mac_out14), .valid_out(mac_out_valid14));
    mac #(WIDTH) fc_16_8_16_1_16_mc15(.clk(clk), .reset(acc_clr), .a(w_in15), .b(x_in), .valid_in(en_acc), .f(mac_out15), .valid_out(mac_out_valid15));

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
                out_v[12] <= mac_out12;
                out_v[13] <= mac_out13;
                out_v[14] <= mac_out14;
                out_v[15] <= mac_out15;
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
                if (mac_out12 > 0)
                    out_v[12] <= mac_out12;
                else
                    out_v[12] <= 0;
                if (mac_out13 > 0)
                    out_v[13] <= mac_out13;
                else
                    out_v[13] <= 0;
                if (mac_out14 > 0)
                    out_v[14] <= mac_out14;
                else
                    out_v[14] <= 0;
                if (mac_out15 > 0)
                    out_v[15] <= mac_out15;
                else
                    out_v[15] <= 0;
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
                12: out = out_v[12];
                13: out = out_v[13];
                14: out = out_v[14];
                15: out = out_v[15];
                default: out = 0;
            endcase
        end
        assign out_data = out;
endmodule

module fc_16_8_16_1_16_control(clk, reset, in_valid, out_ready,
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

module fc_16_8_16_1_16_W_rom0(clk, addr, z);
   input clk;
   input [2:0] addr;
   output logic signed [15:0] z;
   always_ff @(posedge clk) begin
      case(addr)
        0: z <= 16'd52;
        1: z <= -16'd28;
        2: z <= -16'd22;
        3: z <= -16'd59;
        4: z <= -16'd126;
        5: z <= 16'd41;
        6: z <= -16'd48;
        7: z <= -16'd55;
      endcase
   end
endmodule

module fc_16_8_16_1_16_W_rom1(clk, addr, z);
   input clk;
   input [2:0] addr;
   output logic signed [15:0] z;
   always_ff @(posedge clk) begin
      case(addr)
        0: z <= 16'd46;
        1: z <= -16'd31;
        2: z <= -16'd52;
        3: z <= -16'd125;
        4: z <= -16'd96;
        5: z <= 16'd19;
        6: z <= -16'd59;
        7: z <= -16'd44;
      endcase
   end
endmodule

module fc_16_8_16_1_16_W_rom2(clk, addr, z);
   input clk;
   input [2:0] addr;
   output logic signed [15:0] z;
   always_ff @(posedge clk) begin
      case(addr)
        0: z <= 16'd13;
        1: z <= -16'd62;
        2: z <= -16'd48;
        3: z <= -16'd116;
        4: z <= 16'd94;
        5: z <= 16'd61;
        6: z <= -16'd63;
        7: z <= 16'd1;
      endcase
   end
endmodule

module fc_16_8_16_1_16_W_rom3(clk, addr, z);
   input clk;
   input [2:0] addr;
   output logic signed [15:0] z;
   always_ff @(posedge clk) begin
      case(addr)
        0: z <= -16'd104;
        1: z <= -16'd36;
        2: z <= 16'd2;
        3: z <= -16'd5;
        4: z <= -16'd89;
        5: z <= 16'd60;
        6: z <= -16'd95;
        7: z <= 16'd91;
      endcase
   end
endmodule

module fc_16_8_16_1_16_W_rom4(clk, addr, z);
   input clk;
   input [2:0] addr;
   output logic signed [15:0] z;
   always_ff @(posedge clk) begin
      case(addr)
        0: z <= -16'd96;
        1: z <= 16'd11;
        2: z <= -16'd95;
        3: z <= -16'd94;
        4: z <= -16'd76;
        5: z <= -16'd15;
        6: z <= -16'd20;
        7: z <= 16'd98;
      endcase
   end
endmodule

module fc_16_8_16_1_16_W_rom5(clk, addr, z);
   input clk;
   input [2:0] addr;
   output logic signed [15:0] z;
   always_ff @(posedge clk) begin
      case(addr)
        0: z <= 16'd82;
        1: z <= 16'd56;
        2: z <= 16'd101;
        3: z <= 16'd114;
        4: z <= -16'd53;
        5: z <= -16'd85;
        6: z <= -16'd57;
        7: z <= 16'd88;
      endcase
   end
endmodule

module fc_16_8_16_1_16_W_rom6(clk, addr, z);
   input clk;
   input [2:0] addr;
   output logic signed [15:0] z;
   always_ff @(posedge clk) begin
      case(addr)
        0: z <= -16'd19;
        1: z <= 16'd23;
        2: z <= 16'd100;
        3: z <= -16'd53;
        4: z <= -16'd43;
        5: z <= -16'd90;
        6: z <= 16'd76;
        7: z <= -16'd19;
      endcase
   end
endmodule

module fc_16_8_16_1_16_W_rom7(clk, addr, z);
   input clk;
   input [2:0] addr;
   output logic signed [15:0] z;
   always_ff @(posedge clk) begin
      case(addr)
        0: z <= 16'd2;
        1: z <= -16'd50;
        2: z <= 16'd104;
        3: z <= 16'd41;
        4: z <= -16'd117;
        5: z <= -16'd119;
        6: z <= 16'd5;
        7: z <= -16'd85;
      endcase
   end
endmodule

module fc_16_8_16_1_16_W_rom8(clk, addr, z);
   input clk;
   input [2:0] addr;
   output logic signed [15:0] z;
   always_ff @(posedge clk) begin
      case(addr)
        0: z <= 16'd20;
        1: z <= 16'd38;
        2: z <= -16'd50;
        3: z <= 16'd73;
        4: z <= -16'd105;
        5: z <= 16'd58;
        6: z <= 16'd43;
        7: z <= 16'd105;
      endcase
   end
endmodule

module fc_16_8_16_1_16_W_rom9(clk, addr, z);
   input clk;
   input [2:0] addr;
   output logic signed [15:0] z;
   always_ff @(posedge clk) begin
      case(addr)
        0: z <= -16'd14;
        1: z <= 16'd17;
        2: z <= 16'd92;
        3: z <= 16'd61;
        4: z <= 16'd60;
        5: z <= -16'd93;
        6: z <= 16'd21;
        7: z <= -16'd87;
      endcase
   end
endmodule

module fc_16_8_16_1_16_W_rom10(clk, addr, z);
   input clk;
   input [2:0] addr;
   output logic signed [15:0] z;
   always_ff @(posedge clk) begin
      case(addr)
        0: z <= 16'd58;
        1: z <= -16'd6;
        2: z <= -16'd12;
        3: z <= -16'd113;
        4: z <= 16'd32;
        5: z <= -16'd63;
        6: z <= -16'd4;
        7: z <= -16'd94;
      endcase
   end
endmodule

module fc_16_8_16_1_16_W_rom11(clk, addr, z);
   input clk;
   input [2:0] addr;
   output logic signed [15:0] z;
   always_ff @(posedge clk) begin
      case(addr)
        0: z <= 16'd15;
        1: z <= -16'd28;
        2: z <= 16'd75;
        3: z <= 16'd26;
        4: z <= -16'd19;
        5: z <= -16'd48;
        6: z <= 16'd70;
        7: z <= -16'd126;
      endcase
   end
endmodule

module fc_16_8_16_1_16_W_rom12(clk, addr, z);
   input clk;
   input [2:0] addr;
   output logic signed [15:0] z;
   always_ff @(posedge clk) begin
      case(addr)
        0: z <= 16'd118;
        1: z <= -16'd108;
        2: z <= 16'd75;
        3: z <= -16'd115;
        4: z <= 16'd78;
        5: z <= -16'd10;
        6: z <= 16'd119;
        7: z <= -16'd64;
      endcase
   end
endmodule

module fc_16_8_16_1_16_W_rom13(clk, addr, z);
   input clk;
   input [2:0] addr;
   output logic signed [15:0] z;
   always_ff @(posedge clk) begin
      case(addr)
        0: z <= -16'd121;
        1: z <= 16'd83;
        2: z <= 16'd125;
        3: z <= 16'd67;
        4: z <= 16'd118;
        5: z <= 16'd18;
        6: z <= 16'd108;
        7: z <= 16'd48;
      endcase
   end
endmodule

module fc_16_8_16_1_16_W_rom14(clk, addr, z);
   input clk;
   input [2:0] addr;
   output logic signed [15:0] z;
   always_ff @(posedge clk) begin
      case(addr)
        0: z <= -16'd116;
        1: z <= -16'd31;
        2: z <= 16'd64;
        3: z <= 16'd44;
        4: z <= 16'd34;
        5: z <= -16'd68;
        6: z <= 16'd78;
        7: z <= -16'd79;
      endcase
   end
endmodule

module fc_16_8_16_1_16_W_rom15(clk, addr, z);
   input clk;
   input [2:0] addr;
   output logic signed [15:0] z;
   always_ff @(posedge clk) begin
      case(addr)
        0: z <= 16'd33;
        1: z <= 16'd26;
        2: z <= 16'd76;
        3: z <= -16'd114;
        4: z <= 16'd106;
        5: z <= 16'd18;
        6: z <= -16'd112;
        7: z <= 16'd97;
      endcase
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

