module fc_16_8_16_1_2(clk, reset, input_valid, input_ready, input_data, output_valid, output_ready, output_data);
    localparam              WIDTH = 16;
    localparam              W_M = 16;
    localparam              W_N = 8;
    localparam              P = 2;
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

    fc_16_8_16_1_2_datapath #(WIDTH, W_M, W_N, P, R) dp( 
                            .in_data(input_data), .clk(clk), .reset(reset), .clear_acc(clear_acc), .en_acc(en_acc), 
                            .addr_x(addr_x), .wr_en_x(wr_en_x), .addr_w(addr_w),
                            .out_addr(out_addr), .delay_ctrl(delay_ctrl),
                            .out_data(output_data));

    fc_16_8_16_1_2_control #(WIDTH, W_M, W_N, P) ctrl(
                            .clk(clk), .reset(reset), .in_valid(input_valid), .out_ready(output_ready),
                            .addr_x(addr_x), .wr_en_x(wr_en_x), .addr_w(addr_w),
                            .clear_acc(clear_acc), .en_acc(en_acc), .in_ready(input_ready),
                            .out_valid(output_valid), .out_addr(out_addr),
                            .delay_ctrl(delay_ctrl));
endmodule

module fc_16_8_16_1_2_datapath(in_data, clk, reset, 
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
    logic signed    [WIDTH-1:0]         x_in;
    logic signed    [WIDTH-1:0]         mac_out0;
    logic signed    [WIDTH-1:0]         mac_out1;
    logic                               mac_out_valid0;
    logic                               mac_out_valid1;

    logic signed    [P:0][WIDTH-1:0]     out_v;
    logic signed    [WIDTH-1:0]          out;

    memory #(WIDTH, X_SIZE) fc_16_8_16_1_2_vector(.clk(clk), .data_in(in_data), .addr(addr_x), .wr_en(wr_en_x), .data_out(x_in));
    fc_16_8_16_1_2_W_rom0 fc_16_8_16_1_2_w0(.clk(clk), .addr(addr_w), .z(w_in0));
    fc_16_8_16_1_2_W_rom1 fc_16_8_16_1_2_w1(.clk(clk), .addr(addr_w), .z(w_in1));
    mac #(WIDTH) fc_16_8_16_1_2_mc0(.clk(clk), .reset(acc_clr), .a(w_in0), .b(x_in), .valid_in(en_acc), .f(mac_out0), .valid_out(mac_out_valid0));
    mac #(WIDTH) fc_16_8_16_1_2_mc1(.clk(clk), .reset(acc_clr), .a(w_in1), .b(x_in), .valid_in(en_acc), .f(mac_out1), .valid_out(mac_out_valid1));

    always_comb begin
        acc_clr = reset || clear_acc;
    end
    always_ff @(posedge clk) begin
        if(delay_ctrl == 2)begin
            if (R==0)begin
                out_v[0] <= mac_out0;
                out_v[1] <= mac_out1;
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
            end
        end
        else
            out_v <= out_v;
        end
        always_comb begin
            case(out_addr)
                0: out = out_v[0];
                1: out = out_v[1];
                default: out = 0;
            endcase
        end
        assign out_data = out;
endmodule

module fc_16_8_16_1_2_control(clk, reset, in_valid, out_ready,
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

module fc_16_8_16_1_2_W_rom0(clk, addr, z);
   input clk;
   input [5:0] addr;
   output logic signed [15:0] z;
   always_ff @(posedge clk) begin
      case(addr)
        0: z <= 16'd102;
        1: z <= 16'd59;
        2: z <= -16'd8;
        3: z <= 16'd46;
        4: z <= 16'd45;
        5: z <= 16'd12;
        6: z <= 16'd89;
        7: z <= 16'd92;
        8: z <= -16'd1;
        9: z <= 16'd46;
        10: z <= 16'd57;
        11: z <= 16'd73;
        12: z <= -16'd46;
        13: z <= -16'd18;
        14: z <= 16'd117;
        15: z <= 16'd27;
        16: z <= -16'd37;
        17: z <= 16'd126;
        18: z <= 16'd4;
        19: z <= -16'd120;
        20: z <= 16'd10;
        21: z <= -16'd35;
        22: z <= 16'd100;
        23: z <= -16'd18;
        24: z <= -16'd120;
        25: z <= -16'd54;
        26: z <= 16'd104;
        27: z <= -16'd38;
        28: z <= 16'd56;
        29: z <= 16'd94;
        30: z <= 16'd118;
        31: z <= -16'd75;
        32: z <= -16'd39;
        33: z <= 16'd1;
        34: z <= -16'd111;
        35: z <= 16'd100;
        36: z <= 16'd94;
        37: z <= 16'd118;
        38: z <= -16'd46;
        39: z <= 16'd60;
        40: z <= 16'd30;
        41: z <= 16'd73;
        42: z <= 16'd118;
        43: z <= -16'd42;
        44: z <= 16'd39;
        45: z <= 16'd108;
        46: z <= 16'd11;
        47: z <= 16'd95;
        48: z <= 16'd4;
        49: z <= 16'd65;
        50: z <= 16'd122;
        51: z <= -16'd30;
        52: z <= 16'd55;
        53: z <= -16'd52;
        54: z <= -16'd98;
        55: z <= 16'd88;
        56: z <= -16'd60;
        57: z <= 16'd45;
        58: z <= -16'd88;
        59: z <= 16'd108;
        60: z <= 16'd26;
        61: z <= 16'd52;
        62: z <= 16'd75;
        63: z <= 16'd19;
      endcase
   end
endmodule

module fc_16_8_16_1_2_W_rom1(clk, addr, z);
   input clk;
   input [5:0] addr;
   output logic signed [15:0] z;
   always_ff @(posedge clk) begin
      case(addr)
        0: z <= 16'd100;
        1: z <= 16'd0;
        2: z <= -16'd58;
        3: z <= -16'd7;
        4: z <= -16'd88;
        5: z <= -16'd11;
        6: z <= -16'd15;
        7: z <= 16'd11;
        8: z <= -16'd4;
        9: z <= -16'd39;
        10: z <= -16'd105;
        11: z <= 16'd32;
        12: z <= 16'd111;
        13: z <= 16'd32;
        14: z <= 16'd6;
        15: z <= 16'd86;
        16: z <= 16'd93;
        17: z <= -16'd85;
        18: z <= 16'd104;
        19: z <= -16'd122;
        20: z <= 16'd32;
        21: z <= -16'd39;
        22: z <= 16'd17;
        23: z <= -16'd97;
        24: z <= -16'd73;
        25: z <= -16'd115;
        26: z <= 16'd85;
        27: z <= -16'd89;
        28: z <= 16'd45;
        29: z <= -16'd37;
        30: z <= 16'd125;
        31: z <= -16'd119;
        32: z <= -16'd95;
        33: z <= -16'd70;
        34: z <= 16'd66;
        35: z <= 16'd65;
        36: z <= 16'd20;
        37: z <= -16'd45;
        38: z <= 16'd97;
        39: z <= 16'd28;
        40: z <= 16'd121;
        41: z <= -16'd31;
        42: z <= -16'd122;
        43: z <= 16'd39;
        44: z <= 16'd60;
        45: z <= -16'd125;
        46: z <= 16'd48;
        47: z <= -16'd106;
        48: z <= 16'd7;
        49: z <= 16'd96;
        50: z <= 16'd26;
        51: z <= -16'd101;
        52: z <= -16'd76;
        53: z <= -16'd5;
        54: z <= 16'd55;
        55: z <= 16'd82;
        56: z <= -16'd107;
        57: z <= 16'd81;
        58: z <= -16'd70;
        59: z <= 16'd81;
        60: z <= 16'd84;
        61: z <= 16'd106;
        62: z <= 16'd103;
        63: z <= -16'd40;
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

