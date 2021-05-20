module fc_8_4_16_1_1(clk, reset, input_valid, input_ready, input_data, output_valid, output_ready, output_data);
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

    fc_8_4_16_1_1_datapath #(WIDTH, W_M, W_N, P, R) dp( 
                            .in_data(input_data), .clk(clk), .reset(reset), .clear_acc(clear_acc), .en_acc(en_acc), 
                            .addr_x(addr_x), .wr_en_x(wr_en_x), .addr_w(addr_w),
                            .out_addr(out_addr), .delay_ctrl(delay_ctrl),
                            .out_data(output_data));

    fc_8_4_16_1_1_control #(WIDTH, W_M, W_N, P) ctrl(
                            .clk(clk), .reset(reset), .in_valid(input_valid), .out_ready(output_ready),
                            .addr_x(addr_x), .wr_en_x(wr_en_x), .addr_w(addr_w),
                            .clear_acc(clear_acc), .en_acc(en_acc), .in_ready(input_ready),
                            .out_valid(output_valid), .out_addr(out_addr),
                            .delay_ctrl(delay_ctrl));
endmodule

module fc_8_4_16_1_1_datapath(in_data, clk, reset, 
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

    memory #(WIDTH, X_SIZE) fc_8_4_16_1_1_vector(.clk(clk), .data_in(in_data), .addr(addr_x), .wr_en(wr_en_x), .data_out(x_in));
    fc_8_4_16_1_1_W_rom fc_8_4_16_1_1_w(.clk(clk), .addr(addr_w), .z(w_in));
    mac #(WIDTH) fc_8_4_16_1_1_mc(.clk(clk), .reset(acc_clr), .a(w_in), .b(x_in), .valid_in(en_acc), .f(mac_out), .valid_out(mac_out_valid));

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

module fc_8_4_16_1_1_control(clk, reset, in_valid, out_ready,
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

module fc_8_4_16_1_1_W_rom(clk, addr, z);
   input clk;
   input [4:0] addr;
   output logic signed [15:0] z;
   always_ff @(posedge clk) begin
      case(addr)
        0: z <= 16'd80;
        1: z <= 16'd81;
        2: z <= -16'd15;
        3: z <= 16'd58;
        4: z <= -16'd68;
        5: z <= -16'd57;
        6: z <= 16'd64;
        7: z <= -16'd107;
        8: z <= 16'd90;
        9: z <= 16'd0;
        10: z <= 16'd19;
        11: z <= -16'd52;
        12: z <= 16'd57;
        13: z <= -16'd65;
        14: z <= 16'd20;
        15: z <= 16'd125;
        16: z <= 16'd102;
        17: z <= -16'd65;
        18: z <= -16'd35;
        19: z <= -16'd123;
        20: z <= -16'd61;
        21: z <= 16'd1;
        22: z <= -16'd41;
        23: z <= 16'd21;
        24: z <= -16'd62;
        25: z <= -16'd8;
        26: z <= 16'd69;
        27: z <= -16'd12;
        28: z <= 16'd60;
        29: z <= -16'd25;
        30: z <= -16'd89;
        31: z <= 16'd13;
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

