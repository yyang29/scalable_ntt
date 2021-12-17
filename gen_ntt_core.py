from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

import math


def gen_ntt_core(ntt_core_type, ntt_config, d_idx, tp_idx):

    out_folder = ntt_config.out_folder + '/design_' + str(d_idx) + '/'
    prefix = 'ntt_core_tp_' + str(tp_idx)
    modulus = ntt_config.moduli[tp_idx]

    io_width = ntt_config.io_width

    gen_pipelined_mult(out_folder+'pipelined_mult.sv', io_width, prefix)
    gen_pipelined_mult(
        out_folder+'pipelined_mult_lowerhalf.sv', io_width, prefix, True)

    num_tf_per_core = int(
        ntt_config.N // ntt_config.pp[d_idx] // ntt_config.dp[d_idx])

    moduli_config = [[
        [28, 37, 2, 0, 0, 0, 1],        # sign: 1 - negative
        [42, 45, 42, 40, 41, 46, 0],
        [46, 53, 49, 0, 0, 0, 1],
    ]]

    adder_tree_inputs = 0
    adder_tree_levels = 0

    # parameters declarations
    params = f'localparam ADD_OP = 1;\n'
    params += '  localparam SUB_OP = 0;\n'
    params += f'  localparam [{io_width}-1:0] Q = {io_width}\'d{modulus};\n'
    for i in range(0, len(moduli_config)):
        adder_tree_inputs = max(len(moduli_config[i])+1, adder_tree_inputs)
        adder_tree_levels = int(math.ceil(math.log(adder_tree_inputs, 2)))
        for j in range(0, len(moduli_config[i])):
            params += f'  localparam Q_{i}_OP_{j}_P1_START_IDX = {moduli_config[i][j][0]};\n'
            params += f'  localparam Q_{i}_OP_{j}_P1_END_IDX = {moduli_config[i][j][1]};\n'
            params += f'  localparam Q_{i}_OP_{j}_P1_OFFSET = {moduli_config[i][j][2]};\n'
            params += f'  localparam Q_{i}_OP_{j}_P2_START_IDX = {moduli_config[i][j][3]};\n'
            params += f'  localparam Q_{i}_OP_{j}_P2_END_IDX = {moduli_config[i][j][4]};\n'
            params += f'  localparam Q_{i}_OP_{j}_P2_OFFSET = {moduli_config[i][j][5]};\n'
            params += f'  localparam Q_{i}_OP_{j}_SIGN = {moduli_config[i][j][6]};\n'

    # internal signals declrations
    internal_signals = '\n'
    for i in range(0, adder_tree_inputs):
        internal_signals += f'  logic [{io_width}-1:0] op_{i};\n'
        internal_signals += f'  logic op_{i}_sign;\n'
        internal_signals += f'  logic [{io_width}-1:0] op_{i}_signed;\n'

    nodes_curr_level = 0
    nodes_prev_level = adder_tree_inputs
    for i in range(0, adder_tree_levels):
        nodes_curr_level = nodes_prev_level // 2 + nodes_prev_level % 2
        for j in range(0, nodes_curr_level):
            internal_signals += f'  logic [{io_width}-1:0] sum_reg_l{i}_{j};\n'
            internal_signals += f'  logic [{io_width}-1:0] sum_wire_l{i}_{j};\n'
        internal_signals += f'  logic data_valid_reg_l{i};\n'
        nodes_prev_level = nodes_curr_level

    # adder tree construct
    adder_tree_construct = '\n'
    for i in range(0, adder_tree_inputs):
        adder_tree_construct += f'  assign op_{i}_signed = (op_{i}_sign == 1) ? -op_{i} : op_{i};\n'

    nodes_curr_level = 0
    nodes_prev_level = adder_tree_inputs
    for i in range(0, adder_tree_levels):
        nodes_curr_level = nodes_prev_level // 2 + nodes_prev_level % 2
        for j in range(0, nodes_curr_level):
            if i == 0:
                adder_tree_construct += f'  {prefix}_modular_adder ma_instance_{i}_{j} (op_{j*2}_signed, op_{j*2+1}_signed, sum_wire_l{i}_{j});\n'
            else:
                adder_tree_construct += f'  {prefix}_modular_adder ma_instance_{i}_{j} (sum_reg_l{i-1}_{j*2}, sum_reg_l{i-1}_{j*2+1}, sum_wire_l{i}_{j});\n'
            if i == adder_tree_levels - 1:
                assert j == 0
                adder_tree_construct += f'  assign out_data = sum_reg_l{i}_{j};\n'
                adder_tree_construct += f'  assign out_data_valid = data_valid_reg_l{i};\n'
        nodes_prev_level = nodes_curr_level

    # adder tree operands updates
    opr_construct = '\n  always_ff @ (posedge clk) begin\n'
    opr_construct += f'    if (mult_data_valid) begin\n'
    for i in range(0, len(moduli_config)):
        for j in range(0, adder_tree_inputs):
            if j < adder_tree_inputs - 1:
                opr_construct += f"      op_{j} <= 28'h0 | (mult_data[Q_{i}_OP_{j}_P1_END_IDX:Q_{i}_OP_{j}_P1_START_IDX] << Q_{i}_OP_{j}_P1_OFFSET) | (mult_data[Q_{i}_OP_{j}_P2_END_IDX:Q_{i}_OP_{j}_P2_START_IDX] << Q_{i}_OP_{j}_P2_OFFSET);\n"
                opr_construct += f'      op_{j}_sign <= Q_{i}_OP_{j}_SIGN;\n'
            else:
                opr_construct += f'      op_{j} <= mult_data[2*{io_width}-1: {io_width}];\n'
                opr_construct += f'      op_{j}_sign <= 0;\n'
    opr_construct += '    end\n'
    opr_construct += '  end\n'

    # adder tree data path
    always_block_dp = '\n'
    nodes_curr_level = 0
    nodes_prev_level = adder_tree_inputs
    for i in range(0, adder_tree_levels):
        nodes_curr_level = nodes_prev_level // 2 + nodes_prev_level % 2
        for j in range(0, nodes_curr_level):
            always_block_dp += f'      sum_reg_l{i}_{j} <= sum_wire_l{i}_{j};\n'
        if i == 0:
            always_block_dp += f'      data_valid_reg_l{i} <= mult_data_valid;\n'
        else:
            always_block_dp += f'      data_valid_reg_l{i} <= data_valid_reg_l{i-1};\n'
        nodes_prev_level = nodes_curr_level

    ntt_core_pipe_regs = '\n'
    ntt_core_pipe_regs += f'  logic [{io_width}-1:0] w_reg;\n'
    ntt_core_pipe_regs += f'  logic [{io_width}-1:0] x1_reg;\n'
    ntt_core_pipe_regs += f'  logic [{io_width}-1:0] x2_reg;\n'
    for i in range(0, adder_tree_levels):
        ntt_core_pipe_regs += f'  logic [{io_width}-1:0] x{i+3}_reg;\n'

    ntt_core_seq_block = '\n'
    ntt_core_seq_block += '  always_ff @(posedge clk) begin\n'
    ntt_core_seq_block += '    if (rst) begin\n'
    ntt_core_seq_block += '      counter <= 0;\n'
    ntt_core_seq_block += '      twiddle_rd_addr_valid <= 0;\n'
    ntt_core_seq_block += '      twiddle_rd_addr <= 0;\n'
    ntt_core_seq_block += '    end else begin\n'
    ntt_core_seq_block += '      if (in_data_valid) begin\n'
    ntt_core_seq_block += '        counter <= counter + 1;\n'
    ntt_core_seq_block += '        twiddle_rd_addr_valid <= 1;\n'
    ntt_core_seq_block += '        twiddle_rd_addr <= counter;\n'
    ntt_core_seq_block += '        x1_reg <= in_data_0;\n'
    ntt_core_seq_block += '        x2_reg <= x1_reg;\n'
    for i in range(0, adder_tree_levels):
        ntt_core_seq_block += f'        x{i+3}_reg <= x{i+2}_reg;\n'
    ntt_core_seq_block += '      end\n'
    ntt_core_seq_block += '    end\n'
    ntt_core_seq_block += '  end\n'

    ma_sv = f"""
module {prefix}_modular_adder #(
    parameter op = 0
  ) (
    x,
    y,
    out
  );

  localparam [{io_width}-1:0] q = {io_width}\'d{modulus};

  input [{io_width}-1:0] x;
  input [{io_width}-1:0] y;
  output [{io_width}-1:0] out;

  logic [{io_width}-1:0] z1;
  logic [{io_width}-1:0] z2;

  assign z1 = (op == 0) ? x + y : x - y;
  assign z2 = z1 - q;

  assign out = (z1 >= q) ? z2[{io_width}-1:0] : z1[{io_width}-1:0];

endmodule
"""

    if ntt_core_type == 'specialized':
        ntt_core_sv = f"""// This verilog file is generated automatically.
// Resource efficient NTT core optimized for solinas primes.
// Author: Yang Yang (yyang172@usc.edu)
`timescale 1ns/1ps

{ma_sv}

module {prefix}_modular_reduction (
    clk,
    rst,
    mult_data_valid,
    mult_data,
    out_data_valid,
    out_data
  );

  {params}

  input clk, rst;

  input mult_data_valid;
  input [2*{io_width}-1:0] mult_data;

  output logic out_data_valid;
  output [{io_width}-1:0] out_data;

  {internal_signals}

  {opr_construct}

  {adder_tree_construct}

  // data path
  always_ff @ (posedge clk) begin
    {always_block_dp}
  end
endmodule

module {prefix} (
    in_data_valid,
    in_data_0,
    in_data_1,
    out_data_0,
    out_data_1,
    twiddle_rd_addr_valid,
    twiddle_rd_addr,
    twiddle_rd_data,
    clk,
    rst
  );

  input clk, rst;
  input         in_data_valid;
  input         [{io_width}-1:0] in_data_0, in_data_1;
  input         [{io_width}-1:0] twiddle_rd_data;
  output logic  twiddle_rd_addr_valid;
  output logic  [{num_tf_per_core.bit_length() - 2}:0] twiddle_rd_addr;
  output logic  [{io_width}-1:0] out_data_0, out_data_1;

  logic [{num_tf_per_core.bit_length() - 2}:0] counter;

  {ntt_core_pipe_regs}

  logic [{2*io_width}-1:0] z;
  logic [{io_width}-1:0] mr_out;

  //assign z = in_data_1 * twiddle_rd_data;
  {prefix}_pipelined_mult pm(
    .clk(clk),
    .rst(rst),
    .in1(in_data_1),
    .in2(twiddle_rd_data),
    .out(z));

  {prefix}_modular_reduction mr(
    .clk(clk),
    .rst(rst),
    .mult_data_valid(1'b1),
    .mult_data(z),
    .out_data(mr_out));

  {prefix}_modular_adder #(
    .op(0)) ma_inst_0 (x{adder_tree_levels+2}_reg, mr_out, out_data_0);

  {prefix}_modular_adder #(
    .op(1)) ma_inst_1 (x{adder_tree_levels+2}_reg, mr_out, out_data_1);

  {ntt_core_seq_block}

endmodule

"""
        with open(out_folder + prefix + '.sv', 'a+') as fid:
            fid.write(ntt_core_sv)

    else:
        ntt_core_sv = f"""// This verilog file is generated automatically.
// General purpose NTT core.
// Author: Yang Yang (yyang172@usc.edu)
`timescale 1ns/1ps

{ma_sv}

module {prefix} (
    in_data_valid,
    in_data_0,
    in_data_1,
    out_data_0,
    out_data_1,
    twiddle_rd_addr_valid,
    twiddle_rd_addr,
    twiddle_rd_data,
    clk,
    rst
  );

  localparam L = {io_width} + 1;
  localparam T = {io_width}\'d{modulus << 1};

  input clk, rst;
  input         in_data_valid;
  input         [{io_width}-1:0] in_data_0, in_data_1;
  input         [{io_width}-1:0] twiddle_rd_data;
  output logic  twiddle_rd_addr_valid;
  output logic  [{num_tf_per_core.bit_length() - 2}:0] twiddle_rd_addr;
  output logic  [{io_width}-1:0] out_data_0, out_data_1;

  logic [{num_tf_per_core.bit_length() - 2}:0] counter;

  logic [{2*io_width}-1:0] U;
  logic [{2*io_width}-1:0] U_reg;
  logic [{io_width}-1:0] V;
  logic [{io_width}-1:0] V_reg;
  logic [{io_width}-1:0] W;
  logic [{io_width}-1:0] W_reg;
  logic [{io_width}:0] X;
  logic [{io_width}:0] X_reg;
  logic [{io_width}-1:0] X_shift_out;
  logic [{io_width}-1:0] Y_shift_out;
  logic [{io_width}-1:0] Y_reg;
  logic [{io_width}:0] Z_0;
  logic [{io_width}:0] Z_0_reg;
  logic [{io_width}:0] Z_1;
  logic [{io_width}:0] Z_1_reg;

  logic [{io_width}-1:0] mr_out;

  logic [{2*io_width}-1:0] temp;


  Shift_Register #(
    .NPIPE_DEPTH(8),
    .DATA_WIDTH({io_width})) sr_inst_0 (
    .clock(clk),
    .reset(rst),
    .input_data(in_data_1),
    .output_data(Y_shift_out));

  Shift_Register #(
    .NPIPE_DEPTH(12),
    .DATA_WIDTH({io_width})) sr_inst_1 (
    .clock(clk),
    .reset(rst),
    .input_data(in_data_0),
    .output_data(X_shift_out));

  //assign U = in_data_1 * twiddle_rd_data;
  {prefix}_pipelined_mult pm1(
    .clk(clk),
    .rst(rst),
    .in1(in_data_1),
    .in2(twiddle_rd_data),
    .out(U));

  assign V = U_reg >> (L - 1);

  //assign W = (V_reg * T) >> (L + 1);
  {prefix}_pipelined_mult pm2(
    .clk(clk),
    .rst(rst),
    .in1(V_reg),
    .in2(T),
    .out(temp));
  assign W = temp >> (L + 1);

  //assign X = W_reg * {io_width}\'d{modulus};
  {prefix}_pipelined_mult_lowerhalf pm3(
    .clk(clk),
    .rst(rst),
    .in1(W_reg),
    .in2({io_width}\'d{modulus}),
    .out(X));

  assign Z_0 = X_reg < Y_reg ? (2 << (L + 1)) + X_reg - Y_reg : X_reg - Y_reg;
  assign Z_1 = Z_0_reg >= 2 * {io_width}\'d{modulus} ? Z_0_reg - 2 * {io_width}\'d{modulus} : Z_0_reg - {io_width}\'d{modulus};

  assign mr_out = Z_1;

  always_ff @(posedge clk) begin
    if (rst) begin
      U_reg <= 0;
      V_reg <= 0;
      W_reg <= 0;
      X_reg <= 0;
      Y_reg <= 0;
      Z_0_reg <= 0;
    end else begin
      U_reg <= U;
      V_reg <= V;
      W_reg <= W;
      X_reg <= X;
      Y_reg <= Y_shift_out;
      Z_0_reg <= Z_0;
    end
  end

  {prefix}_modular_adder #(
    .op(0)) ma_inst_0 (X_shift_out, mr_out, out_data_0);

  {prefix}_modular_adder #(
    .op(1)) ma_inst_1 (X_shift_out, mr_out, out_data_1);

endmodule

"""
        with open(out_folder + prefix + '.sv', 'a+') as fid:
            fid.write(ntt_core_sv)


def gen_pipelined_mult(filename, bitwidth, prefix, lower_half=False):

    module_name = prefix + '_pipelined_mult'
    if lower_half:
        module_name += '_lowerhalf'

    pipeline_depth = 0
    if bitwidth == 28 and not lower_half:
        pipeline_depth = 4
    elif bitwidth == 28 and lower_half:
        pipeline_depth = 4
    elif bitwidth == 52 and not lower_half:
        pipeline_depth = 6
    elif bitwidth == 52 and lower_half:
        pipeline_depth = 5
    else:
        assert False

    mult_out_signals = '\n'
    for i in range(0, pipeline_depth):
        mult_out_signals += f'    logic [45-1:0] mult_out_{i};\n'

    timing_block = '\n'
    for i in range(0, pipeline_depth-1):
        if i == 0:
            timing_block += 'in1_regs[0] <= in1;\n'
            timing_block += 'in2_regs[0] <= in2;\n'
        else:
            timing_block += f'in1_regs[{i}] <= in1_regs[{i-1}];\n'
            timing_block += f'in2_regs[{i}] <= in2_regs[{i-1}];\n'

    for i in range(0, pipeline_depth):
        if i == 0:
            timing_block += 'pdt[0] <= mult_out_0;\n'
        else:
            timing_block += f'pdt[{i}] <= mult_out_{i} + pdt[{i-1}];\n'

    assign_block = '\n'
    if bitwidth == 28:
        assign_block += f"""
    (* use_dsp = "yes" *) assign mult_out_0 = in1[19:0] * in2[15:0];
    (* use_dsp = "yes" *) assign mult_out_1 = in1_regs[0][27:20] * in2_regs[0][27:16];
    (* use_dsp = "yes" *) assign mult_out_2 = in1_regs[1][19:0] * in2_regs[1][27:16];
    (* use_dsp = "yes" *) assign mult_out_3 = in1_regs[2][27:20] * in2_regs[2][15:0];
    """
    elif bitwidth == 52 and not lower_half:
        assign_block += f"""
    (* use_dsp = "yes" *) assign mult_out_0 = in1[26:0] * in2[15:0];
    (* use_dsp = "yes" *) assign mult_out_1 = in1_regs[0][26:0] * in2_regs[0][31:16];
    (* use_dsp = "yes" *) assign mult_out_2 = in1_regs[1][26:0] * in2_regs[1][47:32];
    (* use_dsp = "yes" *) assign mult_out_3 = in1_regs[2][51:27] * in2_regs[2][16:0];
    (* use_dsp = "yes" *) assign mult_out_4 = in1_regs[3][51:27] * in2_regs[3][31:16];
    (* use_dsp = "yes" *) assign mult_out_5 = in1_regs[4][51:27] * in2_regs[4][47:32];
    """
    elif bitwidth == 52 and lower_half:
        assign_block += f"""
    (* use_dsp = "yes" *) assign mult_out_0 = in1[26:0] * in2[15:0];
    (* use_dsp = "yes" *) assign mult_out_1 = in1_regs[0][26:0] * in2_regs[0][31:16];
    (* use_dsp = "yes" *) assign mult_out_2 = in1_regs[1][26:0] * in2_regs[1][47:32];
    (* use_dsp = "yes" *) assign mult_out_3 = in1_regs[2][51:27] * in2_regs[2][16:0];
    (* use_dsp = "yes" *) assign mult_out_4 = in1_regs[3][51:27] * in2_regs[3][31:16];
    """
    else:
        assert False

    pipelined_mult_sv = f"""
module {module_name} #(
    parameter WIDTH = {bitwidth},
    parameter PIPELINE_DEPTH = {pipeline_depth},
    parameter lower_half = 0
    ) (
    clk,
    rst,
    in1,
    in2,
    out
    );
    
    input clk;
    input rst;
    
    input [WIDTH-1:0] in1;
    input [WIDTH-1:0] in2;
    output [2*WIDTH-1:0] out;
    
    logic [2*WIDTH-1:0] pdt [PIPELINE_DEPTH-1:0];
    logic [WIDTH-1:0] in1_regs [PIPELINE_DEPTH-2:0];
    logic [WIDTH-1:0] in2_regs [PIPELINE_DEPTH-2:0];
    
    {mult_out_signals}
    
    assign out = pdt[PIPELINE_DEPTH-1];
    
    always_ff @ (posedge clk) begin
        {timing_block}
    end
    
    {assign_block}
    
endmodule
"""
    with open(filename, 'a+') as fid:
        fid.write(pipelined_mult_sv)
