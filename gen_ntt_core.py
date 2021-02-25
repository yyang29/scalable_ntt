from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

import math


def gen_ntt_core(out_file, ntt_core_type='resource_efficient', io_width=28,
                 moduli_config=None):

    moduli_config = [[
        [28, 37, 2, 0, 0, 0, 1],        # sign: 1 - negative
        [42, 45, 42, 40, 41, 46, 0],
        [46, 53, 49, 0, 0, 0, 1],
    ], [
        [28, 37, 2, 0, 0, 0, 1],
        [42, 45, 42, 40, 41, 46, 0],
        [46, 53, 49, 0, 0, 0, 1],
    ]]

    adder_tree_inputs = 0
    adder_tree_levels = 0

    # parameters declarations
    params = f'localparam NUM_Q = {len(moduli_config)};\n'
    params += '  localparam Q_ID_BITS = $clog2(NUM_Q);\n'
    params += '  localparam ADD_OP = 1;\n'
    params += '  localparam SUB_OP = 0;\n'
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
                adder_tree_construct += f'  modular_adder ma_instance_{i}_{j} (op_{j*2}_signed, op_{j*2+1}_signed, sum_wire_l{i}_{j}, q, 1\'b0);\n'
            else:
                adder_tree_construct += f'  modular_adder ma_instance_{i}_{j} (sum_reg_l{i-1}_{j*2}, sum_reg_l{i-1}_{j*2+1}, sum_wire_l{i}_{j}, q, 1\'b0);\n'
            if i == adder_tree_levels - 1:
                assert j == 0
                adder_tree_construct += f'  assign out_data = sum_reg_l{i}_{j};\n'
                adder_tree_construct += f'  assign out_data_valid = data_valid_reg_l{i};\n'
        nodes_prev_level = nodes_curr_level

    # adder tree operands updates
    opr_construct = '\n  always @ (q_id, mult_data) begin\n'
    for i in range(0, len(moduli_config)):
        if i == 0:
            opr_construct += f'    if (q_id == {i}) begin\n'
        else:
            opr_construct += f'    end else if (q_id == {i}) begin\n'
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

    if ntt_core_type == 'resource_efficient':
        ntt_core_sv = f"""// This verilog file is generated automatically.
// Resource efficient NTT core optimized for solinas primes.
// Author: Yang Yang (yyang172@usc.edu)
`timescale 1ns/1ps

module modular_adder (
    x,
    y,
    out,
    q,
    op
  );

  input [{io_width}-1:0] x;
  input [{io_width}-1:0] y;
  input [{io_width}-1:0] q;
  input op;
  output [{io_width}-1:0] out;

  logic [{io_width}-1:0] z1;
  logic [{io_width}-1:0] z2;

  assign z1 = (op == 0) ? x + y : x - y;
  assign z2 = z1 - q;

  assign out = (z1 >= q) ? z2[{io_width}-1:0] : z1[{io_width}-1:0];

endmodule


module modular_reduction (
    clk,
    rst,
    mult_data_valid,
    mult_data,
    out_data_valid,
    out_data,
    q_id,
    q
  );

  {params}

  input clk, rst;

  input mult_data_valid;
  input [2*{io_width}-1:0] mult_data;

  output logic out_data_valid;
  output [{io_width}-1:0] out_data;

  input [Q_ID_BITS-1:0] q_id;
  input [{io_width}-1:0] q;

  {internal_signals}

  {opr_construct}

  {adder_tree_construct}

  // data path
  always_ff @ (posedge clk) begin
    {always_block_dp}
  end
endmodule


// module ntt_core (
//     in_data_valid,
//     in_data_0,
//     in_data_1,
//     out_data_valid,
//     out_data_0,
//     out_data_1,
//     twiddle_rd_addr_valid,
//     twiddle_rd_addr,
//     twiddle_rd_data_valid,
//     twiddle_rd_data,
//     q_id,
//     clk,
//     rst
//   );
// 
//   input ctrl, clk, rst;
//   input         [{io_width}-1:0] inData_0, inData_1;
//   output logic  [{io_width}-1:0] outData_0, outData_1;
// 
//   always_ff @ (posedge clk) begin
//     if (rst) begin
//       outData_0 <= 0;
//       outData_1 <= 0;
//     end else begin
//       outData_0 <= (!ctrl) ? inData_0 : inData_1;
//       outData_1 <= (!ctrl) ? inData_1 : inData_0;
//     end
//   end
// 
// endmodule

"""
        with open(out_file + '.sv', 'a+') as fid:
            fid.write(ntt_core_sv)

        ntt_core_test_sv = f"""// This verilog file is generated automatically.
// Resource efficient NTT core optimized for solinas primes.
// Author: Yang Yang (yyang172@usc.edu)
`timescale 1ns/1ps

module ntt_core_tb;

  localparam NUM_Q = {len(moduli_config)};
  localparam Q_ID_BITS = $clog2(NUM_Q);
  localparam CLK_PERIOD = 10;
  logic clk, rst;

  logic mult_data_valid;
  logic [2*{io_width}-1:0] mult_data;

  logic out_data_valid;
  logic [{io_width}-1:0] out_data;

  logic [Q_ID_BITS-1:0] q_id;
  logic [{io_width}-1:0] q;

  // clock generation
  initial begin
    clk = 1;
    forever begin
      #(CLK_PERIOD/2) clk = ~clk;
      q_id = 0;
      q = 28'hFFFC001;
    end 
  end

  modular_reduction mr_instance (
    .clk(clk),
    .rst(rst),
    .mult_data_valid(mult_data_valid),
    .mult_data(mult_data),
    .out_data_valid(out_data_valid),
    .out_data(out_data),
    .q_id(q_id),
    .q(q)
  );

  // reset generation
  initial begin
    rst = 1;
    #(2 * CLK_PERIOD) rst = 0;
  end

  // testing
  integer i;
  bit [2*{io_width}-1:0] rand_num;
  initial begin
    #(3.5 * CLK_PERIOD);
    for (i = 0; i < 16; i = i + 1) begin
      mult_data_valid = 1'b1;
      std::randomize(rand_num);
      mult_data = rand_num;
      #CLK_PERIOD;
    end
  end
 
endmodule

"""
        with open(out_file + '_test.sv', 'a+') as fid:
            fid.write(ntt_core_test_sv)

    else:
        ntt_core_sv = f"""// This verilog file is generated automatically.
// General purpose NTT core.
// Author: Yang Yang (yyang172@usc.edu)
"""


gen_ntt_core('/home/yang/Desktop/test')
