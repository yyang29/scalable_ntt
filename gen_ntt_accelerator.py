from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

import argparse
import math
import os

import gen_ntt_spn


def gen_interconnect(ntt_config):
    """Generate 2x2 switch to form spn stage."""

    gen_ntt_spn.gen_common_modules(ntt_config)

    # stride for each SPN
    perm_routing = []
    for i in range(0, ntt_config.pp):
        perm_routing.append([])

    stride = ntt_config.N
    i = 0
    while stride > 1:
        stride = stride // 2
        perm_routing[i % ntt_config.pp].append(stride)
        i = i + 1

    # Generate SPN and connect them together
    for i in range(0, ntt_config.pp):
        # i is SPN id
        gen_ntt_spn.gen_spn(ntt_config, i, perm_routing[i])


def gen_ntt_core():
    """Generate NTT core."""
    pass


def gen_ntt_super_pipeline(ntt_config, pipe_id):
    """Generate NTT pipeline that contains one spn and multiple NTT cores."""
    pass


def gen_ntt_top(ntt_config):
    """Generate NTT top."""

    wire_declrs = ''
    for i in range(0, ntt_config.pp - 1):
        wire_declrs += f'  logic [DATA_WIDTH-1:0] spn_{i}_out_data[DP-1:0];\n'

    spn_insts = ''
    for i in range(0, ntt_config.pp):
        spn_insts += f'  spn_{i} spn_{i}_instance (\n'
        if i == 0:
            for j in range(0, ntt_config.dp):
                spn_insts += f'    .inData_{j}(in_data[{j}]),\n'
            for j in range(0, ntt_config.dp):
                spn_insts += f'    .outData_{j}(spn_{i}_out_data[{j}]),\n'
        elif i == ntt_config.pp - 1:
            for j in range(0, ntt_config.dp):
                spn_insts += f'    .inData_{j}(spn_{i-1}_out_data[{j}]),\n'
            for j in range(0, ntt_config.dp):
                spn_insts += f'    .outData_{j}(out_data[{j}]),\n'
        else:
            for j in range(0, ntt_config.dp):
                spn_insts += f'    .inData_{j}(spn_{i-1}_out_data[{j}]),\n'
            for j in range(0, ntt_config.dp):
                spn_insts += f'    .outData_{j}(spn_{i}_out_data[{j}]),\n'
        spn_insts += '    .clk(clk),\n'
        spn_insts += '    .rst(rst),\n'
        spn_insts += '    .in_start(in_start),\n'
        spn_insts += '    .out_start(out_start),\n'
        spn_insts += '    .ntt_sel(ntt_sel),\n'
        spn_insts += '    .modulus(modulus));\n\n'

    # TODO: Add tap-in logic
    top_sv = f"""

module ntt_top(
    in_data,
    out_data,
    ntt_sel,
    modulus,
    in_start,
    out_start,
    clk,
    rst
  );

  localparam DATA_WIDTH = {ntt_config.io_width};
  localparam DP = {ntt_config.dp}
  localparam DP_WIDTH = $clog2(DP);
  localparam NTT_SEL_WIDTH = $clog2($clog2({ntt_config.N}));

  input in_start, clk, rst;
  output out_start;

  input [NTT_SEL_WIDTH-1:0] ntt_sel;
  input [DATA_WIDTH-1:0] modulus;

  input [DATA_WIDTH-1:0] in_data;
  output [DATA_WIDTH-1:0] out_data;

{wire_declrs}

{spn_insts}

endmodule

"""
    with open(ntt_config.out_file, 'a+') as fid:
        fid.write(top_sv)


def gen_ntt_top_wrapper(ntt_config):
    """Generate NTT top wrapper."""

    wrapper_sv = f"""

module ntt_top_wrapper (
    in_start,
    out_start,
    in_data,
    out_data,
    clk,
    rst,
    ntt_sel,
    modulus
  );
  
  localparam DATA_WIDTH = {ntt_config.io_width};
  localparam DP = {ntt_config.dp}
  localparam DP_WIDTH = $clog2(DP);
  localparam NTT_SEL_WIDTH = $clog2($clog2({ntt_config.N}));

  input in_start, clk, rst;
  output out_start;

  input [NTT_SEL_WIDTH-1:0] ntt_sel;
  input [DATA_WIDTH-1:0] modulus;

  input [DATA_WIDTH-1:0] in_data;
  output [DATA_WIDTH-1:0] out_data;

  logic [DP_WIDTH-1:0] counter;
  
  logic [DATA_WIDTH-1:0] in_data_reg[DP-1:0];
  logic [DATA_WIDTH-1:0] out_data_reg[DP-1:0];

  ntt_top ntt_top_instance (
    .in_data(in_data_reg),
    .out_data(out_data_reg),
    .clk(clk),
    .rst(rst),
    .in_start(in_start),
    .out_start(out_start),
    .ntt_sel(ntt_sel),
    .modulus(modulus));
  
  integer i;
  always_ff @ (posedge clk) begin
    if (rst) begin
      counter <= 0;
      for (i = 0; i < DP; i = i + 1) begin
        in_data_reg[i] <= 0;
      end
    end else begin
      counter <= counter + 1;
      in_data_reg[counter] <= in_data;
    end
  end

  assign out_data = out_data_reg[counter];

endmodule

"""
    with open(ntt_config.out_file, 'a+') as fid:
        fid.write(wrapper_sv)


class NttGenConfig:
    def __init__(self, N, ntt_core_type, dp, pp, moduli, io_width, out_folder):
        self.N = N
        self.ntt_core_type = ntt_core_type
        self.dp = dp
        self.pp = pp
        self.moduli = moduli
        self.io_width = io_width
        self.out_folder = out_folder

    @property
    def ntt_stages(self):
        return self.N.bit_length() - 1

    @property
    def ntt_stages_per_pipe(self):
        return self.ntt_stages // self.pp

    @property
    def spn_stages_per_pipe(self):
        return 2 * (self.dp.bit_length() - 1)

    @property
    def spn_stages_per_spatial_perm(self):
        return self.dp.bit_length() - 1

    @property
    def spn_switches_per_stage(self):
        return int(self.dp // 2)


def main():

    parser = argparse.ArgumentParser(description='NTT Accelerator Generator.')
    parser.add_argument('-t', '--ntt-core-type', type=str, default='compact',
                        help='Type of ntt core. compact|full')
    parser.add_argument('-N', '--N', type=int, default=1024,
                        help='Max polynomial degree to support.')
    parser.add_argument('-dp', '--data-parallelism', type=int, default=4,
                        help='Data parallelism to support.')
    parser.add_argument('-pp', '--pipeline-parallelism', type=int, default=2,
                        help='Pipeline parallelism to support.')
    parser.add_argument('-m', '--moduli', type=int, nargs='+', default=[],
                        help='List of modulus.')
    parser.add_argument('-w', '--io-width', type=int, default=64,
                        help='Input and output bitwidth.')
    parser.add_argument('-o', '--output-folder', type=str, default='ntt.sv',
                        help='Output verilog file directory.')

    args = parser.parse_args()

    if not os.path.exists(args.output_folder):
        os.makedirs(args.output_folder)

    ntt_config = NttGenConfig(args.N, args.ntt_core_type, args.data_parallelism, args.pipeline_parallelism,
                              args.moduli, args.io_width, args.output_folder)

    # Number of beats for one polynomial to flow through one pipeline
    #beats = args.N // args.data_parallelism

    # Number of switches per spn stage
    #switches_per_spn_stage = args.data_parallelism // 2

    # Ctrl bits per switch
    #ctrl_bits_per_switch = beats * ntt_stages_per_pipe

    # Total control bits in KB
    # total_ctrl_bits = (ctrl_bits_per_switch * switches_per_spn_stage *
    #                   spn_stages * args.pipeline_parallelism) / 8 / 1024

    print('##### Architecture parameters #####')
    print('Max NTT stages: %d' % ntt_config.ntt_stages)
    print('Max NTT stages per pipeline: %d' % ntt_config.ntt_stages_per_pipe)
    print('SPN stages: %d' % ntt_config.spn_stages_per_pipe)

    gen_interconnect(ntt_config)
    # gen_ntt_core()
    #for i in range(args.pipeline_parallelism):
    #    gen_ntt_super_pipeline(ntt_config, i)
    #gen_ntt_top(ntt_config)
    #gen_ntt_top_wrapper(ntt_config)


if __name__ == '__main__':
    main()
