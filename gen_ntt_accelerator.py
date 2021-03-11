from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

import argparse
import math
import os

import dse
import gen_ntt_core
import gen_ntt_spn

def gen_tp_top(ntt_config, d_idx, tp_idx):
    out_folder = ntt_config.out_folder + '/design_' + str(d_idx)
    out_file = out_folder + '/task_' + str(tp_idx) + '.sv'
    wire_declrs = ''

    spn_insts = ''
    spn_insts += '    .clk(clk),\n'
    spn_insts += '    .rst(rst),\n'
    spn_insts += '    .in_start(in_start),\n'
    spn_insts += '    .out_start(out_start),\n'
    spn_insts += '    .ntt_sel(ntt_sel),\n'
    spn_insts += '    .modulus(modulus));\n\n'

    task_sv = f"""

module task_{tp_idx} (
    in_data,
    out_data,
    in_start,
    out_start,
    tw_valid,
    tw_addr,
    tw_data,
    clk,
    rst
  );

  localparam DATA_WIDTH = {ntt_config.io_width};
  localparam DP = {ntt_config.dp[d_idx]}
  localparam DP_WIDTH = $clog2(DP[d_idx]);

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
    with open(out_file, 'a+') as fid:
        fid.write(task_sv)


def gen_ntt_super_pipeline(ntt_config, d_idx, tp_idx, pp_idx):
    """Generate NTT pipeline that contains one spn and multiple NTT cores."""
    out_folder = ntt_config.out_folder + '/design_' + str(d_idx)
    out_file = out_folder + '/super_pipe_' + str(tp_idx) + '_' + str(pp_idx) + '.sv'

    wire_declrs = ''

    spn_insts = ''
    spn_insts += '    .clk(clk),\n'
    spn_insts += '    .rst(rst),\n'
    spn_insts += '    .in_start(in_start),\n'
    spn_insts += '    .out_start(out_start),\n'
    spn_insts += '    .ntt_sel(ntt_sel),\n'
    spn_insts += '    .modulus(modulus));\n\n'


    super_pipe_sv = f"""

module super_pipe_{tp_idx}_{pp_idx} (
    in_data,
    out_data,
    in_start,
    out_start,
    tw_valid,
    tw_addr,
    tw_data,
    clk,
    rst
  );

  localparam DATA_WIDTH = {ntt_config.io_width};
  localparam DP = {ntt_config.dp[d_idx]}
  localparam DP_WIDTH = $clog2(DP[d_idx]);

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
    with open(out_file, 'a+') as fid:
        fid.write(super_pipe_sv)



def gen_ntt_top(ntt_config, d_idx):
    """Generate NTT top."""
    out_folder = ntt_config.out_folder + '/design_' + str(d_idx)

    wire_declrs = ''
    for i in range(0, ntt_config.pp[d_idx] - 1):
        wire_declrs += f'  logic [DATA_WIDTH-1:0] spn_{i}_out_data[DP-1:0];\n'

    spn_insts = ''
    for i in range(0, ntt_config.pp[d_idx]):
        spn_insts += f'  spn_{i} spn_{i}_instance (\n'
        if i == 0:
            for j in range(0, ntt_config.dp[d_idx]):
                spn_insts += f'    .inData_{j}(in_data[{j}]),\n'
            for j in range(0, ntt_config.dp[d_idx]):
                spn_insts += f'    .outData_{j}(spn_{i}_out_data[{j}]),\n'
        elif i == ntt_config.pp[d_idx] - 1:
            for j in range(0, ntt_config.dp[d_idx]):
                spn_insts += f'    .inData_{j}(spn_{i-1}_out_data[{j}]),\n'
            for j in range(0, ntt_config.dp[d_idx]):
                spn_insts += f'    .outData_{j}(out_data[{j}]),\n'
        else:
            for j in range(0, ntt_config.dp[d_idx]):
                spn_insts += f'    .inData_{j}(spn_{i-1}_out_data[{j}]),\n'
            for j in range(0, ntt_config.dp[d_idx]):
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
  localparam DP = {ntt_config.dp[d_idx]}
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
    with open(out_folder + '/ntt_top.sv', 'a+') as fid:
        fid.write(top_sv)


def gen_ntt_top_wrapper(ntt_config, d_idx):
    """Generate NTT top wrapper."""
    out_folder = ntt_config.out_folder + '/design_' + str(d_idx)

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
  localparam DP = {ntt_config.dp[d_idx]}
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
    with open(out_folder + '/ntt_top_wrapper.sv', 'a+') as fid:
        fid.write(wrapper_sv)


class NttGenConfig:
    def __init__(self, N, moduli, io_width, latency, out_folder):
        # input parameters
        self.N = N
        self.moduli = moduli
        self.io_width = io_width
        self.latency = latency
        self.out_folder = out_folder
        # input constraints
        self.limit_lut = 1182240
        self.limit_bram = 2160
        self.limit_ff = 2364480
        self.limit_dsp = 6800
        self.bw_gbps = 70
        self.freq_mhz = 200
        # DSE output
        self.ntt_core_type = None
        self.tp = 0
        self.dp = []
        self.pp = []

    @property
    def ntt_stages(self):
        return self.N.bit_length() - 1

    @property
    def ntt_stages_per_pipe(self):
        return self.ntt_stages // self.pp

    def spn_stages_per_pipe(self, idx):
        return 2 * (self.dp[idx].bit_length() - 1)

    def spn_stages_per_spatial_perm(self, idx):
        return self.dp[idx].bit_length() - 1

    def spn_switches_per_stage(self, idx):
        return int(self.dp[idx] // 2)


def main():

    parser = argparse.ArgumentParser(description='NTT Accelerator Generator.')
    parser.add_argument('-N', '--N', type=int, default=1024,
                        help='Max polynomial degree to support.')
    parser.add_argument('-m', '--moduli', type=int, nargs='+', default=[],
                        help='List of modulus.')
    parser.add_argument('-w', '--io-width', type=int, default=60,
                        help='Input and output bitwidth.')
    parser.add_argument('-o', '--output-folder', type=str, default='ntt.sv',
                        help='Output verilog file directory.')
    parser.add_argument('-l', '--latency', type=float, default=1.5,
                        help='Latency in us.')

    args = parser.parse_args()

    if not os.path.exists(args.output_folder):
        os.makedirs(args.output_folder)

    ntt_config = NttGenConfig(args.N, args.moduli, args.io_width, args.latency,
                              args.output_folder)

    ntt_config = dse.dse(ntt_config)

    valid_designs = len(ntt_config.dp)

    print('\nDSE identifies %d valid designs.' % valid_designs)

    for i in range(valid_designs):
        out_folder = ntt_config.out_folder + '/design_' + str(i)
        if not os.path.exists(out_folder):
            os.makedirs(out_folder)
        print('\ngenerating design point %d' % i)
        print('design point tp %d dp %d pp %d mod %s' % (
            ntt_config.tp, ntt_config.dp[i], ntt_config.pp[i], ntt_config.moduli))

        gen_ntt_spn.gen_common_modules(ntt_config, i)

        for j in range(ntt_config.tp):
            N = ntt_config.N
            pp = ntt_config.pp[i]
            stride = []
            while N > 1:
                N = N // 2
                stride.append(N)
            for k in range(pp):
                my_stride = []
                for offset in range(k, ntt_config.ntt_stages, pp):
                    my_stride.append(stride[offset])
                gen_ntt_spn.gen_spn(ntt_config, i, j, k, my_stride)
                # gen_ntt_core()
                gen_ntt_super_pipeline(ntt_config, i, j, k)
            gen_tp_top(ntt_config, i, j)
        gen_ntt_top(ntt_config, i)
        gen_ntt_top_wrapper(ntt_config, i)


if __name__ == '__main__':
    main()
