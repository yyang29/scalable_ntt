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

    num_tf_per_core = ntt_config.N // ntt_config.pp[d_idx] // ntt_config.dp[d_idx]

    io_width = ntt_config.io_width
    dp = ntt_config.dp[d_idx]
    pp = ntt_config.pp[d_idx]

    wire_declars = '\n'
    wire_declars += f'  logic sp_out_start[{pp}-1:0];\n'
    for i in range(1, pp):
        wire_declars += f'  logic [DATA_WIDTH-1:0] sp_{i-1}_out[DP-1:0];\n'

    for i in range(0, pp):
        wire_declars += f'  logic [DATA_WIDTH-1:0] sp_{i}_tw_rd_data[DP_2-1:0];\n'
        wire_declars += f'  logic [{num_tf_per_core.bit_length() - 2}:0] sp_{i}_tw_rd_addr[DP_2-1:0];\n'
        wire_declars += f'  logic sp_{i}_tw_rd_valid[DP_2-1:0];\n'

    sp_insts = '\n'
    if pp > 1:
        for i in range(0, pp):
            if i == 0:
                sp_insts += f'  super_pipe_{tp_idx}_0 sp_inst_0 (\n'
                sp_insts += f'    .clk(clk),\n'
                sp_insts += f'    .rst(rst),\n'
                sp_insts += f'    .in_data(first_sp_in),\n'
                sp_insts += f'    .out_data(sp_0_out),\n'
                sp_insts += f'    .in_start(in_start),\n'
                sp_insts += f'    .out_start(sp_out_start[0]),\n'
                sp_insts += f'    .twiddle_rd_addr_valid(sp_0_tw_rd_valid),\n'
                sp_insts += f'    .twiddle_rd_addr(sp_0_tw_rd_addr),\n'
                sp_insts += f'    .twiddle_rd_data(sp_0_tw_rd_data));\n'
            elif i == pp - 1:
                sp_insts += f'  super_pipe_{tp_idx}_{i} sp_inst_{i} (\n'
                sp_insts += f'    .clk(clk),\n'
                sp_insts += f'    .rst(rst),\n'
                sp_insts += f'    .in_data(sp_{i-1}_out),\n'
                sp_insts += f'    .out_data(last_sp_out),\n'
                sp_insts += f'    .in_start(sp_out_start[{i-1}]),\n'
                sp_insts += f'    .out_start(sp_out_start[{i}]),\n'
                sp_insts += f'    .twiddle_rd_addr_valid(sp_{i}_tw_rd_valid),\n'
                sp_insts += f'    .twiddle_rd_addr(sp_{i}_tw_rd_addr),\n'
                sp_insts += f'    .twiddle_rd_data(sp_{i}_tw_rd_data));\n'
            else:
                sp_insts += f'  super_pipe_{tp_idx}_{i} sp_inst_{i} (\n'
                sp_insts += f'    .clk(clk),\n'
                sp_insts += f'    .rst(rst),\n'
                sp_insts += f'    .in_data(sp_{i-1}_out),\n'
                sp_insts += f'    .out_data(sp_{i}_out),\n'
                sp_insts += f'    .in_start(sp_out_start[{i-1}]),\n'
                sp_insts += f'    .out_start(sp_out_start[{i}]),\n'
                sp_insts += f'    .twiddle_rd_addr_valid(sp_{i}_tw_rd_valid),\n'
                sp_insts += f'    .twiddle_rd_addr(sp_{i}_tw_rd_addr),\n'
                sp_insts += f'    .twiddle_rd_data(sp_{i}_tw_rd_data));\n'
    else:
        sp_insts += f'  super_pipe_{tp_idx}_0 sp_inst_0 (\n'
        sp_insts += f'    .clk(clk),\n'
        sp_insts += f'    .rst(rst),\n'
        sp_insts += f'    .in_data(first_sp_in),\n'
        sp_insts += f'    .out_data(last_sp_out),\n'
        sp_insts += f'    .in_start(in_start),\n'
        sp_insts += f'    .out_start(sp_out_start[0]),\n'
        sp_insts += f'    .twiddle_rd_addr_valid(sp_0_tw_rd_valid),\n'
        sp_insts += f'    .twiddle_rd_addr(sp_0_tw_rd_addr),\n'
        sp_insts += f'    .twiddle_rd_data(sp_0_tw_rd_data));\n'

    tw_stmt = '\n'
    for i in range(0, pp):
        tw_stmt += f'      if (pp_counter == {i}) begin\n'
        tw_stmt += f'        sp_{i}_tw_rd_data[tw_counter] <= twiddle_rd_data;\n'
        tw_stmt += f'        twiddle_rd_addr_valid <= sp_{i}_tw_rd_valid[tw_counter];\n'
        tw_stmt += f'        twiddle_rd_addr <= sp_{i}_tw_rd_addr[tw_counter];\n'
        tw_stmt += '      end\n'

    task_sv = f"""

module task_{tp_idx} (
    in_data,
    out_data,
    in_start,
    out_start,
    twiddle_rd_addr_valid,
    twiddle_rd_addr,
    twiddle_rd_data,
    clk,
    rst
  );

  localparam DATA_WIDTH = {ntt_config.io_width};
  localparam DP = {ntt_config.dp[d_idx]};
  localparam DP_2 = {ntt_config.dp[d_idx] // 2};
  localparam DP_WIDTH = $clog2({dp});
  localparam DP_2_WIDTH = $clog2(DP_2);
  localparam PP_WIDTH = $clog2({pp});

  input clk, rst;
  
  input in_start;
  output logic out_start;

  input [DATA_WIDTH-1:0] in_data;
  output logic [DATA_WIDTH-1:0] out_data;

  input         [DATA_WIDTH-1:0] twiddle_rd_data;
  output logic  twiddle_rd_addr_valid;
  output logic  [{num_tf_per_core.bit_length() - 2}:0] twiddle_rd_addr;

  logic [DP_WIDTH-1:0] io_counter;
  logic [DP_2_WIDTH-1:0] tw_counter;
  logic [PP_WIDTH-1:0] pp_counter;

  logic [DATA_WIDTH-1:0] first_sp_in[DP-1:0];
  logic [DATA_WIDTH-1:0] last_sp_out[DP-1:0];

  {wire_declars}

  assign out_start = sp_out_start[{pp}-1];

  {sp_insts}

  always_ff @ (posedge clk) begin
    if (rst) begin
      io_counter <= 0;
      tw_counter <= 0;
      pp_counter <= 0;
    end else begin
      io_counter <= io_counter + 1;
      tw_counter <= tw_counter + 1;
      pp_counter <= pp_counter + 1;

      if (pp_counter == {pp}) begin
        pp_counter <= 0;
      end

      first_sp_in[io_counter] <= in_data;
      out_data <= last_sp_out[io_counter];

      {tw_stmt}
    end
  end
 
endmodule

"""
    with open(out_file, 'a+') as fid:
        fid.write(task_sv)


def gen_ntt_super_pipeline(ntt_config, d_idx, tp_idx, pp_idx):
    """Generate NTT pipeline that contains one spn and multiple NTT cores."""
    out_folder = ntt_config.out_folder + '/design_' + str(d_idx)
    out_file = out_folder + '/super_pipe_' + str(tp_idx) + '_' + str(pp_idx) + '.sv'

    num_tf_per_core = ntt_config.N // ntt_config.pp[d_idx] // ntt_config.dp[d_idx]

    io_width = ntt_config.io_width
    dp = ntt_config.dp[d_idx]

    wire_declrs = ''
    wire_declrs += f'  logic [{io_width}-1:0] spn_out[DP-1:0];\n'
    wire_declrs += f'  logic spn_out_start;\n'

    spn_insts = ''
    spn_insts += f'  spn_{tp_idx}_{pp_idx} spn_inst (\n'
    for i in range(0, dp):
        spn_insts += f'    .inData_{i}(in_data[{i}]),\n'
        spn_insts += f'    .outData_{i}(spn_out[{i}]),\n'
    spn_insts += '    .clk(clk),\n'
    spn_insts += '    .rst(rst),\n'
    spn_insts += '    .in_start(in_start),\n'
    spn_insts += '    .out_start(spn_out_start));\n'

    core_insts = ''
    for i in range(0, dp // 2):
        core_insts += f'  ntt_core_tp_{tp_idx} ntt_core_inst_{i} (\n'
        core_insts += f'    .in_data_valid(spn_out_start),\n'
        core_insts += f'    .in_data_0(spn_out[{i*2+0}]),\n'
        core_insts += f'    .in_data_1(spn_out[{i*2+1}]),\n'
        core_insts += f'    .out_data_0(out_data[{i*2+0}]),\n'
        core_insts += f'    .out_data_1(out_data[{i*2+1}]),\n'
        core_insts += f'    .twiddle_rd_addr_valid(twiddle_rd_addr_valid[{i}]),\n'
        core_insts += f'    .twiddle_rd_addr(twiddle_rd_addr[{i}]),\n'
        core_insts += f'    .twiddle_rd_data(twiddle_rd_data[{i}]),\n'
        core_insts += f'    .clk(clk),\n'
        core_insts += f'    .rst(rst));\n'

    super_pipe_sv = f"""

module super_pipe_{tp_idx}_{pp_idx} (
    in_data,
    out_data,
    in_start,
    out_start,
    twiddle_rd_addr_valid,
    twiddle_rd_addr,
    twiddle_rd_data,
    clk,
    rst
  );

  localparam DATA_WIDTH = {ntt_config.io_width};
  localparam DP = {ntt_config.dp[d_idx]};
  localparam DP_2 = {ntt_config.dp[d_idx] // 2};
  localparam DP_WIDTH = $clog2({dp});

  input in_start, clk, rst;
  output logic out_start;

  input [DATA_WIDTH-1:0] in_data[DP-1:0];
  output [DATA_WIDTH-1:0] out_data[DP-1:0];

  input         [DATA_WIDTH-1:0] twiddle_rd_data[DP_2-1:0];
  output logic  twiddle_rd_addr_valid[DP_2-1:0];
  output logic  [{num_tf_per_core.bit_length() - 2}:0] twiddle_rd_addr[DP_2-1:0];

{wire_declrs}

{spn_insts}

{core_insts}

endmodule

"""
    with open(out_file, 'a+') as fid:
        fid.write(super_pipe_sv)



def gen_ntt_top(ntt_config, d_idx):
    """Generate NTT top."""
    out_folder = ntt_config.out_folder + '/design_' + str(d_idx)

    num_tf_per_core = ntt_config.N // ntt_config.pp[d_idx] // ntt_config.dp[d_idx]

    io_width = ntt_config.io_width
    dp = ntt_config.dp[d_idx]
    tp = ntt_config.tp

    wire_declars = '\n'
    for i in range(0, tp):
        wire_declars += f'  logic [DATA_WIDTH-1:0] tp_{i}_in;\n'
        wire_declars += f'  logic [DATA_WIDTH-1:0] tp_{i}_out;\n'

    for i in range(0, tp):
        wire_declars += f'  logic [DATA_WIDTH-1:0] tp_{i}_tw_rd_data;\n'
        wire_declars += f'  logic [{num_tf_per_core.bit_length() - 2}:0] tp_{i}_tw_rd_addr;\n'
        wire_declars += f'  logic tp_{i}_tw_rd_valid;\n'

    tp_insts = '\n'
    for i in range(0, tp):
        tp_insts += f'  task_{i} task_{i}_inst (\n'
        tp_insts += f'    .clk(clk),\n'
        tp_insts += f'    .rst(rst),\n'
        tp_insts += f'    .in_data(tp_{i}_in),\n'
        tp_insts += f'    .out_data(tp_{i}_out),\n'
        tp_insts += f'    .in_start(in_start[{i}]),\n'
        tp_insts += f'    .out_start(out_start[{i}]),\n'
        tp_insts += f'    .twiddle_rd_addr_valid(tp_{i}_tw_rd_valid),\n'
        tp_insts += f'    .twiddle_rd_addr(tp_{i}_tw_rd_addr),\n'
        tp_insts += f'    .twiddle_rd_data(tp_{i}_tw_data));\n'

    seq_stmt = '\n'
    for i in range(0, tp):
        seq_stmt += f'      if (tp_counter == {i}) begin\n'
        seq_stmt += f'        tp_{i}_tw_rd_data <= twiddle_rd_data;\n'
        seq_stmt += f'        twiddle_rd_addr_valid <= tp_{i}_tw_rd_valid;\n'
        seq_stmt += f'        twiddle_rd_addr <= tp_{i}_tw_rd_addr;\n'
        seq_stmt += f'        tp_{i}_in <= in_data;\n'
        seq_stmt += f'        out_data <= tp_{i}_out;\n'
        seq_stmt += '      end\n'

    # TODO: Add tap-in logic
    top_sv = f"""

module ntt_top(
    in_data,
    out_data,
    in_start,
    out_start,
    twiddle_rd_addr_valid,
    twiddle_rd_addr,
    twiddle_rd_data,
    clk,
    rst
  );

  localparam DATA_WIDTH = {ntt_config.io_width};
  localparam DP = {ntt_config.dp[d_idx]};
  localparam DP_2 = {ntt_config.dp[d_idx] // 2};
  localparam DP_WIDTH = $clog2({dp});
  localparam DP_2_WIDTH = $clog2(DP_2);
  localparam TP_WIDTH = $clog2({tp});

  input clk, rst;
  
  input in_start[{tp}-1:0];
  output logic out_start[{tp}-1:0];

  input [DATA_WIDTH-1:0] in_data;
  output logic [DATA_WIDTH-1:0] out_data;

  input         [DATA_WIDTH-1:0] twiddle_rd_data;
  output logic  twiddle_rd_addr_valid;
  output logic  [{num_tf_per_core.bit_length() - 2}:0] twiddle_rd_addr;

  logic [TP_WIDTH-1:0] tp_counter;

  {wire_declars}

  {tp_insts}

  always_ff @ (posedge clk) begin
    if (rst) begin
      tp_counter <= 0;
    end else begin
      tp_counter <= tp_counter + 1;

      if (tp_counter == {tp}) begin
        tp_counter <= 0;
      end

      {seq_stmt}

    end
  end
 
endmodule
"""
    with open(out_folder + '/ntt_top.sv', 'a+') as fid:
        fid.write(top_sv)


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
        self.limit_bram = 2160 * 0.2
        self.limit_ff = 2364480
        self.limit_dsp = 6840 * 0.2
        self.bw_gbps = 70
        self.freq_mhz = 250
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
            gen_ntt_core.gen_ntt_core('general_purpose', ntt_config, i, j)
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
                gen_ntt_super_pipeline(ntt_config, i, j, k)
            gen_tp_top(ntt_config, i, j)
        gen_ntt_top(ntt_config, i)


if __name__ == '__main__':
    main()
