from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

import argparse
import math

from gen_ntt_spn import *


def gen_switch(out_file, N, dp, pp, io_width):
    """Generate 2x2 switch to form spn stage."""

    ntt_stages = N.bit_length() - 1
    ntt_stages_per_pipe = ntt_stages // pp

    # stride for each SPN
    perm_routing = []
    for i in range(0, pp):
        perm_routing.append([])

    stride = N
    i = 0
    while stride > 1:
        stride = stride // 2
        perm_routing[i % pp].append(stride)
        i = i + 1

    # Generate SPN and connect them together
    for i in range(0, pp):
        # i is SPN id
        gen_spn(out_file, i, perm_routing[i], io_width, dp)


def gen_ntt_core():
    """Generate NTT core."""
    pass


def gen_ntt_super_pipeline(out_file, pipe_id):
    """Generate NTT pipeline that contains one spn and multiple NTT cores."""
    pipeline_sv = f"""

module control_unit_{pipe_id}
endmodule

module super_pipeline_{pipe_id} (
endmodule

"""
    with open(out_file, 'a+') as fid:
        fid.write(pipeline_sv)


def gen_ntt_top():
    """Generate NTT top."""
    pass


def gen_ntt_top_wrapper():
    """Generate NTT top wrapper."""
    pass


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
    parser.add_argument('-o', '--output-file', type=str, default='ntt.sv',
                        help='Output verilog file name and location.')

    args = parser.parse_args()

    # Calculate high level architecture parameters

    # Total number of NTT stages of this algorithm.
    ntt_stages = args.N.bit_length() - 1

    # Number of NTT stages per pipeline.
    # Pipeline here refers to one SPN and dp/2 number of NTT cores.
    ntt_stages_per_pipe = ntt_stages // args.pipeline_parallelism

    # Number of beats for one polynomial to flow through one pipeline
    beats = args.N // args.data_parallelism

    # Number of stages of each SPN
    spn_stages = 2 * (args.data_parallelism.bit_length() - 1)

    # Number of switches per spn stage
    switches_per_spn_stage = args.data_parallelism // 2

    # Ctrl bits per switch
    ctrl_bits_per_switch = beats * ntt_stages_per_pipe

    # Total control bits in KB
    total_ctrl_bits = (ctrl_bits_per_switch * switches_per_spn_stage *
                       spn_stages * args.pipeline_parallelism) / 8 / 1024

    print('##### Architecture parameters #####')
    print('Max NTT stages: %d' % ntt_stages)
    print('Max NTT stages per pipeline: %d' % ntt_stages_per_pipe)
    print('Max beats per polynomial: %d' % beats)
    print('SPN stages: %d' % spn_stages)
    print('switches per SPN stage: %d' % switches_per_spn_stage)
    print('ctrl bits per switch [bits]: %d' % ctrl_bits_per_switch)
    print('total ctrl bits [KB]: %d' % total_ctrl_bits)

    # Add tap-out logic

    gen_switch(args.output_file, args.N, args.data_parallelism,
               args.pipeline_parallelism, args.io_width)
    # gen_spn_stage()
    # gen_spn_top()
    # gen_ntt_core()
    # for i in range(args.pipeline_parallelism):
    #    gen_ntt_pipeline(args.output_file, i)
    gen_ntt_top()
    gen_ntt_top_wrapper()


if __name__ == '__main__':
    main()
