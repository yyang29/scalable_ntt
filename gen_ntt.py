from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

import argparse


def gen_switch(out_file, N, dp, pp, io_width):
    """Generate 2x2 switch to form spn stage."""
    switch_sv = f"""// This verilog file is generated automatically.
// Author: Yang Yang (yyang172@usc.edu)

module switch
endmodule
"""
    with open(out_file, 'a+') as fid:
        fid.write(switch_sv)


def gen_spn_stage():
    """Generate each spn stage."""
    pass


def gen_spn_top():
    """Generate entire spn with multiple stages."""
    pass


def gen_ntt_core():
    """Generate NTT core."""
    pass


def gen_ntt_stage():
    """Generate NTT stage that contains one spn and multiple NTT cores."""
    pass


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

    gen_switch(args.output_file, args.N, args.data_parallelism, args.pipeline_parallelism, args.io_width)
    gen_spn_stage()
    gen_spn_top()
    gen_ntt_core()
    gen_ntt_stage()
    gen_ntt_top()
    gen_ntt_top_wrapper()


if __name__ == '__main__':
    main()
