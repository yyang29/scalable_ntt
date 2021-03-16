# Generate SPN

import math
import random

import gressGen
import memGen


def gen_common_modules(ntt_config, d_idx):
    out_folder = ntt_config.out_folder + '/design_' + str(d_idx)
    genCounter(out_folder + '/counter.sv',
               int(2*ntt_config.N/ntt_config.dp[d_idx]))
    gen_switch_2_2(ntt_config, d_idx)
    gen_sr(ntt_config, d_idx)
    genDataSPRam(out_folder + '/mem.sv', ntt_config.dp[d_idx],
                 ntt_config.N, ntt_config.io_width, 'block_ram_sp')
    genDataSPRam(out_folder + '/mem.sv', ntt_config.dp[d_idx],
                 ntt_config.N, ntt_config.io_width, 'dist_ram_sp')
    genDataDPRam(out_folder + '/mem.sv', ntt_config.dp[d_idx],
                 ntt_config.N, ntt_config.io_width, 'block_ram_dp')
    genDataDPRam(out_folder + '/mem.sv', ntt_config.dp[d_idx],
                 ntt_config.N, ntt_config.io_width, 'dist_ram_dp')


def getCtrl(ntt_config, d_idx, strides):
    numIntStage = ntt_config.spn_stages_per_spatial_perm
    numSwitchPerStage = ntt_config.spn_switches_per_stage
    numCtrlBitsPerSwitch = int(ntt_config.N // ntt_config.dp[d_idx]) * len(strides)
    numMem = ntt_config.dp[d_idx]

    # control bit vectors for ingress stage
    ingressStageBits = [[[random.randint(0, 1) for k in range(numCtrlBitsPerSwitch)]
                         for j in range(numSwitchPerStage(d_idx))]
                        for i in range(numIntStage(d_idx))]
    # control bit vectors for egress stage
    egressStageBits = [[[random.randint(0, 1) for k in range(numCtrlBitsPerSwitch)]
                        for j in range(numSwitchPerStage(d_idx))]
                       for i in range(numIntStage(d_idx))]
    memAddr = [[] for k in range(0, numMem)]

    for i in range(numMem):
        memAddr[i] = [random.randint(0, int(ntt_config.N // ntt_config.dp[d_idx]) - 1)
                for k in range(int(ntt_config.N // ntt_config.dp[d_idx]) * len(strides))]

    return ingressStageBits, egressStageBits, memAddr


def gen_spn(ntt_config, d_idx, tp_idx, pp_idx, routing):

    print('generating design %d tp %d pp %d SPN, io_width %d, dp %d...'
          % (d_idx, tp_idx, pp_idx, ntt_config.io_width, ntt_config.dp[d_idx]))

    out_folder = ntt_config.out_folder + '/design_' + str(d_idx)
    filename = out_folder + '/spn_' + str(tp_idx) + '_' + str(pp_idx) + '.sv'

    ingressStageBits, egressStageBits, memAddr = getCtrl(ntt_config, d_idx, routing)

    # Connect wireIn with LB, RB, MemStage
    regOutSwitch, regOutWireCon = [], []
    # Determine if register one switch stage or one wire stage
    for i in range(0, int(math.log(ntt_config.dp[d_idx], 2))):
        a, b = 0, 0
        # Currently, to simplify switch control, only one pipeline stage enabled
        if(i == int(math.log(ntt_config.dp[d_idx], 2))-1):
            regOutSwitch.append(a)
        else:
            regOutSwitch.append(b)
        regOutWireCon.append(b)

    module_prefix = 'spn_' + str(tp_idx) + '_' + str(pp_idx)

    nameLB = gressGen.genStagesBlock(filename, ntt_config.dp[d_idx], routing[0], ntt_config.N / ntt_config.dp[d_idx],
                                     'L', ntt_config.io_width, ingressStageBits, regOutSwitch, regOutWireCon,
                                     False, 0, module_prefix)

    nameRB = gressGen.genStagesBlock(filename, ntt_config.dp[d_idx], routing[0], ntt_config.N / ntt_config.dp[d_idx],
                                     'R', ntt_config.io_width, egressStageBits, regOutSwitch[::-1],
                                     regOutWireCon, False, 0, module_prefix)

    nameMemStage = memGen.genMemStage(filename, ntt_config.dp[d_idx], ntt_config.N, routing[0], ntt_config.io_width,
                                      memAddr, False, 0, module_prefix)

    # Gen permutation module
    with open(filename, 'a+') as wrFile:
        wrFile.write('\n')
        wrFile.write('module {}'.format(module_prefix))
        wrFile.write('(\n')

    genMultiPortName(filename, ntt_config.dp[d_idx], 'inData', False)
    with open(filename, 'a') as wrFile:
        wrFile.write(',\n')
    genMultiPortName(filename, ntt_config.dp[d_idx], 'outData', False)
    with open(filename, 'a') as wrFile:
        wrFile.write(',\n')

    with open(filename, 'a') as wrFile:
        wrFile.write('in_start,                        \n')
        wrFile.write('out_start,                       \n')
        wrFile.write('clk,                             \n')
        wrFile.write('rst                              \n')
        wrFile.write(');                               \n  ')
        wrFile.write('parameter DATA_WIDTH = ')
        wrFile.write(str(ntt_config.io_width))
        wrFile.write(';                                \n  ')
        wrFile.write('input in_start, clk, rst;        \n  ')
        wrFile.write('input [DATA_WIDTH-1:0] ')

    genMultiPortName(filename, ntt_config.dp[d_idx], 'inData', True)
    with open(filename, 'a') as wrFile:
        wrFile.write(';\n  ')

    with open(filename, 'a') as wrFile:
        wrFile.write('output [DATA_WIDTH-1:0] ')

    genMultiPortName(filename, ntt_config.dp[d_idx], 'outData', True)

    with open(filename, 'a') as wrFile:
        wrFile.write('; \n  ')
        wrFile.write('output out_start; ')

    with open(filename, 'a') as wrFile:
        wrFile.write('\n  ')

    nameLB = module_prefix + '_ingressStage_p' + str(ntt_config.dp[d_idx])
    nameRB = module_prefix + '_egressStage_p' + str(ntt_config.dp[d_idx])
    nameMemStage = module_prefix + '_mem_stage_dp' + str(ntt_config.dp[d_idx])
    counter_width = int(math.log(ntt_config.N / ntt_config.dp[d_idx], 2.0))

    with open(filename, 'a') as wrFile:
        wrFile.write('\n  ')
        wrFile.write('wire [DATA_WIDTH-1:0] wireIn [' +
                     str(ntt_config.dp[d_idx]-1)+':0];                  \n  ')
        wrFile.write('wire [DATA_WIDTH-1:0] wireOut [' +
                     str(ntt_config.dp[d_idx]-1)+':0];                 \n  ')
        wrFile.write('wire [DATA_WIDTH-1:0] wireOut_LB [' +
                     str(ntt_config.dp[d_idx]-1)+':0];              \n  ')
        wrFile.write('wire [DATA_WIDTH-1:0] wireIn_RB [' +
                     str(ntt_config.dp[d_idx]-1)+':0];               \n  ')
        wrFile.write('wire out_start_LB;               \n  ')
        wrFile.write('wire out_start_MemStage;               \n  ')
        wrFile.write('wire out_start_RB;               \n\n  ')
        wrFile.write('wire ['+str(counter_width-1) +
                     ':0] counter_out_w;               \n  ')

        # Connect wireIn with inData
        for i in range(0, ntt_config.dp[d_idx]):
            wrFile.write(
                'assign wireIn['+str(i)+'] = inData_'+str(i)+';    \n  ')
        wrFile.write('\n  ')

        # counter for control
        wrFile.write('counter_'+str(2 * ntt_config.N // ntt_config.dp[d_idx]))
        wrFile.write(' ctrl_unit'+'(')
        wrFile.write('.in_start(in_start),')
        wrFile.write(
            ' .counter_out(counter_out_w), .clk(clk), .rst(rst));\n\n  ')

        # Connect wireIn with ingress Stage
        wrFile.write(nameLB+' ' + nameLB+'_inst(')
        for j in range(0, ntt_config.dp[d_idx]):
            wrFile.write('.inData_'+str(j)+'(wireIn['+str(j)+']), ')
        for j in range(0, ntt_config.dp[d_idx]):
            wrFile.write('.outData_'+str(j)+'(wireOut_LB['+str(j)+']), ')
        wrFile.write('.in_start(in_start), ')
        wrFile.write('.out_start(out_start_LB), ')
        wrFile.write('.counter_in(counter_out_w), ')
        wrFile.write('.clk(clk), .rst(rst));\n  \n  ')

        # Connect ingress Stage with memory stage
        wrFile.write(nameMemStage+' ' + nameMemStage+'_inst(')
        for j in range(0, ntt_config.dp[d_idx]):
            wrFile.write('.inData_'+str(j)+'(wireOut_LB['+str(j)+']), ')
        for j in range(0, ntt_config.dp[d_idx]):
            wrFile.write('.outData_'+str(j)+'(wireIn_RB['+str(j)+']), ')
        wrFile.write('.in_start(out_start_LB), ')
        wrFile.write('.out_start(out_start_MemStage), ')
        wrFile.write('.clk(clk), \n ')
        wrFile.write('.counter_in(counter_out_w), ')
        wrFile.write('.rst(rst));\n  \n  ')

        # Connect memory stage with egress stage
        wrFile.write(nameRB+' ' + nameRB+'_inst(')
        for j in range(0, ntt_config.dp[d_idx]):
            wrFile.write('.inData_'+str(j)+'(wireIn_RB['+str(j)+']), ')
        for j in range(0, ntt_config.dp[d_idx]):
            wrFile.write('.outData_'+str(j)+'(wireOut['+str(j)+']), ')
        wrFile.write('.in_start(out_start_MemStage), ')
        wrFile.write('.out_start(out_start_RB), ')
        wrFile.write('.counter_in(counter_out_w), ')
        wrFile.write('.clk(clk), .rst(rst));\n  \n  ')

    with open(filename, 'a') as wrFile:
        wrFile.write('\n  ')
        # Connect wireOut with OutData
        for i in range(0, ntt_config.dp[d_idx]):
            wrFile.write('assign outData_'+str(i) +
                         ' = wireOut['+str(i)+'];    \n  ')
        wrFile.write('assign out_start = out_start_RB;    \n  ')

        wrFile.write('\n')
        wrFile.write('endmodule                        \n\n')


def genMultiPortName(out_file, numPorts, namePre, addTabs):
    # Generate multi port names
    with open(out_file, 'a') as wrFile:
        for i in range(0, numPorts):
            wrFile.write(namePre)
            wrFile.write('_')
            wrFile.write(str(i))
            if(i == numPorts - 1):
                wrFile.write('')
            else:
                wrFile.write(',\n')
                if(addTabs):
                    wrFile.write('      ')

# Generate memory blocks in the middle stage
# SP: single port RAM


def genDataSPRam(fileName, dp, sizeN, dataWidth, ramStyle):
    memSize = sizeN / dp
    addrWidth = int(math.ceil(math.log(memSize, 2.0)))

    # Gen connection module
    with open(fileName, 'a') as wrFile:
        wrFile.write('\n')
        wrFile.write('module  '+ramStyle)
        wrFile.write('(\n')
        wrFile.write('wen,                              \n')
        if(ramStyle == 'block_ram_sp'):
            wrFile.write('en,                              \n')
        wrFile.write('clk,                             \n')
        wrFile.write('addr,                            \n')
        wrFile.write('din,                            \n')
        wrFile.write('dout                             \n')
        wrFile.write(');                               \n  ')
        wrFile.write('parameter DATA_WIDTH = ')
        wrFile.write(str(dataWidth))
        wrFile.write(';                                \n  ')
        wrFile.write('parameter ADDR_WIDTH = ')
        wrFile.write(str(addrWidth))
        wrFile.write(';                                \n  ')
        wrFile.write('parameter RAM_SIZE = 1 << ADDR_WIDTH')
        wrFile.write(';                                \n  ')
        wrFile.write('input wen, clk;                   \n  ')
        if(ramStyle == 'block_ram_sp'):
            wrFile.write('input en;                              \n  ')
        wrFile.write(
            'input [ADDR_WIDTH-1:0] addr;                        \n  ')
        wrFile.write('input [DATA_WIDTH-1:0] din;                        \n  ')
        wrFile.write('output ')
        if(ramStyle == 'block_ram_sp'):
            wrFile.write('reg ')
        wrFile.write('[DATA_WIDTH-1:0] dout;        \n  ')

    # Wires
    with open(fileName, 'a') as wrFile:
        wrFile.write('\n  ')
        wrFile.write('reg [DATA_WIDTH-1:0] ram[RAM_SIZE-1:0];        \n  ')
        wrFile.write('\n  ')
        # Update ctrl_out cycle by cycle
        wrFile.write('always@(posedge clk)             \n  ')
        wrFile.write('begin                            \n    ')
        if(ramStyle == 'block_ram_sp'):
            wrFile.write(
                "// synthesis attribute ram_style of ram is \"block\" \n  ")
            wrFile.write('if(en) begin                    \n      ')
            wrFile.write('if(wen)                         \n        ')
            wrFile.write('ram[addr] <= din ;              \n      ')
            wrFile.write('dout <= ram[addr];              \n  ')
            wrFile.write('end\n  ')
            wrFile.write('end                             \n  ')
        elif(ramStyle == 'dist_ram_sp'):
            wrFile.write(
                "// synthesis attribute ram_style of ram is \"distributed\" \n  ")
            wrFile.write('if(wen)                         \n      ')
            wrFile.write('ram[addr] <= din ;              \n  ')
            wrFile.write('end                             \n \n  ')
            wrFile.write('assign dout = ram[addr];         \n  ')

        wrFile.write('\n')
        wrFile.write('endmodule                        \n\n')


# Generate dual-port ram
def genDataDPRam(fileName, dp, sizeN, dataWidth, ramStyle):
    memSize = sizeN / dp
    addrWidth = int(math.ceil(math.log(memSize, 2.0)))

    # Gen connection module
    with open(fileName, 'a') as wrFile:
        wrFile.write('\n')
        wrFile.write('module  '+ramStyle)
        wrFile.write('(\n')
        wrFile.write('wen,                              \n')
        if(ramStyle == 'block_ram_dp'):
            wrFile.write('en,                              \n')
        wrFile.write('clk,                             \n')
        wrFile.write('addr_r,                            \n')
        wrFile.write('addr_w,                            \n')
        wrFile.write('din,                            \n')
        wrFile.write('dout                             \n')
        wrFile.write(');                               \n  ')
        wrFile.write('parameter DATA_WIDTH = ')
        wrFile.write(str(dataWidth))
        wrFile.write(';                                \n  ')
        wrFile.write('parameter ADDR_WIDTH = ')
        wrFile.write(str(addrWidth))
        wrFile.write(';                                \n  ')
        wrFile.write('parameter RAM_SIZE = 1 << ADDR_WIDTH')
        wrFile.write(';                                \n  ')
        wrFile.write('input wen, clk;                   \n  ')
        if(ramStyle == 'block_ram_dp'):
            wrFile.write('input en;                              \n  ')
        wrFile.write(
            'input [ADDR_WIDTH-1:0] addr_r;                        \n  ')
        wrFile.write(
            'input [ADDR_WIDTH-1:0] addr_w;                        \n  ')
        wrFile.write('input [DATA_WIDTH-1:0] din;                        \n  ')
        wrFile.write('output ')
        if(ramStyle == 'block_ram_dp'):
            wrFile.write('reg ')
        wrFile.write('[DATA_WIDTH-1:0] dout;        \n  ')

    # Wires
    with open(fileName, 'a') as wrFile:
        wrFile.write('\n  ')
        wrFile.write('reg [DATA_WIDTH-1:0] ram[RAM_SIZE-1:0];        \n  ')
        wrFile.write('\n  ')
        # Update ctrl_out cycle by cycle
        wrFile.write('always@(posedge clk)             \n  ')
        wrFile.write('begin                            \n    ')
        if(ramStyle == 'block_ram_dp'):
            wrFile.write(
                "// synthesis attribute ram_style of ram is \"block\" \n  ")
            wrFile.write('if(en) begin                    \n      ')
            wrFile.write('if(wen)                         \n        ')
            wrFile.write('ram[addr_w] <= din ;              \n      ')
            wrFile.write('dout <= ram[addr_r];              \n  ')
            wrFile.write('end\n  ')
            wrFile.write('end                             \n  ')
        elif(ramStyle == 'dist_ram_dp'):
            wrFile.write(
                "// synthesis attribute ram_style of ram is \"distributed\" \n  ")
            wrFile.write('if(wen)                         \n      ')
            wrFile.write('ram[addr_w] <= din ;              \n  ')
            wrFile.write('end                             \n \n  ')
            wrFile.write('assign dout = ram[addr_r];         \n  ')

        wrFile.write('\n')
        wrFile.write('endmodule                        \n\n')


def gen_sr(ntt_config, d_idx):

    filename = ntt_config.out_folder + \
        '/design_' + str(d_idx) + '/shift_register.sv'

    sr_sv = f"""// This verilog file is generated automatically.
// Author: Yang Yang (yyang172@usc.edu)

module Shift_Register #(
    NPIPE_DEPTH = 2,
    DATA_WIDTH = 32) (
    clock,
    reset,
    input_data,
    output_data);

  input                         clock;
  input                         reset;
  input logic[DATA_WIDTH-1:0]   input_data;
  output logic[DATA_WIDTH-1:0]  output_data;

  logic[DATA_WIDTH-1:0]         pipe_reg[NPIPE_DEPTH-1:0];

  integer i;

  always_ff @ (posedge clock) begin
    if (reset) begin
      output_data <= 0;
    end else begin
      output_data <= pipe_reg[NPIPE_DEPTH-1];
      pipe_reg[0] <= input_data;
      for (i = 0; i < NPIPE_DEPTH-1; i = i+1)
        pipe_reg[i+1] <= pipe_reg[i];
    end
  end

endmodule
"""
    with open(filename, 'a+') as fid:
        fid.write(sr_sv)



def gen_switch_2_2(ntt_config, d_idx):

    filename = ntt_config.out_folder + \
        '/design_' + str(d_idx) + '/switch_2x2.sv'

    switch_sv = f"""// This verilog file is generated automatically.
// Author: Yang Yang (yyang172@usc.edu)

module switch_2_2 (
    inData_0,
    inData_1,
    outData_0,
    outData_1,
    ctrl,
    clk,
    rst
  );

  input ctrl, clk, rst;
  input         [{ntt_config.io_width}-1:0] inData_0, inData_1;
  output logic  [{ntt_config.io_width}-1:0] outData_0, outData_1;

  always_ff @ (posedge clk) begin
    if (rst) begin
      outData_0 <= 0;
      outData_1 <= 0;
    end else begin
      outData_0 <= (!ctrl) ? inData_0 : inData_1;
      outData_1 <= (!ctrl) ? inData_1 : inData_0;
    end
  end

endmodule

"""
    with open(filename, 'a+') as fid:
        fid.write(switch_sv)


# Generate a counter
def genCounter(fileName, maxVal):
    dataWidth = int(math.ceil(math.log(maxVal, 2.0)))

    with open(fileName, 'a') as wrFile:
        wrFile.write('\n')
        wrFile.write('module  '+'counter_'+str(maxVal))
        wrFile.write('(\n')
        wrFile.write('in_start'+',                         \n')
        wrFile.write('counter_out,                         \n')
        wrFile.write('clk,                             \n')
        wrFile.write('rst                              \n')
        wrFile.write(');                               \n  ')
        wrFile.write('input in_start, clk, rst;                   \n  ')
        wrFile.write('output ['+str(dataWidth-1) +
                     ':0] counter_out;            \n  ')
        wrFile.write('\n  ')
        wrFile.write('reg ['+str(dataWidth-1)+':0] counter_r;        \n  ')
        wrFile.write('reg status_couting;        \n\n  ')
        wrFile.write('assign counter_out = counter_r;        \n  ')
        wrFile.write('\n  ')
        wrFile.write('always@(posedge clk)             \n  ')
        wrFile.write('begin                            \n    ')
        wrFile.write('if(rst) begin                    \n      ')
        wrFile.write('counter_r <= '+str(dataWidth)+"'b0;    \n      ")
        wrFile.write("status_couting <= 1'b0;            \n    ")
        wrFile.write('end\n    ')
        wrFile.write('else begin                        \n      ')
        # Make sure in_start has been registered
        wrFile.write("if (status_couting == 1'b1)                \n        ")
        wrFile.write(
            "counter_r <= counter_r + 1'b1;                   \n      ")
        ##addrRomWidth = x * addrRamWidth
        wrFile.write(
            'if (counter_r['+str(dataWidth-1)+':0] == '+str(maxVal-1)+') begin  \n        ')
        wrFile.write("status_couting <= 1'b0;                 \n        ")
        wrFile.write('counter_r <= '+str(dataWidth)+"'b0;         \n      ")
        wrFile.write('end                                    \n      ')
        wrFile.write('if (in_start) begin                     \n        ')
        wrFile.write("status_couting <= 1'b1;                 \n      ")
        wrFile.write('counter_r <= '+str(dataWidth) +
                     "'b0;                \n      ")
        #wrFile.write("counter_r <= "+str(dataWidth)+"'b0;         \n      ")
        wrFile.write('end                                    \n    ')
        wrFile.write('end\n  ')
        wrFile.write('end                              \n')

        wrFile.write('\n')
        wrFile.write('endmodule                        \n\n')

    return 'counter_'+str(maxVal)
