# Streaming Permutation Network Routing Configuration Generation."

import numpy as np

class switch:

  # list of (0, 1) to indicate pass or cross of the switch for each cycle
  config_table = []

  # coordinate of the switch, top-left is (0, 0), bottom-right is (p/2-1, plogp-1)
  coord = []

def generate_dataflow(N, p, stage, isNTT=True):
  index_in = []
  index_out = []

  if isNTT == True:
    stride = N // (2**(stage+1))
    if stage == 0:
      index_in = [i for i in range(0, N)]
      index_in = np.array(index_in).reshape(-1, p)
    else:
      stride_prev = stride * 2

      f_in0 = lambda x:x
      f_in1 = lambda x:x + stride_prev
      for start_idx in range(0, N-stride_prev, stride_prev*2):
        index_in += [f(i) for i in range(start_idx, start_idx+stride_prev) for f in (f_in0, f_in1)]
      index_in = np.array(index_in).reshape(-1, p)

    f_in0 = lambda x:x
    f_in1 = lambda x:x + stride
    for start_idx in range(0, N-stride, stride*2):
      index_out += [f(i) for i in range(start_idx, start_idx+stride) for f in (f_in0, f_in1)]
    index_out = np.array(index_out).reshape(-1, p)

    print('Input dataflow:')
    print(index_in)
    print('\nOutput dataflow:')
    print(index_out)
    return index_in, index_out

  else:
    # This is INTT,
    # TODO
    pass

def main():
  generate_dataflow(64, 8, 0)

if __name__ == "__main__":
  main()
