import math


def dse(ntt_config, core_type='general'):
    cycle_ns = 1 / ntt_config.freq_mhz * 1e3
    latency_cycles = ntt_config.latency * 1e3 // cycle_ns

    # EDIT EDIT EDIT
    # set address tree depth to 3 --> 8 input operands
    adder_tree_depth = 3
    ntt_config.tp = len(ntt_config.moduli)
    ntt_config.ntt_core_type = core_type

    print('latency target: %d cycles, NTT Core type: %s.' %
          (latency_cycles, core_type))

    # EDIT EDIT EDIT
    # const
    if ntt_config.io_width <= 32:
        special_core_latency = 4 + adder_tree_depth + 2
        general_core_latency = 14
        dsp_per_special_core = 4
        dsp_per_general_core = 12
        spn_factor = 1
    elif ntt_config.io_width <= 52:
        special_core_latency = 6 + adder_tree_depth + 2
        general_core_latency = 19
        dsp_per_special_core = 6
        dsp_per_general_core = 17
        spn_factor = 2

    dsp_per_core = (dsp_per_special_core if ntt_config.ntt_core_type ==
                    'specialized' else dsp_per_general_core)
    core_lat = (special_core_latency if ntt_config.ntt_core_type ==
                'specialized' else general_core_latency)

    twiddle_factors = []
    for i in range(1, ntt_config.ntt_stages+1):
        twiddle_factors.append(ntt_config.N // (2 ** i))

    # account for both input and output polynomial
    total_bytes = 2 * ntt_config.tp * ntt_config.N * ntt_config.io_width / 8

    for dp in [2, 4, 8, 16, 32, 64, 128]:
        print('\nexploring dp %d' % dp)
        max_pp_dsp = 0
        max_pp_bram = 0

        num_cores_super_pipe = dp // 2
        dsp_super_pipe = dsp_per_core * num_cores_super_pipe

        max_pp_dsp = ntt_config.limit_dsp // dsp_super_pipe // ntt_config.tp
        max_pp_dsp = min(max_pp_dsp, ntt_config.ntt_stages)
        print('max pp with dsp limitor... ', max_pp_dsp)

        # 1024 comes from: BRAM is configured in 1K * 32-bit mode
        spn_brams = 0.5 * \
            math.ceil(2.0 * ntt_config.N / dp / 1024) * spn_factor * dp

        for pp in range(1, ntt_config.ntt_stages+1):

            twiddle_factors_pp = [0] * pp
            for i in range(0, ntt_config.ntt_stages):
                twiddle_factors_pp[i % pp] += twiddle_factors[i]
            # print(twiddle_factors_pp)

            super_pipe_bram = []
            for i in range(0, pp):
                unique_factors = min(dp//2, twiddle_factors[i])
                num_rows = math.ceil(twiddle_factors_pp[i] / unique_factors)
                super_pipe_bram.append(
                    math.ceil(num_rows / 1024) * unique_factors * spn_factor)
            #print('tf per pipe: ', twiddle_factors)
            #print('bram per pipe: ', super_pipe_bram)
            tf = sum(super_pipe_bram) * ntt_config.tp

            spn = spn_brams * pp * ntt_config.tp
            print('pp', pp, 'tf brams', tf, 'spn brams', spn)

            bram_needed = tf + spn
            if bram_needed < ntt_config.limit_bram:
                max_pp_bram = pp
        print('max pp with bram limitor... ', max_pp_bram)

        pp = min(max_pp_dsp, max_pp_bram)
        assert(pp <= ntt_config.ntt_stages)
        print('pp... ', pp)

        if pp == 0:
            # not a valid design point
            continue

        num_passes = math.ceil(ntt_config.ntt_stages / pp)
        beats = ntt_config.N // dp

        spn_lrb = 2 * int(math.log2(dp)) + 4
        spn_lat = {}
        for i in range(0, 17):
            if 2**i >= dp:
                spn_lat[2**i] = 2**i // dp + spn_lrb
            else:
                spn_lat[2**i] = spn_lrb
        print(spn_lat)

        passes_lat = []
        for i in range(0, num_passes):
            lat = 0
            for j in range(0, int(pp)):
                stride = ntt_config.N // 2 // 2**(i * pp + j)
                if stride > 0:
                    lat += spn_lat[stride] + core_lat
            if lat > beats or i == num_passes - 1:
                passes_lat.append(lat)
            else:
                passes_lat.append(beats)
            print('pass %d lat %d final %d' % (i, lat, passes_lat[-1]))

        total_lat = sum(passes_lat) + beats
        print('bytes', total_bytes)
        total_bw = total_bytes / (total_lat / ntt_config.freq_mhz) / 1e3

        print('projected latency [cycles] ', total_lat,
              'projected bandwidth [GB/s] ', total_bw)

        if total_lat < latency_cycles and total_bw < ntt_config.bw_gbps:
            print('find a valid design point. tp %d dp %d pp %d' %
                  (ntt_config.tp, dp, pp))
            ntt_config.dp.append(dp)
            ntt_config.pp.append(pp)

    return ntt_config
