import math

def dse(ntt_config):
    cycle_ns = 1 / ntt_config.freq_mhz * 1e3
    latency_cycles = ntt_config.latency * 1e3 // cycle_ns

    # EDIT EDIT EDIT
    # set address tree depth to 3 --> 8 input operands
    adder_tree_depth = 3
    ntt_config.tp = len(ntt_config.moduli)
    print('setting tp to... ', ntt_config.tp)
    if ntt_config.tp > 2:
        ntt_config.ntt_core_type = 'general'
    else:
        ntt_config.ntt_core_type = 'specialized'

    ntt_config.ntt_core_type = 'specialized'

    print('Latency target: %d cycles.' % latency_cycles)

    # EDIT EDIT EDIT
    # const
    if ntt_config.io_width <= 32:
        special_core_latency = 4 + adder_tree_depth + 2
        general_core_latency = 14
        dsp_per_special_core = 4
        dsp_per_general_core = 12
    elif ntt_config.io_width <= 52:
        special_core_latency = 6 + adder_tree_depth + 2
        general_core_latency =  19
        dsp_per_special_core = 6
        dsp_per_general_core = 17

    dsp_per_core = dsp_per_special_core if ntt_config.ntt_core_type == 'specialized' else dsp_per_general_core
    core_lat = special_core_latency if ntt_config.ntt_core_type == 'specialized' else general_core_latency

    twiddle_factors = []
    for i in range(1, ntt_config.ntt_stages+1):
        twiddle_factors.append(ntt_config.N // (2 ** i))

    for dp in [2, 4, 8, 16, 32, 64, 128]:
        print('\nexploring dp %d' %dp)

        num_cores_super_pipe = dp // 2
        dsp_super_pipe = dsp_per_core * num_cores_super_pipe

        max_pp_dsp = ntt_config.limit_dsp // dsp_super_pipe // ntt_config.tp
        print('max pp with dsp limitor... ', max_pp_dsp)

        spn_brams = 0.5 * math.ceil(2.0 * ntt_config.N * ntt_config.io_width / dp / 32000) * dp
        max_pp_bram = 0

        for pp in range(1, ntt_config.ntt_stages+1):

            twiddle_factors_pp = [0] * pp
            for i in range(0, ntt_config.ntt_stages):
                twiddle_factors_pp[i%pp] += twiddle_factors[i]
            #print(twiddle_factors_pp)

            for i in range(0, len(twiddle_factors_pp)):
                twiddle_factors_pp[i] = math.ceil(twiddle_factors_pp[i] * ntt_config.io_width / 32000)
            #print(twiddle_factors_pp)

            tf_prime = sum(twiddle_factors_pp) * ntt_config.tp

            #tf = 0.5 * math.ceil(2.0 * ntt_config.N * ntt_config.io_width / 32000) * ntt_config.tp
            tf = 0.5 * math.ceil(2.0 * ntt_config.N * ntt_config.io_width / pp / 32000) * pp * ntt_config.tp
            #tf_per_core = math.ceil(ntt_config.ntt_stages  / pp) * (ntt_config.N / dp)
            #bram_per_core = 0.5 * math.ceil(2.0 * tf_per_core * ntt_config.io_width / 32000)
            #tf = bram_per_core * dp // 2 * pp * ntt_config.tp
            spn = spn_brams * pp * ntt_config.tp
            print('pp', pp, 'tf brams', tf, 'spn brams', spn, 'tf_prime', tf_prime)
            bram_needed = tf + spn
            if bram_needed < ntt_config.limit_bram:
                max_pp_bram = pp
        print('max pp with bram limitor... ', max_pp_bram)

        pp = min(max_pp_dsp, max_pp_bram)
        pp = min(pp, ntt_config.ntt_stages)
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
            print('pass %d lat %d final %d' %(i, lat, passes_lat[-1]))

        total_lat = sum(passes_lat) + beats
        print('projected latency.. ', total_lat)

        if total_lat < latency_cycles:
            print('find a valid design point. tp %d dp %d pp %d' %
                  (ntt_config.tp, dp, pp))
            ntt_config.dp.append(dp)
            ntt_config.pp.append(pp)

    return ntt_config
