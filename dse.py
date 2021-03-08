import math

def dse(ntt_config):
    cycle_ns = 1 / ntt_config.freq_mhz * 1e3
    latency_cycles = ntt_config.latency * 1e3 // cycle_ns

    print('Latency target: %d cycles.' % latency_cycles)

    # EDIT EDIT EDIT
    # const
    special_core_latency = 6
    general_core_latency = 10
    if ntt_config.io_width == 32:
        dsp_per_special_core = 4
        lut_per_special_core = 0
        ff_per_special_core = 0
        dsp_per_general_core = 8
        lut_per_general_core = 0
        ff_per_general_core = 0
    elif ntt_config.io_width == 52:
        dsp_per_special_core = 9
        lut_per_special_core = 0
        ff_per_special_core = 0
        dsp_per_general_core = 18
        lut_per_general_core = 0
        ff_per_general_core = 0
    elif ntt_config.io_width == 60:
        dsp_per_special_core = 16
        lut_per_special_core = 0
        ff_per_special_core = 0
        dsp_per_general_core = 22
        lut_per_general_core = 0
        ff_per_general_core = 0
    #######################################

    ntt_config.tp = len(ntt_config.moduli)
    print('setting tp to... ', ntt_config.tp)
    if ntt_config.tp > 5:
        ntt_config.ntt_core_type = 'general'
    else:
        ntt_config.ntt_core_type = 'specialized'

    dsp_per_core = dsp_per_special_core if ntt_config.ntt_core_type == 'specialized' else dsp_per_general_core
    lut_per_core = lut_per_special_core if ntt_config.ntt_core_type == 'specialized' else lut_per_general_core
    ff_per_core = ff_per_special_core if ntt_config.ntt_core_type == 'specialized' else ff_per_general_core
    core_lat = special_core_latency if ntt_config.ntt_core_type == 'specialized' else general_core_latency

    bytes_coefficient = math.ceil(ntt_config.io_width / 8)

    for dp in [2, 4, 8, 16, 32, 64, 128]:
        print('\nexploring dp %d' %dp)
        # EDIT EDIT EDIT
        if ntt_config.io_width == 28:
            lut_per_spn = 0
            ff_per_spn = 0
        elif ntt_config.io_width == 52:
            lut_per_spn = 0
            ff_per_spn = 0
        elif ntt_config.io_width == 60:
            lut_per_spn = 0
            ff_per_spn = 0
        #######################################

        num_cores_super_pipe = dp // 2
        dsp_super_pipe = dsp_per_core * num_cores_super_pipe

        max_pp_dsp = ntt_config.limit_dsp // dsp_super_pipe // ntt_config.tp
        print('max pp with dsp limitor... ', max_pp_dsp)

        tf_mem_size = ntt_config.N * bytes_coefficient * 8
        tf_brams = math.ceil(tf_mem_size / 32000)
        spn_mem_size = ntt_config.N * bytes_coefficient * 8
        spn_brams = math.ceil(spn_mem_size / 32000)
        bram_super_pipe = spn_brams

        max_pp_bram = (ntt_config.limit_bram - tf_brams *
                       ntt_config.tp) // spn_brams // ntt_config.tp
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
        #print(spn_lat)

        passes_lat = []
        for i in range(0, num_passes):
            lat = 0
            for j in range(0, pp):
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
