#!/usr/bin/env python3
# -*- coding: utf-8 -*-

#
# SPDX-License-Identifier: GPL-3.0
#
# GNU Radio Python Flow Graph
# Title: Pulsar Filterbank Uhd
# GNU Radio version: 3.9.8.0

from gnuradio import analog
from gnuradio import blocks
import pmt
from gnuradio import fft
from gnuradio.fft import window
from gnuradio import filter
from gnuradio import gr
from gnuradio.filter import firdes
import sys
import signal
from argparse import ArgumentParser
from gnuradio.eng_arg import eng_float, intx
from gnuradio import eng_notation
import atexit
import math
import numpy
import osmosdr
import time
import platform
import pulsar_filterbank_uhd_Folder_Block as Folder_Block  # embedded python block
import pulsar_filterbank_uhd_fb_helper as fb_helper  # embedded python module
import threading




class pulsar_filterbank_uhd(gr.top_block):

    def __init__(self, bbgain=10, dec=54.58, dm=26.76, dscale=1, freq=1400e6, hp=1, hpgain=0.6, ifgain=10, integrator=0, p0=0.71452, pinterval=30, pps="internal", prefix="./", profile=0, pw50=6.6e-3, ra=53.25, refclock="internal", resolution=10, rfgain=45, rfilist="", rolloff=0, runtime=60*5, sky=0, source="B0329+54", srate=40.0e6, subdev="A:0", tbins=250, thresh=2.5, trial_ppms="0.0", wide=0):
        gr.top_block.__init__(self, "Pulsar Filterbank Uhd", catch_exceptions=True)

        ##################################################
        # Parameters
        ##################################################
        self.bbgain = bbgain
        self.dec = dec
        self.dm = dm
        self.dscale = dscale
        self.freq = freq
        self.hp = hp
        self.hpgain = hpgain
        self.ifgain = ifgain
        self.integrator = integrator
        self.p0 = p0
        self.pinterval = pinterval
        self.pps = pps
        self.prefix = prefix
        self.profile = profile
        self.pw50 = pw50
        self.ra = ra
        self.refclock = refclock
        self.resolution = resolution
        self.rfgain = rfgain
        self.rfilist = rfilist
        self.rolloff = rolloff
        self.runtime = runtime
        self.sky = sky
        self.source = source
        self.srate = srate
        self.subdev = subdev
        self.tbins = tbins
        self.thresh = thresh
        self.trial_ppms = trial_ppms
        self.wide = wide

        ##################################################
        # Variables
        ##################################################
        self.samp_rate = samp_rate = srate
        self.ltp = ltp = time.gmtime(time.time())
        self.fbsize = fbsize = fb_helper.dm_to_bins(dm,fb_helper.get_sky_freq(sky,freq),samp_rate,pw50)*resolution
        self.timestr = timestr = "%04d%02d%02d%02d%02d" % (ltp.tm_year, ltp.tm_mon, ltp.tm_mday,ltp.tm_hour,ltp.tm_min)
        self.fbdecim = fbdecim = fb_helper.determine_rate(samp_rate,fbsize,pw50)
        self.an_fftsize = an_fftsize = 2048
        self.swidth = swidth = 1 if wide == 0 else 2
        self.rx_time_tags = rx_time_tags = []
        self.rfi_mask = rfi_mask = fb_helper.static_mask(fb_helper.get_sky_freq(sky,freq),samp_rate,fbsize,rfilist)
        self.profile_dec = profile_dec = 2
        self.pfname = pfname = prefix+"psr-"+timestr
        self.parmname = parmname = prefix+"psr-"+timestr+".header"
        self.fft_poll2 = fft_poll2 = [1.0e-15]*fbsize
        self.fft_poll = fft_poll = [1.0e-15]*fbsize
        self.fbrate = fbrate = (float(samp_rate)/float(fbsize))/float(fbdecim)
        self.analysis_poll_fast = analysis_poll_fast = [0.0]*an_fftsize
        self.update_header = update_header = fb_helper.update_header(rx_time_tags, runtime, swidth)
        self.tag_stash_result = tag_stash_result = fb_helper.process_tag(rx_time_tags)
        self.shaper = shaper = fb_helper.get_correction(fbsize,rolloff,fft_poll2)
        self.sfname = sfname = "/dev/null" if wide == 0 else pfname+".filtmp"
        self.rfi_mask2 = rfi_mask2 = fb_helper.dynamic_mask(fft_poll,rfi_mask,thresh)
        self.reduced_rate = reduced_rate = fbrate/profile_dec
        self.prof_file = prof_file = prefix+"psr-"+timestr+"-profile.json" if profile else '/dev/null'
        self.magn = magn = fb_helper.get_current_estimate()
        self.lpcorner = lpcorner = (1.0/pw50)*2.5
        self.hpcorner = hpcorner = (1.0/p0)/20.0
        self.hdstatus = hdstatus = fb_helper.write_header(parmname, fb_helper.get_sky_freq(sky,freq)/1.0e6, samp_rate/1.0e6, fbsize, fbrate,swidth)
        self.hdr2_stat = hdr2_stat = fb_helper.build_header_info(pfname+".fil",source,ra,dec,fb_helper.get_sky_freq(sky,freq),srate,fbrate,fbsize,None,swidth)
        self.fft_rate = fft_rate = samp_rate/fbsize
        self.fft_log_status = fft_log_status = fb_helper.log_fft(fb_helper.get_sky_freq(sky,freq),samp_rate,prefix,fft_poll2)
        self.cfname = cfname = "/dev/null" if wide == 1 else pfname+".filtmp"
        self.analysis_poll_slow = analysis_poll_slow = [0.0]*an_fftsize
        self.analyser_result = analyser_result = fb_helper.analyser(analysis_poll_fast,fbsize)

        ##################################################
        # Blocks
        ##################################################
        self.fft_probe = blocks.probe_signal_vf(fbsize)
        self.analysis_probe = blocks.probe_signal_vf(an_fftsize)
        self.tag_retrieve = blocks.tag_debug(gr.sizeof_gr_complex*1, '', '')
        self.tag_retrieve.set_display(False)
        def _fft_poll2_probe():
          while True:

            val = self.fft_probe.level()
            try:
              try:
                self.doc.add_next_tick_callback(functools.partial(self.set_fft_poll2,val))
              except AttributeError:
                self.set_fft_poll2(val)
            except AttributeError:
              pass
            time.sleep(1.0 / (5))
        _fft_poll2_thread = threading.Thread(target=_fft_poll2_probe)
        _fft_poll2_thread.daemon = True
        _fft_poll2_thread.start()
        def _analysis_poll_slow_probe():
          while True:

            val = self.analysis_probe.level()
            try:
              try:
                self.doc.add_next_tick_callback(functools.partial(self.set_analysis_poll_slow,val))
              except AttributeError:
                self.set_analysis_poll_slow(val)
            except AttributeError:
              pass
            time.sleep(1.0 / (0.3333))
        _analysis_poll_slow_thread = threading.Thread(target=_analysis_poll_slow_probe)
        _analysis_poll_slow_thread.daemon = True
        _analysis_poll_slow_thread.start()
        self.single_pole_iir_filter_xx_4 = filter.single_pole_iir_filter_ff(1.0/5.0, 1)
        self.single_pole_iir_filter_xx_3 = filter.single_pole_iir_filter_ff(0.1, an_fftsize)
        self.single_pole_iir_filter_xx_2 = filter.single_pole_iir_filter_ff(1.0-math.pow(math.e,-2.0*(20.0/fbrate)), fbsize)
        self.single_pole_iir_filter_xx_1 = filter.single_pole_iir_filter_ff(1.0-math.pow(math.e,-2.0*(hpcorner/(fbrate))), fbsize)
        self.single_pole_iir_filter_xx_0 = filter.single_pole_iir_filter_ff(1.0-math.pow(math.e,-2.0*(lpcorner/(fft_rate))), fbsize)
        def _rx_time_tags_probe():
          while True:

            val = self.tag_retrieve.current_tags()
            try:
              try:
                self.doc.add_next_tick_callback(functools.partial(self.set_rx_time_tags,val))
              except AttributeError:
                self.set_rx_time_tags(val)
            except AttributeError:
              pass
            time.sleep(1.0 / (30))
        _rx_time_tags_thread = threading.Thread(target=_rx_time_tags_probe)
        _rx_time_tags_thread.daemon = True
        _rx_time_tags_thread.start()
        self.osmo_radio = osmosdr.source(
            args="numchan=" + str(1) + " " + "bladerf=0,nchan=1"
        )
        self.osmo_radio.set_time_source(pps, 0)
        self.osmo_radio.set_time_unknown_pps(osmosdr.time_spec_t())
        self.osmo_radio.set_sample_rate(samp_rate)
        self.osmo_radio.set_center_freq(freq, 0)
        self.osmo_radio.set_freq_corr(0, 0)
        self.osmo_radio.set_dc_offset_mode(0, 0)
        self.osmo_radio.set_iq_balance_mode(0, 0)
        self.osmo_radio.set_gain_mode(False, 0)
        self.osmo_radio.set_gain(rfgain, 0)
        self.osmo_radio.set_if_gain(ifgain, 0)
        self.osmo_radio.set_bb_gain(bbgain, 0)
        self.osmo_radio.set_antenna('', 0)
        self.osmo_radio.set_bandwidth(samp_rate, 0)
        self.fft_vxx_0_0 = fft.fft_vcc(fbsize, True, window.blackmanharris(fbsize), True, 1)
        self.fft_vxx_0 = fft.fft_vfc(an_fftsize, True, window.blackmanharris(an_fftsize), True, 1)
        def _fft_poll_probe():
          while True:

            val = self.fft_probe.level()
            try:
              try:
                self.doc.add_next_tick_callback(functools.partial(self.set_fft_poll,val))
              except AttributeError:
                self.set_fft_poll(val)
            except AttributeError:
              pass
            time.sleep(1.0 / (50))
        _fft_poll_thread = threading.Thread(target=_fft_poll_probe)
        _fft_poll_thread.daemon = True
        _fft_poll_thread.start()
        self.copyblock = blocks.copy(gr.sizeof_gr_complex*1)
        self.copyblock.set_enabled(True)
        self.blocks_vector_to_stream_1 = blocks.vector_to_stream(gr.sizeof_float*1, fbsize)
        self.blocks_vector_to_stream_0 = blocks.vector_to_stream(gr.sizeof_float*1, fbsize)
        self.blocks_tags_strobe_0 = blocks.tags_strobe(gr.sizeof_gr_complex*1, pmt.from_bool(True), 1, pmt.intern("local_tick"))
        self.blocks_sub_xx_0 = blocks.sub_ff(fbsize)
        self.blocks_stream_to_vector_2 = blocks.stream_to_vector(gr.sizeof_float*1, an_fftsize)
        self.blocks_stream_to_vector_1 = blocks.stream_to_vector(gr.sizeof_float*1, fbsize)
        self.blocks_stream_to_vector_0 = blocks.stream_to_vector(gr.sizeof_gr_complex*1, fbsize)
        self.blocks_multiply_const_vxx_5 = blocks.multiply_const_vff(fb_helper.get_current_channel(analysis_poll_slow,fbsize))
        self.blocks_multiply_const_vxx_4 = blocks.multiply_const_vff([1.0]*fbsize if integrator == 1 else [0.0]*fbsize)
        self.blocks_multiply_const_vxx_3 = blocks.multiply_const_vff([float(fbdecim)]*fbsize if integrator == 0 else [0.0]*fbsize)
        self.blocks_multiply_const_vxx_2 = blocks.multiply_const_vff(fb_helper.invert_rfi_mask(rfi_mask2)*numpy.array([magn]*fbsize))
        self.blocks_multiply_const_vxx_1 = blocks.multiply_const_vff([0.0] * fbsize if hp == 0 else [hpgain]*fbsize)
        self.blocks_multiply_const_vxx_0 = blocks.multiply_const_vff(numpy.multiply(rfi_mask2,shaper))
        self.blocks_keep_one_in_n_3 = blocks.keep_one_in_n(gr.sizeof_float*fbsize, profile_dec if profile == 1 else 100)
        self.blocks_keep_one_in_n_2 = blocks.keep_one_in_n(gr.sizeof_float*fbsize, 10)
        self.blocks_keep_one_in_n_1 = blocks.keep_one_in_n(gr.sizeof_gr_complex*1, int(samp_rate/15.0))
        self.blocks_keep_one_in_n_0 = blocks.keep_one_in_n(gr.sizeof_float*fbsize, fbdecim)
        self.blocks_integrate_xx_1 = blocks.integrate_ff(fbsize, 1)
        self.blocks_integrate_xx_0 = blocks.integrate_ff(fbdecim, fbsize)
        self.blocks_head_0 = blocks.head(gr.sizeof_gr_complex*1, int(runtime*samp_rate))
        self.blocks_float_to_short_0 = blocks.float_to_short(fbsize, float(dscale) if dscale >= 0 else fb_helper.autoscale(abs(dscale),fft_poll2))
        self.blocks_float_to_complex_0 = blocks.float_to_complex(1)
        self.blocks_float_to_char_0 = blocks.float_to_char(fbsize, float(dscale) if dscale >= 0 else fb_helper.autoscale(abs(dscale),fft_poll2))
        self.blocks_file_sink_0_0 = blocks.file_sink(gr.sizeof_short*fbsize, sfname, True)
        self.blocks_file_sink_0_0.set_unbuffered(False)
        self.blocks_file_sink_0 = blocks.file_sink(gr.sizeof_char*fbsize, cfname, True)
        self.blocks_file_sink_0.set_unbuffered(False)
        self.blocks_complex_to_mag_squared_1 = blocks.complex_to_mag_squared(an_fftsize)
        self.blocks_complex_to_mag_squared_0 = blocks.complex_to_mag_squared(fbsize)
        self.blocks_complex_to_float_0 = blocks.complex_to_float(1)
        self.blocks_add_xx_2 = blocks.add_vff(fbsize)
        self.blocks_add_xx_1 = blocks.add_vff(fbsize)
        self.blocks_add_xx_0 = blocks.add_vcc(1)
        self.blocks_abs_xx_1 = blocks.abs_ff(fbsize)
        self.blocks_abs_xx_0 = blocks.abs_ff(fbsize)
        def _analysis_poll_fast_probe():
          while True:

            val = self.analysis_probe.level()
            try:
              try:
                self.doc.add_next_tick_callback(functools.partial(self.set_analysis_poll_fast,val))
              except AttributeError:
                self.set_analysis_poll_fast(val)
            except AttributeError:
              pass
            time.sleep(1.0 / (4))
        _analysis_poll_fast_thread = threading.Thread(target=_analysis_poll_fast_probe)
        _analysis_poll_fast_thread.daemon = True
        _analysis_poll_fast_thread.start()
        self.analog_fastnoise_source_x_0 = analog.fastnoise_source_f(analog.GR_GAUSSIAN, 1, 0, int(2.0e6))
        self.Folder_Block = Folder_Block.blk(fbsize=fbsize, smear=fb_helper.dm_to_smear(freq,samp_rate,dm), period=p0, filename=prof_file, fbrate=reduced_rate, tbins=tbins, interval=pinterval, tppms=trial_ppms)


        ##################################################
        # Connections
        ##################################################
        self.connect((self.analog_fastnoise_source_x_0, 0), (self.blocks_stream_to_vector_1, 0))
        self.connect((self.blocks_abs_xx_0, 0), (self.blocks_multiply_const_vxx_2, 0))
        self.connect((self.blocks_abs_xx_1, 0), (self.blocks_float_to_char_0, 0))
        self.connect((self.blocks_abs_xx_1, 0), (self.blocks_float_to_short_0, 0))
        self.connect((self.blocks_abs_xx_1, 0), (self.blocks_keep_one_in_n_3, 0))
        self.connect((self.blocks_add_xx_0, 0), (self.tag_retrieve, 0))
        self.connect((self.blocks_add_xx_1, 0), (self.blocks_multiply_const_vxx_0, 0))
        self.connect((self.blocks_add_xx_1, 0), (self.single_pole_iir_filter_xx_2, 0))
        self.connect((self.blocks_add_xx_2, 0), (self.blocks_sub_xx_0, 0))
        self.connect((self.blocks_add_xx_2, 0), (self.single_pole_iir_filter_xx_1, 0))
        self.connect((self.blocks_complex_to_float_0, 0), (self.blocks_float_to_complex_0, 1))
        self.connect((self.blocks_complex_to_float_0, 1), (self.blocks_float_to_complex_0, 0))
        self.connect((self.blocks_complex_to_mag_squared_0, 0), (self.blocks_integrate_xx_0, 0))
        self.connect((self.blocks_complex_to_mag_squared_0, 0), (self.single_pole_iir_filter_xx_0, 0))
        self.connect((self.blocks_complex_to_mag_squared_1, 0), (self.single_pole_iir_filter_xx_3, 0))
        self.connect((self.blocks_float_to_char_0, 0), (self.blocks_file_sink_0, 0))
        self.connect((self.blocks_float_to_complex_0, 0), (self.blocks_head_0, 0))
        self.connect((self.blocks_float_to_short_0, 0), (self.blocks_file_sink_0_0, 0))
        self.connect((self.blocks_head_0, 0), (self.blocks_stream_to_vector_0, 0))
        self.connect((self.blocks_integrate_xx_0, 0), (self.blocks_multiply_const_vxx_4, 0))
        self.connect((self.blocks_integrate_xx_1, 0), (self.single_pole_iir_filter_xx_4, 0))
        self.connect((self.blocks_keep_one_in_n_0, 0), (self.blocks_multiply_const_vxx_3, 0))
        self.connect((self.blocks_keep_one_in_n_1, 0), (self.blocks_add_xx_0, 0))
        self.connect((self.blocks_keep_one_in_n_2, 0), (self.blocks_multiply_const_vxx_5, 0))
        self.connect((self.blocks_keep_one_in_n_3, 0), (self.blocks_vector_to_stream_1, 0))
        self.connect((self.blocks_multiply_const_vxx_0, 0), (self.blocks_add_xx_2, 0))
        self.connect((self.blocks_multiply_const_vxx_1, 0), (self.blocks_sub_xx_0, 1))
        self.connect((self.blocks_multiply_const_vxx_2, 0), (self.blocks_add_xx_2, 1))
        self.connect((self.blocks_multiply_const_vxx_3, 0), (self.blocks_add_xx_1, 0))
        self.connect((self.blocks_multiply_const_vxx_4, 0), (self.blocks_add_xx_1, 1))
        self.connect((self.blocks_multiply_const_vxx_5, 0), (self.blocks_vector_to_stream_0, 0))
        self.connect((self.blocks_stream_to_vector_0, 0), (self.fft_vxx_0_0, 0))
        self.connect((self.blocks_stream_to_vector_1, 0), (self.blocks_abs_xx_0, 0))
        self.connect((self.blocks_stream_to_vector_2, 0), (self.fft_vxx_0, 0))
        self.connect((self.blocks_sub_xx_0, 0), (self.blocks_abs_xx_1, 0))
        self.connect((self.blocks_tags_strobe_0, 0), (self.blocks_add_xx_0, 1))
        self.connect((self.blocks_vector_to_stream_0, 0), (self.blocks_integrate_xx_1, 0))
        self.connect((self.blocks_vector_to_stream_1, 0), (self.Folder_Block, 0))
        self.connect((self.copyblock, 0), (self.blocks_complex_to_float_0, 0))
        self.connect((self.copyblock, 0), (self.blocks_keep_one_in_n_1, 0))
        self.connect((self.fft_vxx_0, 0), (self.blocks_complex_to_mag_squared_1, 0))
        self.connect((self.fft_vxx_0_0, 0), (self.blocks_complex_to_mag_squared_0, 0))
        self.connect((self.osmo_radio, 0), (self.copyblock, 0))
        self.connect((self.single_pole_iir_filter_xx_0, 0), (self.blocks_keep_one_in_n_0, 0))
        self.connect((self.single_pole_iir_filter_xx_0, 0), (self.blocks_keep_one_in_n_2, 0))
        self.connect((self.single_pole_iir_filter_xx_1, 0), (self.blocks_multiply_const_vxx_1, 0))
        self.connect((self.single_pole_iir_filter_xx_2, 0), (self.fft_probe, 0))
        self.connect((self.single_pole_iir_filter_xx_3, 0), (self.analysis_probe, 0))
        self.connect((self.single_pole_iir_filter_xx_4, 0), (self.blocks_stream_to_vector_2, 0))


    def get_bbgain(self):
        return self.bbgain

    def set_bbgain(self, bbgain):
        self.bbgain = bbgain
        self.osmo_radio.set_bb_gain(self.bbgain, 0)

    def get_dec(self):
        return self.dec

    def set_dec(self, dec):
        self.dec = dec
        self.set_hdr2_stat(fb_helper.build_header_info(self.pfname+".fil",self.source,self.ra,self.dec,fb_helper.get_sky_freq(self.sky,self.freq),self.srate,self.fbrate,self.fbsize,None,self.swidth))

    def get_dm(self):
        return self.dm

    def set_dm(self, dm):
        self.dm = dm
        self.set_fbsize(fb_helper.dm_to_bins(self.dm,fb_helper.get_sky_freq(self.sky,self.freq),self.samp_rate,self.pw50)*self.resolution)

    def get_dscale(self):
        return self.dscale

    def set_dscale(self, dscale):
        self.dscale = dscale
        self.blocks_float_to_char_0.set_scale(float(self.dscale) if self.dscale >= 0 else fb_helper.autoscale(abs(self.dscale),self.fft_poll2))
        self.blocks_float_to_short_0.set_scale(float(self.dscale) if self.dscale >= 0 else fb_helper.autoscale(abs(self.dscale),self.fft_poll2))

    def get_freq(self):
        return self.freq

    def set_freq(self, freq):
        self.freq = freq
        self.set_fbsize(fb_helper.dm_to_bins(self.dm,fb_helper.get_sky_freq(self.sky,self.freq),self.samp_rate,self.pw50)*self.resolution)
        self.set_fft_log_status(fb_helper.log_fft(fb_helper.get_sky_freq(self.sky,self.freq),self.samp_rate,self.prefix,self.fft_poll2))
        self.set_hdr2_stat(fb_helper.build_header_info(self.pfname+".fil",self.source,self.ra,self.dec,fb_helper.get_sky_freq(self.sky,self.freq),self.srate,self.fbrate,self.fbsize,None,self.swidth))
        self.set_hdstatus(fb_helper.write_header(self.parmname, fb_helper.get_sky_freq(self.sky,self.freq)/1.0e6, self.samp_rate/1.0e6, self.fbsize, self.fbrate,self.swidth))
        self.set_rfi_mask(fb_helper.static_mask(fb_helper.get_sky_freq(self.sky,self.freq),self.samp_rate,self.fbsize,self.rfilist))
        self.osmo_radio.set_center_freq(self.freq, 0)

    def get_hp(self):
        return self.hp

    def set_hp(self, hp):
        self.hp = hp
        self.blocks_multiply_const_vxx_1.set_k([0.0] * self.fbsize if self.hp == 0 else [self.hpgain]*self.fbsize)

    def get_hpgain(self):
        return self.hpgain

    def set_hpgain(self, hpgain):
        self.hpgain = hpgain
        self.blocks_multiply_const_vxx_1.set_k([0.0] * self.fbsize if self.hp == 0 else [self.hpgain]*self.fbsize)

    def get_ifgain(self):
        return self.ifgain

    def set_ifgain(self, ifgain):
        self.ifgain = ifgain
        self.osmo_radio.set_if_gain(self.ifgain, 0)

    def get_integrator(self):
        return self.integrator

    def set_integrator(self, integrator):
        self.integrator = integrator
        self.blocks_multiply_const_vxx_3.set_k([float(self.fbdecim)]*self.fbsize if self.integrator == 0 else [0.0]*self.fbsize)
        self.blocks_multiply_const_vxx_4.set_k([1.0]*self.fbsize if self.integrator == 1 else [0.0]*self.fbsize)

    def get_p0(self):
        return self.p0

    def set_p0(self, p0):
        self.p0 = p0
        self.set_hpcorner((1.0/self.p0)/20.0)

    def get_pinterval(self):
        return self.pinterval

    def set_pinterval(self, pinterval):
        self.pinterval = pinterval

    def get_pps(self):
        return self.pps

    def set_pps(self, pps):
        self.pps = pps

    def get_prefix(self):
        return self.prefix

    def set_prefix(self, prefix):
        self.prefix = prefix
        self.set_fft_log_status(fb_helper.log_fft(fb_helper.get_sky_freq(self.sky,self.freq),self.samp_rate,self.prefix,self.fft_poll2))
        self.set_parmname(self.prefix+"psr-"+self.timestr+".header")
        self.set_pfname(self.prefix+"psr-"+self.timestr)
        self.set_prof_file(self.prefix+"psr-"+self.timestr+"-profile.json" if self.profile else '/dev/null')

    def get_profile(self):
        return self.profile

    def set_profile(self, profile):
        self.profile = profile
        self.set_prof_file(self.prefix+"psr-"+self.timestr+"-profile.json" if self.profile else '/dev/null')
        self.blocks_keep_one_in_n_3.set_n(self.profile_dec if self.profile == 1 else 100)

    def get_pw50(self):
        return self.pw50

    def set_pw50(self, pw50):
        self.pw50 = pw50
        self.set_fbdecim(fb_helper.determine_rate(self.samp_rate,self.fbsize,self.pw50))
        self.set_fbsize(fb_helper.dm_to_bins(self.dm,fb_helper.get_sky_freq(self.sky,self.freq),self.samp_rate,self.pw50)*self.resolution)
        self.set_lpcorner((1.0/self.pw50)*2.5)

    def get_ra(self):
        return self.ra

    def set_ra(self, ra):
        self.ra = ra
        self.set_hdr2_stat(fb_helper.build_header_info(self.pfname+".fil",self.source,self.ra,self.dec,fb_helper.get_sky_freq(self.sky,self.freq),self.srate,self.fbrate,self.fbsize,None,self.swidth))

    def get_refclock(self):
        return self.refclock

    def set_refclock(self, refclock):
        self.refclock = refclock

    def get_resolution(self):
        return self.resolution

    def set_resolution(self, resolution):
        self.resolution = resolution
        self.set_fbsize(fb_helper.dm_to_bins(self.dm,fb_helper.get_sky_freq(self.sky,self.freq),self.samp_rate,self.pw50)*self.resolution)

    def get_rfgain(self):
        return self.rfgain

    def set_rfgain(self, rfgain):
        self.rfgain = rfgain
        self.osmo_radio.set_gain(self.rfgain, 0)

    def get_rfilist(self):
        return self.rfilist

    def set_rfilist(self, rfilist):
        self.rfilist = rfilist
        self.set_rfi_mask(fb_helper.static_mask(fb_helper.get_sky_freq(self.sky,self.freq),self.samp_rate,self.fbsize,self.rfilist))

    def get_rolloff(self):
        return self.rolloff

    def set_rolloff(self, rolloff):
        self.rolloff = rolloff
        self.set_shaper(fb_helper.get_correction(self.fbsize,self.rolloff,self.fft_poll2))

    def get_runtime(self):
        return self.runtime

    def set_runtime(self, runtime):
        self.runtime = runtime
        self.set_update_header(fb_helper.update_header(self.rx_time_tags, self.runtime, self.swidth))
        self.blocks_head_0.set_length(int(self.runtime*self.samp_rate))

    def get_sky(self):
        return self.sky

    def set_sky(self, sky):
        self.sky = sky
        self.set_fbsize(fb_helper.dm_to_bins(self.dm,fb_helper.get_sky_freq(self.sky,self.freq),self.samp_rate,self.pw50)*self.resolution)
        self.set_fft_log_status(fb_helper.log_fft(fb_helper.get_sky_freq(self.sky,self.freq),self.samp_rate,self.prefix,self.fft_poll2))
        self.set_hdr2_stat(fb_helper.build_header_info(self.pfname+".fil",self.source,self.ra,self.dec,fb_helper.get_sky_freq(self.sky,self.freq),self.srate,self.fbrate,self.fbsize,None,self.swidth))
        self.set_hdstatus(fb_helper.write_header(self.parmname, fb_helper.get_sky_freq(self.sky,self.freq)/1.0e6, self.samp_rate/1.0e6, self.fbsize, self.fbrate,self.swidth))
        self.set_rfi_mask(fb_helper.static_mask(fb_helper.get_sky_freq(self.sky,self.freq),self.samp_rate,self.fbsize,self.rfilist))

    def get_source(self):
        return self.source

    def set_source(self, source):
        self.source = source
        self.set_hdr2_stat(fb_helper.build_header_info(self.pfname+".fil",self.source,self.ra,self.dec,fb_helper.get_sky_freq(self.sky,self.freq),self.srate,self.fbrate,self.fbsize,None,self.swidth))

    def get_srate(self):
        return self.srate

    def set_srate(self, srate):
        self.srate = srate
        self.set_hdr2_stat(fb_helper.build_header_info(self.pfname+".fil",self.source,self.ra,self.dec,fb_helper.get_sky_freq(self.sky,self.freq),self.srate,self.fbrate,self.fbsize,None,self.swidth))
        self.set_samp_rate(self.srate)

    def get_subdev(self):
        return self.subdev

    def set_subdev(self, subdev):
        self.subdev = subdev

    def get_tbins(self):
        return self.tbins

    def set_tbins(self, tbins):
        self.tbins = tbins

    def get_thresh(self):
        return self.thresh

    def set_thresh(self, thresh):
        self.thresh = thresh
        self.set_rfi_mask2(fb_helper.dynamic_mask(self.fft_poll,self.rfi_mask,self.thresh))

    def get_trial_ppms(self):
        return self.trial_ppms

    def set_trial_ppms(self, trial_ppms):
        self.trial_ppms = trial_ppms

    def get_wide(self):
        return self.wide

    def set_wide(self, wide):
        self.wide = wide
        self.set_cfname("/dev/null" if self.wide == 1 else self.pfname+".filtmp")
        self.set_sfname("/dev/null" if self.wide == 0 else self.pfname+".filtmp")
        self.set_swidth(1 if self.wide == 0 else 2)

    def get_samp_rate(self):
        return self.samp_rate

    def set_samp_rate(self, samp_rate):
        self.samp_rate = samp_rate
        self.set_fbdecim(fb_helper.determine_rate(self.samp_rate,self.fbsize,self.pw50))
        self.set_fbrate((float(self.samp_rate)/float(self.fbsize))/float(self.fbdecim))
        self.set_fbsize(fb_helper.dm_to_bins(self.dm,fb_helper.get_sky_freq(self.sky,self.freq),self.samp_rate,self.pw50)*self.resolution)
        self.set_fft_log_status(fb_helper.log_fft(fb_helper.get_sky_freq(self.sky,self.freq),self.samp_rate,self.prefix,self.fft_poll2))
        self.set_fft_rate(self.samp_rate/self.fbsize)
        self.set_hdstatus(fb_helper.write_header(self.parmname, fb_helper.get_sky_freq(self.sky,self.freq)/1.0e6, self.samp_rate/1.0e6, self.fbsize, self.fbrate,self.swidth))
        self.set_rfi_mask(fb_helper.static_mask(fb_helper.get_sky_freq(self.sky,self.freq),self.samp_rate,self.fbsize,self.rfilist))
        self.blocks_head_0.set_length(int(self.runtime*self.samp_rate))
        self.blocks_keep_one_in_n_1.set_n(int(self.samp_rate/15.0))
        self.osmo_radio.set_sample_rate(self.samp_rate)
        self.osmo_radio.set_bandwidth(self.samp_rate, 0)

    def get_ltp(self):
        return self.ltp

    def set_ltp(self, ltp):
        self.ltp = ltp

    def get_fbsize(self):
        return self.fbsize

    def set_fbsize(self, fbsize):
        self.fbsize = fbsize
        self.set_analyser_result(fb_helper.analyser(self.analysis_poll_fast,self.fbsize))
        self.set_fbdecim(fb_helper.determine_rate(self.samp_rate,self.fbsize,self.pw50))
        self.set_fbrate((float(self.samp_rate)/float(self.fbsize))/float(self.fbdecim))
        self.set_fft_poll([1.0e-15]*self.fbsize)
        self.set_fft_poll2([1.0e-15]*self.fbsize)
        self.set_fft_rate(self.samp_rate/self.fbsize)
        self.set_hdr2_stat(fb_helper.build_header_info(self.pfname+".fil",self.source,self.ra,self.dec,fb_helper.get_sky_freq(self.sky,self.freq),self.srate,self.fbrate,self.fbsize,None,self.swidth))
        self.set_hdstatus(fb_helper.write_header(self.parmname, fb_helper.get_sky_freq(self.sky,self.freq)/1.0e6, self.samp_rate/1.0e6, self.fbsize, self.fbrate,self.swidth))
        self.set_rfi_mask(fb_helper.static_mask(fb_helper.get_sky_freq(self.sky,self.freq),self.samp_rate,self.fbsize,self.rfilist))
        self.set_shaper(fb_helper.get_correction(self.fbsize,self.rolloff,self.fft_poll2))
        self.blocks_multiply_const_vxx_1.set_k([0.0] * self.fbsize if self.hp == 0 else [self.hpgain]*self.fbsize)
        self.blocks_multiply_const_vxx_2.set_k(fb_helper.invert_rfi_mask(self.rfi_mask2)*numpy.array([self.magn]*self.fbsize))
        self.blocks_multiply_const_vxx_3.set_k([float(self.fbdecim)]*self.fbsize if self.integrator == 0 else [0.0]*self.fbsize)
        self.blocks_multiply_const_vxx_4.set_k([1.0]*self.fbsize if self.integrator == 1 else [0.0]*self.fbsize)
        self.blocks_multiply_const_vxx_5.set_k(fb_helper.get_current_channel(self.analysis_poll_slow,self.fbsize))

    def get_timestr(self):
        return self.timestr

    def set_timestr(self, timestr):
        self.timestr = timestr
        self.set_parmname(self.prefix+"psr-"+self.timestr+".header")
        self.set_pfname(self.prefix+"psr-"+self.timestr)
        self.set_prof_file(self.prefix+"psr-"+self.timestr+"-profile.json" if self.profile else '/dev/null')

    def get_fbdecim(self):
        return self.fbdecim

    def set_fbdecim(self, fbdecim):
        self.fbdecim = fbdecim
        self.set_fbrate((float(self.samp_rate)/float(self.fbsize))/float(self.fbdecim))
        self.blocks_keep_one_in_n_0.set_n(self.fbdecim)
        self.blocks_multiply_const_vxx_3.set_k([float(self.fbdecim)]*self.fbsize if self.integrator == 0 else [0.0]*self.fbsize)

    def get_an_fftsize(self):
        return self.an_fftsize

    def set_an_fftsize(self, an_fftsize):
        self.an_fftsize = an_fftsize
        self.set_analysis_poll_fast([0.0]*self.an_fftsize)
        self.set_analysis_poll_slow([0.0]*self.an_fftsize)

    def get_swidth(self):
        return self.swidth

    def set_swidth(self, swidth):
        self.swidth = swidth
        self.set_hdr2_stat(fb_helper.build_header_info(self.pfname+".fil",self.source,self.ra,self.dec,fb_helper.get_sky_freq(self.sky,self.freq),self.srate,self.fbrate,self.fbsize,None,self.swidth))
        self.set_hdstatus(fb_helper.write_header(self.parmname, fb_helper.get_sky_freq(self.sky,self.freq)/1.0e6, self.samp_rate/1.0e6, self.fbsize, self.fbrate,self.swidth))
        self.set_update_header(fb_helper.update_header(self.rx_time_tags, self.runtime, self.swidth))

    def get_rx_time_tags(self):
        return self.rx_time_tags

    def set_rx_time_tags(self, rx_time_tags):
        self.rx_time_tags = rx_time_tags
        self.set_tag_stash_result(fb_helper.process_tag(self.rx_time_tags))
        self.set_update_header(fb_helper.update_header(self.rx_time_tags, self.runtime, self.swidth))

    def get_rfi_mask(self):
        return self.rfi_mask

    def set_rfi_mask(self, rfi_mask):
        self.rfi_mask = rfi_mask
        self.set_rfi_mask2(fb_helper.dynamic_mask(self.fft_poll,self.rfi_mask,self.thresh))

    def get_profile_dec(self):
        return self.profile_dec

    def set_profile_dec(self, profile_dec):
        self.profile_dec = profile_dec
        self.set_reduced_rate(self.fbrate/self.profile_dec)
        self.blocks_keep_one_in_n_3.set_n(self.profile_dec if self.profile == 1 else 100)

    def get_pfname(self):
        return self.pfname

    def set_pfname(self, pfname):
        self.pfname = pfname
        self.set_cfname("/dev/null" if self.wide == 1 else self.pfname+".filtmp")
        self.set_hdr2_stat(fb_helper.build_header_info(self.pfname+".fil",self.source,self.ra,self.dec,fb_helper.get_sky_freq(self.sky,self.freq),self.srate,self.fbrate,self.fbsize,None,self.swidth))
        self.set_sfname("/dev/null" if self.wide == 0 else self.pfname+".filtmp")

    def get_parmname(self):
        return self.parmname

    def set_parmname(self, parmname):
        self.parmname = parmname
        self.set_hdstatus(fb_helper.write_header(self.parmname, fb_helper.get_sky_freq(self.sky,self.freq)/1.0e6, self.samp_rate/1.0e6, self.fbsize, self.fbrate,self.swidth))

    def get_fft_poll2(self):
        return self.fft_poll2

    def set_fft_poll2(self, fft_poll2):
        self.fft_poll2 = fft_poll2
        self.set_fft_log_status(fb_helper.log_fft(fb_helper.get_sky_freq(self.sky,self.freq),self.samp_rate,self.prefix,self.fft_poll2))
        self.set_shaper(fb_helper.get_correction(self.fbsize,self.rolloff,self.fft_poll2))
        self.blocks_float_to_char_0.set_scale(float(self.dscale) if self.dscale >= 0 else fb_helper.autoscale(abs(self.dscale),self.fft_poll2))
        self.blocks_float_to_short_0.set_scale(float(self.dscale) if self.dscale >= 0 else fb_helper.autoscale(abs(self.dscale),self.fft_poll2))

    def get_fft_poll(self):
        return self.fft_poll

    def set_fft_poll(self, fft_poll):
        self.fft_poll = fft_poll
        self.set_rfi_mask2(fb_helper.dynamic_mask(self.fft_poll,self.rfi_mask,self.thresh))

    def get_fbrate(self):
        return self.fbrate

    def set_fbrate(self, fbrate):
        self.fbrate = fbrate
        self.set_hdr2_stat(fb_helper.build_header_info(self.pfname+".fil",self.source,self.ra,self.dec,fb_helper.get_sky_freq(self.sky,self.freq),self.srate,self.fbrate,self.fbsize,None,self.swidth))
        self.set_hdstatus(fb_helper.write_header(self.parmname, fb_helper.get_sky_freq(self.sky,self.freq)/1.0e6, self.samp_rate/1.0e6, self.fbsize, self.fbrate,self.swidth))
        self.set_reduced_rate(self.fbrate/self.profile_dec)
        self.single_pole_iir_filter_xx_1.set_taps(1.0-math.pow(math.e,-2.0*(self.hpcorner/(self.fbrate))))
        self.single_pole_iir_filter_xx_2.set_taps(1.0-math.pow(math.e,-2.0*(20.0/self.fbrate)))

    def get_analysis_poll_fast(self):
        return self.analysis_poll_fast

    def set_analysis_poll_fast(self, analysis_poll_fast):
        self.analysis_poll_fast = analysis_poll_fast
        self.set_analyser_result(fb_helper.analyser(self.analysis_poll_fast,self.fbsize))

    def get_update_header(self):
        return self.update_header

    def set_update_header(self, update_header):
        self.update_header = update_header

    def get_tag_stash_result(self):
        return self.tag_stash_result

    def set_tag_stash_result(self, tag_stash_result):
        self.tag_stash_result = tag_stash_result

    def get_shaper(self):
        return self.shaper

    def set_shaper(self, shaper):
        self.shaper = shaper
        self.blocks_multiply_const_vxx_0.set_k(numpy.multiply(self.rfi_mask2,self.shaper))

    def get_sfname(self):
        return self.sfname

    def set_sfname(self, sfname):
        self.sfname = sfname
        self.blocks_file_sink_0_0.open(self.sfname)

    def get_rfi_mask2(self):
        return self.rfi_mask2

    def set_rfi_mask2(self, rfi_mask2):
        self.rfi_mask2 = rfi_mask2
        self.blocks_multiply_const_vxx_0.set_k(numpy.multiply(self.rfi_mask2,self.shaper))
        self.blocks_multiply_const_vxx_2.set_k(fb_helper.invert_rfi_mask(self.rfi_mask2)*numpy.array([self.magn]*self.fbsize))

    def get_reduced_rate(self):
        return self.reduced_rate

    def set_reduced_rate(self, reduced_rate):
        self.reduced_rate = reduced_rate

    def get_prof_file(self):
        return self.prof_file

    def set_prof_file(self, prof_file):
        self.prof_file = prof_file

    def get_magn(self):
        return self.magn

    def set_magn(self, magn):
        self.magn = magn
        self.blocks_multiply_const_vxx_2.set_k(fb_helper.invert_rfi_mask(self.rfi_mask2)*numpy.array([self.magn]*self.fbsize))

    def get_lpcorner(self):
        return self.lpcorner

    def set_lpcorner(self, lpcorner):
        self.lpcorner = lpcorner
        self.single_pole_iir_filter_xx_0.set_taps(1.0-math.pow(math.e,-2.0*(self.lpcorner/(self.fft_rate))))

    def get_hpcorner(self):
        return self.hpcorner

    def set_hpcorner(self, hpcorner):
        self.hpcorner = hpcorner
        self.single_pole_iir_filter_xx_1.set_taps(1.0-math.pow(math.e,-2.0*(self.hpcorner/(self.fbrate))))

    def get_hdstatus(self):
        return self.hdstatus

    def set_hdstatus(self, hdstatus):
        self.hdstatus = hdstatus

    def get_hdr2_stat(self):
        return self.hdr2_stat

    def set_hdr2_stat(self, hdr2_stat):
        self.hdr2_stat = hdr2_stat

    def get_fft_rate(self):
        return self.fft_rate

    def set_fft_rate(self, fft_rate):
        self.fft_rate = fft_rate
        self.single_pole_iir_filter_xx_0.set_taps(1.0-math.pow(math.e,-2.0*(self.lpcorner/(self.fft_rate))))

    def get_fft_log_status(self):
        return self.fft_log_status

    def set_fft_log_status(self, fft_log_status):
        self.fft_log_status = fft_log_status

    def get_cfname(self):
        return self.cfname

    def set_cfname(self, cfname):
        self.cfname = cfname
        self.blocks_file_sink_0.open(self.cfname)

    def get_analysis_poll_slow(self):
        return self.analysis_poll_slow

    def set_analysis_poll_slow(self, analysis_poll_slow):
        self.analysis_poll_slow = analysis_poll_slow
        self.blocks_multiply_const_vxx_5.set_k(fb_helper.get_current_channel(self.analysis_poll_slow,self.fbsize))

    def get_analyser_result(self):
        return self.analyser_result

    def set_analyser_result(self, analyser_result):
        self.analyser_result = analyser_result



def argument_parser():
    parser = ArgumentParser()
    parser.add_argument(
        "--bbgain", dest="bbgain", type=eng_float, default=eng_notation.num_to_str(float(10)),
        help="Set Baseband Gain [default=%(default)r]")
    parser.add_argument(
        "--dec", dest="dec", type=eng_float, default=eng_notation.num_to_str(float(54.58)),
        help="Set Source DEC [default=%(default)r]")
    parser.add_argument(
        "--dm", dest="dm", type=eng_float, default=eng_notation.num_to_str(float(26.76)),
        help="Set Dispersion Measure [default=%(default)r]")
    parser.add_argument(
        "--dscale", dest="dscale", type=intx, default=1,
        help="Set Detector scaling [default=%(default)r]")
    parser.add_argument(
        "--freq", dest="freq", type=eng_float, default=eng_notation.num_to_str(float(1400e6)),
        help="Set Tuner Frequency [default=%(default)r]")
    parser.add_argument(
        "--hp", dest="hp", type=intx, default=1,
        help="Set HP pass enable [default=%(default)r]")
    parser.add_argument(
        "--hpgain", dest="hpgain", type=eng_float, default=eng_notation.num_to_str(float(0.6)),
        help="Set High Pass pseudo-gain [default=%(default)r]")
    parser.add_argument(
        "--ifgain", dest="ifgain", type=eng_float, default=eng_notation.num_to_str(float(10)),
        help="Set IF Gain [default=%(default)r]")
    parser.add_argument(
        "--integrator", dest="integrator", type=intx, default=0,
        help="Set Enable straight integrator instead of IIR [default=%(default)r]")
    parser.add_argument(
        "--p0", dest="p0", type=eng_float, default=eng_notation.num_to_str(float(0.71452)),
        help="Set Pulsar P0 [default=%(default)r]")
    parser.add_argument(
        "--pinterval", dest="pinterval", type=intx, default=30,
        help="Set Interval for logging profile [default=%(default)r]")
    parser.add_argument(
        "--profile", dest="profile", type=intx, default=0,
        help="Set Log Rough Profile [default=%(default)r]")
    parser.add_argument(
        "--pw50", dest="pw50", type=eng_float, default=eng_notation.num_to_str(float(6.6e-3)),
        help="Set Pulsar PW50 [default=%(default)r]")
    parser.add_argument(
        "--ra", dest="ra", type=eng_float, default=eng_notation.num_to_str(float(53.25)),
        help="Set Source RA [default=%(default)r]")
    parser.add_argument(
        "--resolution", dest="resolution", type=intx, default=10,
        help="Set FB resolution multiplier [default=%(default)r]")
    parser.add_argument(
        "--rfgain", dest="rfgain", type=eng_float, default=eng_notation.num_to_str(float(45)),
        help="Set RF Gain [default=%(default)r]")
    parser.add_argument(
        "--rolloff", dest="rolloff", type=intx, default=0,
        help="Set Enable Roll-off correction [default=%(default)r]")
    parser.add_argument(
        "--runtime", dest="runtime", type=intx, default=60*5,
        help="Set Total runtime (seconds) [default=%(default)r]")
    parser.add_argument(
        "--sky", dest="sky", type=eng_float, default=eng_notation.num_to_str(float(0)),
        help="Set Sky Frequency [default=%(default)r]")
    parser.add_argument(
        "--srate", dest="srate", type=eng_float, default=eng_notation.num_to_str(float(40.0e6)),
        help="Set Hardware Sample Rate [default=%(default)r]")
    parser.add_argument(
        "--tbins", dest="tbins", type=intx, default=250,
        help="Set Number of time bins for profile [default=%(default)r]")
    parser.add_argument(
        "--thresh", dest="thresh", type=eng_float, default=eng_notation.num_to_str(float(2.5)),
        help="Set RFI detection threshold--linear factor [default=%(default)r]")
    parser.add_argument(
        "--wide", dest="wide", type=intx, default=0,
        help="Set Enable 16-bit output [default=%(default)r]")
    return parser


def main(top_block_cls=pulsar_filterbank_uhd, options=None):
    if options is None:
        options = argument_parser().parse_args()
    tb = top_block_cls(bbgain=options.bbgain, dec=options.dec, dm=options.dm, dscale=options.dscale, freq=options.freq, hp=options.hp, hpgain=options.hpgain, ifgain=options.ifgain, integrator=options.integrator, p0=options.p0, pinterval=options.pinterval, profile=options.profile, pw50=options.pw50, ra=options.ra, resolution=options.resolution, rfgain=options.rfgain, rolloff=options.rolloff, runtime=options.runtime, sky=options.sky, srate=options.srate, tbins=options.tbins, thresh=options.thresh, wide=options.wide)

    def sig_handler(sig=None, frame=None):
        tb.stop()
        tb.wait()

        sys.exit(0)

    signal.signal(signal.SIGINT, sig_handler)
    signal.signal(signal.SIGTERM, sig_handler)

    tb.start()

    tb.wait()


if __name__ == '__main__':
    main()
