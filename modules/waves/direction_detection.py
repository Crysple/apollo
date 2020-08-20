#!/usr/bin/env python3

# ****************************************************************************
# Copyright 2020 The Apollo Authors. All Rights Reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
#  You may obtain a copy of the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
# ****************************************************************************
# -*- coding: utf-8 -*-
import os
import wave
from cyber.python.cyber_py3 import cyber
from modules.drivers.microphone.proto.audio_pb2 import AudioData
import numpy as np
import math

SOUND_SPEED = 343.2
MIC_DISTANCE_4 = 0.08127
MAX_TDOA_4 = MIC_DISTANCE_4 / float(SOUND_SPEED)

RESPEAKER_CHANNEL = "/apollo/sensor/microphone"
WAV_SAVING_PATH = "/tmp"

def gcc_phat(sig, refsig, fs=1, max_tau=None, interp=16):
    n = sig.shape[0] + refsig.shape[0]

    SIG = np.fft.rfft(sig, n=n)
    REFSIG = np.fft.rfft(refsig, n=n)
    R = SIG * np.conj(REFSIG)

    cc = np.fft.irfft(R / np.abs(R), n=(interp * n))

    max_shift = int(interp * n / 2)
    if max_tau:
        max_shift = np.minimum(int(interp * fs * max_tau), max_shift)

    cc = np.concatenate((cc[-max_shift:], cc[:max_shift+1]))

    shift = np.argmax(np.abs(cc)) - max_shift
    tau = shift / float(interp * fs)
    #print("shift & maxshift ", shift, max_shift)
    return tau, cc

def get_direction(channels, sample_rate):
    #for c in channels:
    #    print(c[:8])
    #print()
    best_guess = None
    
    MIC_GROUP_N = 2
    MIC_GROUP = [[0, 2], [1, 3]]

    tau = [0] * MIC_GROUP_N
    theta = [0] * MIC_GROUP_N
    for i, v in enumerate(MIC_GROUP):
        tau[i], _ = gcc_phat(channels[v[0]], channels[v[1]], fs=sample_rate, max_tau=MAX_TDOA_4, interp=1)
        theta[i] = math.asin(tau[i] / MAX_TDOA_4) * 180 / math.pi

    print(tau, theta)
    if np.abs(theta[0]) < np.abs(theta[1]):
        if theta[1] > 0:
            best_guess = (theta[0] + 360) % 360
        else:
            best_guess = (180 - theta[0])
    else:
        if theta[0] < 0:
            best_guess = (theta[1] + 360) % 360
        else:
            best_guess = (180 - theta[1])

        best_guess = (best_guess + 90 + 180) % 360


    best_guess = (-best_guess + 120) % 360

    return best_guess



def callback(audio):
    sample_width = audio.microphone_config.sample_width
    sample_rate = audio.microphone_config.sample_rate
    channels = []
    for idx, channel_data in enumerate(audio.channel_data):
        if channel_data.channel_type != 2:
            continue
        #print(channel_data.data[:10], np.fromstring(channel_data.data[:10], dtype='int16'))
        channels.append(np.fromstring(channel_data.data, dtype='int16'))
        #file_path = os.path.join(WAV_SAVING_PATH, "seq_{}_channel_{}.wav".format(audio.header.sequence_num, idx))
        #save_to_wave(channel_data.data, file_path, sample_width, sample_rate, 1)
    #for c in channels:
    #    for i in range(1, 6):
    #        print(c[i], end=" ")
    #    print()
    #print('length is: ', len(channels[1]), "And: ", len(audio.channel_data[0]))
    #print(len(channels))
    i = 0
    chunk = audio.microphone_config.chunk
    while (i+1) * chunk < len(channels[0]):
        print("Current direction is: ", get_direction([c[i*chunk:(i+1)*chunk] for c in channels], audio.microphone_config.sample_rate))
        i += 1


def run():
    print("=" * 120)
    test_node = cyber.Node("audiosaver")
    test_node.create_reader(RESPEAKER_CHANNEL, AudioData, callback)
    test_node.spin()


if __name__ == '__main__':
    cyber.init()
    run()
    cyber.shutdown()

