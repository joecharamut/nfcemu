##
## This file is part of the libsigrokdecode project.
##
## Copyright (C) 2018 Steve R <steversig@virginmedia.com>
##
## This program is free software; you can redistribute it and/or modify
## it under the terms of the GNU General Public License as published by
## the Free Software Foundation; either version 2 of the License, or
## (at your option) any later version.
##
## This program is distributed in the hope that it will be useful,
## but WITHOUT ANY WARRANTY; without even the implied warranty of
## MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
## GNU General Public License for more details.
##
## You should have received a copy of the GNU General Public License
## along with this program; if not, see <http://www.gnu.org/licenses/>.
##

import sigrokdecode as srd

'''
OUTPUT_PYTHON format:
Samples:    The Samples array is sent when a DECODE_TIMEOUT occurs.
[<start>, <finish>, <state>]
<start> is the sample number of the start of the decoded bit. This may not line
up with the pulses that were converted into the decoded bit particularly for
Manchester encoding.
<finish> is the sample number of the end of the decoded bit.
<state> is a single character string which is the state of the decoded bit.
This can be
'0'   zero or low
'1'   one or high
'E'   Error or invalid. This can be caused by missing transitions or the wrong
pulse lengths according to the rules for the particular encoding. In some cases
this is intentional (Oregon 1 preamble) and is part of the sync pattern. In
other cases the signal could simply be broken.

If there are more than self.max_errors (default 5) in decoding then the
OUTPUT_PYTHON is not sent as the data is assumed to be worthless.
There also needs to be a low for five times the preamble period at the end of
each set of pulses to trigger a DECODE_TIMEOUT and get the OUTPUT_PYTHON sent.
'''

class SamplerateError(Exception):
    pass

def roundto(x, k=1.0):
    return round(x / k) * k

class Decoder(srd.Decoder):
    api_version = 3
    id = 'nfc-ook'
    name = 'NFC Listen'
    longname = 'NFC Listen->Poll communication (Manchester encoding)'
    desc = 'Manchester encoding protocol'
    license = 'gplv2+'
    inputs = ['logic']
    outputs = []
    tags = ['Encoding']
    channels = (
        {'id': 'data', 'name': 'Data', 'desc': 'Data line'},
    )
    annotations = (
        ('bits', 'Bits'),
        ('framing', 'Framing'),
        ('commands', 'Commands'),
    )
    annotation_rows = (
        ('bits', 'Bits', (0,)),
        ('framing', 'Framing', (1,)),
        ('commands', 'Commands', (2,)),
    )
    binary = (
        ('data', 'Data Bytes'),
    )
    options = (
        {'id': 'baudrate', 'desc': 'Baud rate', 'default': 106000},
    )

    def __init__(self):
        self.reset()

    def reset(self):
        self.samplerate = None
        self.ss = self.es = -1

    def metadata(self, key, value):
        if key == srd.SRD_CONF_SAMPLERATE:
            self.samplerate = value

    def start(self):
        self.out_ann = self.register(srd.OUTPUT_ANN)
        self.out_binary = self.register(srd.OUTPUT_BINARY)

    def decode_bits(self):
        timeunit = self.samplerate / self.options['baudrate']
        bitperiod = 2*timeunit
        
        lastedge = None
        lastedgetype = None
        expectednext = None
        decoding = False

        # wait for initial rising edge
        value, = self.wait({0: 'r'})
        yield (str(value), self.samplenum, self.samplenum+bitperiod)
        lastbitend = self.samplenum+bitperiod

        while True:
            next_bit, = self.wait({"skip": int(0.75*bitperiod)})
            self.wait([{0: "e"}, {"skip": int(bitperiod)}])

            if self.matched[1]:
                yield (False, lastbitend, lastbitend+bitperiod)
                break
            yield (str(next_bit), lastbitend, lastbitend+bitperiod)
            lastbitend = lastbitend+bitperiod

    def run_decode(self):
        timeunit = self.samplerate / self.options['baudrate']
        last_edge = None
        edge_time = None

        last_bit = None
        next_start = None

        active = False

        cur_byte = 0
        bytes_read = []
        byte_start = None
        byte_end = None
        bitpos = 0
        for value, ss, es in self.decode_bits():
            if value is False:
                active = False
                self.put(int(ss), int(es), self.out_ann, [1, ["End of Frame", "EoF", "E"]])
                if bitpos > 1:
                    bytes_read.append(cur_byte)
                    cur_byte = 0
                    bitpos = 0
            elif not active and value == '1':
                active = True
                self.put(int(ss), int(es), self.out_ann, [0, ["{}".format(value)]])
                self.put(int(ss), int(es), self.out_ann, [1, ["Start of Frame", "SoF", "S"]])
            else:
                self.put(int(ss), int(es), self.out_ann, [0, ["{}".format(value)]])
                if bitpos == 0:
                    byte_start = ss
                
                if bitpos == 8:
                    byte_end = ss
                    self.put(int(byte_start), int(byte_end), self.out_ann, [1, ["{:02X}".format(cur_byte)]])
                    self.put(int(ss), int(es), self.out_ann, [1, ["Parity", "P"]])
                    bytes_read.append(cur_byte)
                    cur_byte = 0
                    bitpos = 0
                else:
                    cur_byte |= (int(value) << bitpos)
                    bitpos += 1

    def decode(self):
        while True:
            self.run_decode()
