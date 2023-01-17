##
## This file is part of the libsigrokdecode project.
##
## Copyright (C) 2017 Christoph Rackwitz <christoph.rackwitz@rwth-aachen.de>
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

# http://www.gorferay.com/type-a-communications-interface/
# https://resources.infosecinstitute.com/introduction-rfid-security/
# https://www.radio-electronics.com/info/wireless/nfc/near-field-communications-modulation-rf-signal-interface.php
# https://www.researchgate.net/figure/Modified-Miller-Code_fig16_283498836

# Miller: either edge
# modified Miller: falling edge

import sigrokdecode as srd

def roundto(x, k=1.0):
    return round(x / k) * k

class Decoder(srd.Decoder):
    api_version = 3
    id = 'nfc-miller'
    name = 'NFC (Miller)'
    longname = 'NFC Miller Protocol'
    desc = 'Decode NFC miller (Poll->Listen) communication'
    license = 'gplv2+'
    inputs = ['logic']
    outputs = []
    tags = ['Encoding']
    channels = (
        {'id': 'data', 'name': 'Data', 'desc': 'Data signal'},
    )
    options = (
        {'id': 'baudrate', 'desc': 'Baud rate', 'default': 106000},
    )
    annotations = (
        ('bit', 'Bits'),
        ('framing', 'Framing'),
        ('commands', 'Commands'),
    )
    annotation_rows = tuple((u, v, (i,)) for i, (u, v) in enumerate(annotations))
    binary = (
        ('raw', 'Raw binary'),
    )

    def __init__(self):
        self.reset()

    def reset(self):
        self.samplerate = None

    def metadata(self, key, value):
        if key == srd.SRD_CONF_SAMPLERATE:
            self.samplerate = value

    def start(self):
        self.out_ann = self.register(srd.OUTPUT_ANN)
        self.out_binary = self.register(srd.OUTPUT_BINARY)

    def decode_bits(self):
        timeunit = self.samplerate / self.options['baudrate']
        edgetype = 'f'

        self.wait({0: edgetype}) # first symbol, beginning of unit
        prevedge = self.samplenum

        # start of message: '0'
        prevbit = 0
        yield (0, prevedge, prevedge + timeunit)
        expectedstart = self.samplenum + timeunit

        # end of message: '0' followed by one idle symbol

        while True:
            self.wait([{0: edgetype}, {'skip': int(3 * timeunit)}])
            got_timeout = self.matched[1]
            sampledelta = (self.samplenum - prevedge)
            prevedge = self.samplenum
            timedelta = roundto(sampledelta / timeunit, 0.5)

            # a mark stands for a 1 bit
            # a mark has an edge in the middle

            # a space stands for a 0 bit
            # a space either has an edge at the beginning or no edge at all
            # after a mark, a space is edge-less
            # after a space, a space has an edge

            # we get 1.0, 1.5, 2.0 times between edges

            # end of transmission is always a space, either edged or edge-less

            if prevbit == 0: # space -> ???
                if timedelta == 1.0: # 1.0 units -> space
                    yield (0, self.samplenum, self.samplenum + timeunit)
                    prevbit = 0
                    expectedstart = self.samplenum + timeunit
                elif timedelta == 1.5: # 1.5 units -> mark
                    yield (1, expectedstart, self.samplenum + 0.5*timeunit)
                    prevbit = 1
                    expectedstart = self.samplenum + timeunit*0.5
                elif timedelta >= 2.0:
                    # idle symbol (end of message)
                    yield None
                else:
                    # assert timedelta >= 2.0
                    yield (False, self.samplenum - sampledelta, self.samplenum)
                    break
            else: # mark -> ???
                if timedelta <= 0.5:
                    yield (False, self.samplenum - sampledelta, self.samplenum)
                    break
                if timedelta == 1.0: # 1.0 units -> mark again (1.5 from start)
                    yield (1, expectedstart, self.samplenum + 0.5*timeunit)
                    prevbit = 1
                    expectedstart = self.samplenum + 0.5*timeunit
                elif timedelta == 1.5: # 1.5 units -> space (no pulse) and space (pulse)
                    yield (0, expectedstart, self.samplenum)
                    yield (0, self.samplenum, self.samplenum + timeunit)
                    prevbit = 0
                    expectedstart = self.samplenum + timeunit
                elif timedelta == 2.0: # 2.0 units -> space (no pulse) and mark (pulse)
                    yield (0, expectedstart, expectedstart + timeunit)
                    yield (1, self.samplenum - 0.5*timeunit, self.samplenum + 0.5*timeunit)
                    prevbit = 1
                    expectedstart = self.samplenum + timeunit*0.5
                else: # longer -> space and end of message
                    yield (0, expectedstart, expectedstart + timeunit)
                    yield None
                    break

    def resolve_cmd(self, start, end, cmd_bytes):
        if len(cmd_bytes) == 1:
            b = cmd_bytes[0]
            if b == 0x52:
                self.put(int(start), int(end), self.out_ann, [2, ['ALL_REQ']])
            elif b == 0x26:
                self.put(int(start), int(end), self.out_ann, [2, ['SENS_REQ']])
        elif len(cmd_bytes) == 2:
            b = cmd_bytes[0]
            s = "SDD_REQ"
            if b == 0x93:
                s += " CL1"
            elif b == 0x95:
                s += " CL2"
            elif b == 0x97:
                s += " CL3"
            self.put(int(start), int(end), self.out_ann, [2, [s]])
        else:
            if cmd_bytes[0:4] == [0x50, 0x00, 0x57, 0xCD]:
                self.put(int(start), int(end), self.out_ann, [2, ['SLP_REQ']])

    def decode_run(self):
        numbits = 0
        bitvalue = 0
        bitstring = ''
        stringstart = None
        stringend = None

        sof = None
        eof = None

        last_bit = None
        byte_start = None
        byte_end = None
        bit_pos = 0
        cur_byte = 0

        cmd_start = None
        cmd_end = None
        bytes_read = []

        for bit in self.decode_bits():
            if bit is None:
                if sof:
                    self.put(int(last_bit[1]), int(last_bit[2]), self.out_ann, [1, ['End of Frame', 'EoF', 'E']])
                    cmd_end = int(last_bit[2])
                    if bit_pos >= 4:
                        self.put(int(byte_start), int(last_bit[1]), self.out_ann, [1, ['{:02X}'.format(cur_byte)]])
                        bytes_read.append(cur_byte)
                        cur_byte = 0
                        bit_pos = 0
                break

            (value, ss, es) = bit
            last_bit = bit

            if value is False:
                self.put(int(ss), int(es), self.out_ann, [1, ['ERROR']])
            else:
                self.put(int(ss), int(es), self.out_ann, [0, ['{}'.format(value)]])

            if value is False:
                numbits = 0
                break

            if sof is None:
                if value == 0:
                    sof = bit
                    self.put(int(ss), int(es), self.out_ann, [1, ['Start of Frame', 'SoF', 'S']])
                    cmd_start = ss
                else:
                    self.put(int(ss), int(es), self.out_ann, [1, ['ERROR']])
            else:
                if bit_pos == 0:
                    byte_start = ss
                
                if bit_pos == 8:
                    self.put(int(byte_start), int(byte_end), self.out_ann, [1, ['{:02X}'.format(cur_byte)]])
                    bytes_read.append(cur_byte)
                    cur_byte = 0
                    bit_pos = 0
                    self.put(int(ss), int(es), self.out_ann, [1, ['Parity', 'P']])
                else:
                    byte_end = es
                    cur_byte |= (value<<bit_pos)
                    bit_pos += 1
                
            if stringstart is None:
                stringstart = ss

            stringend = es

            bitvalue |= value << numbits
            numbits += 1

            bitstring += '{}'.format(value)
            if numbits % 4 == 0:
                bitstring += ' '

        self.resolve_cmd(cmd_start, cmd_end, bytes_read)

        if not numbits:
            return

        # self.put(int(stringstart), int(stringend), self.out_ann, [1, ['{}'.format(bitstring)]])

        numbytes = numbits // 8 + (numbits % 8 > 0)
        bytestring = bitvalue.to_bytes(numbytes, 'little')
        self.put(int(stringstart), int(stringend), self.out_binary, [0, bytestring])

    def decode(self):
        while True:
            self.decode_run()
