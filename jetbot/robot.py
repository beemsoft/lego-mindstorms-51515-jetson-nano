#!/usr/bin/env python3
#
# This file is part of the MicroPython project, http://micropython.org/
#
# The MIT License (MIT)
#
# Copyright (c) 2020 Hans Beemsterboer
# This file has been modified by Hans Beemsterboer to be used in the
# lego-mindstorms-51515-jetson-nano project.
# Original file:
# https://github.com/micropython/micropython/blob/master/tools/pyboard.py
#
# Copyright (c) 2014-2019 Damien P. George
# Copyright (c) 2017 Paul Sokolovsky
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
# THE SOFTWARE.

import sys
import time
import traitlets
from traitlets.config.configurable import SingletonConfigurable


class PyboardError(Exception):
    pass


class Pyboard:
    no_follow = False
    follow = False

    def __init__(self, device, baudrate=115200, wait=0):
        print('Initialize Pyboard')
        self.use_raw_paste = True
        import serial

        delayed = False
        for attempt in range(wait + 1):
            try:
                self.serial = serial.Serial(device, baudrate=baudrate, interCharTimeout=1)
                break
            except (OSError, IOError):  # Py2 and Py3 have different errors
                if wait == 0:
                    continue
                if attempt == 0:
                    sys.stdout.write("Waiting {} seconds for pyboard ".format(wait))
                    delayed = True
            time.sleep(1)
            sys.stdout.write(".")
            sys.stdout.flush()
        else:
            if delayed:
                print("")
            raise PyboardError("failed to access " + device)
        if delayed:
            print("")

    def close(self):
        self.serial.close()

    def read_until(self, min_num_bytes, ending, timeout=10, data_consumer=None):
        # if data_consumer is used then data is not accumulated and the ending must be 1 byte long
        assert data_consumer is None or len(ending) == 1

        data = self.serial.read(min_num_bytes)
        if data_consumer:
            data_consumer(data)
        timeout_count = 0
        while True:
            if data.endswith(ending):
                break
            elif self.serial.inWaiting() > 0:
                new_data = self.serial.read(1)
                if data_consumer:
                    data_consumer(new_data)
                    data = new_data
                else:
                    data = data + new_data
                timeout_count = 0
            else:
                timeout_count += 1
                if timeout is not None and timeout_count >= 100 * timeout:
                    break
                time.sleep(0.01)
        return data

    def enter_raw_repl(self):
        self.serial.write(b"\r\x03\x03")  # ctrl-C twice: interrupt any running program

        # flush input (without relying on serial.flushInput())
        n = self.serial.inWaiting()
        while n > 0:
            self.serial.read(n)
            n = self.serial.inWaiting()

        self.serial.write(b"\r\x01")  # ctrl-A: enter raw REPL
        data = self.read_until(1, b"raw REPL; CTRL-B to exit\r\n>")
        if not data.endswith(b"raw REPL; CTRL-B to exit\r\n>"):
            print(data)
            raise PyboardError("could not enter raw repl")

        self.serial.write(b"\x04")  # ctrl-D: soft reset
        data = self.read_until(1, b"soft reboot\r\n")
        if not data.endswith(b"soft reboot\r\n"):
            print(data)
            raise PyboardError("could not enter raw repl")
        # By splitting this into 2 reads, it allows boot.py to print stuff,
        # which will show up after the soft reboot and before the raw REPL.
        data = self.read_until(1, b"raw REPL; CTRL-B to exit\r\n")
        if not data.endswith(b"raw REPL; CTRL-B to exit\r\n"):
            print(data)
            raise PyboardError("could not enter raw repl")

    def exit_raw_repl(self):
        self.serial.write(b"\r\x02")  # ctrl-B: enter friendly REPL

    def follow(self, timeout, data_consumer=None):
        # wait for normal output
        data = self.read_until(1, b"\x04", timeout=timeout, data_consumer=data_consumer)
        if not data.endswith(b"\x04"):
            raise PyboardError("timeout waiting for first EOF reception")
        data = data[:-1]

        # wait for error output
        data_err = self.read_until(1, b"\x04", timeout=timeout)
        if not data_err.endswith(b"\x04"):
            raise PyboardError("timeout waiting for second EOF reception")
        data_err = data_err[:-1]

        # return normal and error output
        return data, data_err

    def raw_paste_write(self, command_bytes):
        # Read initial header, with window size.
        data = self.serial.read(2)
        window_size = data[0] | data[1] << 8
        window_remain = window_size

        # Write out the command_bytes data.
        i = 0
        while i < len(command_bytes):
            while window_remain == 0 or self.serial.inWaiting():
                data = self.serial.read(1)
                if data == b"\x01":
                    # Device indicated that a new window of data can be sent.
                    window_remain += window_size
                elif data == b"\x04":
                    # Device indicated abrupt end.  Acknowledge it and finish.
                    self.serial.write(b"\x04")
                    return
                else:
                    # Unexpected data from device.
                    raise PyboardError("unexpected read during raw paste: {}".format(data))
            # Send out as much data as possible that fits within the allowed window.
            b = command_bytes[i: min(i + window_remain, len(command_bytes))]
            self.serial.write(b)
            window_remain -= len(b)
            i += len(b)

        # Indicate end of data.
        self.serial.write(b"\x04")

        # Wait for device to acknowledge end of data.
        data = self.read_until(1, b"\x04")
        if not data.endswith(b"\x04"):
            raise PyboardError("could not complete raw paste: {}".format(data))

    def exec_raw_no_follow(self, command):
        if isinstance(command, bytes):
            command_bytes = command
        else:
            command_bytes = bytes(command, encoding="utf8")

        # check we have a prompt
        data = self.read_until(1, b">")
        if not data.endswith(b">"):
            raise PyboardError("could not enter raw repl")

        if self.use_raw_paste:
            # Try to enter raw-paste mode.
            self.serial.write(b"\x05A\x01")
            data = self.serial.read(2)
            if data == b"R\x00":
                # Device understood raw-paste command but doesn't support it.
                pass
            elif data == b"R\x01":
                # Device supports raw-paste mode, write out the command using this mode.
                return self.raw_paste_write(command_bytes)
            else:
                # Device doesn't support raw-paste, fall back to normal raw REPL.
                data = self.read_until(1, b"w REPL; CTRL-B to exit\r\n>")
                if not data.endswith(b"w REPL; CTRL-B to exit\r\n>"):
                    print(data)
                    raise PyboardError("could not enter raw repl")
            # Don't try to use raw-paste mode again for this connection.
            self.use_raw_paste = False

        # Write command using standard raw REPL, 256 bytes every 10ms.
        for i in range(0, len(command_bytes), 256):
            self.serial.write(command_bytes[i: min(i + 256, len(command_bytes))])
            time.sleep(0.01)
        self.serial.write(b"\x04")

        # check if we could exec command
        data = self.serial.read(2)
        if data != b"OK":
            raise PyboardError("could not exec command (response: %r)" % data)

    def exec_raw(self, command, timeout=10, data_consumer=None):
        self.exec_raw_no_follow(command)
        return self.follow(timeout, data_consumer)

    def eval(self, expression):
        ret = self.exec_("print({})".format(expression))
        ret = ret.strip()
        return ret

    def exec_(self, command, data_consumer=None):
        ret, ret_err = self.exec_raw(command, data_consumer=data_consumer)
        if ret_err:
            raise PyboardError("exception", ret, ret_err)
        return ret

    def execfile(self, filename):
        with open(filename, "rb") as f:
            pyfile = f.read()
        return self.exec_(pyfile)


class Motor(SingletonConfigurable):

    value = traitlets.Float()

    def __init__(self, pyb, motor_id, **kwargs):
        super(Motor, self).__init__(**kwargs)
        self.pyb = pyb
        self.motor_id = motor_id
        self._actual = 0

    @traitlets.observe('value')
    def _observe_value(self, change):
        self._write_value(change['new'])

    def _write_value(self, value):
        print ("value changed to", value)
        self._actual = value
        if value > 0.1 or value < -0.1:
            command = f"""
from mindstorms import Motor
motor = Motor('{self.motor_id}')
motor.start(int(100*float('{value}')))"""
        else:
            command = f"""
from mindstorms import Motor
motor = Motor('{self.motor_id}')
motor.stop()"""
        self.pyb.exec_(command)


class Robot(SingletonConfigurable):
    device = '/dev/rfcomm0'
    baudrate = 115200
    wait = 0

    left_motor = traitlets.Instance(Motor)
    right_motor = traitlets.Instance(Motor)

    def __init__(self, **kwargs):
        super(Robot, self).__init__(**kwargs)
        self.pyb = None
        try:
            self.pyb = Pyboard(self.device, self.baudrate, self.wait)
            try:
                self.pyb.enter_raw_repl()
                self.left_motor = Motor(self.pyb, 'A')
                self.right_motor = Motor(self.pyb, 'B')

            except PyboardError as er:
                print(er)
                self.pyb.close()
                sys.exit(1)
            command = """from mindstorms import MSHub
hub = MSHub()
hub.light_matrix.write("Hi!")"""
            self.pyb.exec_(command)
            print('Hello from the Mindstorms robot!')
        except PyboardError as er:
            print(er)
            sys.exit(1)

    def set_motors(self, left_speed, right_speed):
        self.move_robot(left_speed, right_speed)

    def move_robot(self, left_speed, right_speed):
        command = f"""from mindstorms import MotorPair
motorEF = MotorPair('A','B')
motorEF.start_tank(int(100*float('{left_speed}')),int(100*float('{right_speed}')))"""
        self.pyb.exec_(command)

    def forward(self, speed=1.0):
        self.move_robot(speed, speed)

    def backward(self, speed=1.0):
        self.move_robot(-speed, -speed)

    def left(self, speed=1.0):
        self.move_robot(-speed, speed)

    def right(self, speed=1.0):
        self.move_robot(speed, -speed)

    def stop(self):
        command = """from mindstorms import MotorPair
motorEF = MotorPair('A','B')
motorEF.stop()"""
        self.pyb.exec_(command)
