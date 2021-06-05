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
import os
import cv2
import face_recognition
import numpy as np
import speech_recognition as sr
from gtts import gTTS

video_capture = None
device = '/dev/rfcomm0'
baudrate = 115200
no_follow = False
follow = False
wait = 0

speak = False
name = ''
nameOld = ''
video_capture = None
known_face_encodings = []
known_face_names = []
recognizer = sr.Recognizer()

try:
    stdout = sys.stdout.buffer
except AttributeError:
    # Python2 doesn't have buffer attr
    stdout = sys.stdout


def stdout_write_bytes(b):
    b = b.replace(b"\x04", b"")
    stdout.write(b)
    stdout.flush()


class PyboardError(Exception):
    pass


class Pyboard:
    def __init__(self, device, baudrate=115200, wait=0):
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


# in Python2 exec is a keyword so one must use "exec_"
# but for Python3 we want to provide the nicer version "exec"
setattr(Pyboard, "exec", Pyboard.exec_)


def say_text(pyb, sentence, display):
    global speak
    speak = True
    output = gTTS(text=sentence, lang='en', slow=False)
    output.save('output.mp3')
    os.system('mpg123 output.mp3')
    if display:
        command = f"""\
from mindstorms import MSHub
hub = MSHub()
hub.light_matrix.write('{sentence}')
"""
        pyb.exec(command)
    speak = False


def ask_name(pyb):
    global name
    say_text(pyb, 'I see a new face! What\'s your name?', False)
    try:
        with sr.Microphone() as source:
            print("Say something!")
            audio = recognizer.listen(source, timeout=5)
            name = recognizer.recognize_google(audio, None, "en")
            print("Google Speech Recognition thinks you said " + name)
            return name
    except sr.WaitTimeoutError:
        print("Google Speech Recognition timeout")
        say_text(pyb, 'Please repeat that in the mic', False)
        return ask_name(pyb)
    except sr.UnknownValueError:
        print("Google Speech Recognition unknown value")
        say_text(pyb, 'Please repeat that in the mic', False)
        return ask_name(pyb)
    except sr.RequestError as e:
        say_text(pyb, 'I could not recognize that', False)
        print("Could not request results from Google Speech Recognition service; {0}".format(e))


def get_jetson_gstreamer_source(capture_width=1280, capture_height=720, display_width=1280, display_height=720,
                                framerate=60, flip_method=0):
    """
    Return an OpenCV-compatible video source description that uses gstreamer to capture video from the RPI camera on a Jetson Nano
    """
    return (
            f'nvarguscamerasrc ! video/x-raw(memory:NVMM), ' +
            f'width=(int){capture_width}, height=(int){capture_height}, ' +
            f'format=(string)NV12, framerate=(fraction){framerate}/1 ! ' +
            f'nvvidconv flip-method={flip_method} ! ' +
            f'video/x-raw, width=(int){display_width}, height=(int){display_height}, format=(string)BGRx ! ' +
            'videoconvert ! video/x-raw, format=(string)BGR ! appsink'
    )


def train_faces():
    global known_face_encodings
    global known_face_names
    print("Training for face recognition")
    harry_image = face_recognition.load_image_file("harry.jpg")
    harry_face_encoding = face_recognition.face_encodings(harry_image)[0]
    legolas_image = face_recognition.load_image_file("legolas.jpeg")
    legolas_face_encoding = face_recognition.face_encodings(legolas_image)[0]
    known_face_encodings = [harry_face_encoding, legolas_face_encoding]
    known_face_names = ["Harry Potter", "Legolas"]


def shake_arm(pyb):
    command = """from mindstorms import MotorPair
import math
motorEF = MotorPair('E','F')
motorEF.move(math.pi, 'cm', steering=80)
motorEF.move(-math.pi, 'cm', steering=80)"""
    pyb.exec(command)


def add_face(pyb, face_encoding):
    new_name = ask_name(pyb)
    if new_name != "stop":
        known_face_encodings.append(face_encoding)
        known_face_names.append(new_name)
        say_text(pyb, 'Nice to meet you, ' + new_name, True)


def do_face_recognition(pyb):
    global speak
    global nameOld
    global video_capture
    process_this_frame = True

    # Accessing the camera with OpenCV on a Jetson Nano requires gstreamer with a custom gstreamer source string
    video_capture = cv2.VideoCapture(get_jetson_gstreamer_source(), cv2.CAP_GSTREAMER)

    while True:
        print('Process frame')
        # Grab a single frame of video
        ret, frame = video_capture.read()

        # Resize frame of video to 1/4 size for faster face recognition processing
        small_frame = cv2.resize(frame, (0, 0), fx=0.25, fy=0.25)

        # Convert the image from BGR color (which OpenCV uses) to RGB color (which face_recognition uses)
        rgb_small_frame = small_frame[:, :, ::-1]

        # Only process every other frame of video to save time
        # Also, don't process during speaking
        if process_this_frame and speak is False:
            # Find all the faces and face encodings in the current frame of video
            face_locations = face_recognition.face_locations(rgb_small_frame)
            face_encodings = face_recognition.face_encodings(rgb_small_frame, face_locations)

            for face_encoding in face_encodings:
                # See if the face is a match for the known face(s)
                matches = face_recognition.compare_faces(known_face_encodings, face_encoding)
                face_distances = face_recognition.face_distance(known_face_encodings, face_encoding)
                best_match_index = np.argmin(face_distances)
                if matches[best_match_index]:
                    known_face_name = known_face_names[best_match_index]
                    if known_face_name != nameOld:
                       shake_arm(pyb)
                       say_text(pyb, 'Hello, ' + known_face_name, True)
                    nameOld = known_face_name
                else:
                    add_face(pyb, face_encoding)

        process_this_frame = not process_this_frame


def main():
    try:
        pyb = Pyboard(device, baudrate, wait)
        try:
            pyb.enter_raw_repl()
        except PyboardError as er:
            print(er)
            pyb.close()
            sys.exit(1)

        train_faces()
        #say_text(pyb, 'Hi there!')
        do_face_recognition(pyb)
    except PyboardError as er:
        print(er)
        sys.exit(1)
    finally:
        video_capture.release()
        pyb.exit_raw_repl()
        pyb.close()


if __name__ == "__main__":
    main()
