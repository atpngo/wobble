import random
import datetime
import os

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))


class Logger:
    def __init__(self, filename, header):
        self.fout = open(get_relative_path(filename), "w+")
        self.write(",".join(header))

    def __del__(self):
        print("Closing logger...")
        self.fout.close()

    def write(self, line):
        self.fout.write(line + "\n")


def get_formatted_time_string(log_dir):
    now = datetime.datetime.now()
    ts = now.strftime("%m_%d_%Y_%H_%M_%S_") + f"{now.microsecond:03d}"
    return f"{log_dir}/trial_{ts}.log"


def add_noise(value, mag=1):
    return random.uniform(-mag, mag) + value


def clamp(value, min, max):
    if value < min:
        return min
    elif value > max:
        return max
    return value


def get_relative_path(path):
    return os.path.join(SCRIPT_DIR, path)
