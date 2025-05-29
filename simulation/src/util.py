import random


class Logger:
    def __init__(self, filename, header):
        self.fout = open(filename, "w+")
        self.write(",".join(header))

    def __del__(self):
        print("Closing logger...")
        self.fout.close()

    def write(self, line):
        self.fout.write(line + "\n")


def add_noise(value, mag=1):
    return random.uniform(-mag, mag) + value


def clamp(value, min, max):
    if value < min:
        return min
    elif value > max:
        return max
    return value
