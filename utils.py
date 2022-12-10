import time


def reading_log_files(filename):
    with open(filename, "r") as f:
        data = f.read().splitlines()
    return data


def log_generator(filename, period=0.5):
    data = reading_log_files(filename)
    while True:
        time.sleep(period)
        new_data = reading_log_files(filename)
        yield new_data[len(data):]
        data = new_data