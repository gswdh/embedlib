import serial
import click
import os
import multiprocessing as mp
import json


class ProgressBar:
    def __init__(self, width, low_limit, high_limit, label, units):
        self.width = width - 40
        self.low_limit = low_limit
        self.high_limit = high_limit
        self.value = low_limit
        self.label = label
        self.units = units

    def update(self, new_value):
        self.value = new_value

    def __str__(self):
        value = self.value
        if value < self.low_limit:
            value = self.low_limit
        if value > self.high_limit:
            value = self.high_limit
        normalized_value = (value - self.low_limit) / (self.high_limit - self.low_limit)
        bar_width = int(self.width * normalized_value)
        bar = "[" + "=" * bar_width + " " * (self.width - bar_width) + "]"
        label_padded = f"{self.label: <20}"
        value_padded = f"{(str(round(self.value, 3)) + ' ' + self.units): <15}"
        return f"{label_padded}{bar} {value_padded}"


def clear_screen():
    os.system("clear")


def port_task(port: str, baudrate: int, data_queue: mp.Queue):
    ser = serial.Serial(port=port, baudrate=baudrate, timeout=1)
    data = b""
    eol = b"***endjson***"
    while True:
        data += ser.read(1)
        if data[-len(eol) :] == eol:
            data = data.strip(eol)
            try:
                packet = json.loads(data.decode())
            except:
                pass
            else:
                data_queue.put(packet)
            data = b""


def display_task(data_queue: mp.Queue):
    objects = {}
    log_queue = []
    objects["log_queue"] = log_queue
    while True:
        objects["terminal_dims"] = os.get_terminal_size()
        data = data_queue.get()
        if data:
            if "create_bar" in data.keys():
                objects[data["create_bar"]["name"]] = ProgressBar(
                    objects["terminal_dims"][0],
                    data["create_bar"]["low_limit"],
                    data["create_bar"]["high_limit"],
                    data["create_bar"]["name"],
                    data["create_bar"]["units"],
                )
            if "set_bar" in data.keys():
                try:
                    objects[data["set_bar"]["name"]].update(data["set_bar"]["value"])
                except:
                    pass
            if "log" in data.keys():
                objects["log_queue"].append(data["log"])
                objects["log_queue"] = objects["log_queue"][-1000:]

            update_display(objects)


def update_display(objects: dict):
    clear_screen()
    print("*" * objects["terminal_dims"][0])
    n_lines = 1
    for key in objects.keys():
        if "log_queue" in key:
            continue
        if "terminal_dims" in key:
            continue
        print(objects[key])
        n_lines += 1
    print("*" * objects["terminal_dims"][0])
    n_lines += 1
    for log in objects["log_queue"][-(objects["terminal_dims"][1] - n_lines - 3) :]:
        print(log, end="")
    print("\u001b[37m", end="")
    print("*" * objects["terminal_dims"][0])


@click.command()
@click.option("--port", type=str, help="Serial port name")
@click.option("--baud", default=115200, type=int, help="Serial port baudrate")
def main(port, baud):
    ser = serial.Serial(port=port, baudrate=baud)
    while True:
        line = ser.readline()
        print(line.decode(), end="")


if __name__ == "__main__":
    main()
