import threading
from dataclasses import dataclass, asdict
from obd import OBD, OBDResponse, commands, OBDStatus
import board
import adafruit_adxl34x
import os
import time
import numpy as np
import copy
import logging
import tkinter as tk
from tkinter import font

logger = logging.getLogger("obd")
logger.addHandler(logging.NullHandler())

ODB_ADDRESS = "66:1E:31:00:B4:76"
RF_CHANNEL = "66"
REFRESH_INTERVAL = 0.25
ACCEL_CALIBRATION_INTERVAL = 10

@dataclass
class CollectedData:
    acc_x: float = 0
    acc_y: float = 0
    acc_z: float = 0

    voltage: OBDResponse = None        # "CONTROL_MODULE_VOLTAGE"
    speed: OBDResponse = None          # "SPEED"
    rpm: OBDResponse = None            # "RPM"
    coolant_temp: OBDResponse = None   # "COOLANT_TEMP"
    intake_temp: OBDResponse = None    # "INTAKE_TEMP"
    throttle_pos: OBDResponse = None   # "THROTTLE_POS"


def obd_worker(lock, data):
    os.system(f'rfcomm bind {RF_CHANNEL} {ODB_ADDRESS}')
    obd = OBD(f"/dev/rfcomm{RF_CHANNEL}", baudrate=38600)
    
    obd_items = {
        "voltage": "CONTROL_MODULE_VOLTAGE",
        "speed": "SPEED",
        "rpm": "RPM",
        "coolant_temp": "COOLANT_TEMP",
        "intake_temp": "INTAKE_TEMP",
        "throttle_pos": "THROTTLE_POS"
    }

    while True:
        start = time.time()
        for result_name, cmd_name in obd_items.items():
            while not obd.is_connected():
                obd = OBD(f"/dev/rfcomm{RF_CHANNEL}", baudrate=38600)
            response = obd.query(commands[cmd_name])
            with lock:
                setattr(data, result_name, response)
        end = time.time()
        duration = end - start
        remain = REFRESH_INTERVAL - duration
        if remain > 0:
            time.sleep(remain)


acc_calibration_basis = np.array(
    [[1, 0, 0],
     [0, 1, 0],
     [0, 0, 1]], dtype=float
)

def rebasis_acc(ax, ay, az):
    a = np.vstack((ax, ay, az))
    a_rebased = acc_calibration_basis @ a
    ax, ay, az = a_rebased[0], a_rebased[1], a_rebased[2]
    return ax, ay, az


def identify_gravity_axis():
    ax, ay, az = 0, 0, 0
    count = 0

    i2c = board.I2C()
    accelerometer = adafruit_adxl34x.ADXL345(i2c)
    accelerometer.data_rate = adafruit_adxl34x.DataRate.RATE_12_5_HZ
    accelerometer.range = adafruit_adxl34x.Range.RANGE_2_G

    start = time.time()
    while True:
        ax_, ay_, az_ = accelerometer.acceleration
        ax += ax_
        ay += ay_
        az += az_
        count += 1
        time.sleep(REFRESH_INTERVAL)
        if time.time() - start > ACCEL_CALIBRATION_INTERVAL:
            break

    ax, ay, az = ax/count, ay/count, az/count
    return az, ay, az


def identify_forward_axis(gravity):
    gx, gy, gz = gravity
    ax, ay, az = 0, 0, 0
    count = 0

    i2c = board.I2C()
    accelerometer = adafruit_adxl34x.ADXL345(i2c)
    accelerometer.data_rate = adafruit_adxl34x.DataRate.RATE_12_5_HZ
    accelerometer.range = adafruit_adxl34x.Range.RANGE_2_G

    start = time.time()
    while True:
        ax_, ay_, az_ = accelerometer.acceleration
        ax += ax_
        ay += ay_
        az += az_
        count += 1
        time.sleep(REFRESH_INTERVAL)
        if time.time() - start > ACCEL_CALIBRATION_INTERVAL:
            break

    ax, ay, az = ax/count, ay/count, az/count
    ax, ay, az = ax-gx, ay-gy, az-gz

    gravity = np.array(gravity)
    forward = np.array((ax, ay, az))
    forward_gravity = (np.dot(gravity, forward) / np.dot(gravity, gravity)) * gravity
    forward_non_gravity = forward - forward_gravity
    return forward_non_gravity


def do_calibration():
    input("now identifying gravity, press any key if you are ready")
    gravity = identify_gravity_axis()
    input("now identifying forward, press any key if you are ready")
    forward = identify_forward_axis(gravity)
    print("identifying done")
    gravity =  np.array(gravity)
    forward = np.array(forward)
    gravity = gravity / np.linalg.norm(gravity)
    forward = forward / np.linalg.norm(forward)
    right = np.cross(gravity, forward)
    calibration_matrix = np.vstack((gravity, forward, right))
    print(calibration_matrix)
    return calibration_matrix


def accel_worker(lock, data):
    i2c = board.I2C()
    accelerometer = adafruit_adxl34x.ADXL345(i2c)
    accelerometer.data_rate = adafruit_adxl34x.DataRate.RATE_12_5_HZ
    accelerometer.range = adafruit_adxl34x.Range.RANGE_2_G

    while True:
        start = time.time()
        ax, ay, az = accelerometer.acceleration
        ax, ay, az = rebasis_acc(ax, ay, az)
        with lock:
            data.acc_x = float(ax)
            data.acc_y = float(ay)
            data.acc_z = float(az)
        end = time.time()
        duration = end - start
        remain = REFRESH_INTERVAL - duration
        if remain > 0:
            time.sleep(remain)

def display_worker2(lock, data):
    while True:
        with lock:
            frozen_data = copy.deepcopy(data)
        print(frozen_data)
        time.sleep(REFRESH_INTERVAL)

def display_worker(lock, data):
    # while True:
    #     with lock:
    #         frozen_data = copy.deepcopy(data)
    #     print(frozen_data)
    #     time.sleep(REFRESH_INTERVAL)
    keys = list(asdict(data).keys())
    root = tk.Tk()
    root.title("OBD")
    def update_text_contents():
        with lock:
            frozen_data = asdict(copy.deepcopy(data))
        for i, text_widget in enumerate(text_widgets):
            text_widget.delete("1.0", tk.END)
            if i < len(keys):
                key = keys[i]
                value = frozen_data[key]
                if isinstance(value, OBDResponse):
                    value = value.value
                if isinstance(value, float):
                    text_widget.insert(tk.END, f"{key}\n", "key")
                    text_widget.insert(tk.END, f"{value:.2f}\n", "value")
                else:
                    text_widget.insert(tk.END, f"{key}\n", "key")
                    text_widget.insert(tk.END, f"{value}\n", "value")
            else:
                text_widget.insert(tk.END, "")
        root.after(int(1000*REFRESH_INTERVAL), update_text_contents)
    text_widgets = []
    for row in range(3):
        for col in range(3):
            text = tk.Text(root, width=32, height=8)
            text.grid(row=row, column=col)
            text_font = font.Font(family="Helvetica", size=48)
            text_font2 = font.Font(family="Helvetica", size=24)
            text.tag_configure("key", font=text_font2)
            text.tag_configure("value", font=text_font)
            text_widgets.append(text)
    update_text_contents()
    root.mainloop()


def main():
    # acc_calibration_basis = do_calibration()
    
    data = CollectedData()
    lock = threading.Lock()

    threads = [
        threading.Thread(target=obd_worker, args=(lock, data)),
        threading.Thread(target=accel_worker, args=(lock, data)),
        threading.Thread(target=display_worker, args=(lock, data))
    ]

    for thread in threads:
        thread.start()

    for thread in threads:
        thread.join()

if __name__ == '__main__':
    main()


