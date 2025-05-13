import threading
import math
from dataclasses import dataclass, asdict
from obd import OBD, OBDResponse, commands, OBDStatus, Unit
import board
import adafruit_adxl34x
import os
import time
import numpy as np
import copy
import logging
import tkinter as tk
from tkinter import font
import json

logger = logging.getLogger("obd")
logger.addHandler(logging.NullHandler())

ODB_ADDRESS = "66:1E:31:00:B4:76"
RF_CHANNEL = "66"
REFRESH_INTERVAL = 0.1
LOGGING_INTERVAL = 0.1
ACCEL_CALIBRATION_INTERVAL = 10
LOGGING_DIR = "/home/changhai/pyobd/obdlog"
logging_now = False
darkmode = False

contrast_color_dict = {
    "purple": "yellow",
    "blue": "white",
    "green": "black",
    "orange": "black",
    "red": "blue"
}

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


def log_worker(lock, data):
    global logging_now
    while True:
        if not logging_now:
            time.sleep(1)
            continue
        def _serialize():
            with lock:
                frozen_data = asdict(copy.deepcopy(data))
                for key in frozen_data.keys():
                    value = frozen_data[key]
                    if isinstance(value, OBDResponse):
                        value = value.value
                    if isinstance(value, Unit.Quantity):
                        value = value.magnitude
                    frozen_data[key] = value
                return frozen_data
            return None

        date_time = time.strftime("%Y%m%d_%H%M%S", time.localtime())
        os.makedirs(LOGGING_DIR, exist_ok=True)
        log_file = os.path.join(LOGGING_DIR, f"{date_time}.obdlog")
        init = time.time()
        last = time.time()
        counter = 0
        with open(log_file, "w") as f:
            while logging_now:
                now = time.time()
                serialized = _serialize()
                if serialized is None:
                    continue
                serialized["timestamp"] = (now - init) * 1e6
                f.write(json.dumps(serialized) + "\n")
                counter += 1
                if counter % 100 == 0:
                    f.flush()
                    os.fsync(f.fileno())
                remains = LOGGING_INTERVAL - (now - last)
                if remains > 0:
                    time.sleep(remains)
                last = now


def obd_worker(lock, data):
    if False:
        os.system(f'rfcomm bind {RF_CHANNEL} {ODB_ADDRESS}')
    obd = OBD(f"/dev/ttyUSB0", baudrate=500000)
    
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
            retry = 5
            while not obd.is_connected():
            # while False:
                obd = OBD(f"/dev/ttyUSB0", baudrate=500000)
                retry -= 1
                if retry <= 0:
                    break
                # print(obd.status())
            response = obd.query(commands[cmd_name])
            with lock:
                setattr(data, result_name, response)
            time.sleep(0.2)
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
        if abs(ax-10) < 1e-3:
            continue
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
    def logging_button_handler():
        global logging_now
        if logging_now:
            button_loggin.config(text="Logging")
            logging_now = False
        else:
            button_loggin.config(text="Stop")
            logging_now = True
    def darkmode_button_handler():
        global darkmode
        if darkmode:
            root.tk_setPalette(background="white")
            darkmode = False
        else:
            root.tk_setPalette(background="black")
            darkmode = True
            
    def update():
        coolent_temp_color()
        update_text_contents()
        root.after(int(1000*REFRESH_INTERVAL), update)
    def coolent_temp_color():
        with lock:
            frozen_data = asdict(copy.deepcopy(data))
        color = "purple"
        temp = frozen_data["coolant_temp"]
        if temp is None:
            color = "purple"
        elif temp.value is None:
            color = "purple"
        elif temp.value < 60:
            color = "blue"
        elif temp.value < 95:
            color = "green"
        elif temp.value < 110:
            color = "orange"
        else:
            color = "red"
        for i, text_widget in enumerate(text_widgets):
            key = keys[i]
            if not key == "coolant_temp":
                continue
            text_widget.configure(bg=color, fg=contrast_color_dict[color])
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
                if isinstance(value, Unit.Quantity):
                    value = value.magnitude
                if isinstance(value, float):
                    text_widget.insert(tk.END, f"{key}\n", "key")
                    text_widget.insert(tk.END, f"{value:.2f}\n", "value")
                else:
                    text_widget.insert(tk.END, f"{key}\n", "key")
                    text_widget.insert(tk.END, f"{value}\n", "value")
            else:
                text_widget.insert(tk.END, "")
    text_widgets = []
    for row in range(3):
        for col in range(3):
            text = tk.Text(root, width=32, height=6)
            text.grid(row=row, column=col)
            text_font = font.Font(family="Helvetica", size=48)
            text_font2 = font.Font(family="Helvetica", size=24)
            text.tag_configure("key", font=text_font2)
            text.tag_configure("value", font=text_font)
            text_widgets.append(text)
    button_loggin = tk.Button(root, text="Logging", command=logging_button_handler, width=1, height=4, font=("Helvetica", 24))
    button_loggin.grid(row=3, column=0)
    button_darkmode = tk.Button(root, text="DarkMode", command=darkmode_button_handler, width=1, height=4, font=("Helvetica", 24))
    button_darkmode.grid(row=3, column=1)
    button_3 = tk.Button(root, text="N/A", width=1, height=4)
    button_3.grid(row=3, column=2)
    update()
    root.mainloop()


def main():
    # acc_calibration_basis = do_calibration()
    
    data = CollectedData()
    lock = threading.Lock()

    threads = [
        threading.Thread(target=obd_worker, args=(lock, data)),
        threading.Thread(target=accel_worker, args=(lock, data)),
        threading.Thread(target=display_worker, args=(lock, data)),
        threading.Thread(target=log_worker, args=(lock, data)),
    ]

    for thread in threads:
        thread.start()

    for thread in threads:
        thread.join()

if __name__ == '__main__':
    main()


