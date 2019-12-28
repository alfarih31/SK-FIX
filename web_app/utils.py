import subprocess
import os
import socket
from datetime import datetime
from time import sleep
import usb_mount as um

_device = ""

def shell_source(script):
    """Sometime you want to emulate the action of "source" in bash,
    settings some environment variables. Here is a way to do it."""
    pipe = subprocess.Popen(". %s; env" % script, stdout=subprocess.PIPE, shell=True)
    output = pipe.communicate()[0]
    lines = output.decode("utf-8").split("\n")
    env = {}
    for line in lines:
        line = line.split("=")
        k = line[0]
        if len(k) == 0:
            continue
        v = "=".join(line[1:])
        env[k] = v
    os.environ.update(env)

def launch_ouster():
    roslaunch_file = "livox_lidar.launch"
    process = subprocess.Popen(['roslaunch', 'livox_ros_driver', roslaunch_file])
    return process

def launch_dji():
    process = subprocess.Popen(['python3', '/home/alfarihfz/Projects/SK-FIX/main_local.py'])
    return process

def launch_record(device, project_name):
    device.mount()
    time = datetime.now().strftime('%Y-%m-%d-%H-%M-%S')
    save_location = "%s/%s/"%(device.mount_point, project_name)
    os.makedirs(save_location, exist_ok=True)
    os.makedirs(save_location+"/"+time, exist_ok=True)
    roslaunch_args = "dir:=%s"%(save_location+time+"/logs")
    process = subprocess.Popen(['roslaunch', '/home/alfarihfz/Projects/SK-FIX/web_app/record.launch', roslaunch_args])
    return process

def get_progress(device):
    size_stat = device.size_status()
    while size_stat['percent'] < 0.95:
        left = size_stat['available']/1000000000.0
        sleep(1)
        size_stat = device.size_status()
        yield left, size_stat['total']

def check_dji():
    devices = os.popen("ls /dev/ttyUSB* | grep ttyUSB*").read()
    device = devices.split("/n")[0].strip()
    if device:
        global _device
        _device = device
        return True
    else:
        return True

def check_lidar():
    #Check OusterLidar
    if os.system("ping -c 1 -W 1 192.168.0.144 >/dev/null") == 0:
        return True
    else:
        return True

def check_usb():
    #Check USB Flashdisk
    USB_devices = um.USBDevices()
    if not USB_devices.devices:
        return None, False
    else:
        device = USB_devices.devices[0]
        device.mount()
        if device.size_status()["percent"] < 0.95:
            return device, True
        else:
            return None, False

def check_device():
    checker = {
        'I': check_dji(),
        'L': check_lidar(),
        'M': check_usb()[1]
    }
    return checker
