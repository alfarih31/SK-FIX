import os
from time import sleep
import threading
from json import dumps, load
from flask_socketio import SocketIO, emit
from flask import Flask, render_template, send_from_directory
import utils as lidar
import signal
app = Flask(__name__)
socket = SocketIO(app=app, cors_allowed_origins="*")

with open('state.json', 'r') as f:
    SESSION = load(f)

SESSION['status'] = {'I': False,
                     'L': False,
                     'M': False}
process_pool = {"LIDAR": {},
                  "INS": {},
                  "MAPPING": {}}
is_first = True

def process_observer(onExit, fn, args):
    def runInThread(onExit, proc):
        code = proc.wait()
        onExit(code)
        return
    if args is None:
        proc = fn()
    else:
        proc = fn(*args)
    thread = threading.Thread(target=runInThread, args=(onExit, proc))
    thread.start()
    # returns immediately after the thread starts
    return {"process": proc, "thread": thread}

def check_device():
    global is_first
    status = lidar.check_device()
    for k, v in status.items():
        SESSION['status'][k] = v
        if is_first:
            SESSION["%s_S"%k] = v
    if is_first:
        is_first = False

@socket.on('connect')
def on_connect():
    if SESSION['ssid'] == '0':
        SESSION['ssid'] = '1'

@socket.on('req')
def on_data(msg):
    check_device()
    alert_msg = ""
    if not SESSION["L_STP"]:
        alert_msg += "LIDAR Not Ready\n"
    if not SESSION["I_STP"]:
        alert_msg += "INS Not Ready\n"
    if not SESSION["status"]["M"]:
        alert_msg += "USB Not Ready, Is there any USB? | is there space minimum 5%?\n"
    SESSION["WARN"] = False
#    SESSION["msg"] = alert_msg
    emit('res', dumps(SESSION), broadcast=True)

@socket.on('button')
def on_button(msg):
    SESSION["WARN"] = False
    if msg == 'L_S':
        status = SESSION["status"]["L"] = lidar.check_lidar()
        if status:
            def onProcessExit(code):
                SESSION["L_S"] = True
                SESSION["L_STP"] = False
                SESSION["msg"] = "Lidar driver has stopped"
                SESSION["WARN"] = True
                process_pool["LIDAR"] = {}
                socket.emit("res", dumps(SESSION), broadcast=True)
            process_pool["LIDAR"] = process_observer(onProcessExit, lidar.launch_ouster, None)
            SESSION["L_S"] = False
            SESSION["L_STP"] = True
        elif not status:
            SESSION["msg"] = "Lidar not found"
            SESSION["WARN"] = True
        socket.emit('res', dumps(SESSION), broadcast=True)
    elif msg == 'L_STP':
        if process_pool["LIDAR"]:
            process_pool["LIDAR"]["process"].terminate()
            process_pool["LIDAR"]["process"].wait()
            SESSION["L_S"] = True
            SESSION["L_STP"] = False
            process_pool["LIDAR"] = {}
            socket.emit('res', dumps(SESSION), broadcast=True)
    elif msg == "I_S":
        status = SESSION["status"]["I"] = lidar.check_dji()
        if status:
            def onProcessExit(code):
                SESSION["I_S"] = True
                SESSION["I_STP"] = False
                SESSION["msg"] = "INS driver has stopped"
                SESSION["WARN"] = True
                process_pool["INS"] = {}
                socket.emit("res", dumps(SESSION), broadcast=True)
            process_pool["INS"] = process_observer(onProcessExit, lidar.launch_dji, None)
            SESSION["I_S"] = False
            SESSION["I_STP"] = True
        elif not status:
            SESSION["msg"] = "INS not found"
            SESSION["WARN"] = True
        socket.emit('res', dumps(SESSION), broadcast=True)
    elif msg == "I_STP":
        if process_pool["INS"]:
            process_pool["INS"]["process"].terminate()
            process_pool["INS"]["process"].wait()
            process_pool["INS"] = {}
            SESSION["I_S"] = True
            SESSION["I_STP"] = False
            socket.emit('res', dumps(SESSION), broadcast=True)
    elif msg == "M_S":
        usb, status = lidar.check_usb()
        SESSION["status"]["M"] = status
        if status:
            def onRecordExit(code):
                print("INI CODENYA", code)
                SESSION["M_S"] = True
                SESSION["M_STP"] = False
                SESSION["msg"] = "Recording has stopped"
                SESSION["WARN"] = True
                process_pool["MAPPING"] = {}
                socket.emit("res", dumps(SESSION), broadcast=True)
            if not process_pool["MAPPING"]:
                if SESSION["name"].split(",")[0] == "Now Ready":
                    foldername = "Default"
                else:
                    foldername = SESSION["name"]
                process_pool["MAPPING"] = process_observer(onRecordExit, lidar.launch_record, (usb, foldername))
            SESSION["M_S"] = False
            SESSION["M_STP"] = True
            SESSION["WARN"] = False
            SESSION["SUBMIT"] = False
            print("HERHEHRHEHREHHHDEHDAD")
        else:
            alert_msg = ""
            if not SESSION["L_STP"]:
                alert_msg += "LIDAR Not Ready\n"
            if not SESSION["I_STP"]:
                alert_msg += "INS Not Ready\n"
            if not status:
                alert_msg += "USB Not Ready, Is there any USB? | is there space minimum 5%?\n"
            SESSION["WARN"] = True
            SESSION["msg"] = alert_msg
        socket.emit('res', dumps(SESSION), broadcast=True)
    elif msg == "M_STP":
        if process_pool["MAPPING"]:
            usb, _ = lidar.check_usb()
            process_pool["MAPPING"]["process"].send_signal(signal.SIGINT)
            process_pool["MAPPING"]["process"].wait()
            process_pool["MAPPING"] = {}
            SESSION["M_S"] = True
            SESSION["M_STP"] = False
            SESSION["SUBMIT"] = True
            SESSION["WARN"] = False
            usb.unmount()
            print("HADSHASKDKJASJDKASKJDJASNKJDASKDHKJHDKJBHASKJDBH")
            socket.emit('res', dumps(SESSION), broadcast=True)

@socket.on('cname')
def on_cname(msg):
    SESSION['name'] = msg
    SESSION["WARN"] = False
    emit('res', dumps(SESSION), broadcast=True)

@app.route('/')
def home():
    return render_template('index.html')

@app.route('/js/<path:path>')
def send_js(path):
    return send_from_directory('templates/js', path)

@app.route('/css/<path:path>')
def send_css(path):
    return send_from_directory('templates/css', path)

if __name__ == "__main__":
    try:
        os.system("/bin/bash -c source /home/alfarihfz/.bashrc")
#        lidar.shell_source("/home/odroid/roslidar-webapp/lidar_ws/devel/setup.sh")
        print('Start app on port 2000')
        #app.run(host='0.0.0.0', port=2000)
        socket.run(app, host='0.0.0.0', port=2000)
    except KeyboardInterrupt:
        with open('state.json', 'w') as f:
            f.write(dumps(SESSION))
