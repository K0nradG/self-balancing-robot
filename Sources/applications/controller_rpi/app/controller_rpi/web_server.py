from flask import Flask, request, jsonify, render_template_string, Response
import subprocess, threading, re, os, asyncio
from nus_service import NUSClient
import zipfile
from camera_driver import CameraDriver
import time
import asyncio


trajectory_ack_event = asyncio.Event()

app = Flask(__name__)

UPLOAD_FOLDER = "uploads"
DFU_SCRIPT = "dfu_ble.py"
os.makedirs(UPLOAD_FOLDER, exist_ok=True)

trajectory_ack_event = asyncio.Event()

# --- DFU state ---
dfu = {
    "status": "idle",
    "percent": 0.0,
    "speed": 0.0,
    "current_step": "",
    "duration": 0.0,
    "avg_speed": 0.0,
    "size": 0.0
}
log_buffer = []

# --- NUS state ---
nus_client: NUSClient | None = None
nus_loop: asyncio.AbstractEventLoop | None = None
nus_log_buffer = []

# --- Asyncio loop w tle ---
def nus_loop_thread():
    global nus_loop
    nus_loop = asyncio.new_event_loop()
    asyncio.set_event_loop(nus_loop)
    nus_loop.run_forever()

threading.Thread(target=nus_loop_thread, daemon=True).start()

# --- DFU functions ---
def run_dfu(path):
    dfu.update({
        "status": "running",
        "percent": 0.0,
        "speed": 0.0,
        "current_step": "Starting DFU...",
        "duration": 0.0,
        "avg_speed": 0.0,
        "size": os.path.getsize(path) / 1024.0
    })
    log_buffer.clear()

    process = subprocess.Popen(
        ['python3', '-u', DFU_SCRIPT, path],
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        bufsize=1,
        universal_newlines=True
    )

    for line in iter(process.stdout.readline, ''):
        if not line.strip():
            continue
        log_buffer.append(line)
        print(line.strip(), flush=True)

        if "[DFU_STEP]" in line:
            dfu["current_step"] = line.strip().replace("[DFU_STEP]", "").strip()

        m = re.search(r"\s([0-9.]+)%\s+([0-9.]+)\s*KiB/s", line)
        if m:
            dfu["percent"] = float(m.group(1))
            dfu["speed"] = float(m.group(2))

        s = re.search(r"\[DFU_STATS\]\s*time=([\d.]+)s\s+avg_speed=([\d.]+)KiB/s", line)
        if s:
            dfu["duration"] = float(s.group(1))
            dfu["avg_speed"] = float(s.group(2))

    process.wait()

    if process.returncode == 0:
        dfu["status"] = "finished"
        dfu["percent"] = 100
        dfu["current_step"] = "DFU finished successfully"
    else:
        dfu["status"] = "error"
        dfu["current_step"] = "Error during DFU"


# --- NUS helpers ---
async def async_connect(addr):
    global nus_client
    nus_client = NUSClient(addr)

    def on_data(text):
        print(f"[NUS_LOG] {text}")
        nus_log_buffer.append(text)
        if len(nus_log_buffer) > 500:
            nus_log_buffer.pop(0)

    nus_client.on_data = on_data
    nus_client.on_trajectory_ack = lambda text: trajectory_ack_event.set()

    await nus_client.connect()

async def async_disconnect():
    if nus_client:
        await nus_client.disconnect()

async def async_send(data):
    if nus_client:
        await nus_client.send(data)

async def async_notify_on():
    if nus_client:
        await nus_client.notifications_on()

async def async_notify_off():
    if nus_client:
        await nus_client.notifications_off()


def run_in_nus_loop(coro):
    """Uruchamia coroutine w tle w tym samym loop Bleak"""
    return asyncio.run_coroutine_threadsafe(coro, nus_loop)


# --- Camera setup ---
camera = CameraDriver()

def generate_camera_stream():
    boundary = "--frame"
    while camera.running:
        frame = camera.get_frame()
        if not frame:
            time.sleep(0.05)
            continue
        yield (b"%s\r\nContent-Type: image/jpeg\r\nContent-Length: %d\r\n\r\n" %
               (boundary.encode(), len(frame)))
        yield frame
        yield b"\r\n"


# --- Routes ---
@app.route("/")
def index():
    return render_template_string(open("templates/index.html").read())

@app.route("/app")
def app_dashboard():
    return render_template_string(open("templates/app_dashboard.html").read())

@app.route("/motion")
def motion_page():
    return render_template_string(open("templates/motion.html").read())

@app.route("/trajectory")
def trajectory_page():
    return render_template_string(open("templates/trajectory.html").read())

# --- DFU endpoints ---
@app.route("/upload", methods=["POST"])
def upload():
    f = request.files.get("file")
    if not f:
        return jsonify({"error": "no file"}), 400

    filename = f.filename
    path = os.path.join(UPLOAD_FOLDER, filename)
    f.save(path)

    log_buffer.clear()
    dfu.update({
        "status": "idle",
        "percent": 0.0,
        "speed": 0.0,
        "current_step": "Idle",
        "duration": 0.0,
        "avg_speed": 0.0,
        "size": 0.0
    })

    extracted_bin = None
    if filename.lower().endswith(".zip"):
        try:
            with zipfile.ZipFile(path, 'r') as z:
                z.extractall(UPLOAD_FOLDER)
                for name in z.namelist():
                    if name.lower().endswith(".bin"):
                        extracted_bin = os.path.join(UPLOAD_FOLDER, name)
                        break
            if not extracted_bin or not os.path.exists(extracted_bin):
                return jsonify({"error": "no .bin file found in ZIP"}), 400
            os.remove(path)
            path = extracted_bin
        except Exception as e:
            return jsonify({"error": f"zip extract failed: {e}"}), 500

    return jsonify({"file_path": path})

@app.route("/start_dfu", methods=["POST"])
def start_dfu():
    path = request.json.get("file_path")
    if not path or not os.path.exists(path):
        return jsonify({"error": "file not found"}), 404
    threading.Thread(target=run_dfu, args=(path,), daemon=True).start()
    return jsonify({"status": "started"})

@app.route("/status")
def status():
    return jsonify(dfu)

@app.route("/logs")
def logs():
    return jsonify({"logs": log_buffer[-200:]})

@app.route("/download_logs")
def download_logs():
    logs = "".join(log_buffer)
    return Response(
        logs,
        mimetype="text/plain",
        headers={"Content-Disposition": "attachment;filename=dfu_logs.txt"}
    )

@app.route("/reset", methods=["POST"])
def reset():
    log_buffer.clear()
    dfu.update({
        "status": "idle",
        "percent": 0.0,
        "speed": 0.0,
        "current_step": "Idle",
        "duration": 0.0,
        "avg_speed": 0.0,
        "size": 0.0
    })
    return jsonify({"status": "reset"})


# --- NUS endpoints ---
@app.route("/nus/connect", methods=["POST"])
def nus_connect():
    addr = request.json.get("address")
    run_in_nus_loop(async_connect(addr))
    return jsonify({"status": "connecting"})

@app.route("/nus/disconnect", methods=["POST"])
def nus_disconnect():
    run_in_nus_loop(async_disconnect())
    return jsonify({"status": "disconnecting"})

@app.route("/nus/send", methods=["POST"])
def nus_send():
    data = request.json.get("data")
    run_in_nus_loop(async_send(data))
    return jsonify({"status": "sending"})

@app.route("/nus/notify_on", methods=["POST"])
def nus_notify_on():
    run_in_nus_loop(async_notify_on())
    return jsonify({"status": "notify_on"})

@app.route("/nus/notify_off", methods=["POST"])
def nus_notify_off():
    run_in_nus_loop(async_notify_off())
    return jsonify({"status": "notify_off"})

@app.route("/nus/logs")
def nus_logs():
    return jsonify({"logs": nus_log_buffer[-200:]})

@app.route("/nus/status")
def nus_status():
    if nus_client:
        return jsonify(nus_client.get_status())
    return jsonify({
        "connected": False,
        "notify_active": False,
        "address": None,
        "device_name": None
    })

# --- Trajectory endpoints ---

@app.route("/nus/wait_ack")
def wait_ack():
    """Czeka na ACK z trajektorii (blokujące, synchronizowane)"""
    try:
        fut = asyncio.run_coroutine_threadsafe(trajectory_ack_event.wait(), nus_loop)
        fut.result() 
        trajectory_ack_event.clear()
        return jsonify({"ack": True})
    except asyncio.TimeoutError:
        return jsonify({"ack": False})

# --- Motion endpoints ---
@app.route("/motion/data", methods=["POST"])
def motion_data():
    payload = request.json
    print(f"Otrzymano dane z telefonu: {payload}")
    return jsonify(status="ok")

@app.route("/camera/start", methods=["POST"])
def camera_start():
    camera.start()
    return jsonify({"status": "started"})

@app.route("/camera/stop", methods=["POST"])
def camera_stop():
    camera.stop()
    return jsonify({"status": "stopped"})

@app.route("/camera/stream")
def camera_stream():
    if not camera.running:
        return "Camera not started", 400
    return Response(generate_camera_stream(),
                    mimetype='multipart/x-mixed-replace; boundary=frame')

if __name__ == "__main__":
    app.run(host="0.0.0.0", port=5000, debug=False)
