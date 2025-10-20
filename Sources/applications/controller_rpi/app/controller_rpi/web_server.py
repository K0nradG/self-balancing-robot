from flask import Flask, request, jsonify, render_template_string, Response
import subprocess, threading, re, os

app = Flask(__name__)

UPLOAD_FOLDER = "uploads"
DFU_SCRIPT = "dfu_ble.py"
os.makedirs(UPLOAD_FOLDER, exist_ok=True)

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


def run_dfu(path):
    import sys

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


@app.route("/")
def index():
    return render_template_string(open("templates/index.html").read())


@app.route("/upload", methods=["POST"])
def upload():
    f = request.files["file"]
    if not f:
        return jsonify({"error": "no file"}), 400
    path = os.path.join(UPLOAD_FOLDER, f.filename)
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


if __name__ == "__main__":
    app.run(host="0.0.0.0", port=5000, debug=False)
