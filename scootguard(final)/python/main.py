import time, csv, os, stat, threading, json, datetime
from zoneinfo import ZoneInfo
HACKSTER_TZ = ZoneInfo("America/Los_Angeles")
from arduino.app_utils import App, Bridge
from arduino.app_bricks.web_ui import WebUI
from fastapi.responses import FileResponse
from edge_impulse_linux.runner import ImpulseRunner

try:
    import requests
except ImportError:
    requests = None
    print("[Warning] The requests module is not installed. Add requests to requirements.txt.")

ui = WebUI()

# Cloud integration settings (Firebase Realtime Database)
CLOUD_ENDPOINT = "https://e-scooter-safety-system-default-rtdb.firebaseio.com/sessions"

# Send the session report to the cloud if this idle time (seconds) persists after riding ends
IDLE_TIMEOUT_SEC = 30 * 60  # 30minutes

# Driving Style thresholds (based on unstable ratio, %)
# Planned to separate these settings for future user-specific adjustment.
DRIVING_STYLE_THRESHOLDS = [
    (2.0,  "Stable"),
    (5.0,  "Cautious"),
    (15.0, "Aggressive"),
]
# If all thresholds are exceeded or an actual ALERT occurs during the session, use "Unstable"

# ============== Session state (one ride = one session) ==============
session_active = False
session_start_time = None
idle_since = None

session_counts = {"normal": 0, "turn": 0, "unstable": 0}
confidence_sum = 0.0
confidence_count = 0


def classify_driving_style(unstable_pct, had_alert):
    """Assign a driving style label based on the unstable ratio and whether an actual alert occurred."""
    if had_alert:
        return "Unstable"
    for threshold, label in DRIVING_STYLE_THRESHOLDS:
        if unstable_pct < threshold:
            return label
    return "Unstable"


def reset_session_state():
    """Reset all accumulated values on both the Python and Arduino sides when a new ride starts."""
    global session_counts, confidence_sum, confidence_count, session_start_time

    session_counts = {"normal": 0, "turn": 0, "unstable": 0}
    confidence_sum = 0.0
    confidence_count = 0
    session_start_time = time.time()

    try:
        Bridge.call("reset_session_counters")
    except Exception as e:
        print("[Cloud] Failed to reset Arduino counters:", e)


def build_session_summary(end_time):
    """Collect data at the end of the session and build the JSON payload for cloud transmission."""

    total_windows = sum(session_counts.values())
    if total_windows > 0:
        normal_pct = round(session_counts["normal"] / total_windows * 100, 1)
        turn_pct = round(session_counts["turn"] / total_windows * 100, 1)
        unstable_pct = round(session_counts["unstable"] / total_windows * 100, 1)
    else:
        normal_pct = turn_pct = unstable_pct = 0.0

    avg_confidence = (
        round(confidence_sum / confidence_count, 2)
        if confidence_count > 0 else 0.0
    )

    alert_count = Bridge.call("get_alert_count")
    recheck_pass_count = Bridge.call("get_recheck_pass_count")
    decel_count = Bridge.call("get_decel_count")
    reaction_sum_ms = Bridge.call("get_reaction_time_sum")
    reaction_count = Bridge.call("get_reaction_time_count")

    avg_reaction_time_sec = (
        round((reaction_sum_ms / reaction_count) / 1000, 2)
        if reaction_count > 0 else None
    )

    # False positive estimate:
    # The ratio of ALERT cases that were confirmed normal (not alcohol-related) after remeasurement.
    # An empirical metric using the alcohol sensor remeasurement as the ground truth.
    false_positive_estimate = {
        "alert_count": alert_count,
        "recheck_pass_count": recheck_pass_count,
        "estimated_error_rate_pct": (
            round(recheck_pass_count / alert_count * 100, 1)
            if alert_count > 0 else None
        )
    }

    driving_style = classify_driving_style(unstable_pct, had_alert=(alert_count > 0))

    # Safety Score: A heuristic metric combining multiple indicators into a single 0-100 score.
    safety_score = 100.0
    safety_score -= unstable_pct * 2.0
    safety_score -= alert_count * 10.0
    safety_score -= decel_count * 15.0
    safety_score = max(0.0, min(100.0, round(safety_score, 1)))

    duration_sec = (
        round(end_time - session_start_time, 1)
        if session_start_time else None
    )

    try:
        alcohol_value = Bridge.call("get_last_measured_alcohol")
        alcohol_status_code = Bridge.call("get_alcohol_status")
        alcohol_status_map = {0: "No Breath", 1: "Normal Breath", 2: "Alcohol Breath"}
        alcohol_status = alcohol_status_map.get(alcohol_status_code, "Unknown")
    except Exception as e:
        print("[Cloud] Failed to retrieve alcohol status:", e)
        alcohol_value = None
        alcohol_status = "Unknown"

    summary = {
        "session_start_time": session_start_time,
        "session_end_time": end_time,
        "duration_sec": duration_sec,

        "ai_classification": {
            "normal_pct": normal_pct,
            "turn_pct": turn_pct,
            "unstable_pct": unstable_pct,
        },
        "driving_style": driving_style,
        "avg_ai_confidence": avg_confidence,

        "unsafe_riding_events": alert_count,
        "safety_deceleration_count": decel_count,
        "avg_reaction_time_sec": avg_reaction_time_sec,

        "false_positive_estimate": false_positive_estimate,

        "alcohol_gate_result": {
            "final_value": alcohol_value,
            "status": alcohol_status,
        },

        "safety_score": safety_score,
    }
    return summary


def _post_to_cloud(payload):
    """Perform the actual HTTP PUT with a date-time-based key. Run it in a separate thread so it does not block the 100 ms loop."""
    if requests is None:
        print("[Cloud] Skipping transmission because requests is not installed:", json.dumps(payload, ensure_ascii=False))
        return

    # Build a human-readable, sortable key from the session's end time, e.g. 2026-09-12_14-30-05
    end_time = payload.get("session_end_time") or time.time()
    session_key = datetime.datetime.fromtimestamp(end_time, tz=HACKSTER_TZ).strftime("%Y-%m-%d_%H-%M-%S")
    url = f"{CLOUD_ENDPOINT}/{session_key}.json"

    try:
        resp = requests.put(url, json=payload, timeout=10)
        print(f"[Cloud] Transmission complete (status={resp.status_code}, key={session_key})")
    except Exception as e:
        print("[Cloud] Transmission failed:", e)


def send_to_cloud_async(payload):
    thread = threading.Thread(target=_post_to_cloud, args=(payload,), daemon=True)
    thread.start()

model_path = "assets/model.eim"
st = os.stat(model_path)
os.chmod(model_path, st.st_mode | stat.S_IEXEC | stat.S_IXGRP | stat.S_IXOTH)

try:
    runner = ImpulseRunner("assets/model.eim")
    model_info = runner.init()
    print("Model loaded successfully:", model_info)
except Exception as e:
    print("Error:", e)

gyro_buffer = []
WINDOW_SAMPLES = 65  # 6500ms / 100ms
current_ai_label = "normal" 
current_ai_confidence = 0.0

RECENT_WINDOW_SEC = 1.2
LOOP_INTERVAL_SEC = 0.1
RECENT_SAMPLES = int(RECENT_WINDOW_SEC / LOOP_INTERVAL_SEC)  # 약 12개
NOISE_FLOOR_GX = 9.5
NOISE_FLOOR_GZ = 37.7
recent_gx = []
recent_gz = []
def is_currently_quiet(gx_list, gz_list, floor_gx, floor_gz):
    if len(gx_list) < RECENT_SAMPLES:
        return False
    recent_gx_max = max(abs(v) for v in gx_list[-RECENT_SAMPLES:])
    recent_gz_max = max(abs(v) for v in gz_list[-RECENT_SAMPLES:])
    return recent_gx_max < floor_gx and recent_gz_max < floor_gz

def get_dashboard_data():
    return {
        "alcohol": Bridge.call("get_last_measured_alcohol"),
        "alcohol_status": Bridge.call("get_alcohol_status"),
        "permission": Bridge.call("get_permission"),
        "state": Bridge.call("get_state"),
        "warn": Bridge.call("get_warn_count"),
        "ai_result": current_ai_label,
        "ai_confidence": round(current_ai_confidence, 2)
        
    }

ui.expose_api(
    "GET",
    "/dashboard",
    get_dashboard_data
)

def list_csv():
    files = os.listdir("/home/arduino/data")
    return {"files": files}

ui.expose_api("GET", "/list", list_csv)

def download_csv(filename: str):
    filepath = f"/home/arduino/data/{filename}"
    return FileResponse(filepath, filename=filename)

ui.expose_api("GET", "/download/{filename}", download_csv)

def collect(label: str, duration: str):
    duration = int(duration)
    os.makedirs("/home/arduino/data", exist_ok=True)
    filename = f"/home/arduino/data/{label}_{int(time.time())}.csv"
    with open(filename, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["timestamp_ms", "gyro_x", "gyro_z"])
        start = time.time()
        while time.time() - start < duration:
            t = int((time.time() - start) * 1000)
            gx = Bridge.call("get_gyro_x")
            gz = Bridge.call("get_gyro_z")
            writer.writerow([t, gx, gz])
            time.sleep(0.1)
    return {"status": "done", "file": filename}

ui.expose_api("GET", "/collect/{label}/{duration}", collect)

# Real-time cumulative statistics for the active session (for checking without waiting 30 minutes)
def get_session_stats():
    total = sum(session_counts.values())
    return {
        "session_active": session_active,
        "session_start_time": session_start_time,
        "elapsed_sec": (
            round(time.time() - session_start_time, 1)
            if session_active and session_start_time else 0
        ),
        "windows_classified": total,
        "counts": session_counts,
        "avg_confidence": (
            round(confidence_sum / confidence_count, 2)
            if confidence_count > 0 else 0.0
        ),
    }


ui.expose_api("GET", "/session_stats", get_session_stats)

# For development/review - send the cloud report immediately without waiting 30 minutes
def force_send_report():
    end_time = time.time()
    summary = build_session_summary(end_time)
    send_to_cloud_async(summary)
    return {"status": "sent", "summary": summary}


ui.expose_api("GET", "/force_send_report", force_send_report)

def loop():
    global current_ai_label, current_ai_confidence
    global session_active, idle_since
    global confidence_sum, confidence_count

    gx = Bridge.call("get_gyro_x")
    gz = Bridge.call("get_gyro_z")
    gyro_buffer.append(gx)
    gyro_buffer.append(gz)
    recent_gx.append(gx)
    recent_gz.append(gz)
    if len(recent_gx) > RECENT_SAMPLES:
        del recent_gx[:len(recent_gx) - RECENT_SAMPLES]
        del recent_gz[:len(recent_gz) - RECENT_SAMPLES]
   
    if len(gyro_buffer) > WINDOW_SAMPLES * 2:
        del gyro_buffer[: len(gyro_buffer) - WINDOW_SAMPLES * 2]

    if len(gyro_buffer) == WINDOW_SAMPLES * 2:
        try:
            res = runner.classify(gyro_buffer)
            result = res["result"]["classification"]
            label = max(result, key=result.get)
            confidence = result[label]

            if "normal" in label:
                code = 0
                count_key = "normal"
            elif "turn" in label:
                code = 1
                count_key = "turn"
            else:
                code = 2
                count_key = "unstable"
            if code == 2 and is_currently_quiet(recent_gx, recent_gz, NOISE_FLOOR_GX, NOISE_FLOOR_GZ):
                print(f"[Gate Intervention]] ML={label}({confidence:.2f}) → quiet{RECENT_WINDOW_SEC}s → normal")
                code = 0
                count_key = "normal"
                label = "normal (recency-override)"
            current_ai_label = label.replace("e-scooter_", "")
            current_ai_confidence = confidence
            print(f"[raw data] {label} ({confidence:.2f}) → Transmission Code: {code}")

            # Accumulate data for cloud statistics while the session is active
            if session_active:
                session_counts[count_key] += 1
                confidence_sum += confidence
                confidence_count += 1

            Bridge.call("set_ai_result", code)  # Send without additional processing
        except Exception as e:
            print("추론 에러:", e)

    # Session start/end + 30-minute idle detection → cloud transmission
    try:
        state = Bridge.call("get_state")

        # 0 = STATE_WAITING, other values = riding states
        if state != 0:
            # Riding (or just resumed) → cancel the idle timer
            idle_since = None

            if not session_active:
                # Detect the start of a new session
                session_active = True
                reset_session_state()
                print("[Cloud] New session started")

        else:
            # STATE_WAITING (stopped state)
            if session_active:
                if idle_since is None:
                    idle_since = time.time()
                elif time.time() - idle_since >= IDLE_TIMEOUT_SEC:
                    end_time = time.time()
                    summary = build_session_summary(end_time)
                    send_to_cloud_async(summary)
                    print("[Cloud] 30-minute idle detected → sending session report")
                    session_active = False
                    idle_since = None

    except Exception as e:
        print("[Cloud] Session detection error:", e)

    time.sleep(0.1)


App.run(user_loop=loop)
