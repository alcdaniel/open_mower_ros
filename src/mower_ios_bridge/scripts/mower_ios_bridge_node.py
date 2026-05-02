#!/usr/bin/env python3
import json
import math
import socket
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import urlparse

import rospy
from geometry_msgs.msg import Point32, Polygon, Pose, Quaternion, Twist
from mower_map.msg import MapArea
from mower_map.srv import (
    AddMowingAreaSrv,
    AddMowingAreaSrvRequest,
    ClearMapSrv,
    ClearMapSrvRequest,
    SetDockingPointSrv,
    SetDockingPointSrvRequest,
)
from mower_msgs.msg import Emergency, ESCStatus, HighLevelStatus, Power, Status
from mower_msgs.srv import HighLevelControlSrv, HighLevelControlSrvRequest, MowerControlSrv
from sensor_msgs.msg import Imu, Range
from std_msgs.msg import Bool, String
from xbot_msgs.msg import AbsolutePose


def now_ms():
    return int(time.time() * 1000)


def clamp(value, low, high):
    return max(low, min(high, value))


def ros_bool(value):
    if isinstance(value, bool):
        return value
    return str(value).strip().lower() in ("1", "true", "yes", "on")


class BridgeState:
    def __init__(self):
        self.lock = threading.RLock()
        self.started_at = time.time()
        self.manual_active = False
        self.manual_until = 0.0
        self.last_setting_updates = {}

        self.high_level = None
        self.low_level = None
        self.power = None
        self.emergency = None
        self.left_esc = None
        self.right_esc = None
        self.pose = None
        self.last_seen = {}

        # Extended Mega sensors
        self.sonar = [999, 999, 999]   # cm: [front, left, right]
        self.bumper = False
        self.rain_mega = False         # separate from Status.rain_detected
        self.tilt = False
        self.wire_detected = True
        self.mega_compass_deg = 0.0
        self.mega_settings = {}        # key → value string, populated by $CFG frames

        # Map / recording
        self.map_json = ""             # last value of /mower_map_service/json_map
        self.map_received_at = 0.0
        self.recording = None          # None | dict(mode, points, started_at, ...)
        self.pending_obstacles = []    # list[list[(x,y)]] buffered between mow stop and save
        self.last_recording_buffer = None  # snapshot of last closed mow session

    def update(self, name, msg):
        with self.lock:
            setattr(self, name, msg)
            self.last_seen[name] = time.time()

    def snapshot(self):
        with self.lock:
            if self.manual_active and time.time() > self.manual_until:
                self.manual_active = False
            return {
                "started_at": self.started_at,
                "manual_active": self.manual_active,
                "high_level": self.high_level,
                "low_level": self.low_level,
                "power": self.power,
                "emergency": self.emergency,
                "left_esc": self.left_esc,
                "right_esc": self.right_esc,
                "pose": self.pose,
                "last_seen": dict(self.last_seen),
                "last_setting_updates": dict(self.last_setting_updates),
                "sonar": list(self.sonar),
                "bumper": self.bumper,
                "rain_mega": self.rain_mega,
                "tilt": self.tilt,
                "wire_detected": self.wire_detected,
                "mega_compass_deg": self.mega_compass_deg,
                "mega_settings": dict(self.mega_settings),
                "map_json": self.map_json,
                "map_received_at": self.map_received_at,
                "recording": dict(self.recording) if self.recording else None,
                "pending_obstacles_count": len(self.pending_obstacles),
                "last_buffer_points": (len(self.last_recording_buffer["points"])
                                       if self.last_recording_buffer else 0),
            }

    def set_manual_active(self, active, hold_seconds=2.0):
        with self.lock:
            self.manual_active = active
            self.manual_until = time.time() + hold_seconds if active else 0.0

    def set_setting(self, setting_id, value):
        with self.lock:
            self.last_setting_updates[setting_id] = value


class IOSBridge:
    def __init__(self):
        self.state = BridgeState()
        self.host = rospy.get_param("~host", "0.0.0.0")
        self.port = int(rospy.get_param("~port", 8080))
        self.discovery_name = rospy.get_param("~discovery_name", "lawnmower")
        self.auth_token = rospy.get_param("~auth_token", "")
        self.linear_speed = float(rospy.get_param("~manual_linear_speed", 0.25))
        self.angular_speed = float(rospy.get_param("~manual_angular_speed", 0.8))
        self.beacon_enabled = ros_bool(rospy.get_param("~udp_beacon_enabled", True))
        self.beacon_port = int(rospy.get_param("~udp_beacon_port", 47820))

        self.action_pub = rospy.Publisher("/xbot/action", String, queue_size=5)
        self.joy_vel_pub = rospy.Publisher("/joy_vel", Twist, queue_size=1)
        self.cfgget_pub = rospy.Publisher("/mega/cfgget", Bool, queue_size=1)
        self.cfgset_pub = rospy.Publisher("/mega/cfgset", String, queue_size=10)

        self.high_level_srv = rospy.ServiceProxy("/mower_service/high_level_control", HighLevelControlSrv)
        self.mower_control_srv = rospy.ServiceProxy("/ll/_service/mow_enabled", MowerControlSrv)
        self.add_area_srv = rospy.ServiceProxy("/mower_map_service/add_mowing_area", AddMowingAreaSrv)
        self.set_dock_srv = rospy.ServiceProxy("/mower_map_service/set_docking_point", SetDockingPointSrv)
        self.clear_map_srv = rospy.ServiceProxy("/mower_map_service/clear_map", ClearMapSrv)

        # Recording config
        self.rec_lock = threading.RLock()
        self.rec_min_distance_m = float(rospy.get_param("~rec_min_distance_m", 0.10))
        self.rec_sample_hz = float(rospy.get_param("~rec_sample_hz", 4.0))
        self.rec_close_tolerance_m = float(rospy.get_param("~rec_close_tolerance_m", 0.30))
        self.rec_min_area_m2 = float(rospy.get_param("~rec_min_area_m2", 0.50))
        self.rec_require_rtk_fixed = ros_bool(rospy.get_param("~rec_require_rtk_fixed", True))

        rospy.Subscriber("/mower_logic/current_state", HighLevelStatus, self.state.update, "high_level")
        rospy.Subscriber("/ll/mower_status", Status, self.state.update, "low_level")
        rospy.Subscriber("/ll/power", Power, self.state.update, "power")
        rospy.Subscriber("/ll/emergency", Emergency, self.state.update, "emergency")
        rospy.Subscriber("/ll/diff_drive/left_esc_status", ESCStatus, self.state.update, "left_esc")
        rospy.Subscriber("/ll/diff_drive/right_esc_status", ESCStatus, self.state.update, "right_esc")
        rospy.Subscriber("/xbot_positioning/xb_pose", AbsolutePose, self.state.update, "pose")

        # Extended Mega sensor topics
        rospy.Subscriber("/mega/sonar/front", Range, lambda m: self._cb_sonar(m, 0))
        rospy.Subscriber("/mega/sonar/left",  Range, lambda m: self._cb_sonar(m, 1))
        rospy.Subscriber("/mega/sonar/right", Range, lambda m: self._cb_sonar(m, 2))
        rospy.Subscriber("/mega/bumper",       Bool, self._cb_bumper)
        rospy.Subscriber("/mega/rain",         Bool, self._cb_rain_mega)
        rospy.Subscriber("/mega/tilt",         Bool, self._cb_tilt)
        rospy.Subscriber("/mega/wire_detected",Bool, self._cb_wire)
        rospy.Subscriber("/mega/imu",          Imu,    self._cb_compass_imu)
        rospy.Subscriber("/mega/cfg",          String, self._cb_cfg)
        rospy.Subscriber("/mega/cfg_loaded",   Bool,   self._cb_cfg_loaded)

        # Map (latched topic — last value arrives on subscribe)
        rospy.Subscriber("/mower_map_service/json_map", String, self._cb_json_map)

        # Recording auto-sampler (4 Hz default)
        sample_period = max(0.05, 1.0 / max(0.5, self.rec_sample_hz))
        rospy.Timer(rospy.Duration(sample_period), self._sample_recording)

        # Request settings from Mega 3 s after startup (gives Mega time to boot)
        rospy.Timer(rospy.Duration(3.0), self._request_settings, oneshot=True)

        self.httpd = None

    def start(self):
        handler = self._make_handler()
        self.httpd = ThreadingHTTPServer((self.host, self.port), handler)
        threading.Thread(target=self.httpd.serve_forever, name="ios-http", daemon=True).start()
        rospy.loginfo("iOS bridge listening on http://%s:%d", self.host, self.port)

        if self.beacon_enabled:
            threading.Thread(target=self._udp_beacon_loop, name="ios-udp-beacon", daemon=True).start()

    def stop(self):
        if self.httpd is not None:
            self.httpd.shutdown()

    def _make_handler(self):
        bridge = self

        class RequestHandler(BaseHTTPRequestHandler):
            server_version = "OpenMowerIOSBridge/0.1"

            def log_message(self, fmt, *args):
                rospy.logdebug("iOS bridge HTTP: " + fmt, *args)

            def do_GET(self):
                bridge.handle_request(self, "GET")

            def do_POST(self):
                bridge.handle_request(self, "POST")

            def do_OPTIONS(self):
                self.send_response(204)
                bridge.send_common_headers(self)
                self.end_headers()

        return RequestHandler

    def handle_request(self, request, method):
        path = urlparse(request.path).path
        try:
            if self.auth_token and not self._authorized(request):
                self._send_json(request, {"error": "unauthorized"}, status=401)
                return

            if method == "GET" and path == "/api/health":
                self._send_json(request, self.health_payload())
            elif method == "GET" and path == "/api/status":
                self._send_json(request, self.status_payload())
            elif method == "GET" and path == "/api/telemetry":
                self._send_json(request, self.telemetry_payload())
            elif method == "GET" and path == "/api/settings":
                self._send_json(request, self.settings_payload())
            elif method == "POST" and path == "/api/command":
                body = self._read_json(request)
                try:
                    self.handle_command(str(body.get("command", "")))
                    self._send_json(request, {"ok": True})
                except (RuntimeError, ValueError) as exc:
                    rospy.logwarn("[ios_bridge] command '%s' rejected: %s", body.get("command", ""), exc)
                    self._send_json(request, {"ok": False, "error": str(exc)})
            elif method == "POST" and path == "/api/manual":
                body = self._read_json(request)
                try:
                    self.handle_manual(str(body.get("direction", "stop")))
                    self._send_json(request, {"ok": True})
                except (RuntimeError, ValueError) as exc:
                    self._send_json(request, {"ok": False, "error": str(exc)})
            elif method == "POST" and path == "/api/settings":
                body = self._read_json(request)
                self.handle_setting(body)
                self._send_json(request, {"ok": True})
            elif method == "GET" and path == "/api/map":
                self._send_json(request, self.map_payload())
            elif method == "GET" and path == "/api/pose":
                self._send_json(request, self.pose_payload())
            elif method == "GET" and path == "/api/recording":
                self._send_json(request, self.recording_status())
            elif method == "POST" and path == "/api/recording/start":
                body = self._read_json(request)
                try:
                    self._start_recording(str(body.get("mode", "")))
                    self._send_json(request, {"ok": True, "recording": self.recording_status()})
                except (RuntimeError, ValueError) as exc:
                    self._send_json(request, {"ok": False, "error": str(exc)})
            elif method == "POST" and path == "/api/recording/stop":
                try:
                    rec = self._stop_recording()
                    self._send_json(request, {
                        "ok": True,
                        "mode": rec["mode"],
                        "points": len(rec["points"]),
                        "recording": self.recording_status(),
                    })
                except RuntimeError as exc:
                    self._send_json(request, {"ok": False, "error": str(exc)})
            elif method == "POST" and path == "/api/recording/cancel":
                self._cancel_recording()
                self._clear_recording_buffer()
                self._send_json(request, {"ok": True})
            elif method == "POST" and path == "/api/areas/mowing":
                body = self._read_json(request)
                try:
                    res = self.save_area(body, is_navigation=False)
                    self._send_json(request, res)
                except (RuntimeError, ValueError) as exc:
                    self._send_json(request, {"ok": False, "error": str(exc)})
            elif method == "POST" and path == "/api/areas/navigation":
                body = self._read_json(request)
                try:
                    res = self.save_area(body, is_navigation=True)
                    self._send_json(request, res)
                except (RuntimeError, ValueError) as exc:
                    self._send_json(request, {"ok": False, "error": str(exc)})
            elif method == "POST" and path == "/api/dock/here":
                try:
                    res = self.set_dock_here()
                    self._send_json(request, res)
                except (RuntimeError, ValueError) as exc:
                    self._send_json(request, {"ok": False, "error": str(exc)})
            elif method == "POST" and path == "/api/dock":
                body = self._read_json(request)
                try:
                    res = self.set_dock_pose(body)
                    self._send_json(request, res)
                except (RuntimeError, ValueError) as exc:
                    self._send_json(request, {"ok": False, "error": str(exc)})
            elif method == "POST" and path == "/api/map/clear":
                body = self._read_json(request)
                if not body.get("confirm", False):
                    self._send_json(request, {"ok": False, "error": "Falta confirmación."})
                else:
                    try:
                        self.clear_full_map()
                        self._send_json(request, {"ok": True})
                    except RuntimeError as exc:
                        self._send_json(request, {"ok": False, "error": str(exc)})
            else:
                self._send_json(request, {"error": "not_found"}, status=404)
        except Exception as exc:
            rospy.logwarn("iOS bridge request failed: %s", exc)
            self._send_json(request, {"error": str(exc)}, status=500)

    def send_common_headers(self, request):
        request.send_header("Content-Type", "application/json")
        request.send_header("Access-Control-Allow-Origin", "*")
        request.send_header("Access-Control-Allow-Headers", "Authorization, Content-Type")
        request.send_header("Access-Control-Allow-Methods", "GET, POST, OPTIONS")

    def _send_json(self, request, payload, status=200):
        data = json.dumps(payload, separators=(",", ":")).encode("utf-8")
        request.send_response(status)
        self.send_common_headers(request)
        request.send_header("Content-Length", str(len(data)))
        request.end_headers()
        request.wfile.write(data)

    def _read_json(self, request):
        length = int(request.headers.get("Content-Length", "0") or "0")
        if length <= 0:
            return {}
        return json.loads(request.rfile.read(length).decode("utf-8"))

    def _authorized(self, request):
        return request.headers.get("Authorization", "") == "Bearer {}".format(self.auth_token)

    def health_payload(self):
        snap = self.state.snapshot()
        return {
            "ok": True,
            "uptimeMs": int((time.time() - snap["started_at"]) * 1000),
            "ip": self._local_ip(),
            "manualActive": bool(snap["manual_active"]),
        }

    def status_payload(self):
        snap = self.state.snapshot()
        high = snap["high_level"]
        low = snap["low_level"]
        power = snap["power"]
        emergency = snap["emergency"]
        left_esc = snap["left_esc"]
        right_esc = snap["right_esc"]

        emergency_active = bool(emergency and (emergency.active_emergency or emergency.latched_emergency))
        charging = bool((low and low.is_charging) or (power and power.charge_current > 0.1))
        battery_voltage = self._first_valid([
            getattr(power, "battery_voltage_adc", 0.0) if power else 0.0,
            getattr(power, "battery_voltage_bms", 0.0) if power else 0.0,
            getattr(power, "battery_voltage_chg", 0.0) if power else 0.0,
        ])
        charge_current = float(power.charge_current) if power else 0.0
        wheel_current = 0.0
        if left_esc:
            wheel_current += abs(float(left_esc.current))
        if right_esc:
            wheel_current += abs(float(right_esc.current))

        state = self._operating_state(high, low, emergency_active, charging, snap["manual_active"])
        running = state in ("mowing", "trackingWire", "exitingDock", "manual")
        docked = state == "docked"
        parked = state == "parked"

        bumper       = snap.get("bumper", False)
        tilt_active  = snap.get("tilt", False)
        wire_det     = snap.get("wire_detected", True)
        rain_mega    = snap.get("rain_mega", False)
        rain_detected = bool((low and low.rain_detected) or rain_mega)

        return {
            "connection": "connected",
            "state": state,
            "robotStatus": int(low.mower_status) if low else 0,
            "errorCode": 3 if emergency_active else (4 if tilt_active else 0),
            "batteryVoltage": battery_voltage,
            "chargeCurrent": charge_current,
            "wheelCurrent": wheel_current,
            "charging": charging,
            "docked": docked,
            "parked": parked,
            "running": running,
            "trackingWire": state == "trackingWire",
            "wireDetected": wire_det,
            "outsideWire": not wire_det and running,
            "rainDetected": rain_detected,
            "bladesOn": bool(low and low.mow_enabled),
            "bumper": bumper,
            "tiltActive": tilt_active,
            "lowBattery": False,
            "wheelBlocked": False,
            "alarm1Enabled": tilt_active,
            "alarm2Enabled": rain_detected,
            "alarm3Enabled": False,
            "lastUpdated": now_ms(),
        }

    def telemetry_payload(self):
        snap = self.state.snapshot()
        pose = snap["pose"]
        left_esc = snap["left_esc"]
        right_esc = snap["right_esc"]
        sonar = snap.get("sonar", [999, 999, 999])   # [front, left, right]

        heading = 0.0
        compass_error = 0.0
        if pose:
            heading = math.degrees(float(pose.vehicle_heading))
            compass_error = float(pose.position_accuracy)

        pwm_left = self._rpm_to_pwm(left_esc.rpm) if left_esc else 0
        pwm_right = self._rpm_to_pwm(right_esc.rpm) if right_esc else 0
        wheel_status = 7
        if pwm_left > 0 or pwm_right > 0:
            wheel_status = 5

        bumper = snap.get("bumper", False)
        tilt   = snap.get("tilt", False)

        return {
            "loopCycle": int(time.time()) % 100000,
            "sonarCenterCm": sonar[0],
            "sonarLeftCm":   sonar[1],
            "sonarRightCm":  sonar[2],
            "sonarLeftHits": 0,
            "sonarCenterHits": 0,
            "sonarRightHits": 0,
            "sonarTriggered": any(s < 30 for s in sonar if s < 999),
            "bumper": bumper,
            "tiltAngle": tilt,
            "tipOver": tilt,
            "gpsInsideFence": self._gps_fix_type(pose) != "none",
            "gpsFixType": self._gps_fix_type(pose),
            "compassHeading": heading,
            "compassError": compass_error,
            "megaCompassDeg": snap.get("mega_compass_deg", 0.0),
            "magNow": 0,
            "pwmLeft": pwm_left,
            "pwmRight": pwm_right,
            "mowerRunBack": 0,
            "turnPhase": 0,
            "sonarPhase": 0,
            "wheelStatusValue": wheel_status,
        }

    def settings_payload(self):
        snap = self.state.snapshot()
        updates = snap["last_setting_updates"]
        ms = snap.get("mega_settings", {})

        def mv(key, default=0):
            """Return float value for a mega setting, preferring live updates."""
            if "mega." + key in updates:
                return float(updates["mega." + key])
            raw = ms.get(key)
            if raw is not None:
                try:
                    return float(raw)
                except (ValueError, TypeError):
                    pass
            return float(default)

        settings = [
            # ── App iOS ────────────────────────────────────────────────────────
            self._setting("bridge.manualLinearSpeed", "Velocidad lineal manual",  "App iOS", "number",  self.linear_speed,        "m/s",   0.05, 1.0,  0.05),
            self._setting("bridge.manualAngularSpeed", "Velocidad giro manual",   "App iOS", "number",  self.angular_speed,       "rad/s", 0.1,  2.0,  0.1),
            self._setting("bridge.udpBeacon",          "Descubrimiento UDP",      "App iOS", "boolean", 1 if self.beacon_enabled else 0, None, None, None, None),

            # ── 1. Motores de rueda ────────────────────────────────────────────
            self._setting("mega.pwmMaxLH",   "PWM máx izquierdo",      "Motores de rueda", "number",  mv("pwmMaxLH",  200), None, 0, 255, 1),
            self._setting("mega.pwmMaxRH",   "PWM máx derecho",        "Motores de rueda", "number",  mv("pwmMaxRH",  200), None, 0, 255, 1),
            self._setting("mega.pwmSlowLH",  "PWM lento izquierdo",    "Motores de rueda", "number",  mv("pwmSlowLH",  80), None, 0, 255, 1),
            self._setting("mega.pwmSlowRH",  "PWM lento derecho",      "Motores de rueda", "number",  mv("pwmSlowRH",  80), None, 0, 255, 1),
            self._setting("mega.wheelsOn",   "Ruedas activadas",       "Motores de rueda", "boolean", mv("wheelsOn",    1), None, None, None, None),

            # ── 2. Temporización ──────────────────────────────────────────────
            self._setting("mega.turnDelayMin",  "Tiempo giro mín (×100 ms)", "Temporización", "number", mv("turnDelayMin",  10), "×100ms", 1,  50, 1),
            self._setting("mega.turnDelayMax",  "Tiempo giro máx (×100 ms)", "Temporización", "number", mv("turnDelayMax",  20), "×100ms", 1, 100, 1),
            self._setting("mega.reverseDelay",  "Tiempo marcha atrás",       "Temporización", "number", mv("reverseDelay",   5), "×100ms", 1,  50, 1),
            self._setting("mega.straightCycles","Ciclos en línea recta",      "Temporización", "number", mv("straightCycles",20), "ciclos", 1, 200, 1),
            self._setting("mega.turn90LH",      "Giro 90° izq (×10 ms)",     "Temporización", "number", mv("turn90LH",     90), "×10ms",  1, 200, 1),
            self._setting("mega.turn90RH",      "Giro 90° der (×10 ms)",     "Temporización", "number", mv("turn90RH",     90), "×10ms",  1, 200, 1),
            self._setting("mega.lineLenCycles", "Ciclos longitud línea",      "Temporización", "number", mv("lineLenCycles", 0), "ciclos", 0, 200, 1),

            # ── 3. Cuchillas ──────────────────────────────────────────────────
            self._setting("mega.pwmBlade", "PWM cuchilla",        "Cuchillas", "number",  mv("pwmBlade", 255), None, 0,   255, 1),
            self._setting("mega.bladeOn",  "Cuchillas activadas", "Cuchillas", "boolean", mv("bladeOn",    1), None, None, None, None),

            # ── 4. Sonares ────────────────────────────────────────────────────
            self._setting("mega.sonar1On",    "Sonar 1 activo",          "Sonares", "boolean", mv("sonar1On",    1), None, None, None, None),
            self._setting("mega.sonar2On",    "Sonar 2 activo",          "Sonares", "boolean", mv("sonar2On",    1), None, None, None, None),
            self._setting("mega.sonar3On",    "Sonar 3 activo",          "Sonares", "boolean", mv("sonar3On",    1), None, None, None, None),
            self._setting("mega.sonarMaxCm",  "Distancia detección (cm)","Sonares", "number",  mv("sonarMaxCm", 30), "cm",    5, 200, 1),
            self._setting("mega.sonarMaxHit", "Sensibilidad sonar",      "Sonares", "number",  mv("sonarMaxHit", 3), "hits",  1,  20, 1),

            # ── 5. Cable perimetral ───────────────────────────────────────────
            self._setting("mega.wireOn",      "Cable perimetral activo", "Cable perimetral", "boolean", mv("wireOn",      1), None,     None, None,  None),
            self._setting("mega.pidP",        "PID P",                   "Cable perimetral", "number",  mv("pidP",     1.50), None,     0.01, 10.0,  0.01),
            self._setting("mega.wireZ1Cyc",   "Ciclos zona 1 (×100)",    "Cable perimetral", "number",  mv("wireZ1Cyc",   5), "×100",   1,   50,    1),
            self._setting("mega.wireZ2Cyc",   "Ciclos zona 2 (×100)",    "Cable perimetral", "number",  mv("wireZ2Cyc",   5), "×100",   1,   50,    1),
            self._setting("mega.wireFwdCyc",  "Ciclos avance cable",     "Cable perimetral", "number",  mv("wireFwdCyc", 20), "×10",    1,  200,    1),
            self._setting("mega.wireBakCyc",  "Ciclos retroceso cable",  "Cable perimetral", "number",  mv("wireBakCyc", 10), "×10",    1,  200,    1),
            self._setting("mega.wireMaxTrR",  "Giros der antes reinicio","Cable perimetral", "number",  mv("wireMaxTrR", 20), "×10",    1,  200,    1),
            self._setting("mega.wireMaxTrL",  "Giros izq antes reinicio","Cable perimetral", "number",  mv("wireMaxTrL", 20), "×10",    1,  200,    1),
            self._setting("mega.cwToCharge",  "CW hacia carga",          "Cable perimetral", "boolean", mv("cwToCharge",  1), None,     None, None,  None),
            self._setting("mega.ccwToCharge", "CCW hacia carga",         "Cable perimetral", "boolean", mv("ccwToCharge", 0), None,     None, None,  None),
            self._setting("mega.cwToStart",   "CW hacia inicio",         "Cable perimetral", "boolean", mv("cwToStart",   0), None,     None, None,  None),
            self._setting("mega.ccwToStart",  "CCW hacia inicio",        "Cable perimetral", "boolean", mv("ccwToStart",  1), None,     None, None,  None),

            # ── 6. Compás ─────────────────────────────────────────────────────
            self._setting("mega.compassOn",    "Compás activo",          "Compás", "boolean", mv("compassOn",    0), None,    None, None,  None),
            self._setting("mega.compassMode",  "Modo compás",            "Compás", "option",  mv("compassMode",  1), None,    1,    3,     1, ["DFRobot QMC5883", "QMC5883 Manual", "QMC5883L"]),
            self._setting("mega.compassHHold", "Mantener rumbo",         "Compás", "boolean", mv("compassHHold", 0), None,    None, None,  None),
            self._setting("mega.compassPower", "Potencia PID compás",    "Compás", "number",  mv("compassPower", 2.0), None,  0.1,  10.0,  0.1),
            self._setting("mega.compassHome",  "Rumbo base (°)",         "Compás", "number",  mv("compassHome",  0), "°",    0,    359,   1),

            # ── 7. Giroscopio ─────────────────────────────────────────────────
            self._setting("mega.gyroOn",    "Giroscopio activo",     "Giroscopio", "boolean", mv("gyroOn",    0), None, None, None,  None),
            self._setting("mega.gyroPower", "Potencia PID giroscopio","Giroscopio","number",  mv("gyroPower", 2.0), None, 0.1, 10.0,  0.1),

            # ── 8. Batería ────────────────────────────────────────────────────
            self._setting("mega.battMin",   "Voltaje mínimo batería",   "Batería", "number",  mv("battMin",  21.0), "V",     10.0, 30.0, 0.1),
            self._setting("mega.battSens",  "Sensibilidad baja batería","Batería", "number",  mv("battSens",    5), "count",    1,   30,   1),

            # ── 9. Protección ruedas ──────────────────────────────────────────
            self._setting("mega.wheelAmpOn",  "Sensor amperios rueda activo","Protección ruedas","boolean", mv("wheelAmpOn",  1), None, None, None, None),
            self._setting("mega.wheelAmpMax", "Amperios máx rueda",          "Protección ruedas","number",  mv("wheelAmpMax", 1.5), "A",  0.1, 10.0, 0.1),

            # ── 10. Bumper ────────────────────────────────────────────────────
            self._setting("mega.bumperOn", "Bumper activo", "Bumper", "boolean", mv("bumperOn", 1), None, None, None, None),

            # ── 11. Patrón de corte ───────────────────────────────────────────
            self._setting("mega.patternMow", "Tipo de patrón", "Patrón de corte", "option", mv("patternMow", 0), None, 0, 2, 1, ["Aleatorio", "Paralelo", "Espiral"]),

            # ── 12. Inclinación ───────────────────────────────────────────────
            self._setting("mega.tiltOn", "Sensor ángulo activo",    "Inclinación", "boolean", mv("tiltOn", 0), None, None, None, None),
            self._setting("mega.tipOn",  "Sensor vuelco activo",    "Inclinación", "boolean", mv("tipOn",  0), None, None, None, None),

            # ── 13. Lluvia ────────────────────────────────────────────────────
            self._setting("mega.rainOn",   "Sensor lluvia instalado",  "Lluvia", "boolean", mv("rainOn",    0), None,    None, None, None),
            self._setting("mega.rainSens", "Sensibilidad lluvia",      "Lluvia", "number",  mv("rainSens",  5), "count",    1,   30,   1),

            # ── 14. Base de carga ─────────────────────────────────────────────
            self._setting("mega.chargeStOn", "Usar base de carga", "Base de carga", "boolean", mv("chargeStOn", 1), None, None, None, None),

            # ── 16. Alarmas ───────────────────────────────────────────────────
            self._setting("mega.alarm1On",  "Alarma 1 activa",  "Alarma 1", "boolean", mv("alarm1On",  0), None,    None, None, None),
            self._setting("mega.alarm1H",   "Alarma 1 hora",    "Alarma 1", "number",  mv("alarm1H",   6), "h",     0,   23,   1),
            self._setting("mega.alarm1M",   "Alarma 1 minuto",  "Alarma 1", "number",  mv("alarm1M",   0), "min",   0,   59,   1),
            self._setting("mega.alarm1Act", "Alarma 1 acción",  "Alarma 1", "option",  mv("alarm1Act", 4), None,    1,    5,   1, ["Zona 1", "Zona 2", "Línea", "Inicio rápido", "Personalizada"]),
            self._setting("mega.alarm1Rep", "Alarma 1 repetir", "Alarma 1", "boolean", mv("alarm1Rep", 0), None,    None, None, None),

            self._setting("mega.alarm2On",  "Alarma 2 activa",  "Alarma 2", "boolean", mv("alarm2On",  0), None,    None, None, None),
            self._setting("mega.alarm2H",   "Alarma 2 hora",    "Alarma 2", "number",  mv("alarm2H",  12), "h",     0,   23,   1),
            self._setting("mega.alarm2M",   "Alarma 2 minuto",  "Alarma 2", "number",  mv("alarm2M",   0), "min",   0,   59,   1),
            self._setting("mega.alarm2Act", "Alarma 2 acción",  "Alarma 2", "option",  mv("alarm2Act", 4), None,    1,    5,   1, ["Zona 1", "Zona 2", "Línea", "Inicio rápido", "Personalizada"]),
            self._setting("mega.alarm2Rep", "Alarma 2 repetir", "Alarma 2", "boolean", mv("alarm2Rep", 0), None,    None, None, None),

            self._setting("mega.alarm3On",  "Alarma 3 activa",  "Alarma 3", "boolean", mv("alarm3On",  0), None,    None, None, None),
            self._setting("mega.alarm3H",   "Alarma 3 hora",    "Alarma 3", "number",  mv("alarm3H",  18), "h",     0,   23,   1),
            self._setting("mega.alarm3M",   "Alarma 3 minuto",  "Alarma 3", "number",  mv("alarm3M",   0), "min",   0,   59,   1),
            self._setting("mega.alarm3Act", "Alarma 3 acción",  "Alarma 3", "option",  mv("alarm3Act", 4), None,    1,    5,   1, ["Zona 1", "Zona 2", "Línea", "Inicio rápido", "Personalizada"]),
            self._setting("mega.alarm3Rep", "Alarma 3 repetir", "Alarma 3", "boolean", mv("alarm3Rep", 0), None,    None, None, None),
        ]

        for item in settings:
            sid = item["id"]
            if sid in updates:
                item["value"] = float(updates[sid])
        return settings

    def _setting(self, setting_id, title, group, kind, value, unit, minimum, maximum, step, option_labels=None):
        obj = {
            "id": setting_id,
            "title": title,
            "group": group,
            "kind": kind,
            "value": float(value),
            "unit": unit,
            "minimum": minimum,
            "maximum": maximum,
            "step": step,
            "isEnabled": None,
        }
        if option_labels is not None:
            obj["optionLabels"] = option_labels
        return obj

    # ── Extended sensor callbacks ──────────────────────────────────────────────

    def _cb_sonar(self, msg, idx):
        cm = 999 if (math.isinf(msg.range) or msg.range >= 9.99) else int(msg.range * 100)
        with self.state.lock:
            self.state.sonar[idx] = cm

    def _cb_bumper(self, msg):
        with self.state.lock:
            self.state.bumper = bool(msg.data)

    def _cb_rain_mega(self, msg):
        with self.state.lock:
            self.state.rain_mega = bool(msg.data)

    def _cb_tilt(self, msg):
        with self.state.lock:
            self.state.tilt = bool(msg.data)

    def _cb_wire(self, msg):
        with self.state.lock:
            self.state.wire_detected = bool(msg.data)

    def _request_settings(self, _event=None):
        self.cfgget_pub.publish(Bool(True))
        rospy.loginfo("[ios_bridge] requested settings from Mega")

    def _cb_cfg(self, msg):
        """Handle one $CFG key=value frame from Mega."""
        if '=' not in msg.data:
            return
        key, _, val = msg.data.partition('=')
        with self.state.lock:
            self.state.mega_settings[key.strip()] = val.strip()

    def _cb_cfg_loaded(self, _msg):
        """Handle $CFGEND — full settings dump received."""
        rospy.loginfo("[ios_bridge] Mega settings loaded (%d keys)",
                      len(self.state.mega_settings))

    def _cb_compass_imu(self, msg):
        q = msg.orientation
        yaw_rad = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                             1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        deg = math.degrees(yaw_rad)
        if deg < 0:
            deg += 360.0
        with self.state.lock:
            self.state.mega_compass_deg = deg

    # ── Map: incoming JSON ─────────────────────────────────────────────────────

    def _cb_json_map(self, msg):
        with self.state.lock:
            self.state.map_json = msg.data
            self.state.map_received_at = time.time()

    # ── Recording state machine ────────────────────────────────────────────────

    def _sample_recording(self, _event=None):
        """Periodic sampler. No-op when no session is active."""
        with self.rec_lock:
            rec = self.state.recording
            if not rec:
                return
            pose = self.state.snapshot()["pose"]
            if pose is None:
                return

            fix = self._gps_fix_type(pose)
            rec["last_fix"] = fix
            if self.rec_require_rtk_fixed and fix != "fixed":
                rec["rtk_lost_count"] = rec.get("rtk_lost_count", 0) + 1
                return

            x = float(pose.pose.pose.position.x)
            y = float(pose.pose.pose.position.y)

            pts = rec["points"]
            if pts:
                lx, ly = pts[-1]
                if math.hypot(x - lx, y - ly) < self.rec_min_distance_m:
                    return
            pts.append((x, y))
            rec["sampled_at"] = time.time()

    def _start_recording(self, mode):
        """mode: 'mow' | 'nav' | 'exclusion'"""
        with self.rec_lock:
            if self.state.recording is not None:
                raise RuntimeError("Ya hay una grabación activa.")
            if mode not in ("mow", "nav", "exclusion"):
                raise ValueError("Modo de grabación inválido: {}".format(mode))
            if mode == "exclusion" and self.state.last_recording_buffer is None:
                raise RuntimeError(
                    "Para grabar una exclusión primero debes grabar y cerrar una zona de corte."
                )
            self.state.recording = {
                "mode": mode,
                "points": [],
                "started_at": time.time(),
                "sampled_at": 0.0,
                "rtk_lost_count": 0,
                "last_fix": "none",
            }
            rospy.loginfo("[ios_bridge] recording started: %s", mode)

    def _stop_recording(self):
        """Closes session and stores in last_recording_buffer (for mow) or pending_obstacles (for exclusion)."""
        with self.rec_lock:
            rec = self.state.recording
            if rec is None:
                raise RuntimeError("No hay grabación activa.")
            self.state.recording = None
            rospy.loginfo("[ios_bridge] recording stopped: %s (%d pts)",
                          rec["mode"], len(rec["points"]))

            mode = rec["mode"]
            if mode == "exclusion":
                if len(rec["points"]) >= 3:
                    self.state.pending_obstacles.append(list(rec["points"]))
            else:
                # mow / nav: keep as last_recording_buffer (only mow uses pending_obstacles)
                self.state.last_recording_buffer = {
                    "mode": mode,
                    "points": list(rec["points"]),
                }
                if mode != "mow":
                    # nav doesn't use obstacles; clear any stale buffer
                    self.state.pending_obstacles = []
            return rec

    def _cancel_recording(self):
        with self.rec_lock:
            if self.state.recording is not None:
                rospy.loginfo("[ios_bridge] recording cancelled")
                self.state.recording = None

    def _clear_recording_buffer(self):
        with self.rec_lock:
            self.state.last_recording_buffer = None
            self.state.pending_obstacles = []

    def recording_status(self):
        with self.rec_lock:
            rec = self.state.recording
            buf = self.state.last_recording_buffer
            return {
                "active": rec is not None,
                "mode": rec["mode"] if rec else None,
                "points": len(rec["points"]) if rec else 0,
                "lastFix": rec["last_fix"] if rec else None,
                "rtkLostCount": rec.get("rtk_lost_count", 0) if rec else 0,
                "bufferedMode": buf["mode"] if buf else None,
                "bufferedPoints": len(buf["points"]) if buf else 0,
                "pendingObstacles": len(self.state.pending_obstacles),
            }

    # ── Validation helpers ─────────────────────────────────────────────────────

    @staticmethod
    def _polygon_area(pts):
        """Signed area (m²). Positive if CCW."""
        n = len(pts)
        if n < 3:
            return 0.0
        s = 0.0
        for i in range(n):
            x1, y1 = pts[i]
            x2, y2 = pts[(i + 1) % n]
            s += (x1 * y2) - (x2 * y1)
        return s / 2.0

    @staticmethod
    def _segments_intersect(a, b, c, d):
        """True if open segments AB and CD properly intersect (no shared endpoint)."""
        def cross(o, p, q):
            return (p[0] - o[0]) * (q[1] - o[1]) - (p[1] - o[1]) * (q[0] - o[0])
        d1 = cross(c, d, a)
        d2 = cross(c, d, b)
        d3 = cross(a, b, c)
        d4 = cross(a, b, d)
        if ((d1 > 0 and d2 < 0) or (d1 < 0 and d2 > 0)) and \
           ((d3 > 0 and d4 < 0) or (d3 < 0 and d4 > 0)):
            return True
        return False

    def _is_self_intersecting(self, pts):
        n = len(pts)
        if n < 4:
            return False
        for i in range(n):
            a = pts[i]
            b = pts[(i + 1) % n]
            for j in range(i + 1, n):
                if abs(i - j) <= 1 or (i == 0 and j == n - 1):
                    continue
                c = pts[j]
                d = pts[(j + 1) % n]
                if self._segments_intersect(a, b, c, d):
                    return True
        return False

    @staticmethod
    def _point_in_polygon(p, poly):
        x, y = p
        inside = False
        n = len(poly)
        j = n - 1
        for i in range(n):
            xi, yi = poly[i]
            xj, yj = poly[j]
            if ((yi > y) != (yj > y)) and \
               (x < (xj - xi) * (y - yi) / ((yj - yi) or 1e-9) + xi):
                inside = not inside
            j = i
        return inside

    def _close_polygon(self, pts):
        """Drop final point if it equals (within tolerance) the first."""
        if len(pts) < 2:
            return pts
        x0, y0 = pts[0]
        xn, yn = pts[-1]
        if math.hypot(xn - x0, yn - y0) < self.rec_close_tolerance_m:
            return pts[:-1]
        return pts

    def _validate_polygon(self, pts, label="polígono"):
        pts = self._close_polygon(pts)
        if len(pts) < 3:
            raise ValueError("{}: necesita al menos 3 puntos.".format(label))
        if abs(self._polygon_area(pts)) < self.rec_min_area_m2:
            raise ValueError("{}: área demasiado pequeña (< {} m²).".format(label, self.rec_min_area_m2))
        if self._is_self_intersecting(pts):
            raise ValueError("{}: el polígono se cruza consigo mismo.".format(label))
        # Normalise to CCW
        if self._polygon_area(pts) < 0:
            pts = list(reversed(pts))
        return pts

    # ── Build ROS messages ─────────────────────────────────────────────────────

    @staticmethod
    def _to_ros_polygon(pts):
        poly = Polygon()
        poly.points = [Point32(x=float(x), y=float(y), z=0.0) for (x, y) in pts]
        return poly

    @staticmethod
    def _yaw_to_quaternion(yaw_rad):
        half = yaw_rad / 2.0
        return Quaternion(x=0.0, y=0.0, z=math.sin(half), w=math.cos(half))

    # ── Map payload ────────────────────────────────────────────────────────────

    def map_payload(self):
        snap = self.state.snapshot()
        raw = snap.get("map_json", "") or ""
        # Try to parse so the iOS side gets a structured object; if the backend
        # JSON is malformed, return the raw string and let the client decide.
        parsed = None
        try:
            parsed = json.loads(raw) if raw else None
        except ValueError:
            parsed = None
        return {
            "map": parsed,
            "raw": raw if parsed is None else None,
            "receivedAt": int(snap.get("map_received_at", 0.0) * 1000),
            "hasMap": bool(parsed),
        }

    def pose_payload(self):
        snap = self.state.snapshot()
        pose = snap["pose"]
        if pose is None:
            return {"hasPose": False}
        x = float(pose.pose.pose.position.x)
        y = float(pose.pose.pose.position.y)
        heading_deg = math.degrees(float(pose.vehicle_heading))
        if heading_deg < 0:
            heading_deg += 360.0
        return {
            "hasPose": True,
            "x": x,
            "y": y,
            "headingDeg": heading_deg,
            "fixType": self._gps_fix_type(pose),
            "positionAccuracy": float(pose.position_accuracy),
        }

    # ── Save area / dock / clear ───────────────────────────────────────────────

    def save_area(self, body, is_navigation):
        """Save a mowing or navigation area.

        Body schema:
          {"name": str, "points"?: [[x,y]…], "obstacles"?: [[[x,y]…]…], "useBuffer"?: bool}
        If `useBuffer` is true, points and obstacles are taken from the bridge's
        last_recording_buffer and pending_obstacles instead of the request body.
        """
        snap = self.state.snapshot()
        self._check_hardware(snap)

        name = str(body.get("name", "")).strip() or ("Area " + time.strftime("%H:%M"))
        use_buffer = bool(body.get("useBuffer", False))

        if use_buffer:
            with self.rec_lock:
                buf = self.state.last_recording_buffer
                if buf is None:
                    raise RuntimeError("No hay grabación previa para guardar.")
                if (is_navigation and buf["mode"] != "nav") or (not is_navigation and buf["mode"] != "mow"):
                    raise RuntimeError("La grabación en buffer no coincide con el tipo solicitado.")
                points = list(buf["points"])
                obstacles = list(self.state.pending_obstacles) if not is_navigation else []
        else:
            points = [(float(p[0]), float(p[1])) for p in body.get("points", [])]
            obstacles = []
            if not is_navigation:
                for raw_obs in body.get("obstacles", []):
                    obstacles.append([(float(p[0]), float(p[1])) for p in raw_obs])

        validated = self._validate_polygon(points, label="zona")

        validated_obstacles = []
        for i, ob in enumerate(obstacles, start=1):
            ob_validated = self._validate_polygon(ob, label="exclusión {}".format(i))
            # Each obstacle vertex must lie inside the main polygon
            for p in ob_validated:
                if not self._point_in_polygon(p, validated):
                    raise ValueError(
                        "La exclusión {} se sale del contorno principal.".format(i)
                    )
            validated_obstacles.append(ob_validated)

        # Build ROS msg and call service
        area_msg = MapArea()
        area_msg.name = name
        area_msg.area = self._to_ros_polygon(validated)
        area_msg.obstacles = [self._to_ros_polygon(ob) for ob in validated_obstacles]

        req = AddMowingAreaSrvRequest()
        req.area = area_msg
        req.isNavigationArea = bool(is_navigation)

        try:
            rospy.wait_for_service("/mower_map_service/add_mowing_area", timeout=2.0)
            self.add_area_srv(req)
        except rospy.ROSException:
            raise RuntimeError("Servicio add_mowing_area no disponible.")
        except rospy.ServiceException as exc:
            raise RuntimeError("add_mowing_area falló: {}".format(exc))

        if use_buffer:
            self._clear_recording_buffer()

        return {
            "ok": True,
            "name": name,
            "points": len(validated),
            "obstacles": len(validated_obstacles),
            "isNavigationArea": bool(is_navigation),
        }

    def set_dock_here(self):
        snap = self.state.snapshot()
        self._check_hardware(snap)
        pose = snap["pose"]
        if pose is None:
            raise RuntimeError("Sin pose disponible.")
        if self.rec_require_rtk_fixed and self._gps_fix_type(pose) != "fixed":
            raise RuntimeError("Sin fix RTK Fixed — no se puede fijar dock.")

        x = float(pose.pose.pose.position.x)
        y = float(pose.pose.pose.position.y)
        yaw = float(pose.vehicle_heading)
        return self._call_set_dock(x, y, yaw)

    def set_dock_pose(self, body):
        try:
            x = float(body["x"])
            y = float(body["y"])
            yaw_deg = float(body.get("headingDeg", 0.0))
        except (KeyError, TypeError, ValueError):
            raise ValueError("Faltan x/y/headingDeg.")
        return self._call_set_dock(x, y, math.radians(yaw_deg))

    def _call_set_dock(self, x, y, yaw_rad):
        req = SetDockingPointSrvRequest()
        req.docking_pose = Pose()
        req.docking_pose.position.x = x
        req.docking_pose.position.y = y
        req.docking_pose.position.z = 0.0
        req.docking_pose.orientation = self._yaw_to_quaternion(yaw_rad)
        try:
            rospy.wait_for_service("/mower_map_service/set_docking_point", timeout=2.0)
            self.set_dock_srv(req)
        except rospy.ROSException:
            raise RuntimeError("Servicio set_docking_point no disponible.")
        except rospy.ServiceException as exc:
            raise RuntimeError("set_docking_point falló: {}".format(exc))
        return {"ok": True, "x": x, "y": y, "headingDeg": math.degrees(yaw_rad) % 360.0}

    def clear_full_map(self):
        try:
            rospy.wait_for_service("/mower_map_service/clear_map", timeout=2.0)
            self.clear_map_srv(ClearMapSrvRequest())
        except rospy.ROSException:
            raise RuntimeError("Servicio clear_map no disponible.")
        except rospy.ServiceException as exc:
            raise RuntimeError("clear_map falló: {}".format(exc))
        self._clear_recording_buffer()
        return {"ok": True}

    def _gps_fix_type(self, pose):
        """Classify RTK fix quality from AbsolutePose.position_accuracy."""
        if pose is None:
            return "none"
        has_recent = bool(pose.flags & AbsolutePose.FLAG_SENSOR_FUSION_RECENT_ABSOLUTE_POSE)
        if not has_recent:
            return "none"
        accuracy = float(pose.position_accuracy)
        if accuracy <= 0.05:
            return "fixed"   # RTK Fixed
        if accuracy <= 0.5:
            return "float"   # RTK Float
        return "single"      # GPS but no RTK

    def _check_hardware(self, snap, require_high_level=False):
        low = snap["low_level"]
        emergency = snap["emergency"]
        high = snap["high_level"]
        last_seen = snap.get("last_seen", {})
        now = time.time()
        low_age = now - last_seen.get("low_level", 0.0)
        if low is None or low_age > 3.0:
            raise RuntimeError(
                "Mega no conectado — sin datos de /ll/mower_status. "
                "Verifica mower_mega_bridge y el cable serie."
            )
        if emergency and (emergency.active_emergency or emergency.latched_emergency):
            raise RuntimeError(
                "Emergencia activa — desactiva la parada de emergencia antes de continuar."
            )
        high_age = now - last_seen.get("high_level", 0.0)
        if require_high_level and (high is None or high_age > 5.0):
            raise RuntimeError(
                "Módulo de lógica no disponible — /mower_logic/current_state sin datos. "
                "Verifica que mower_logic esté en ejecución."
            )

    def handle_command(self, command):
        snap = self.state.snapshot()
        rospy.loginfo("[ios_bridge] command: %s", command)
        if command in ("start", "exitDock"):
            self._check_hardware(snap, require_high_level=True)
            self._call_high_level(HighLevelControlSrvRequest.COMMAND_START)
        elif command == "dock":
            self._check_hardware(snap, require_high_level=True)
            self._call_high_level(HighLevelControlSrvRequest.COMMAND_HOME)
        elif command == "stop":
            self._check_hardware(snap)
            self.action_pub.publish(String("mower_logic:mowing/pause"))
            self._publish_manual_twist("stop")
            self.state.set_manual_active(False)
        elif command == "autoMode":
            self.state.set_manual_active(False)
        elif command == "manualMode":
            self.state.set_manual_active(True, hold_seconds=60.0)
        elif command == "bladeOn":
            self._check_hardware(snap)
            self._call_mower_control(True)
        elif command == "bladeOff":
            self._check_hardware(snap)
            self._call_mower_control(False)
        else:
            raise ValueError("Comando '{}' no reconocido.".format(command))

    def handle_manual(self, direction):
        self._check_hardware(self.state.snapshot())
        self._publish_manual_twist(direction)
        self.state.set_manual_active(direction != "stop")

    def handle_setting(self, body):
        setting_id = str(body.get("id", ""))
        value = float(body.get("value", 0))
        self.state.set_setting(setting_id, value)
        if setting_id == "bridge.manualLinearSpeed":
            self.linear_speed = clamp(value, 0.05, 1.0)
        elif setting_id == "bridge.manualAngularSpeed":
            self.angular_speed = clamp(value, 0.1, 2.0)
        elif setting_id.startswith("mega."):
            mega_key = setting_id[len("mega."):]
            # Format as integer if it's a whole number to keep payloads clean
            if value == int(value):
                val_str = str(int(value))
            else:
                val_str = "{:.4g}".format(value)
            self.cfgset_pub.publish(String("{}={}".format(mega_key, val_str)))

    def _call_high_level(self, command):
        svc = "/mower_service/high_level_control"
        try:
            rospy.wait_for_service(svc, timeout=1.0)
            self.high_level_srv(command)
        except rospy.ROSException:
            raise RuntimeError(
                "Servicio {} no disponible. "
                "¿Está mower_logic en ejecución?".format(svc)
            )
        except rospy.ServiceException as exc:
            raise RuntimeError("Error en {}: {}".format(svc, exc))

    def _call_mower_control(self, enabled):
        svc = "/ll/_service/mow_enabled"
        try:
            rospy.wait_for_service(svc, timeout=1.0)
            self.mower_control_srv(1 if enabled else 0, 0)
        except rospy.ROSException:
            raise RuntimeError(
                "Servicio {} no disponible. "
                "¿Está mower_mega_bridge en ejecución?".format(svc)
            )
        except rospy.ServiceException as exc:
            raise RuntimeError("Error en {}: {}".format(svc, exc))

    def _publish_manual_twist(self, direction):
        twist = Twist()
        if direction == "forward":
            twist.linear.x = self.linear_speed
        elif direction == "reverse":
            twist.linear.x = -self.linear_speed
        elif direction == "left":
            twist.angular.z = self.angular_speed
        elif direction == "right":
            twist.angular.z = -self.angular_speed
        elif direction == "stop":
            pass
        else:
            raise ValueError("unsupported manual direction '{}'".format(direction))
        self.joy_vel_pub.publish(twist)

    def _operating_state(self, high, low, emergency_active, charging, manual_active):
        if emergency_active:
            return "error"
        if manual_active:
            return "manual"
        if not high:
            return "docked" if charging else "unknown"

        name = (high.state_name or "").upper()
        sub = (high.sub_state_name or "").upper()
        text = "{} {}".format(name, sub)

        if "UNDOCK" in text:
            return "exitingDock"
        if "DOCK" in text and not charging:
            return "trackingWire"
        state_base = high.state & 0b11111
        if "MOW" in text or state_base == HighLevelStatus.HIGH_LEVEL_STATE_AUTONOMOUS:
            return "mowing"
        if state_base == HighLevelStatus.HIGH_LEVEL_STATE_RECORDING:
            return "setup"
        if state_base == HighLevelStatus.HIGH_LEVEL_STATE_IDLE:
            return "docked" if charging or (low and low.is_charging) else "parked"
        return "unknown"

    def _udp_beacon_loop(self):
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        while not rospy.is_shutdown():
            ip = self._local_ip()
            auth = "token" if self.auth_token else "open"
            payload = "MOWER|{}|{}:{}|auth={}".format(self.discovery_name, ip, self.port, auth).encode("utf-8")
            try:
                sock.sendto(payload, ("255.255.255.255", self.beacon_port))
            except OSError as exc:
                rospy.logdebug("iOS bridge UDP beacon failed: %s", exc)
            time.sleep(2.0)

    def _local_ip(self):
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            sock.connect(("8.8.8.8", 80))
            return sock.getsockname()[0]
        except OSError:
            return "127.0.0.1"
        finally:
            sock.close()

    def _first_valid(self, values):
        for value in values:
            if value and value > 0:
                return float(value)
        return 0.0

    def _rpm_to_pwm(self, rpm):
        if rpm == 0:
            return 0
        return int(clamp(abs(int(rpm)) / 20, 1, 255))


def main():
    rospy.init_node("mower_ios_bridge")
    bridge = IOSBridge()
    bridge.start()
    rospy.on_shutdown(bridge.stop)
    rospy.spin()


if __name__ == "__main__":
    main()
