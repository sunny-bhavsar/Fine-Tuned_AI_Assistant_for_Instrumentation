# =============================================================================
#  dashboard_app5.py  —  AI Industrial Plant Controller (FINAL VERSION)
#  Three closed-loop PID control loops: Tank Level · Boiler Temp · Pressure
# =============================================================================

import streamlit as st
import re
import threading
import time
from datetime import datetime
from collections import deque

try:
    from opcua import Client, ua
except ImportError:
    st.error("Missing: pip install opcua"); st.stop()
try:
    from openai import OpenAI
except ImportError:
    st.error("Missing: pip install openai"); st.stop()
try:
    from streamlit_autorefresh import st_autorefresh
    AUTOREFRESH_OK = True
except ImportError:
    AUTOREFRESH_OK = False

# ── PAGE CONFIG ────────────────────────────────────────────────────────────────
st.set_page_config(
    page_title="AI Plant Controller",
    page_icon="🏭",
    layout="wide",
    initial_sidebar_state="collapsed",
)

# ── AUTO REFRESH (WITH TOGGLE TO PREVENT CHAT INTERRUPTIONS) ───────────────────
colA, colB = st.columns([1, 1])
with colA:
    auto_refresh_on = st.toggle("🟢 Live Dashboard Auto-Refresh", value=True, 
                                help="Turn this OFF while chatting with the AI so the page doesn't refresh and interrupt it.")

if AUTOREFRESH_OK and auto_refresh_on:
    st_autorefresh(interval=5000, key="plant_refresh")

# ── CSS ────────────────────────────────────────────────────────────────────────
st.markdown("""
<style>
  html, body, [data-testid="stApp"]            { background:#0b0f19; color:#d0dbe8; }
  [data-testid="stSidebar"]                    { background:#0f1525; }
  section[data-testid="stMain"]                { padding-top:1rem; }
  h1,h2,h3,h4  { font-family:'Courier New',monospace; letter-spacing:.03em; color:#e8f0fe; }
  h2  { font-size:1.05rem !important; }
  h3  { font-size:0.95rem !important; }
  p, div, span, label { font-size:0.88rem; }
  [data-testid="stMetricValue"] { font-size:1.05rem !important; color:#00d4ff !important; font-family:monospace; }
  [data-testid="stMetricLabel"] { font-size:0.75rem !important; color:#6e8098 !important; }
  [data-testid="stMetricDelta"] { font-size:0.75rem !important; }
  [data-testid="stProgress"] > div > div       { background:#1e2d45 !important; }
  [data-testid="stProgress"] > div > div > div { background:linear-gradient(90deg,#0077b6,#00d4ff) !important; border-radius:4px !important; }
  [data-testid="stChatMessage"] { background:#131929; border-radius:8px; border:1px solid #1e2d45; }
  [data-testid="stButton"] > button { background:#131929; color:#00d4ff; border:1px solid #00d4ff; border-radius:6px; font-size:0.82rem; }
  [data-testid="stButton"] > button:hover { background:#0f2040; }
  hr { border-color:#1e2d45 !important; }
  .badge-ok   { color:#23d47e; font-weight:700; font-size:0.8rem; }
  .badge-warn { color:#f0883e; font-weight:700; font-size:0.8rem; }
  .badge-crit { color:#f85149; font-weight:700; font-size:0.8rem; }
  .pid-row    { background:#131929; border:1px solid #1e2d45; padding:5px 10px; border-radius:4px; margin:4px 0; font-size:0.78rem; font-family:'Courier New',monospace; color:#8ecae6; }
  .alarm-msg  { background:#1a0e0e; border-left:3px solid #f85149; padding:5px 10px; border-radius:4px; margin:2px 0; font-size:0.82rem; color:#f85149; }
  .warn-msg   { background:#1a140a; border-left:3px solid #f0883e; padding:5px 10px; border-radius:4px; margin:2px 0; font-size:0.82rem; color:#f0883e; }
</style>
""", unsafe_allow_html=True)

# ── CONFIG ─────────────────────────────────────────────────────────────────────
PLC_URL   = "opc.tcp://localhost:4840"
NS        = 4
BASE      = "|var|CODESYS Control Win V3 x64.Application.PLC_PRG."
LLM_URL   = "http://localhost:1234/v1"
LLM_KEY   = "lm-studio"
LLM_MODEL = "local-model"

NODES = {
    "TankLevel":        f"ns={NS};s={BASE}TankLevel",
    "TankSetpoint":     f"ns={NS};s={BASE}TankSetpoint",
    "TankValveCmd":     f"ns={NS};s={BASE}TankValveCmd",
    "BoilerTemp":       f"ns={NS};s={BASE}BoilerTemp",
    "BoilerSetpoint":   f"ns={NS};s={BASE}BoilerSetpoint",
    "BoilerHeaterCmd":  f"ns={NS};s={BASE}BoilerHeaterCmd",
    "PressureVal":      f"ns={NS};s={BASE}PressureVal",
    "PressureSetpoint": f"ns={NS};s={BASE}PressureSetpoint",
    "PressurePumpCmd":  f"ns={NS};s={BASE}PressurePumpCmd",
}

THRESH_WARN = {"tank": 15.0, "boiler": 15.0, "pressure": 1.2}
THRESH_CRIT = {"tank": 30.0, "boiler": 30.0, "pressure": 2.5}

# ── PID CONTROLLER (DEADBAND AWARE) ────────────────────────────────────────────
class PIDController:
    """
    Combines your friend's higher gains with Deadband Compensation.
    If the PLC needs to move, we instantly skip over the 45-55 dead zone 
    so the PLC actually reacts, preventing "fake stability".
    """
    def __init__(self, kp, ki, kd, deadband=5.2, tolerance=0.2, lo=0.0, hi=100.0):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.db = deadband       # Offset to bypass CODESYS 45-55 deadband
        self.tol = tolerance     # Acceptable error margin before safely idling at 50%
        self.lo = lo
        self.hi = hi
        self._integral = 0.0
        self._prev_pv  = None

    def reset(self):
        self._integral = 0.0
        self._prev_pv  = None

    def compute(self, pv, sp, dt=1.0):
        error = sp - pv

        # 1. Perfectly at setpoint? Idle at 50 so PLC goes to STABLE
        if abs(error) <= self.tol:
            self._integral = 0.0
            return 50.0

        # 2. Derivative
        d_term = 0.0
        if self._prev_pv is not None:
            d_term = -self.kd * (pv - self._prev_pv) / dt
        self._prev_pv = pv

        # 3. Integral (capped to prevent wild wind-up)
        self._integral += error * dt
        self._integral = max(-15.0, min(self._integral, 15.0))

        # 4. Raw PID Effort
        effort = self.kp * error + self.ki * self._integral + d_term

        # 5. Deadband jump
        if effort > 0:
            raw = 50.0 + self.db + effort
        elif effort < 0:
            raw = 50.0 - self.db + effort
        else:
            raw = 50.0

        output = max(self.lo, min(raw, self.hi))
        return round(output, 2)

    def get_terms(self, pv, sp):
        error = sp - pv
        p = round(self.kp * error, 2)
        i = round(self.ki * self._integral, 2)
        d = 0.0 if self._prev_pv is None else round(-self.kd * (pv - self._prev_pv), 2)
        return p, i, d


# ── PERSISTENT STATE ───────────────────────────────────────────────────────────
@st.cache_resource
def get_state():
    return {
        "plc":        {},
        "online":     False,
        "alarms":     deque(maxlen=40),
        "start_time": {
            "tank":     datetime.now(),
            "boiler":   datetime.now(),
            "pressure": datetime.now(),
        },
        "threads_up":   False,
        "lock":         threading.Lock(),
        "_alarm_dedup": "",
    }

@st.cache_resource
def get_pids():
    # Higher gains (friend's idea) + Deadband comp + dt=1.0s
    return {
        "tank":     PIDController(kp=2.5,  ki=0.15, kd=1.5, deadband=5.2, tolerance=0.2),
        "boiler":   PIDController(kp=1.2,  ki=0.08, kd=0.8, deadband=5.2, tolerance=0.2),
        "pressure": PIDController(kp=10.0, ki=0.50, kd=3.0, deadband=5.2, tolerance=0.05),
    }

@st.cache_resource
def get_ai_client():
    return OpenAI(base_url=LLM_URL, api_key=LLM_KEY)

STATE = get_state()
PIDS  = get_pids()
AI    = get_ai_client()

def write_plc(var_name, value):
    try:
        # UI uses short-lived connection, doesn't interfere with background thread
        c = Client(PLC_URL); c.connect()
        node = c.get_node(NODES[var_name])
        node.set_value(ua.DataValue(ua.Variant(float(value), ua.VariantType.Float)))
        c.disconnect()
        return True
    except Exception:
        return False

# ── PID THREAD WITH PERSISTENT CONNECTION ──────────────────────────────────────
def controller_thread():
    dt = 1.0  # Increased to 1.0s for stability, per your friend's correct suggestion
    client = Client(PLC_URL)
    is_connected = False

    while True:
        try:
            if not is_connected:
                client.connect()
                is_connected = True

            # Persistent fast read
            data = {k: round(client.get_node(v).get_value(), 3) for k, v in NODES.items()}

            if data:
                with STATE["lock"]:
                    STATE["plc"]    = data
                    STATE["online"] = True

                # Compute new outputs
                out_tank = PIDS["tank"].compute(data["TankLevel"], data["TankSetpoint"], dt)
                out_boil = PIDS["boiler"].compute(data["BoilerTemp"], data["BoilerSetpoint"], dt)
                out_pres = PIDS["pressure"].compute(data["PressureVal"], data["PressureSetpoint"], dt)

                # Persistent fast write
                client.get_node(NODES["TankValveCmd"]).set_value(ua.DataValue(ua.Variant(float(out_tank), ua.VariantType.Float)))
                client.get_node(NODES["BoilerHeaterCmd"]).set_value(ua.DataValue(ua.Variant(float(out_boil), ua.VariantType.Float)))
                client.get_node(NODES["PressurePumpCmd"]).set_value(ua.DataValue(ua.Variant(float(out_pres), ua.VariantType.Float)))

                # Alarm check
                now = datetime.now().strftime("%H:%M:%S")
                checks = [
                    ("tank",     data["TankSetpoint"]     - data["TankLevel"],   "Tank",     "%"),
                    ("boiler",   data["BoilerSetpoint"]   - data["BoilerTemp"],  "Boiler",   "°C"),
                    ("pressure", data["PressureSetpoint"] - data["PressureVal"], "Pressure", " bar"),
                ]
                for loop_key, error, label, unit in checks:
                    ae = abs(error)
                    if   ae > THRESH_CRIT[loop_key]: sev = "CRITICAL"
                    elif ae > THRESH_WARN[loop_key]: sev = "WARNING"
                    else: continue
                    msg      = f"[{now}] {sev} — {label} deviation {round(ae,1)}{unit} from SP"
                    dedup    = f"{loop_key}{sev}"
                    with STATE["lock"]:
                        if dedup != STATE["_alarm_dedup"]:
                            STATE["alarms"].append((sev, msg))
                            STATE["_alarm_dedup"] = dedup
            else:
                with STATE["lock"]:
                    STATE["online"] = False

        except Exception as e:
            # If OPC drops, reset connection flag and try again next loop
            is_connected = False
            with STATE["lock"]:
                STATE["online"] = False
            try:
                client.disconnect()
            except: pass

        time.sleep(dt)

# ── START THREAD ONCE ──────────────────────────────────────────────────────────
with STATE["lock"]:
    if not STATE["threads_up"]:
        threading.Thread(target=controller_thread, daemon=True, name="pid").start()
        STATE["threads_up"] = True

# ── HELPERS ────────────────────────────────────────────────────────────────────
def fmt_err(err):
    return ("+" if err >= 0 else "") + str(round(err, 1))

def loop_badge(error, loop):
    ae = abs(error)
    if ae > THRESH_CRIT[loop]: return '<span class="badge-crit">● CRITICAL</span>'
    if ae > THRESH_WARN[loop]: return '<span class="badge-warn">● WARNING</span>'
    return '<span class="badge-ok">● OK</span>'

def act_label(cmd, up, dn, hold):
    if cmd > 55: return f'<span class="badge-warn">{up}</span>'
    if cmd < 45: return f'<span class="badge-warn">{dn}</span>'
    return f'<span class="badge-ok">{hold}</span>'

def get_uptime(loop):
    start = STATE["start_time"].get(loop)
    if not start: return "--"
    secs = int((datetime.now() - start).total_seconds())
    h, rem = divmod(secs, 3600)
    m, s   = divmod(rem, 60)
    return f"{h}h {m:02d}m" if h > 0 else f"{m}m {s:02d}s"

# ── SNAPSHOT ───────────────────────────────────────────────────────────────────
with STATE["lock"]:
    D      = dict(STATE["plc"])
    online = STATE["online"]
    alarms = list(STATE["alarms"])

if "sp_init" not in st.session_state:
    st.session_state.sp_tank     = int(round(D.get("TankSetpoint",    50.0)))
    st.session_state.sp_boiler   = int(round(D.get("BoilerSetpoint",  70.0)))
    st.session_state.sp_pressure = round(D.get("PressureSetpoint", 5.0) * 2) / 2
    st.session_state.sp_init     = True

# ══════════════════════════════════════════════════════════════════════════════
#  HEADER
# ══════════════════════════════════════════════════════════════════════════════
hdr_l, hdr_r = st.columns([6, 1])
with hdr_l:
    st.markdown("## 🏭 AI Plant Controller")
    st.caption("Tank Level · Boiler Temperature · Pressure  |  PID closed-loop control")
with hdr_r:
    if online:
        st.success("● ONLINE")
    else:
        st.error("● OFFLINE")
    if st.button("↺ Refresh", use_container_width=True):
        st.rerun()

st.divider()

# ══════════════════════════════════════════════════════════════════════════════
#  LOOP PANELS
# ══════════════════════════════════════════════════════════════════════════════
if online and D:
    c1, c2, c3 = st.columns(3)

    # ── TANK ──────────────────────────────────────────────────────────────────
    with c1:
        lvl = D.get("TankLevel",    0.0)
        sp  = D.get("TankSetpoint", 50.0)
        vlv = D.get("TankValveCmd", 50.0)
        err = sp - lvl
        p, i, d = PIDS["tank"].get_terms(lvl, sp)

        st.markdown("### 💧 Tank Level")
        m1, m2, m3 = st.columns(3)
        m1.metric("Level",    f"{lvl:.1f} %",  fmt_err(err) + " %")
        m2.metric("Setpoint", f"{sp:.1f} %")
        m3.metric("Valve",    f"{vlv:.1f} %")
        st.progress(max(0, min(int(lvl), 100)))
        st.markdown(
            loop_badge(err, "tank") + " &nbsp; "
            + act_label(vlv, "FILLING", "DRAINING", "STABLE")
            + f'  <span style="color:#6e8098;font-size:0.75rem">Up:{get_uptime("tank")}</span>',
            unsafe_allow_html=True,
        )
        st.markdown(f'<div class="pid-row">P={p}  I={i}  D={d}</div>', unsafe_allow_html=True)

        new_sp = st.slider("Tank Setpoint (%)", 0, 100,
                           value=st.session_state.sp_tank, step=1, key="sl_tank")
        if new_sp != st.session_state.sp_tank:
            st.session_state.sp_tank = new_sp
            write_plc("TankSetpoint", float(new_sp))
            PIDS["tank"].reset()

    # ── BOILER ────────────────────────────────────────────────────────────────
    with c2:
        tmp = D.get("BoilerTemp",      0.0)
        sp  = D.get("BoilerSetpoint",  70.0)
        htr = D.get("BoilerHeaterCmd", 50.0)
        err = sp - tmp
        p, i, d = PIDS["boiler"].get_terms(tmp, sp)

        st.markdown("### 🔥 Boiler Temp")
        m1, m2, m3 = st.columns(3)
        m1.metric("Temp",     f"{tmp:.1f} °C", fmt_err(err) + " °C")
        m2.metric("Setpoint", f"{sp:.1f} °C")
        m3.metric("Heater",   f"{htr:.1f} %")
        st.progress(max(0, min(int((tmp / 150) * 100), 100)))
        st.markdown(
            loop_badge(err, "boiler") + " &nbsp; "
            + act_label(htr, "HEATING", "COOLING", "STABLE")
            + f'  <span style="color:#6e8098;font-size:0.75rem">Up:{get_uptime("boiler")}</span>',
            unsafe_allow_html=True,
        )
        st.markdown(f'<div class="pid-row">P={p}  I={i}  D={d}</div>', unsafe_allow_html=True)

        new_sp = st.slider("Boiler Setpoint (°C)", 0, 150,
                           value=st.session_state.sp_boiler, step=1, key="sl_boiler")
        if new_sp != st.session_state.sp_boiler:
            st.session_state.sp_boiler = new_sp
            write_plc("BoilerSetpoint", float(new_sp))
            PIDS["boiler"].reset()

    # ── PRESSURE ──────────────────────────────────────────────────────────────
    with c3:
        prs = D.get("PressureVal",      0.0)
        sp  = D.get("PressureSetpoint", 5.0)
        pmp = D.get("PressurePumpCmd",  50.0)
        err = sp - prs
        p, i, d = PIDS["pressure"].get_terms(prs, sp)

        st.markdown("### 💨 Pressure")
        m1, m2, m3 = st.columns(3)
        m1.metric("Pressure", f"{prs:.2f} bar", fmt_err(err))
        m2.metric("Setpoint", f"{sp:.1f} bar")
        m3.metric("Pump",     f"{pmp:.1f} %")
        st.progress(max(0, min(int((prs / 10) * 100), 100)))
        st.markdown(
            loop_badge(err, "pressure") + " &nbsp; "
            + act_label(pmp, "PRESSURISING", "VENTING", "STABLE")
            + f'  <span style="color:#6e8098;font-size:0.75rem">Up:{get_uptime("pressure")}</span>',
            unsafe_allow_html=True,
        )
        st.markdown(f'<div class="pid-row">P={p}  I={i}  D={d}</div>', unsafe_allow_html=True)

        new_sp = st.slider("Pressure Setpoint (bar)", 0.0, 10.0, step=0.5,
                           value=st.session_state.sp_pressure, key="sl_prs")
        if abs(new_sp - st.session_state.sp_pressure) > 0.01:
            st.session_state.sp_pressure = new_sp
            write_plc("PressureSetpoint", float(new_sp))
            PIDS["pressure"].reset()

else:
    st.warning(
        "PLC offline — open CODESYS → F11 → Online → Login → Start. "
        "Ensure OPC UA anonymous login is enabled on port 4840."
    )

st.divider()

# ══════════════════════════════════════════════════════════════════════════════
#  ALARM LOG
# ══════════════════════════════════════════════════════════════════════════════
with st.expander("🔔 Alarm Log", expanded=bool(alarms)):
    if alarms:
        for sev, msg in reversed(alarms):
            css = "alarm-msg" if sev == "CRITICAL" else "warn-msg"
            st.markdown(f'<div class="{css}">⚠ {msg}</div>', unsafe_allow_html=True)
    else:
        st.caption("No alarms. All loops within tolerance.")

st.divider()

# ══════════════════════════════════════════════════════════════════════════════
#  AI OPERATOR CHAT
# ══════════════════════════════════════════════════════════════════════════════
st.markdown("### 💬 AI Operator")

if "messages" not in st.session_state:
    st.session_state.messages = []

for msg in st.session_state.messages:
    with st.chat_message(msg["role"]):
        st.markdown(msg["content"])

def build_system_prompt(data):
    if not data:
        plant_block = "ALL SENSORS OFFLINE"
    else:
        def row(name, pv, sp, act, pu, au):
            return (
                f"  {name:<10}PV={round(pv,2):>7}{pu}"
                f"  SP={round(sp,2):>7}{pu}"
                f"  ACT={round(act,1):>5}{au}"
                f"  ERR={fmt_err(sp-pv):>7}{pu}"
            )
        plant_block = "\n".join([
            row("TANK",     data["TankLevel"],   data["TankSetpoint"],    data["TankValveCmd"],    "%",    "% open"),
            row("BOILER",   data["BoilerTemp"],  data["BoilerSetpoint"],  data["BoilerHeaterCmd"], "°C",   "% power"),
            row("PRESSURE", data["PressureVal"], data["PressureSetpoint"],data["PressurePumpCmd"], " bar", "% pump"),
        ])

    alarm_block = "\n".join(msg for _, msg in list(alarms)[-5:]) if alarms else "No active alarms."

    return (
        "You are an expert AI process controller for an industrial plant with PID control loops.\n"
        "LIVE PLANT DATA:\n" + plant_block + "\n\n"
        "ACTIVE ALARMS:\n" + alarm_block + "\n\n"
        "COMMANDS (embed in reply when action needed):\n"
        "  TANK_SP=<0-100>\n  BOILER_SP=<0-150>\n  PRESSURE_SP=<0-10>\n"
        "RESPONSE FORMAT: 1-2 sentences, then command tags if needed."
    )

SP_CMDS = [
    (r"TANK_SP\s*=\s*([\d.]+)",     "TankSetpoint",    0,   100, "Tank Setpoint"),
    (r"BOILER_SP\s*=\s*([\d.]+)",   "BoilerSetpoint",  0,   150, "Boiler Setpoint"),
    (r"PRESSURE_SP\s*=\s*([\d.]+)", "PressureSetpoint",0,    10, "Pressure Setpoint"),
]
PID_KEY    = {"TankSetpoint": "tank", "BoilerSetpoint": "boiler", "PressureSetpoint": "pressure"}
SP_SS_KEY  = {"TankSetpoint": "sp_tank", "BoilerSetpoint": "sp_boiler", "PressureSetpoint": "sp_pressure"}

def extract_and_execute(reply):
    actions = []
    for pattern, var, lo, hi, label in SP_CMDS:
        m = re.search(pattern, reply, re.IGNORECASE)
        if m:
            val = round(max(float(lo), min(float(m.group(1)), float(hi))), 2)
            ok  = write_plc(var, val)
            PIDS[PID_KEY[var]].reset()
            st.session_state[SP_SS_KEY[var]] = (
                round(val * 2) / 2 if var == "PressureSetpoint" else int(round(val))
            )
            actions.append((label, val, ok))
    return actions

def ask_llm(user_text, data):
    try:
        resp = AI.chat.completions.create(
            model=LLM_MODEL,
            messages=[
                {"role": "system", "content": build_system_prompt(data)},
                {"role": "user",   "content": user_text},
            ],
            temperature=0.1,
            max_tokens=300,
        )
        return resp.choices[0].message.content
    except Exception as exc:
        return f"LLM error: {exc}"

if prompt := st.chat_input("e.g. 'fill the tank', 'set boiler to 110°C'"):
    st.session_state.messages.append({"role": "user", "content": prompt})
    with st.chat_message("user"):
        st.markdown(prompt)

    with STATE["lock"]:
        fresh = dict(STATE["plc"]) or None

    with st.spinner("Thinking…"):
        reply = ask_llm(prompt, fresh)

    actions = extract_and_execute(reply)

    action_md = ""
    if actions:
        action_md = "\n\n**Actions taken:**\n"
        for name, val, ok in actions:
            action_md += f"- {'✅' if ok else '❌'} **{name}** → `{val}`\n"

    final_reply = reply + action_md
    st.session_state.messages.append({"role": "assistant", "content": final_reply})
    with st.chat_message("assistant"):
        st.markdown(final_reply)