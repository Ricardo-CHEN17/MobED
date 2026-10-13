#!/usr/bin/env python3
"""
MobED Dedicated Floating Teleop Dashboard (Tkinter + pynput)
============================================================
A standalone, lightweight floating GUI dashboard for controlling MobED.
Decouples keyboard teleoperation from the MuJoCo 3D viewer, eliminating
all shortcut conflicts (A, D, C, 1-9, [, ], Space).

Features:
- Floating always-on-top window (stays visible alongside MuJoCo)
- Multi-key rollover with KeyPress / KeyRelease tracking
- Optional Global Background Key Capture (via pynput) so keys work even
  when looking at terminal logs
- Real-time dark HUD with speed bars, height sliders, posture badges, curb height
- Dual-cast UDP: sends to mac_mujoco_sim (:9002) and Docker udp_interface (:9001)
- Receives live physics telemetry from mac_mujoco_sim (:9003)

Usage:
    .venv/bin/python mac_teleop_gui.py
"""

import sys
import os
import time
import math
import socket
import json
import threading
import tkinter as tk
from tkinter import ttk

# Optional global keyboard listener via pynput
try:
    from pynput import keyboard as pynput_kbd
    PYNPUT_AVAILABLE = True
except ImportError:
    PYNPUT_AVAILABLE = False


# ----------------------------------------------------------------------
# Configuration & Network Constants
# ----------------------------------------------------------------------
UDP_SIM_TARGET = ('127.0.0.1', 9002)     # mac_mujoco_sim.py teleop port
UDP_DOCKER_TARGET = ('127.0.0.1', 9001)  # Docker udp_interface.py port
TELEMETRY_PORT = 9003                    # Receive feedback from sim

# Dark Palette
BG_COLOR = "#141821"
CARD_BG = "#1f2430"
CARD_BORDER = "#2e3748"
TEXT_MAIN = "#f0f4f8"
TEXT_MUTED = "#8a99ad"
ACCENT_CYAN = "#00e5ff"
ACCENT_GREEN = "#00e676"
ACCENT_AMBER = "#ffab00"
ACCENT_RED = "#ff1744"
BTN_BG = "#2a3243"
BTN_ACTIVE = "#0097a7"


class TeleopDashboardApp:
    def __init__(self, root):
        self.root = root
        self.root.title("MobED Teleop Dashboard")
        self.root.geometry("540x480")
        self.root.configure(bg=BG_COLOR)
        self.root.resizable(False, False)

        # Make window stay on top of MuJoCo & terminals
        self.root.attributes("-topmost", True)

        # ---------------- State Variables ----------------
        self.target_linear = 0.5    # m/s
        self.target_angular = 1.0   # rad/s
        self.vx = 0.0
        self.vy = 0.0
        self.wz = 0.0

        self.height = 0.205         # m (Mode 1 nominal 37 deg outward cruising height)
        self.roll = 0.0             # rad
        self.pitch = 0.0            # rad

        self.posture_mode = 0       # 0: OUTWARD, 1: INWARD, 2: SPIN
        self.curb_height = 0.075    # m [0.04m, 0.15m]
        self.climb_mode = 1         # 1: LOW 80mm / ADAPTIVE, 2: HIGH 200mm
        self.e_stop = False

        self.active_keys = set()
        self.global_capture_enabled = tk.BooleanVar(value=True if PYNPUT_AVAILABLE else False)

        # Telemetry from sim
        self.sim_connected = False
        self.last_telemetry_time = 0.0
        self.actual_height = 0.205
        self.actual_roll = 0.0
        self.actual_pitch = 0.0
        self.actual_vx = 0.0
        self.actual_wz = 0.0

        # UDP Sockets
        self.send_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.send_sock.setblocking(False)

        self.recv_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            self.recv_sock.bind(('0.0.0.0', TELEMETRY_PORT))
            self.recv_sock.setblocking(False)
        except OSError:
            pass

        # Build UI
        self._build_ui()

        # Keyboard event bindings (Window focus)
        self.root.bind("<KeyPress>", self._on_key_press)
        self.root.bind("<KeyRelease>", self._on_key_release)

        # Background listener (Global focus)
        self.pynput_listener = None
        if PYNPUT_AVAILABLE:
            self._start_pynput_listener()

        # Periodic timer (50 Hz / 20ms)
        self.root.after(20, self._tick_loop)

    def _build_ui(self):
        # Header Frame
        header = tk.Frame(self.root, bg=BG_COLOR)
        header.pack(fill="x", padx=16, pady=(12, 6))

        title = tk.Label(header, text="MobED TELEOP DASHBOARD", font=("Helvetica", 14, "bold"),
                         fg=TEXT_MAIN, bg=BG_COLOR)
        title.pack(side="left")

        self.lbl_status = tk.Label(header, text="● STANDALONE READY", font=("Helvetica", 10, "bold"),
                                   fg=ACCENT_AMBER, bg=BG_COLOR)
        self.lbl_status.pack(side="right")

        # Row 1: Motion Card & Posture Card
        row1 = tk.Frame(self.root, bg=BG_COLOR)
        row1.pack(fill="x", padx=16, pady=4)

        # Motion Panel (Left)
        card_motion = tk.LabelFrame(row1, text=" MOTION (WASD / QE / ZC) ", font=("Helvetica", 10, "bold"),
                                    fg=ACCENT_CYAN, bg=CARD_BG, bd=1, relief="solid")
        card_motion.pack(side="left", fill="both", expand=True, padx=(0, 6))

        self.lbl_presets = tk.Label(card_motion, text=f"Preset: Lin {self.target_linear:.1f} m/s (1/2) | Ang {self.target_angular:.1f} r/s (3/4)",
                                    font=("Courier", 9), fg=TEXT_MUTED, bg=CARD_BG)
        self.lbl_presets.pack(anchor="w", padx=10, pady=(6, 2))

        self.lbl_vx = tk.Label(card_motion, text=f"Cmd Vx:  {self.vx:+.2f} m/s",
                               font=("Helvetica", 11, "bold"), fg=TEXT_MAIN, bg=CARD_BG)
        self.lbl_vx.pack(anchor="w", padx=10)

        self.lbl_wz = tk.Label(card_motion, text=f"Cmd Wz:  {self.wz:+.2f} rad/s",
                               font=("Helvetica", 11, "bold"), fg=TEXT_MAIN, bg=CARD_BG)
        self.lbl_wz.pack(anchor="w", padx=10)

        self.lbl_vy = tk.Label(card_motion, text=f"Cmd Vy:  {self.vy:+.2f} m/s",
                               font=("Helvetica", 10), fg=TEXT_MUTED, bg=CARD_BG)
        self.lbl_vy.pack(anchor="w", padx=10)

        self.lbl_sim_act = tk.Label(card_motion, text="Sim Act: Vx=+0.00, Wz=+0.00",
                                    font=("Courier", 9), fg=TEXT_MUTED, bg=CARD_BG)
        self.lbl_sim_act.pack(anchor="w", padx=10, pady=(4, 6))

        # Attitude & Suspension Panel (Right)
        card_att = tk.LabelFrame(row1, text=" ATTITUDE & HEIGHT ", font=("Helvetica", 10, "bold"),
                                 fg=ACCENT_CYAN, bg=CARD_BG, bd=1, relief="solid")
        card_att.pack(side="right", fill="both", expand=True, padx=(6, 0))

        self.lbl_height = tk.Label(card_att, text=f"Height (U/J):  {self.height*1000:.1f} mm",
                                   font=("Helvetica", 11, "bold"), fg=ACCENT_CYAN, bg=CARD_BG)
        self.lbl_height.pack(anchor="w", padx=10, pady=(6, 2))

        self.lbl_roll = tk.Label(card_att, text=f"Roll  (I/K):  {self.roll:+.2f} rad (0.0°)",
                                 font=("Helvetica", 10), fg=TEXT_MAIN, bg=CARD_BG)
        self.lbl_roll.pack(anchor="w", padx=10)

        self.lbl_pitch = tk.Label(card_att, text=f"Pitch (O/L):  {self.pitch:+.2f} rad (0.0°)",
                                  font=("Helvetica", 10), fg=TEXT_MAIN, bg=CARD_BG)
        self.lbl_pitch.pack(anchor="w", padx=10)

        self.lbl_sim_att = tk.Label(card_att, text="IMU Act: R=+0.0°, P=+0.0°",
                                    font=("Courier", 9), fg=TEXT_MUTED, bg=CARD_BG)
        self.lbl_sim_att.pack(anchor="w", padx=10, pady=(12, 6))

        # Row 2: Posture Mode Buttons
        row2 = tk.Frame(self.root, bg=BG_COLOR)
        row2.pack(fill="x", padx=16, pady=4)

        lbl_posture = tk.Label(row2, text="Posture Configuration (7/8/9):", font=("Helvetica", 10, "bold"),
                               fg=TEXT_MAIN, bg=BG_COLOR)
        lbl_posture.pack(side="left", padx=(0, 10))

        self.btn_posture_out = tk.Button(row2, text="OUTWARD (7)", font=("Helvetica", 9, "bold"),
                                         bg=BTN_ACTIVE, fg="white", activebackground=BTN_ACTIVE,
                                         command=lambda: self._set_posture(0), relief="flat", padx=6)
        self.btn_posture_out.pack(side="left", padx=3)

        self.btn_posture_in = tk.Button(row2, text="INWARD (8)", font=("Helvetica", 9),
                                        bg=BTN_BG, fg=TEXT_MUTED, activebackground=BTN_ACTIVE,
                                        command=lambda: self._set_posture(1), relief="flat", padx=6)
        self.btn_posture_in.pack(side="left", padx=3)

        self.btn_posture_spin = tk.Button(row2, text="SPIN (9)", font=("Helvetica", 9),
                                          bg=BTN_BG, fg=TEXT_MUTED, activebackground=BTN_ACTIVE,
                                          command=lambda: self._set_posture(2), relief="flat", padx=6)
        self.btn_posture_spin.pack(side="left", padx=3)

        # Row 3: Curb Height & Climb Mode
        row3 = tk.Frame(self.root, bg=BG_COLOR)
        row3.pack(fill="x", padx=16, pady=4)

        self.lbl_curb = tk.Label(row3, text=f"Curb Target [ / ]:  {self.curb_height*1000:.0f} mm",
                                 font=("Helvetica", 10, "bold"), fg=ACCENT_AMBER, bg=BG_COLOR)
        self.lbl_curb.pack(side="left", padx=(0, 6))

        btn_curb_down = tk.Button(row3, text=" - ", font=("Helvetica", 9, "bold"), bg=BTN_BG, fg=TEXT_MAIN,
                                  command=self._curb_down, relief="flat", padx=4)
        btn_curb_down.pack(side="left", padx=2)

        btn_curb_up = tk.Button(row3, text=" + ", font=("Helvetica", 9, "bold"), bg=BTN_BG, fg=TEXT_MAIN,
                                command=self._curb_up, relief="flat", padx=4)
        btn_curb_up.pack(side="left", padx=2)

        self.btn_climb = tk.Button(row3, text="CLIMB: ENABLED (Adaptive \\)", font=("Helvetica", 9, "bold"),
                                   bg=BTN_ACTIVE, fg=TEXT_MAIN, command=self._cycle_climb, relief="flat", padx=10)
        self.btn_climb.pack(side="right")

        btn_neutral = tk.Button(row3, text="RESET POSE (0)", font=("Helvetica", 9), bg=BTN_BG, fg=TEXT_MUTED,
                                command=self._reset_pose, relief="flat", padx=6)
        btn_neutral.pack(side="right", padx=6)

        # Row 4: Big E-Stop Button & Banner
        row4 = tk.Frame(self.root, bg=BG_COLOR)
        row4.pack(fill="x", padx=16, pady=6)

        self.btn_estop = tk.Button(row4, text="● EMERGENCY STOP (PRESS X / SPACE TO ENGAGE)",
                                   font=("Helvetica", 11, "bold"), bg="#1e3a2b", fg=ACCENT_GREEN,
                                   activebackground=ACCENT_RED, activeforeground="white",
                                   command=self._toggle_estop, relief="flat", pady=6)
        self.btn_estop.pack(fill="x")

        # Row 5: Global Key Listener Option & Keymap Hint
        row5 = tk.Frame(self.root, bg=BG_COLOR)
        row5.pack(fill="x", padx=16, pady=(4, 8))

        if PYNPUT_AVAILABLE:
            chk = tk.Checkbutton(row5, text="Global Background Key Capture (Works when focusing terminal/MuJoCo)",
                                 variable=self.global_capture_enabled, font=("Helvetica", 9),
                                 fg=TEXT_MAIN, bg=BG_COLOR, selectcolor=CARD_BG, activebackground=BG_COLOR,
                                 activeforeground=TEXT_MAIN)
            chk.pack(anchor="w")

        hint_l1 = "Drive: WASD / Arrows (Multi-key hold) | Strafe: Z/C | Spin: Q/E | Stop: Release all"
        hint_l2 = "Height: U/J (80~220mm) | Tilt: I/K, O/L | Posture: 7/8/9 | Curb: [ / ] | E-Stop: X"
        tk.Label(row5, text=hint_l1, font=("Courier", 8), fg=TEXT_MUTED, bg=BG_COLOR).pack(anchor="w", pady=(4, 0))
        tk.Label(row5, text=hint_l2, font=("Courier", 8), fg=TEXT_MUTED, bg=BG_COLOR).pack(anchor="w")

    # ------------------------------------------------------------------
    # User Interactions & Mode Toggles
    # ------------------------------------------------------------------
    def _set_posture(self, mode):
        self.posture_mode = mode
        self.btn_posture_out.configure(bg=BTN_ACTIVE if mode == 0 else BTN_BG,
                                      fg="white" if mode == 0 else TEXT_MUTED)
        self.btn_posture_in.configure(bg=BTN_ACTIVE if mode == 1 else BTN_BG,
                                     fg="white" if mode == 1 else TEXT_MUTED)
        self.btn_posture_spin.configure(bg=BTN_ACTIVE if mode == 2 else BTN_BG,
                                       fg="white" if mode == 2 else TEXT_MUTED)

    def _curb_down(self):
        self.curb_height = max(0.04, round(self.curb_height - 0.005, 3))
        self.lbl_curb.configure(text=f"Curb Target [ / ]:  {self.curb_height*1000:.0f} mm")

    def _curb_up(self):
        self.curb_height = min(0.22, round(self.curb_height + 0.005, 3))
        self.lbl_curb.configure(text=f"Curb Target [ / ]:  {self.curb_height*1000:.0f} mm")

    def _cycle_climb(self):
        # Toggle between 1 (ADAPTIVE ENABLED) and 0 (DISABLED)
        self.climb_mode = 0 if self.climb_mode == 1 else 1
        text = "CLIMB: ENABLED (Adaptive \\)" if self.climb_mode == 1 else "CLIMB: DISABLED (\\)"
        self.btn_climb.configure(text=text,
                                 bg=BTN_ACTIVE if self.climb_mode == 1 else BTN_BG)

    def _reset_pose(self):
        self.height = 0.205
        self.roll = 0.0
        self.pitch = 0.0
        self._update_attitude_labels()

    def _toggle_estop(self):
        self.e_stop = not self.e_stop
        if self.e_stop:
            self.btn_estop.configure(text="■ EMERGENCY STOP ENGAGED — CLICK / PRESS X TO RESUME",
                                     bg=ACCENT_RED, fg="white")
        else:
            self.btn_estop.configure(text="● EMERGENCY STOP (PRESS X / SPACE TO ENGAGE)",
                                     bg="#1e3a2b", fg=ACCENT_GREEN)

    def _update_attitude_labels(self):
        self.lbl_height.configure(text=f"Height (U/J):  {self.height*1000:.1f} mm")
        r_deg = math.degrees(self.roll)
        p_deg = math.degrees(self.pitch)
        self.lbl_roll.configure(text=f"Roll  (I/K):  {self.roll:+.2f} rad ({r_deg:+.1f}°)")
        self.lbl_pitch.configure(text=f"Pitch (O/L):  {self.pitch:+.2f} rad ({p_deg:+.1f}°)")

    # ------------------------------------------------------------------
    # Keyboard Event Handlers (Tkinter Focus)
    # ------------------------------------------------------------------
    def _on_key_press(self, event):
        k = event.keysym.lower()
        if k in ('w', 's', 'a', 'd', 'q', 'e', 'z', 'c', 'up', 'down', 'left', 'right'):
            self.active_keys.add(k)
        else:
            self._handle_discrete_key(k)

    def _on_key_release(self, event):
        k = event.keysym.lower()
        self.active_keys.discard(k)

    def _handle_discrete_key(self, k):
        if k == 'u':
            self.height = min(0.22, round(self.height + 0.005, 3))
            self._update_attitude_labels()
        elif k == 'j':
            self.height = max(0.08, round(self.height - 0.005, 3))
            self._update_attitude_labels()
        elif k == 'i':
            self.roll = min(0.30, round(self.roll + 0.02, 2))
            self._update_attitude_labels()
        elif k == 'k':
            self.roll = max(-0.30, round(self.roll - 0.02, 2))
            self._update_attitude_labels()
        elif k == 'o':
            self.pitch = min(0.38, round(self.pitch + 0.02, 2))
            self._update_attitude_labels()
        elif k == 'l':
            self.pitch = max(-0.38, round(self.pitch - 0.02, 2))
            self._update_attitude_labels()
        elif k == '0':
            self._reset_pose()
        elif k == '1':
            self.target_linear = max(0.1, round(self.target_linear - 0.1, 1))
            self.lbl_presets.configure(text=f"Preset: Lin {self.target_linear:.1f} m/s (1/2) | Ang {self.target_angular:.1f} r/s (3/4)")
        elif k == '2':
            self.target_linear = min(2.0, round(self.target_linear + 0.1, 1))
            self.lbl_presets.configure(text=f"Preset: Lin {self.target_linear:.1f} m/s (1/2) | Ang {self.target_angular:.1f} r/s (3/4)")
        elif k == '3':
            self.target_angular = max(0.2, round(self.target_angular - 0.2, 1))
            self.lbl_presets.configure(text=f"Preset: Lin {self.target_linear:.1f} m/s (1/2) | Ang {self.target_angular:.1f} r/s (3/4)")
        elif k == '4':
            self.target_angular = min(4.0, round(self.target_angular + 0.2, 1))
            self.lbl_presets.configure(text=f"Preset: Lin {self.target_linear:.1f} m/s (1/2) | Ang {self.target_angular:.1f} r/s (3/4)")
        elif k in ('bracketleft', '['):
            self._curb_down()
        elif k in ('bracketright', ']'):
            self._curb_up()
        elif k in ('backslash', '\\'):
            self._cycle_climb()
        elif k == '7':
            self._set_posture(0)
        elif k == '8':
            self._set_posture(1)
        elif k == '9':
            self._set_posture(2)
        elif k in ('x', 'space'):
            self._toggle_estop()

    # ------------------------------------------------------------------
    # Background Keyboard Listener (pynput)
    # ------------------------------------------------------------------
    def _start_pynput_listener(self):
        def on_pynput_press(key):
            if not self.global_capture_enabled.get():
                return
            try:
                char = key.char.lower()
                if char in ('w', 's', 'a', 'd', 'q', 'e', 'z', 'c'):
                    self.active_keys.add(char)
                else:
                    self.root.after(0, lambda: self._handle_discrete_key(char))
            except AttributeError:
                # Special keys
                name = key.name.lower()
                if name in ('up', 'down', 'left', 'right'):
                    self.active_keys.add(name)
                elif name == 'space':
                    self.root.after(0, lambda: self._handle_discrete_key('space'))

        def on_pynput_release(key):
            if not self.global_capture_enabled.get():
                return
            try:
                char = key.char.lower()
                self.active_keys.discard(char)
            except AttributeError:
                name = key.name.lower()
                self.active_keys.discard(name)

        self.pynput_listener = pynput_kbd.Listener(
            on_press=on_pynput_press,
            on_release=on_pynput_release
        )
        self.pynput_listener.daemon = True
        self.pynput_listener.start()

    # ------------------------------------------------------------------
    # Periodic Tick Loop (50 Hz / 20ms)
    # ------------------------------------------------------------------
    def _tick_loop(self):
        # 1. Update continuous velocity from active keys
        if self.e_stop:
            self.vx = 0.0
            self.vy = 0.0
            self.wz = 0.0
        else:
            fwd = ('w' in self.active_keys) or ('up' in self.active_keys)
            bwd = ('s' in self.active_keys) or ('down' in self.active_keys)
            if fwd and not bwd:
                target_vx = self.target_linear
            elif bwd and not fwd:
                target_vx = -self.target_linear
            else:
                target_vx = 0.0

            left = ('a' in self.active_keys) or ('left' in self.active_keys)
            right = ('d' in self.active_keys) or ('right' in self.active_keys)
            if left and not right:
                target_wz = self.target_angular
            elif right and not left:
                target_wz = -self.target_angular
            else:
                target_wz = 0.0

            spin_ccw = 'q' in self.active_keys
            spin_cw = 'e' in self.active_keys
            if spin_ccw and not spin_cw:
                target_vx = 0.0
                target_wz = self.target_angular
            elif spin_cw and not spin_ccw:
                target_vx = 0.0
                target_wz = -self.target_angular

            strafe_l = 'z' in self.active_keys
            strafe_r = 'c' in self.active_keys
            if strafe_l and not strafe_r:
                target_vy = self.target_linear
            elif strafe_r and not strafe_l:
                target_vy = -self.target_linear
            else:
                target_vy = 0.0

            self.vx = target_vx
            self.vy = target_vy
            self.wz = target_wz

        # 2. Update GUI labels
        vx_color = ACCENT_GREEN if abs(self.vx) > 0.01 else TEXT_MAIN
        wz_color = ACCENT_GREEN if abs(self.wz) > 0.01 else TEXT_MAIN
        vy_color = ACCENT_GREEN if abs(self.vy) > 0.01 else TEXT_MUTED

        self.lbl_vx.configure(text=f"Cmd Vx:  {self.vx:+.2f} m/s", fg=vx_color)
        self.lbl_wz.configure(text=f"Cmd Wz:  {self.wz:+.2f} rad/s", fg=wz_color)
        self.lbl_vy.configure(text=f"Cmd Vy:  {self.vy:+.2f} m/s", fg=vy_color)

        # 3. Poll Telemetry from mac_mujoco_sim
        self._poll_telemetry()

        # 4. Broadcast UDP Packet
        self._send_udp()

        # Reschedule next frame
        self.root.after(20, self._tick_loop)

    def _poll_telemetry(self):
        while True:
            try:
                data, _ = self.recv_sock.recvfrom(4096)
                parsed = json.loads(data.decode('utf-8'))
                if 'telemetry' in parsed:
                    t = parsed['telemetry']
                    self.sim_connected = True
                    self.last_telemetry_time = time.time()
                    self.actual_height = float(t.get('height', self.height))
                    self.actual_roll = float(t.get('roll', self.roll))
                    self.actual_pitch = float(t.get('pitch', self.pitch))
                    self.actual_vx = float(t.get('linear_vel', 0.0))
                    self.actual_wz = float(t.get('angular_vel', 0.0))

                    self.lbl_sim_act.configure(text=f"Sim Act: Vx={self.actual_vx:+.2f}, Wz={self.actual_wz:+.2f}")
                    act_r = math.degrees(self.actual_roll)
                    act_p = math.degrees(self.actual_pitch)
                    self.lbl_sim_att.configure(text=f"IMU Act: R={act_r:+.1f}°, P={act_p:+.1f}°, H={self.actual_height*1000:.0f}mm")
            except (BlockingIOError, OSError, json.JSONDecodeError):
                break

        if time.time() - self.last_telemetry_time < 1.5:
            self.lbl_status.configure(text="● SIM LINKED", fg=ACCENT_GREEN)
        else:
            self.lbl_status.configure(text="● STANDALONE READY", fg=ACCENT_AMBER)

    def _send_udp(self):
        packet_dict = {
            'teleop': {
                'vx': self.vx,
                'vy': self.vy,
                'wz': self.wz,
                'height': self.height,
                'roll': self.roll,
                'pitch': self.pitch,
                'posture_mode': self.posture_mode,
                'curb_height': self.curb_height,
                'climb_mode': self.climb_mode,
                'e_stop': self.e_stop,
            }
        }
        packet_bytes = json.dumps(packet_dict).encode('utf-8')

        try:
            self.send_sock.sendto(packet_bytes, UDP_SIM_TARGET)
        except OSError:
            pass

        try:
            self.send_sock.sendto(packet_bytes, UDP_DOCKER_TARGET)
        except OSError:
            pass


def main():
    root = tk.Tk()
    app = TeleopDashboardApp(root)
    root.mainloop()


if __name__ == '__main__':
    main()
