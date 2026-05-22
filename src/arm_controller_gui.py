#!/usr/bin/env python3
"""
arm_controller_gui.py — RadarGuard Arm Controller GUI
Runs on Jetson desktop. Publishes commands to /arm_cmd (std_msgs/String).
Requires arm_controller_node.py to be running first.

Usage:
    python3 arm_controller_gui.py

Dependencies:
    tkinter (stdlib), rclpy, std_msgs
    ROS 2 Humble sourced in environment

Future:
    - DNTD Dynamics branded launcher integration
    - Foxglove live point cloud panel embed
    - Per-joint encoder feedback display (Phase 9+)
"""

import tkinter as tk
from tkinter import ttk, scrolledtext
import threading
import time
import sys

# ROS 2 import — graceful fallback for offline testing
try:
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import String
    from sensor_msgs.msg import JointState
    ROS_AVAILABLE = True
except ImportError:
    ROS_AVAILABLE = False

# ---------------------------------------------------------------------------
# Config
# ---------------------------------------------------------------------------
JOINT_NAMES   = ["Base", "Shoulder", "Elbow", "Forearm", "Wrist 1", "Wrist 2"]
NUM_JOINTS    = 6
DEFAULT_SPEED = 800    # microseconds
MIN_SPEED_US  = 200
MAX_SPEED_US  = 2000
DEFAULT_AMP   = 800    # steps
DEFAULT_COUNT = 5

# DNTD color palette — placeholder until full brand kit applied
C_BG          = "#0f1117"
C_PANEL       = "#1a1d27"
C_BORDER      = "#2a2d3a"
C_ACCENT      = "#00c8ff"
C_ACCENT2     = "#0077aa"
C_STOP        = "#e63946"
C_STOP_HOVER  = "#ff6b6b"
C_HOME        = "#2ec4b6"
C_TEXT        = "#e8eaf0"
C_TEXT_DIM    = "#6b7280"
C_SUCCESS     = "#4ade80"
C_WARNING     = "#facc15"
C_JOINT_SEL   = "#1e3a5f"

FONT_MAIN   = ("Courier New", 10)
FONT_LABEL  = ("Courier New", 9)
FONT_TITLE  = ("Courier New", 13, "bold")
FONT_SMALL  = ("Courier New", 8)
FONT_MONO   = ("Courier New", 9)


# ---------------------------------------------------------------------------
# ROS 2 node (runs in background thread)
# ---------------------------------------------------------------------------

class ArmGUINode:
    """Thin ROS 2 wrapper — publishes /arm_cmd, subscribes /joint_states."""

    def __init__(self, log_callback, joint_callback):
        self._log      = log_callback
        self._jcb      = joint_callback
        self._node     = None
        self._pub      = None
        self._running  = False
        self._thread   = None
        self.connected = False

    def start(self):
        if not ROS_AVAILABLE:
            self._log("WARN: rclpy not found — running in offline mode", "warn")
            return
        self._running = True
        self._thread  = threading.Thread(target=self._spin, daemon=True)
        self._thread.start()

    def _spin(self):
        try:
            rclpy.init()
            self._node = rclpy.create_node("arm_controller_gui")
            self._pub  = self._node.create_publisher(String, "/arm_cmd", 10)
            self._node.create_subscription(
                JointState, "/joint_states", self._js_callback, 10
            )
            self.connected = True
            self._log("ROS 2 connected — /arm_cmd publisher ready", "ok")
            while self._running:
                rclpy.spin_once(self._node, timeout_sec=0.05)
        except Exception as e:
            self._log(f"ROS 2 error: {e}", "err")
            self.connected = False

    def _js_callback(self, msg):
        self._jcb(list(msg.position))

    def send(self, cmd: str):
        cmd = cmd.strip()
        if not cmd:
            return
        if self._pub and self.connected:
            msg      = String()
            msg.data = cmd
            self._pub.publish(msg)
            self._log(f"→ {cmd}", "cmd")
        else:
            self._log(f"[OFFLINE] {cmd}", "dim")

    def stop(self):
        self._running = False
        if self._node:
            self._node.destroy_node()
        if ROS_AVAILABLE:
            try:
                rclpy.shutdown()
            except Exception:
                pass


# ---------------------------------------------------------------------------
# Main GUI
# ---------------------------------------------------------------------------

class ArmControllerGUI:

    def __init__(self, root: tk.Tk):
        self.root        = root
        self.root.title("RadarGuard — Arm Controller")
        self.root.configure(bg=C_BG)
        self.root.resizable(True, True)
        self.root.minsize(700, 640)

        # State
        self.selected_joint  = tk.IntVar(value=0)
        self.speed_var        = tk.IntVar(value=DEFAULT_SPEED)
        self.amp_var          = tk.StringVar(value=str(DEFAULT_AMP))
        self.count_var        = tk.StringVar(value=str(DEFAULT_COUNT))
        self.jog_steps_var    = tk.StringVar(value="400")
        self.joint_angles     = [0.0] * NUM_JOINTS
        self.angle_vars       = [tk.StringVar(value="0.0000") for _ in range(NUM_JOINTS)]
        self.ros_status_var   = tk.StringVar(value="● Connecting...")

        # ROS 2 node
        self.ros = ArmGUINode(
            log_callback=self._log,
            joint_callback=self._update_angles
        )

        self._build_ui()
        self.ros.start()

        # Poll ROS status indicator
        self._poll_ros_status()

        self.root.protocol("WM_DELETE_WINDOW", self._on_close)

    # -----------------------------------------------------------------------
    # UI Construction
    # -----------------------------------------------------------------------

    def _build_ui(self):
        root = self.root

        # ── Title bar ──────────────────────────────────────────────────────
        title_frame = tk.Frame(root, bg=C_BG)
        title_frame.pack(fill="x", padx=12, pady=(10, 4))

        tk.Label(
            title_frame, text="RADARGUARD", font=("Courier New", 16, "bold"),
            fg=C_ACCENT, bg=C_BG
        ).pack(side="left")
        tk.Label(
            title_frame, text="  ARM CONTROLLER", font=("Courier New", 16),
            fg=C_TEXT, bg=C_BG
        ).pack(side="left")

        # ROS status right-aligned
        tk.Label(
            title_frame, textvariable=self.ros_status_var,
            font=FONT_SMALL, fg=C_TEXT_DIM, bg=C_BG
        ).pack(side="right", padx=4)

        tk.Label(
            title_frame, text="DNTD Dynamics",
            font=FONT_SMALL, fg=C_TEXT_DIM, bg=C_BG
        ).pack(side="right", padx=8)

        # Thin accent line under title
        tk.Frame(root, bg=C_ACCENT, height=1).pack(fill="x", padx=12)

        # ── Main layout: left column + right console ───────────────────────
        main = tk.Frame(root, bg=C_BG)
        main.pack(fill="both", expand=True, padx=12, pady=8)

        left  = tk.Frame(main, bg=C_BG)
        left.pack(side="left", fill="both", expand=True)

        right = tk.Frame(main, bg=C_BG, width=230)
        right.pack(side="right", fill="both", padx=(8, 0))
        right.pack_propagate(False)

        # ── Joint selector ─────────────────────────────────────────────────
        self._build_joint_selector(left)

        # ── Controls ───────────────────────────────────────────────────────
        self._build_jog_panel(left)
        self._build_sweep_panel(left)
        self._build_speed_panel(left)
        self._build_home_panel(left)

        # ── STOP button ────────────────────────────────────────────────────
        self._build_stop_button(left)

        # ── Right: joint angle readout + console ───────────────────────────
        self._build_angle_readout(right)
        self._build_console(right)

    def _panel(self, parent, title):
        """Styled labeled panel frame."""
        outer = tk.Frame(parent, bg=C_BORDER, bd=0)
        outer.pack(fill="x", pady=4)
        inner = tk.Frame(outer, bg=C_PANEL, bd=0)
        inner.pack(fill="x", padx=1, pady=1)
        tk.Label(
            inner, text=f"  {title}",
            font=("Courier New", 9, "bold"), fg=C_ACCENT, bg=C_PANEL,
            anchor="w"
        ).pack(fill="x", pady=(6, 2))
        tk.Frame(inner, bg=C_BORDER, height=1).pack(fill="x", padx=8)
        body = tk.Frame(inner, bg=C_PANEL)
        body.pack(fill="x", padx=10, pady=8)
        return body

    def _btn(self, parent, text, cmd, color=C_ACCENT2, width=10, font=FONT_MAIN):
        b = tk.Button(
            parent, text=text, command=cmd,
            bg=color, fg=C_TEXT, font=font,
            relief="flat", bd=0, padx=8, pady=4,
            activebackground=C_ACCENT, activeforeground=C_BG,
            cursor="hand2", width=width
        )
        return b

    def _entry(self, parent, textvariable, width=7):
        return tk.Entry(
            parent, textvariable=textvariable, width=width,
            bg=C_BG, fg=C_TEXT, insertbackground=C_ACCENT,
            relief="flat", bd=1, highlightbackground=C_BORDER,
            highlightthickness=1, font=FONT_MONO
        )

    # ── Joint selector ──────────────────────────────────────────────────────

    def _build_joint_selector(self, parent):
        body = self._panel(parent, "JOINT SELECT")
        row  = tk.Frame(body, bg=C_PANEL)
        row.pack(fill="x")

        self._joint_btns = []
        for j, name in enumerate(JOINT_NAMES):
            b = tk.Button(
                row, text=f"{j}\n{name.upper()}",
                font=("Courier New", 8, "bold"),
                bg=C_JOINT_SEL if j == 0 else C_BG,
                fg=C_ACCENT    if j == 0 else C_TEXT_DIM,
                relief="flat", bd=0, padx=6, pady=6,
                cursor="hand2", width=8,
                command=lambda jj=j: self._select_joint(jj)
            )
            b.pack(side="left", padx=2)
            self._joint_btns.append(b)

    def _select_joint(self, j):
        self.selected_joint.set(j)
        for idx, b in enumerate(self._joint_btns):
            if idx == j:
                b.configure(bg=C_JOINT_SEL, fg=C_ACCENT)
            else:
                b.configure(bg=C_BG, fg=C_TEXT_DIM)

    # ── Jog panel ───────────────────────────────────────────────────────────

    def _build_jog_panel(self, parent):
        body = self._panel(parent, "JOG")
        row  = tk.Frame(body, bg=C_PANEL)
        row.pack(fill="x")

        self._btn(row, "◀  JOG −", self._jog_neg, width=10).pack(side="left", padx=(0, 6))

        tk.Label(row, text="steps:", font=FONT_LABEL,
                 fg=C_TEXT_DIM, bg=C_PANEL).pack(side="left")
        self._entry(row, self.jog_steps_var, width=7).pack(side="left", padx=4)

        self._btn(row, "JOG +  ▶", self._jog_pos, width=10).pack(side="left", padx=(6, 0))

    def _jog_neg(self):
        steps = self._get_int(self.jog_steps_var, "jog steps")
        if steps is None: return
        self.ros.send(f"JOG {self.selected_joint.get()} -{steps}")

    def _jog_pos(self):
        steps = self._get_int(self.jog_steps_var, "jog steps")
        if steps is None: return
        self.ros.send(f"JOG {self.selected_joint.get()} {steps}")

    # ── Sweep panel ─────────────────────────────────────────────────────────

    def _build_sweep_panel(self, parent):
        body = self._panel(parent, "SWEEP")
        row  = tk.Frame(body, bg=C_PANEL)
        row.pack(fill="x")

        tk.Label(row, text="amplitude (steps):", font=FONT_LABEL,
                 fg=C_TEXT_DIM, bg=C_PANEL).pack(side="left")
        self._entry(row, self.amp_var, width=7).pack(side="left", padx=(4, 12))

        tk.Label(row, text="count:", font=FONT_LABEL,
                 fg=C_TEXT_DIM, bg=C_PANEL).pack(side="left")
        self._entry(row, self.count_var, width=5).pack(side="left", padx=4)

        self._btn(row, "▶ SWEEP", self._do_sweep, width=10).pack(side="left", padx=(12, 0))

    def _do_sweep(self):
        amp   = self._get_int(self.amp_var,   "amplitude")
        count = self._get_int(self.count_var, "count")
        if amp is None or count is None: return
        self.ros.send(f"SWEEP {self.selected_joint.get()} {amp} {count}")

    # ── Speed panel ─────────────────────────────────────────────────────────

    def _build_speed_panel(self, parent):
        body = self._panel(parent, "SPEED  (µs per step — lower = faster)")
        row  = tk.Frame(body, bg=C_PANEL)
        row.pack(fill="x")

        self._speed_label = tk.Label(
            row, text=f"{DEFAULT_SPEED} µs", width=8,
            font=FONT_MONO, fg=C_ACCENT, bg=C_PANEL, anchor="w"
        )
        self._speed_label.pack(side="left", padx=(0, 8))

        scale = tk.Scale(
            row, from_=MIN_SPEED_US, to=MAX_SPEED_US,
            orient="horizontal", variable=self.speed_var,
            bg=C_PANEL, fg=C_TEXT, troughcolor=C_BG,
            highlightthickness=0, sliderrelief="flat",
            activebackground=C_ACCENT, length=200,
            command=self._speed_changed, showvalue=False
        )
        scale.pack(side="left", padx=(0, 8))

        row2 = tk.Frame(body, bg=C_PANEL)
        row2.pack(fill="x", pady=(4, 0))

        self._btn(row2, "SET joint", self._speed_set_joint, width=10).pack(side="left", padx=(0, 6))
        self._btn(row2, "SET all",   self._speed_set_all,   width=10).pack(side="left")

    def _speed_changed(self, val):
        self._speed_label.configure(text=f"{val} µs")

    def _speed_set_joint(self):
        self.ros.send(f"SPEED {self.selected_joint.get()} {self.speed_var.get()}")

    def _speed_set_all(self):
        self.ros.send(f"SPEED ALL {self.speed_var.get()}")

    # ── Home panel ──────────────────────────────────────────────────────────

    def _build_home_panel(self, parent):
        body = self._panel(parent, "HOMING")
        row  = tk.Frame(body, bg=C_PANEL)
        row.pack(fill="x")

        self._btn(
            row, "HOME selected", self._home_joint,
            color=C_HOME, width=14
        ).pack(side="left", padx=(0, 8))

        self._btn(
            row, "HOME all joints", self._home_all,
            color=C_HOME, width=14
        ).pack(side="left", padx=(0, 8))

        self._btn(
            row, "STATUS", self._status,
            color=C_ACCENT2, width=8
        ).pack(side="left")

    def _home_joint(self):
        self.ros.send(f"HOME {self.selected_joint.get()}")

    def _home_all(self):
        self.ros.send("HOME")

    def _status(self):
        self.ros.send("STATUS")

    # ── Stop button ─────────────────────────────────────────────────────────

    def _build_stop_button(self, parent):
        frame = tk.Frame(parent, bg=C_BG)
        frame.pack(fill="x", pady=8)

        stop_btn = tk.Button(
            frame, text="⬛  STOP",
            command=self._stop,
            bg=C_STOP, fg="white",
            font=("Courier New", 13, "bold"),
            relief="flat", bd=0, pady=10,
            activebackground=C_STOP_HOVER, activeforeground="white",
            cursor="hand2"
        )
        stop_btn.pack(fill="x")

    def _stop(self):
        self.ros.send("STOP")

    # ── Angle readout ───────────────────────────────────────────────────────

    def _build_angle_readout(self, parent):
        tk.Label(
            parent, text="JOINT ANGLES (rad)",
            font=("Courier New", 9, "bold"), fg=C_ACCENT, bg=C_BG, anchor="w"
        ).pack(fill="x", pady=(0, 2))
        tk.Frame(parent, bg=C_BORDER, height=1).pack(fill="x")

        grid = tk.Frame(parent, bg=C_PANEL)
        grid.pack(fill="x", pady=4)

        for j in range(NUM_JOINTS):
            tk.Label(
                grid, text=f"J{j} {JOINT_NAMES[j][:8]:<8}",
                font=FONT_SMALL, fg=C_TEXT_DIM, bg=C_PANEL, anchor="w", width=12
            ).grid(row=j, column=0, sticky="w", padx=(6, 4), pady=1)

            tk.Label(
                grid, textvariable=self.angle_vars[j],
                font=FONT_MONO, fg=C_TEXT, bg=C_PANEL, anchor="e", width=9
            ).grid(row=j, column=1, sticky="e", padx=(0, 6), pady=1)

    # ── Console ─────────────────────────────────────────────────────────────

    def _build_console(self, parent):
        tk.Label(
            parent, text="CONSOLE",
            font=("Courier New", 9, "bold"), fg=C_ACCENT, bg=C_BG, anchor="w"
        ).pack(fill="x", pady=(10, 2))
        tk.Frame(parent, bg=C_BORDER, height=1).pack(fill="x")

        self._console = scrolledtext.ScrolledText(
            parent, width=26, height=18,
            bg=C_BG, fg=C_TEXT, font=FONT_MONO,
            insertbackground=C_ACCENT, relief="flat",
            bd=0, wrap="word", state="disabled"
        )
        self._console.pack(fill="both", expand=True, pady=4)

        self._console.tag_configure("cmd",  foreground=C_ACCENT)
        self._console.tag_configure("ok",   foreground=C_SUCCESS)
        self._console.tag_configure("warn", foreground=C_WARNING)
        self._console.tag_configure("err",  foreground=C_STOP)
        self._console.tag_configure("dim",  foreground=C_TEXT_DIM)

        # Clear button
        self._btn(parent, "clear console", self._clear_console, width=14).pack()

    def _clear_console(self):
        self._console.configure(state="normal")
        self._console.delete("1.0", "end")
        self._console.configure(state="disabled")

    # -----------------------------------------------------------------------
    # Callbacks / helpers
    # -----------------------------------------------------------------------

    def _log(self, msg: str, tag: str = ""):
        def _write():
            self._console.configure(state="normal")
            ts = time.strftime("%H:%M:%S")
            self._console.insert("end", f"[{ts}] {msg}\n", tag or "")
            self._console.see("end")
            self._console.configure(state="disabled")
        self.root.after(0, _write)

    def _update_angles(self, angles):
        def _write():
            for j, a in enumerate(angles[:NUM_JOINTS]):
                self.angle_vars[j].set(f"{a:+.4f}")
        self.root.after(0, _write)

    def _poll_ros_status(self):
        if self.ros.connected:
            self.ros_status_var.set("● ROS 2 connected")
        elif not ROS_AVAILABLE:
            self.ros_status_var.set("● offline mode")
        else:
            self.ros_status_var.set("● connecting...")
        self.root.after(1000, self._poll_ros_status)

    def _get_int(self, var: tk.StringVar, name: str):
        try:
            val = int(var.get())
            if val <= 0:
                raise ValueError
            return val
        except ValueError:
            self._log(f"ERR: invalid {name} — must be a positive integer", "err")
            return None

    def _on_close(self):
        self.ros.stop()
        self.root.destroy()


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def main():
    root = tk.Tk()
    app  = ArmControllerGUI(root)
    root.mainloop()


if __name__ == "__main__":
    main()
