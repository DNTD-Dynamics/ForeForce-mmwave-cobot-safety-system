"""
arm_config_gui.py
DNTD Dynamics — RadarGuard Arm Configuration Tool

Lets you enter your arm's physical measurements and writes the correct
joint_geometry, joint_names, zone distances, and sensor mount directly
into configs/dntd_mmwave_config.yaml — surgically, preserving every
comment and setting you haven't touched.

What it configures:
  - Number of joints, names, link lengths, axis directions
  - Sensor mount: which joint the sensor is on, XYZ offset
  - Zone distances: stop and caution radii
  - Writes only the changed keys — all other YAML is untouched

Usage:
  python3 src/arm_config_gui.py
  python3 src/arm_config_gui.py --config /path/to/dntd_mmwave_config.yaml

Requires: tkinter (stdlib)
  Ubuntu: sudo apt install python3-tk
"""

import argparse
import math
import os
import re
import sys
import tkinter as tk
from tkinter import ttk, messagebox, filedialog

# ---------------------------------------------------------------------------
# Config path — relative to this file's location
# ---------------------------------------------------------------------------
_HERE = os.path.dirname(os.path.abspath(__file__))
DEFAULT_CONFIG = os.path.join(_HERE, "..", "configs", "dntd_mmwave_config.yaml")


# ---------------------------------------------------------------------------
# Axis presets
# ---------------------------------------------------------------------------
AXIS_PRESETS = {
    "Rotates left/right  (vertical — base turntable)":    [0.0, 0.0, 1.0],
    "Rotates up/down     (horizontal — shoulder/elbow)":  [0.0, 1.0, 0.0],
    "Rolls forward/back  (roll — wrist)":                 [1.0, 0.0, 0.0],
    "Custom  (enter X Y Z below)":                        None,
}


# ---------------------------------------------------------------------------
# Surgical YAML writer — preserves ALL comments
# ---------------------------------------------------------------------------

def _read(path):
    if os.path.exists(path):
        with open(path) as f:
            return f.read()
    return ""

def _write(path, content):
    os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
    with open(path, "w") as f:
        f.write(content)

def _replace_scalar(text, key, value):
    """Replace a scalar value, preserving any inline comment after it."""
    def sub(m):
        return f"{m.group(1)}{value}{m.group(3) or ''}"
    return re.sub(
        rf"(    {key}:\s*)([^\n#]+)(#[^\n]*)?",
        sub, text, count=1
    )

def _replace_list(text, key, items):
    """Replace a YAML list block under a key."""
    new = f"    {key}:\n" + "".join(f'      - "{i}"\n' for i in items)
    return re.sub(
        rf"    {key}:\n(?:      - [^\n]*\n)+",
        new, text, count=1
    )

def _replace_joint_geometry(text, geom):
    """Replace the joint_geometry block only, preserving surrounding content."""
    lines = ["    joint_geometry:\n"]
    for name, d in geom.items():
        x, y, z   = [f"{v}" for v in d["origin_xyz"]]
        rx, ry, rz = [f"{v}" for v in d["origin_rpy"]]
        ax, ay, az = [f"{v}" for v in d["axis"]]
        lines += [
            f"      {name}:\n",
            f"        type: revolute\n",
            f"        origin_xyz: [{x}, {y}, {z}]\n",
            f"        origin_rpy: [{rx}, {ry}, {rz}]\n",
            f"        axis: [{ax}, {ay}, {az}]\n",
        ]
    new = "".join(lines)
    return re.sub(
        r"    joint_geometry:\n(?:      \S[^\n]*\n(?:        [^\n]*\n)*)*",
        new, text, count=1
    )

def _replace_xyz_list(text, key, xyz):
    """Replace a [x, y, z] inline list value."""
    new_val = f"[{xyz[0]}, {xyz[1]}, {xyz[2]}]"
    return re.sub(
        rf"(    {key}:\s*)\[[^\]]*\]",
        rf"\g<1>{new_val}", text, count=1
    )

def patch_config(path, data):
    """
    Surgically patch only the keys the GUI controls.
    Every comment, blank line, and untouched setting is preserved exactly.

    data keys:
      joint_names        list[str]
      joint_geometry     dict  {name: {origin_xyz, origin_rpy, axis}}
      sensor_mount_link  str
      sensor_mount_xyz   [x, y, z]  (meters)
      stop_range_m       float
      caution_range_m    float
      swept_volume_mount_joint_idx  int
      swept_volume_max_reach_m      float
    """
    content = _read(path)
    if not content:
        raise FileNotFoundError(
            f"Config file not found: {path}\n"
            "Run from the repo root or pass --config with the correct path."
        )

    content = _replace_list(content, "joint_names", data["joint_names"])
    content = _replace_joint_geometry(content, data["joint_geometry"])
    content = _replace_scalar(content, "sensor_mount_link",
                              f'"{data["sensor_mount_link"]}"')
    content = _replace_xyz_list(content, "sensor_mount_xyz",
                                data["sensor_mount_xyz"])
    content = _replace_scalar(content, "stop_range_m",
                              f'{data["stop_range_m"]:.3f}')
    content = _replace_scalar(content, "caution_range_m",
                              f'{data["caution_range_m"]:.3f}')
    content = _replace_scalar(content, "swept_volume_mount_joint_idx",
                              str(data["swept_volume_mount_joint_idx"]))
    content = _replace_scalar(content, "swept_volume_max_reach_m",
                              f'{data["swept_volume_max_reach_m"]:.3f}')

    _write(path, content)


# ---------------------------------------------------------------------------
# Geometry helpers
# ---------------------------------------------------------------------------

def build_joint_geometry(joint_rows):
    geom = {}
    for i, row in enumerate(joint_rows):
        link_mm = float(row.get("link_mm") or 0)
        z_mm    = float(row.get("z_mm") or 0)
        axis    = row.get("axis") or [0.0, 0.0, 1.0]

        origin_xyz = (
            [0.0, 0.0, round(z_mm / 1000, 4)]
            if i == 0 else
            [round(link_mm / 1000, 4), 0.0, 0.0]
        )

        if axis == [0.0, 0.0, 1.0]:
            origin_rpy = [0.0, 0.0, 0.0]
        elif axis == [0.0, 1.0, 0.0]:
            origin_rpy = [1.5708, 0.0, 0.0]
        elif axis == [1.0, 0.0, 0.0]:
            origin_rpy = [0.0, -1.5708, 0.0]
        else:
            origin_rpy = [0.0, 0.0, 0.0]

        geom[row["name"]] = {
            "origin_xyz": origin_xyz,
            "origin_rpy": origin_rpy,
            "axis":       [float(v) for v in axis],
        }
    return geom

def estimate_reach(joint_rows):
    return sum(float(r.get("link_mm") or 0) for r in joint_rows) / 1000.0


# ---------------------------------------------------------------------------
# GUI
# ---------------------------------------------------------------------------

class ArmConfigGUI:
    # ── Colours ──────────────────────────────────────────────────────────
    BG      = "#0f1117"
    PANEL   = "#1a1d27"
    ACCENT  = "#00c8ff"
    SUCCESS = "#00e676"
    TEXT    = "#e8eaf0"
    DIM     = "#7b8096"
    INPUT   = "#252838"
    BORDER  = "#2a2d3e"
    WARN    = "#ffab40"

    FONT_HEAD  = ("Arial", 13, "bold")
    FONT_BODY  = ("Arial", 10)
    FONT_SMALL = ("Arial",  9)
    FONT_MONO  = ("Courier", 10)

    def __init__(self, root, config_path):
        self.root        = root
        self.config_path = config_path
        self._joint_rows = []   # list of dicts of tk.Vars per joint

        root.title("RadarGuard — Arm Configuration")
        root.configure(bg=self.BG)
        root.minsize(700, 560)
        root.geometry("820x740")

        self._apply_style()
        self._build(root)

    # ── ttk style ────────────────────────────────────────────────────────
    def _apply_style(self):
        s = ttk.Style()
        s.theme_use("clam")
        for name, opts in [
            ("TFrame",       {"background": self.BG}),
            ("TLabel",       {"background": self.BG,    "foreground": self.TEXT, "font": self.FONT_BODY}),
            ("Dim.TLabel",   {"background": self.PANEL, "foreground": self.DIM,  "font": self.FONT_SMALL}),
            ("Head.TLabel",  {"background": self.BG,    "foreground": self.ACCENT,"font": self.FONT_HEAD}),
            ("TSeparator",   {"background": self.BORDER}),
        ]:
            s.configure(name, **opts)

        s.configure("TButton",
                    background=self.ACCENT, foreground=self.BG,
                    font=("Arial", 10, "bold"), relief="flat", padding=(14,7))
        s.map("TButton", background=[("active", "#0099cc")])

        s.configure("Ghost.TButton",
                    background=self.PANEL, foreground=self.DIM,
                    font=self.FONT_SMALL, relief="flat", padding=(10,5))
        s.map("Ghost.TButton", background=[("active", self.BORDER)])

        s.configure("TCombobox",
                    fieldbackground=self.INPUT, background=self.INPUT,
                    foreground=self.TEXT, arrowcolor=self.ACCENT)
        s.map("TCombobox", fieldbackground=[("readonly", self.INPUT)])

    # ── Layout ───────────────────────────────────────────────────────────
    def _build(self, root):
        # Header
        hdr = tk.Frame(root, bg=self.BG)
        hdr.pack(fill="x", padx=24, pady=(18, 0))
        tk.Label(hdr, text="RadarGuard", bg=self.BG, fg=self.ACCENT,
                 font=self.FONT_HEAD).pack(anchor="w")
        tk.Label(hdr, text="Arm Configuration  ·  writes joint geometry to dntd_mmwave_config.yaml",
                 bg=self.BG, fg=self.DIM, font=self.FONT_SMALL).pack(anchor="w", pady=(2,0))
        ttk.Separator(root, orient="horizontal").pack(fill="x", padx=24, pady=10)

        # Scrollable body
        outer = tk.Frame(root, bg=self.BG)
        outer.pack(fill="both", expand=True, padx=24)

        canvas = tk.Canvas(outer, bg=self.BG, highlightthickness=0)
        sb     = ttk.Scrollbar(outer, orient="vertical", command=canvas.yview)
        canvas.configure(yscrollcommand=sb.set)
        sb.pack(side="right", fill="y")
        canvas.pack(side="left", fill="both", expand=True)

        self._body = tk.Frame(canvas, bg=self.BG)
        win_id     = canvas.create_window((0,0), window=self._body, anchor="nw")

        self._body.bind("<Configure>",
            lambda e: canvas.configure(scrollregion=canvas.bbox("all")))
        canvas.bind("<Configure>",
            lambda e: canvas.itemconfig(win_id, width=e.width))
        for seq in ("<MouseWheel>", "<Button-4>", "<Button-5>"):
            canvas.bind_all(seq, lambda e, c=canvas: c.yview_scroll(
                -1 if e.num != 5 else 1, "units"))

        self._fill_body(self._body)

        # Footer
        ttk.Separator(root, orient="horizontal").pack(fill="x", padx=24)
        foot = tk.Frame(root, bg=self.BG)
        foot.pack(fill="x", padx=24, pady=10)

        self._status = tk.StringVar(value="")
        self._status_lbl = tk.Label(foot, textvariable=self._status,
                                    bg=self.BG, fg=self.SUCCESS,
                                    font=self.FONT_SMALL, anchor="w")
        self._status_lbl.pack(side="left", fill="x", expand=True)

        ttk.Button(foot, text="Preview changes",
                   command=self._preview, style="Ghost.TButton").pack(side="right", padx=(0,8))
        ttk.Button(foot, text="Save to config",
                   command=self._save).pack(side="right")

    def _fill_body(self, parent):
        # ── 1. Arm basics ─────────────────────────────────────────────
        self._section(parent, "1  Arm basics")

        row = tk.Frame(parent, bg=self.BG)
        row.pack(fill="x", pady=(0,10))

        tk.Label(row, text="Number of joints", bg=self.BG,
                 fg=self.TEXT, font=self.FONT_BODY).grid(row=0, column=0, sticky="w", padx=(0,10))
        self._n_joints = tk.IntVar(value=6)
        tk.Spinbox(row, from_=1, to=12, width=4, textvariable=self._n_joints,
                   bg=self.INPUT, fg=self.TEXT, buttonbackground=self.PANEL,
                   highlightthickness=0, relief="flat", font=self.FONT_MONO,
                   command=self._rebuild_joints).grid(row=0, column=1, sticky="w")

        tk.Label(row, text="Arm name", bg=self.BG,
                 fg=self.TEXT, font=self.FONT_BODY).grid(row=0, column=2, sticky="w", padx=(28,8))
        self._arm_name = tk.StringVar(value="My Robot Arm")
        self._entry(row, self._arm_name, 20).grid(row=0, column=3, sticky="w")

        # ── 2. Joint geometry ─────────────────────────────────────────
        self._section(parent,
            "2  Joint geometry   (measure between joint center points)")
        tk.Label(parent,
                 text="  Tip: link length = distance from this joint's rotation center "
                      "to the next joint's rotation center.",
                 bg=self.BG, fg=self.DIM, font=self.FONT_SMALL,
                 anchor="w", justify="left").pack(fill="x", pady=(0,6))

        self._joints_frame = tk.Frame(parent, bg=self.BG)
        self._joints_frame.pack(fill="x")
        self._rebuild_joints()

        # ── 3. Sensor mount ───────────────────────────────────────────
        self._section(parent, "3  Sensor mount")

        mnt = tk.Frame(parent, bg=self.BG)
        mnt.pack(fill="x", pady=(0,10))

        tk.Label(mnt, text="Sensor is mounted on joint #  (1 = base)",
                 bg=self.BG, fg=self.TEXT,
                 font=self.FONT_BODY).grid(row=0, column=0, sticky="w", padx=(0,8))
        self._mount_joint = tk.IntVar(value=3)
        tk.Spinbox(mnt, from_=1, to=12, width=4, textvariable=self._mount_joint,
                   bg=self.INPUT, fg=self.TEXT, buttonbackground=self.PANEL,
                   highlightthickness=0, relief="flat",
                   font=self.FONT_MONO).grid(row=0, column=1, sticky="w")

        tk.Label(mnt, text="Offset from mount point (mm)   X",
                 bg=self.BG, fg=self.TEXT,
                 font=self.FONT_BODY).grid(row=1, column=0, sticky="w", pady=(8,0))
        offs = tk.Frame(mnt, bg=self.BG)
        offs.grid(row=1, column=1, columnspan=3, sticky="w", pady=(8,0))
        for label, attr, default in [("X", "_mx", 0), ("Y", "_my", 0), ("Z", "_mz", 50)]:
            tk.Label(offs, text=label, bg=self.BG, fg=self.DIM,
                     font=self.FONT_SMALL).pack(side="left", padx=(0,3))
            v = tk.DoubleVar(value=default)
            setattr(self, attr, v)
            self._entry(offs, v, 6).pack(side="left", padx=(0,12))

        # ── 4. Zone distances ─────────────────────────────────────────
        self._section(parent, "4  Zone distances")
        tk.Label(parent,
                 text="  Stop range: arm halts immediately.  "
                      "Caution range: arm slows.  "
                      "Both measured from sensor face.",
                 bg=self.BG, fg=self.DIM, font=self.FONT_SMALL,
                 anchor="w").pack(fill="x", pady=(0,6))

        zrow = tk.Frame(parent, bg=self.BG)
        zrow.pack(fill="x", pady=(0,10))
        for label, attr, default in [
            ("Stop range", "_stop_mm", 500),
            ("Caution range", "_caution_mm", 1200),
        ]:
            tk.Label(zrow, text=label, bg=self.BG, fg=self.TEXT,
                     font=self.FONT_BODY).pack(side="left", padx=(0,5))
            v = tk.IntVar(value=default)
            setattr(self, attr, v)
            self._entry(zrow, v, 6).pack(side="left")
            tk.Label(zrow, text="mm", bg=self.BG, fg=self.DIM,
                     font=self.FONT_SMALL).pack(side="left", padx=(3,20))

        # ── 5. Config path ────────────────────────────────────────────
        self._section(parent, "5  Config file")
        frow = tk.Frame(parent, bg=self.BG)
        frow.pack(fill="x", pady=(0,16))
        self._cfg_var = tk.StringVar(value=self.config_path)
        self._entry(frow, self._cfg_var, 55).pack(side="left", padx=(0,8))
        ttk.Button(frow, text="Browse…", style="Ghost.TButton",
                   command=self._browse).pack(side="left")

    # ── Joint table ───────────────────────────────────────────────────────
    def _rebuild_joints(self, *_):
        for w in self._joints_frame.winfo_children():
            w.destroy()
        self._joint_rows = []

        n = self._n_joints.get()

        # Header
        hdr = tk.Frame(self._joints_frame, bg=self.BG)
        hdr.pack(fill="x", pady=(0,3))
        for col, (text, w) in enumerate([
            ("#", 3), ("Name", 12), ("Link length\n(mm)", 9),
            ("Z offset\n(mm — base only)", 9), ("Rotation axis", 40),
            ("Custom X Y Z", 16),
        ]):
            tk.Label(hdr, text=text, bg=self.BG, fg=self.DIM,
                     font=self.FONT_SMALL, width=w,
                     anchor="w", justify="left").grid(row=0, column=col, sticky="w", padx=(0,6))

        ttk.Separator(self._joints_frame, orient="horizontal").pack(fill="x", pady=(0,4))

        default_axes = [
            "Rotates left/right  (vertical — base turntable)",
            "Rotates up/down     (horizontal — shoulder/elbow)",
            "Rotates up/down     (horizontal — shoulder/elbow)",
            "Rotates left/right  (vertical — base turntable)",
            "Rotates up/down     (horizontal — shoulder/elbow)",
            "Rotates left/right  (vertical — base turntable)",
        ]
        default_links = [0, 127, 425, 392, 109, 93]
        default_z     = [127, 0, 0, 0, 0, 0]

        for i in range(n):
            row_data = self._joint_row(
                i, n,
                default_axes[i] if i < len(default_axes) else default_axes[-1],
                default_links[i] if i < len(default_links) else 100,
                default_z[i]    if i < len(default_z)     else 0,
            )
            self._joint_rows.append(row_data)

    def _joint_row(self, i, n, default_axis, default_link, default_z):
        f = tk.Frame(self._joints_frame, bg=self.BG)
        f.pack(fill="x", pady=2)

        # Index
        tk.Label(f, text=str(i+1), bg=self.BG, fg=self.DIM,
                 font=self.FONT_MONO, width=3).grid(row=0, column=0, sticky="w")

        # Name
        name_v = tk.StringVar(value=f"joint{i+1}")
        self._entry(f, name_v, 12).grid(row=0, column=1, sticky="w", padx=(0,6))

        # Link length
        link_v = tk.IntVar(value=default_link)
        le = self._entry(f, link_v, 7,
                         state="disabled" if i == 0 else "normal",
                         bg=self.BORDER if i == 0 else self.INPUT)
        le.grid(row=0, column=2, sticky="w", padx=(0,6))

        # Z offset
        z_v = tk.IntVar(value=default_z)
        ze = self._entry(f, z_v, 7,
                         state="normal" if i == 0 else "disabled",
                         bg=self.INPUT if i == 0 else self.BORDER)
        ze.grid(row=0, column=3, sticky="w", padx=(0,6))

        # Axis dropdown
        axis_keys = list(AXIS_PRESETS.keys())
        axis_v = tk.StringVar(value=default_axis)
        combo = ttk.Combobox(f, textvariable=axis_v, values=axis_keys,
                             width=42, state="readonly")
        combo.grid(row=0, column=4, sticky="w", padx=(0,6))

        # Custom XYZ (hidden unless Custom selected)
        cframe = tk.Frame(f, bg=self.BG)
        cframe.grid(row=0, column=5, sticky="w")
        cx = tk.DoubleVar(value=0.0)
        cy = tk.DoubleVar(value=0.0)
        cz = tk.DoubleVar(value=1.0)
        custom_entries = []
        for label, v in [("X", cx), ("Y", cy), ("Z", cz)]:
            tk.Label(cframe, text=label, bg=self.BG, fg=self.DIM,
                     font=self.FONT_SMALL).pack(side="left", padx=(0,2))
            e = self._entry(cframe, v, 4)
            e.pack(side="left", padx=(0,6))
            custom_entries.append(e)

        def _toggle(*_):
            is_custom = axis_v.get().startswith("Custom")
            st = "normal" if is_custom else "disabled"
            bg = self.INPUT if is_custom else self.BORDER
            for e in custom_entries:
                e.configure(state=st, bg=bg)
        combo.bind("<<ComboboxSelected>>", _toggle)
        _toggle()

        return dict(name_v=name_v, link_v=link_v, z_v=z_v,
                    axis_v=axis_v, cx=cx, cy=cy, cz=cz)

    # ── Helpers ───────────────────────────────────────────────────────────
    def _section(self, parent, text):
        f = tk.Frame(parent, bg=self.BG)
        f.pack(fill="x", pady=(14,4))
        tk.Label(f, text=text, bg=self.BG, fg=self.ACCENT,
                 font=("Arial",11,"bold")).pack(side="left")
        ttk.Separator(f, orient="horizontal").pack(
            side="left", fill="x", expand=True, padx=(10,0))

    def _entry(self, parent, var, width, state="normal", bg=None):
        return tk.Entry(parent, textvariable=var, width=width,
                        bg=bg or self.INPUT, fg=self.TEXT,
                        insertbackground=self.ACCENT,
                        disabledbackground=self.BORDER,
                        disabledforeground=self.DIM,
                        relief="flat", font=self.FONT_MONO,
                        state=state)

    def _browse(self):
        p = filedialog.asksaveasfilename(
            defaultextension=".yaml",
            filetypes=[("YAML", "*.yaml"), ("All", "*.*")],
            initialfile="dntd_mmwave_config.yaml",
            title="Choose config file",
        )
        if p:
            self._cfg_var.set(p)

    # ── Collect form data ─────────────────────────────────────────────────
    def _collect(self):
        rows = []
        for r in self._joint_rows:
            sel = r["axis_v"].get()
            if sel.startswith("Custom"):
                axis = [r["cx"].get(), r["cy"].get(), r["cz"].get()]
            else:
                axis = AXIS_PRESETS.get(sel) or [0.0, 0.0, 1.0]
            rows.append(dict(
                name    = r["name_v"].get().strip(),
                link_mm = r["link_v"].get(),
                z_mm    = r["z_v"].get(),
                axis    = axis,
            ))
        return rows

    def _build_payload(self, rows):
        names    = [r["name"] for r in rows]
        geom     = build_joint_geometry(rows)
        reach    = estimate_reach(rows)
        idx      = max(0, self._mount_joint.get() - 1)
        mount_lk = names[idx] if idx < len(names) else names[-1]
        mount_m  = [round(v/1000, 4) for v in
                    [self._mx.get(), self._my.get(), self._mz.get()]]
        return dict(
            joint_names                  = names,
            joint_geometry               = geom,
            sensor_mount_link            = mount_lk,
            sensor_mount_xyz             = mount_m,
            stop_range_m                 = self._stop_mm.get() / 1000,
            caution_range_m              = self._caution_mm.get() / 1000,
            swept_volume_mount_joint_idx = idx,
            swept_volume_max_reach_m     = round(reach, 3),
        )

    def _validate(self, rows):
        names = [r["name"] for r in rows]
        if any(not n for n in names):
            raise ValueError("All joints must have a name.")
        if len(set(names)) != len(names):
            raise ValueError("Joint names must be unique.")

    # ── Actions ───────────────────────────────────────────────────────────
    def _preview(self):
        try:
            rows    = self._collect()
            self._validate(rows)
            payload = self._build_payload(rows)
        except Exception as e:
            messagebox.showerror("Error", str(e))
            return

        # Show a diff-style preview of what will change
        win = tk.Toplevel(self.root)
        win.title("Preview — changes to dntd_mmwave_config.yaml")
        win.configure(bg=self.BG)
        win.geometry("660x500")

        tk.Label(win,
                 text="These keys will be updated. All comments and other settings are preserved.",
                 bg=self.BG, fg=self.DIM, font=self.FONT_SMALL,
                 anchor="w").pack(fill="x", padx=16, pady=(12,4))

        txt = tk.Text(win, bg=self.INPUT, fg=self.TEXT, font=self.FONT_MONO,
                      relief="flat", padx=12, pady=10, wrap="none",
                      insertbackground=self.ACCENT)
        sb  = ttk.Scrollbar(win, command=txt.yview)
        txt.configure(yscrollcommand=sb.set)
        sb.pack(side="right", fill="y")
        txt.pack(fill="both", expand=True, padx=16, pady=(0,12))

        lines  = [f"sensor_mount_link: \"{payload['sensor_mount_link']}\"",
                  f"sensor_mount_xyz:  {payload['sensor_mount_xyz']}",
                  f"stop_range_m:      {payload['stop_range_m']:.3f}",
                  f"caution_range_m:   {payload['caution_range_m']:.3f}",
                  f"swept_volume_mount_joint_idx: {payload['swept_volume_mount_joint_idx']}",
                  f"swept_volume_max_reach_m:     {payload['swept_volume_max_reach_m']:.3f}",
                  "",
                  "joint_names:"]
        lines += [f"  - {n}" for n in payload["joint_names"]]
        lines += ["", "joint_geometry:"]
        for jname, d in payload["joint_geometry"].items():
            lines += [
                f"  {jname}:",
                f"    origin_xyz: {d['origin_xyz']}",
                f"    origin_rpy: {d['origin_rpy']}",
                f"    axis: {d['axis']}",
            ]

        txt.insert("1.0", "\n".join(lines))
        txt.configure(state="disabled")

    def _save(self):
        try:
            rows    = self._collect()
            self._validate(rows)
            payload = self._build_payload(rows)
            patch_config(self._cfg_var.get(), payload)
        except FileNotFoundError as e:
            messagebox.showerror("File not found", str(e))
            return
        except Exception as e:
            messagebox.showerror("Save failed", str(e))
            return

        reach = payload["swept_volume_max_reach_m"]
        n     = len(payload["joint_names"])
        self._status.set(
            f"✓  Saved — {n} joints, reach ≈ {reach*100:.0f} cm  |  "
            f"stop {payload['stop_range_m']*100:.0f} cm  "
            f"caution {payload['caution_range_m']*100:.0f} cm"
        )
        self._status_lbl.configure(fg=self.SUCCESS)


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def main():
    p = argparse.ArgumentParser(description="RadarGuard arm configuration GUI")
    p.add_argument("--config", default=DEFAULT_CONFIG,
                   help="Path to dntd_mmwave_config.yaml")
    args = p.parse_args()

    config_path = os.path.abspath(args.config)
    if not os.path.exists(config_path):
        print(f"Warning: config not found at {config_path}")
        print("Run from repo root or pass --config with the correct path.")

    root = tk.Tk()
    ArmConfigGUI(root, config_path)
    root.mainloop()


if __name__ == "__main__":
    import argparse
    main()
