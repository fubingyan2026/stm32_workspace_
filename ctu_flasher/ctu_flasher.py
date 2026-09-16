# -*- coding: utf-8 -*-
"""E1 CTU 固件打包 / J-Link 烧录工具（简易界面，仅标准库 + tkinter）

用途:
  1. 把 E1_CTU_BOOT 引导固件与 E1_MASTER/E1_SLAVER_POWER_CTU App 固件按 Flash
     布局拼接为可直接烧录的整包，并做尺寸/向量表/App 签名预检；
  2. 调用 SEGGER J-Link Commander(JLink.exe) 把整包（或单独 App）烧录到目标板。

Flash 布局（见 E1_CTU_BOOT/docs/boot_485_ymodem.md）:
    Boot   0x08000000   32K (0x8000)
    App A  0x08008000   96K (0x18000)   App 固定链接地址（含 +0x200 处 .app_sig 签名）

运行: python ctu_flasher.py
依赖: 仅 Python 标准库；烧录需已安装 SEGGER J-Link 软件（自动检测 JLink.exe）。
"""

from __future__ import annotations

import json
import os
import queue
import re
import struct
import subprocess
import sys
import threading
import time
import zlib
from pathlib import Path
from tkinter import BOTH, END, LEFT, RIGHT, X, Y, StringVar, BooleanVar, IntVar
from tkinter import filedialog, messagebox
import tkinter as tk
from tkinter import ttk

# ---------------- 常量 ----------------
# 打包为 exe 运行时 __file__ 指向临时解压目录，须以 exe 实际位置为工具目录
if getattr(sys, "frozen", False):
    EXE_DIR = Path(sys.executable).resolve().parent
else:
    EXE_DIR = Path(__file__).resolve().parent
# 仓库根：开发时 <root>/ctu_flasher/ · exe 在 <root>/ctu_flasher/dist/
ROOT = EXE_DIR.parent if EXE_DIR.name.lower() != "dist" else EXE_DIR.parent.parent
DEFAULT_OUT = EXE_DIR / "out"

BOOT_ADDR = 0x08000000
APP_ADDR = 0x08008000
BOOT_SIZE = 0x8000
APP_SIZE = 0x18000
APP_SIG_OFFSET = 0x200
APP_SIG_MAGIC = 0x41505031
RAM_START = 0x20000000
RAM_END = 0x2000C000

BOOT_PROJECT = "E1_CTU_BOOT"
BOARDS = {
    "E1_MASTER": {
        "project": "E1_MASTER_POWER_CTU",
        "bin": "E1_MASTER_POWER_CTU.bin",
        "label": "E1_MASTER 主控板",
    },
    "E1_SLAVER": {
        "project": "E1_SLAVER_POWER_CTU",
        "bin": "E1_SLAVER_POWER_CTU.bin",
        "label": "E1_SLAVER 副电源板",
    },
}

CONFIG_PATH = Path.home() / ".ctu_flasher.json"

ERROR_MARKERS = (
    "error:", "failed", "cannot ", "can not", "could not",
    "not found", "no j-link", "unknown device", "verification failed",
)


def human_size(n: int) -> str:
    return f"{n} B ({n / 1024:.1f} KB)"


def load_config() -> dict:
    try:
        return json.loads(CONFIG_PATH.read_text(encoding="utf-8"))
    except Exception:  # noqa: BLE001
        return {}


def save_config(data: dict) -> None:
    try:
        CONFIG_PATH.write_text(json.dumps(data, ensure_ascii=False, indent=2),
                               encoding="utf-8")
    except Exception:  # noqa: BLE001
        pass


def find_newest_bin(project: str, name_hint: str) -> Path | None:
    """在 <project>/build 下查找最近修改的匹配 .bin（Debug/Release 均可）"""
    build = ROOT / project / "build"
    cands: list[Path] = []
    if build.is_dir():
        for p in build.rglob("*.bin"):
            if name_hint.lower() in p.name.lower():
                cands.append(p)
    if not cands:
        return None
    return max(cands, key=lambda p: p.stat().st_mtime)


def autodetect_jlink() -> Path | None:
    """自动检测 SEGGER J-Link Commander（取版本号最大者）"""
    patterns = [
        Path(os.environ.get("ProgramFiles", r"C:\Program Files")) / "SEGGER",
        Path(os.environ.get("ProgramFiles(x86)", r"C:\Program Files (x86)")) / "SEGGER",
        Path(r"C:\Program Files") / "SEGGER",
    ]
    found: list[Path] = []
    for base in patterns:
        if base.is_dir():
            found.extend(base.glob("JLink_V*/JLink.exe"))
    if not found:
        return None

    def version_key(p: Path) -> tuple:
        m = re.search(r"JLink_V(\d+)", p.parent.name)
        return (int(m.group(1)) if m else 0, str(p))

    return max(found, key=version_key)


def inspect_image(path: Path, base_addr: int, max_size: int) -> tuple[dict, list[str]]:
    """读取并预检固件：返回 (信息字典, 警告列表)；硬错误抛 ValueError"""
    if not path.is_file():
        raise ValueError(f"文件不存在: {path}")
    data = path.read_bytes()
    if not data:
        raise ValueError(f"文件为空: {path}")
    if len(data) > max_size:
        raise ValueError(
            f"{path.name} 超出分区容量: {human_size(len(data))} > {human_size(max_size)}")

    warns: list[str] = []
    if len(data) < 0x20:
        warns.append(f"{path.name} 仅 {len(data)} 字节，疑似空白/未链接固件")

    sp = rv = 0
    if len(data) >= 8:
        sp, rv = struct.unpack_from("<II", data, 0)
        if not (RAM_START <= sp <= RAM_END):
            warns.append(f"{path.name} 向量表初值 SP=0x{sp:08X} 不在 RAM 范围")
        if not (rv & 1) or not (base_addr <= (rv & ~1) < base_addr + len(data)):
            warns.append(f"{path.name} 复位向量 0x{rv:08X} 不在 0x{base_addr:08X} 区段")

    sig = None
    if base_addr == APP_ADDR and len(data) >= APP_SIG_OFFSET + 4:
        sig = struct.unpack_from("<I", data, APP_SIG_OFFSET)[0]
        if sig != APP_SIG_MAGIC:
            warns.append(
                f"{path.name} 缺少 App 签名（+0x200 魔数=0x{sig:08X}，期望 0x{APP_SIG_MAGIC:08X}）"
                "：量产 Boot 全校验模式下将无法启动，仅 BOOT_SKIP_APP_VERIFY=1 调试模式可用")

    info = {
        "size": len(data),
        "crc32": zlib.crc32(data) & 0xFFFFFFFF,
        "sum": sum(data) & 0xFFFFFFFF,
        "mtime": time.strftime("%Y-%m-%d %H:%M:%S", time.localtime(path.stat().st_mtime)),
        "sig": sig,
    }
    return info, warns


def merge_firmware(boot_path: Path, app_path: Path, out_path: Path) -> dict:
    """Boot + App 拼接为整包（Boot 不足 32K 处以 0xFF 补齐）"""
    boot_info, boot_warns = inspect_image(boot_path, BOOT_ADDR, BOOT_SIZE)
    app_info, app_warns = inspect_image(app_path, APP_ADDR, APP_SIZE)

    boot = boot_path.read_bytes()
    app = app_path.read_bytes()
    merged = boot + b"\xff" * (BOOT_SIZE - len(boot)) + app

    out_path.parent.mkdir(parents=True, exist_ok=True)
    out_path.write_bytes(merged)

    return {
        "out": out_path,
        "boot": boot_info,
        "app": app_info,
        "warns": boot_warns + app_warns,
        "size": len(merged),
        "crc32": zlib.crc32(merged) & 0xFFFFFFFF,
        "addr": BOOT_ADDR,
    }


class FlasherApp(tk.Tk):
    def __init__(self) -> None:
        super().__init__()
        self.title("E1 CTU 固件打包 / J-Link 烧录工具")
        self.geometry("980x760")
        self.minsize(900, 680)

        cfg = load_config()
        self._q: queue.Queue = queue.Queue()
        self._proc: subprocess.Popen | None = None
        self._worker: threading.Thread | None = None
        self._stopping = False
        self._op_name = "烧录"
        # 最近使用的固件/JLink 路径（每个输入框最多 10 条，持久化）
        raw_recent = cfg.get("recent", {}) or {}
        self._recent: dict[str, list[str]] = {
            k: [str(x) for x in raw_recent.get(k, []) if x]
            for k in ("boot", "master", "slaver", "jlink")
        }
        self._recent_boxes: dict[str, ttk.Combobox] = {}
        self._save_timer: str | None = None

        # 变量
        self.var_boot = StringVar(value=str(cfg.get("boot", "")))
        self.var_master = StringVar(value=str(cfg.get("master", "")))
        self.var_slaver = StringVar(value=str(cfg.get("slaver", "")))
        self.var_outdir = StringVar(value=str(cfg.get("outdir", DEFAULT_OUT)))
        self.var_jlink = StringVar(value=str(cfg.get("jlink", "") or (autodetect_jlink() or "")))
        self.var_device = StringVar(value=str(cfg.get("device", "STM32F103RC")))
        self.var_interface = StringVar(value=str(cfg.get("interface", "SWD")))
        self.var_speed = IntVar(value=int(cfg.get("speed", 4000)))
        self.var_board = StringVar(value=str(cfg.get("board", "E1_MASTER")))
        self.var_mode = StringVar(value=str(cfg.get("mode", "pack")))
        self.var_erase = BooleanVar(value=bool(cfg.get("erase", True)))
        self.var_verify = BooleanVar(value=bool(cfg.get("verify", True)))
        self.var_reset = BooleanVar(value=bool(cfg.get("reset", True)))
        self.var_status = StringVar(value="就绪：选择固件 → 打包 → 烧录")

        self._build_ui()
        self._bind_autosave()
        self.protocol("WM_DELETE_WINDOW", self._on_close)
        self._poll_timer: str | None = self.after(80, self._poll_queue)
        self._startup_timer: str | None = self.after(200, self._log_startup)

    # ---------------- UI ----------------
    def _build_ui(self) -> None:
        style = ttk.Style(self)
        try:
            style.theme_use("vista")
        except tk.TclError:
            pass

        outer = ttk.Frame(self, padding=10)
        outer.pack(fill=BOTH, expand=True)

        ttk.Label(outer, text="E1 CTU 固件打包 / J-Link 烧录",
                  font=("Microsoft YaHei UI", 14, "bold")).pack(anchor="w")
        ttk.Label(outer, text=("Boot 0x08000000 (32K) + App A 0x08008000 (96K)；"
                               "拼接整包后由 J-Link 烧录"),
                  foreground="#666").pack(anchor="w", pady=(0, 8))

        # --- 固件输入 ---
        box_fw = ttk.LabelFrame(outer, text="固件输入", padding=8)
        box_fw.pack(fill=X)
        self._file_row(box_fw, 0, "Boot 固件 (E1_CTU_BOOT):", self.var_boot, "boot")
        self._file_row(box_fw, 1, "E1_MASTER App:", self.var_master, "master")
        self._file_row(box_fw, 2, "E1_SLAVER App:", self.var_slaver, "slaver")

        row = ttk.Frame(box_fw)
        row.grid(row=3, column=0, columnspan=3, sticky="w", pady=(6, 0))
        ttk.Button(row, text="自动检测（各工程 build 目录）",
                   command=self._on_autodetect).pack(side=LEFT)
        ttk.Button(row, text="清空", command=self._on_clear).pack(side=LEFT, padx=6)
        self.lbl_fw = ttk.Label(row, text="", foreground="#666")
        self.lbl_fw.pack(side=LEFT, padx=10)

        # --- 打包 ---
        box_pack = ttk.LabelFrame(outer, text="打包（Boot + App 拼接）", padding=8)
        box_pack.pack(fill=X, pady=(8, 0))
        ttk.Label(box_pack, text="输出目录:").grid(row=0, column=0, sticky="w")
        ttk.Entry(box_pack, textvariable=self.var_outdir).grid(
            row=0, column=1, sticky="ew", padx=4)
        ttk.Button(box_pack, text="浏览", command=self._pick_outdir).grid(row=0, column=2)
        box_pack.columnconfigure(1, weight=1)

        row = ttk.Frame(box_pack)
        row.grid(row=1, column=0, columnspan=3, sticky="w", pady=(6, 0))
        ttk.Button(row, text="打包固件（两块板各一份整包）",
                   command=self._on_pack).pack(side=LEFT)
        ttk.Button(row, text="打开输出目录",
                   command=self._open_outdir).pack(side=LEFT, padx=6)
        self.lbl_pack = ttk.Label(row, text="", foreground="#666")
        self.lbl_pack.pack(side=LEFT, padx=10)

        # --- J-Link 烧录 ---
        box_jl = ttk.LabelFrame(outer, text="J-Link 烧录", padding=8)
        box_jl.pack(fill=X, pady=(8, 0))

        ttk.Label(box_jl, text="JLink.exe:").grid(row=0, column=0, sticky="w")
        cb_jlink = ttk.Combobox(box_jl, textvariable=self.var_jlink,
                                values=self._recent.get("jlink", []))
        cb_jlink.grid(row=0, column=1, sticky="ew", padx=4)
        self._recent_boxes["jlink"] = cb_jlink
        ttk.Button(box_jl, text="自动检测", command=self._on_detect_jlink).grid(row=0, column=2)
        ttk.Button(box_jl, text="浏览", command=self._pick_jlink).grid(row=0, column=3)

        row = ttk.Frame(box_jl)
        row.grid(row=1, column=0, columnspan=4, sticky="w", pady=(6, 0))
        ttk.Label(row, text="Device:").pack(side=LEFT)
        ttk.Entry(row, textvariable=self.var_device, width=16).pack(side=LEFT, padx=4)
        ttk.Label(row, text="Interface:").pack(side=LEFT)
        ttk.Combobox(row, textvariable=self.var_interface, width=7,
                     values=("SWD", "JTAG"), state="readonly").pack(side=LEFT, padx=4)
        ttk.Label(row, text="Speed(kHz):").pack(side=LEFT)
        ttk.Spinbox(row, textvariable=self.var_speed, from_=100, to=12000,
                    increment=100, width=8).pack(side=LEFT, padx=4)

        row = ttk.Frame(box_jl)
        row.grid(row=2, column=0, columnspan=4, sticky="w", pady=(6, 0))
        ttk.Label(row, text="目标板:").pack(side=LEFT)
        for key, meta in BOARDS.items():
            ttk.Radiobutton(row, text=meta["label"], value=key,
                            variable=self.var_board).pack(side=LEFT, padx=(4, 10))
        ttk.Label(row, text="烧录内容:").pack(side=LEFT, padx=(10, 0))
        ttk.Radiobutton(row, text="整包(Boot+App)", value="pack", variable=self.var_mode,
                        command=self._sync_mode).pack(side=LEFT)
        ttk.Radiobutton(row, text="仅 App", value="app", variable=self.var_mode,
                        command=self._sync_mode).pack(side=LEFT, padx=(4, 0))

        row = ttk.Frame(box_jl)
        row.grid(row=3, column=0, columnspan=4, sticky="w", pady=(6, 0))
        self.chk_erase = ttk.Checkbutton(row, text="烧录前全片擦除",
                                         variable=self.var_erase)
        self.chk_erase.pack(side=LEFT)
        ttk.Checkbutton(row, text="校验写入", variable=self.var_verify).pack(side=LEFT, padx=10)
        ttk.Checkbutton(row, text="复位并运行", variable=self.var_reset).pack(side=LEFT, padx=10)

        row = ttk.Frame(box_jl)
        row.grid(row=4, column=0, columnspan=4, sticky="w", pady=(8, 0))
        self.btn_flash = ttk.Button(row, text="开始烧录", command=self._on_flash)
        self.btn_flash.pack(side=LEFT)
        self.btn_erase = ttk.Button(row, text="仅擦除全片", command=self._on_erase)
        self.btn_erase.pack(side=LEFT, padx=6)
        self.btn_stop = ttk.Button(row, text="中止", command=self._on_stop, state="disabled")
        self.btn_stop.pack(side=LEFT, padx=6)
        box_jl.columnconfigure(1, weight=1)

        # --- 日志 ---
        box_log = ttk.LabelFrame(outer, text="日志", padding=6)
        box_log.pack(fill=BOTH, expand=True, pady=(8, 0))
        self.txt = tk.Text(box_log, height=14, wrap="none", font=("Consolas", 9),
                           background="#FBFBF8")
        yscroll = ttk.Scrollbar(box_log, orient="vertical", command=self.txt.yview)
        self.txt.configure(yscrollcommand=yscroll.set)
        self.txt.pack(side=LEFT, fill=BOTH, expand=True)
        yscroll.pack(side=RIGHT, fill=Y)
        self.txt.tag_configure("info", foreground="#333333")
        self.txt.tag_configure("ok", foreground="#2E7D32")
        self.txt.tag_configure("warn", foreground="#B26A00")
        self.txt.tag_configure("error", foreground="#C62828")
        self.txt.tag_configure("cmd", foreground="#1565C0")
        self.txt.configure(state="disabled")

        ttk.Label(outer, textvariable=self.var_status,
                  relief="sunken", anchor="w", padding=4).pack(fill=X, pady=(6, 0))
        ttk.Label(outer, text=f"路径/设置自动保存: {CONFIG_PATH}",
                  foreground="#888").pack(anchor="w", pady=(2, 0))
        self._sync_mode()

    def _file_row(self, parent: ttk.LabelFrame, row: int, label: str,
                  var: StringVar, kind: str) -> None:
        ttk.Label(parent, text=label).grid(row=row, column=0, sticky="w", pady=2)
        cb = ttk.Combobox(parent, textvariable=var, values=self._recent.get(kind, []))
        cb.grid(row=row, column=1, sticky="ew", padx=4)
        self._recent_boxes[kind] = cb
        ttk.Button(parent, text="浏览",
                   command=lambda: self._pick_file(var)).grid(row=row, column=2)
        parent.columnconfigure(1, weight=1)

    # ---------------- 配置持久化（路径自动保存） ----------------
    def _bind_autosave(self) -> None:
        for var in (self.var_boot, self.var_master, self.var_slaver,
                    self.var_outdir, self.var_jlink, self.var_device,
                    self.var_interface, self.var_speed, self.var_board,
                    self.var_mode, self.var_erase, self.var_verify,
                    self.var_reset):
            var.trace_add("write", self._on_var_changed)

    def _on_var_changed(self, *_args) -> None:
        for kind, var in (("boot", self.var_boot), ("master", self.var_master),
                          ("slaver", self.var_slaver), ("jlink", self.var_jlink)):
            self._remember(kind, var.get())
        self._schedule_save()

    def _remember(self, kind: str, value: str) -> None:
        """记录一条最近使用的路径（去重、置顶、最多 10 条）"""
        value = (value or "").strip()
        if not value or not Path(value).is_file():
            return
        lst = self._recent.setdefault(kind, [])
        if value in lst:
            lst.remove(value)
        lst.insert(0, value)
        del lst[10:]
        box = self._recent_boxes.get(kind)
        if box is not None:
            box.configure(values=lst)

    def _schedule_save(self) -> None:
        """改动后防抖保存（600ms），避免逐字符写盘"""
        if self._save_timer is not None:
            try:
                self.after_cancel(self._save_timer)
            except Exception:  # noqa: BLE001
                pass
        self._save_timer = self.after(600, self._save_now)

    def _collect_config(self) -> dict:
        return {
            "boot": self.var_boot.get(),
            "master": self.var_master.get(),
            "slaver": self.var_slaver.get(),
            "outdir": self.var_outdir.get(),
            "jlink": self.var_jlink.get(),
            "device": self.var_device.get(),
            "interface": self.var_interface.get(),
            "speed": int(self.var_speed.get()),
            "board": self.var_board.get(),
            "mode": self.var_mode.get(),
            "erase": bool(self.var_erase.get()),
            "verify": bool(self.var_verify.get()),
            "reset": bool(self.var_reset.get()),
            "recent": {k: v for k, v in self._recent.items() if v},
        }

    def _save_now(self) -> None:
        self._save_timer = None
        save_config(self._collect_config())

    def _log_startup(self) -> None:
        self._startup_timer = None
        self.log(f"配置: {CONFIG_PATH}（路径改动自动保存）")
        restored = []
        for label, var, base, size in (
                ("Boot", self.var_boot, BOOT_ADDR, BOOT_SIZE),
                ("MASTER App", self.var_master, APP_ADDR, APP_SIZE),
                ("SLAVER App", self.var_slaver, APP_ADDR, APP_SIZE)):
            p = Path(var.get())
            if p.is_file():
                restored.append(f"{label}={p.name}")
            elif var.get():
                self.log(f"⚠ 上次的 {label} 路径已失效: {var.get()}", "warn")
        if restored:
            self.log("已载入上次固件: " + "  ".join(restored), "ok")
        self._update_fw_summary()

    # ---------------- 日志 ----------------
    def log(self, text: str, level: str = "info") -> None:
        ts = time.strftime("%H:%M:%S")
        self.txt.configure(state="normal")
        self.txt.insert(END, f"[{ts}] {text}\n", level)
        self.txt.see(END)
        self.txt.configure(state="disabled")

    def _poll_queue(self) -> None:
        self._poll_timer = None
        try:
            while True:
                kind, payload = self._q.get_nowait()
                if kind == "@done":
                    self._finish_ui(bool(payload))
                else:
                    self.log(str(payload), kind)
        except queue.Empty:
            pass
        self._poll_timer = self.after(80, self._poll_queue)

    # ---------------- 文件选择 ----------------
    def _pick_file(self, var: StringVar) -> None:
        path = filedialog.askopenfilename(
            title="选择固件 .bin",
            filetypes=[("Firmware", "*.bin"), ("All files", "*.*")])
        if path:
            var.set(path)

    def _pick_outdir(self) -> None:
        path = filedialog.askdirectory(title="选择输出目录")
        if path:
            self.var_outdir.set(path)

    def _pick_jlink(self) -> None:
        path = filedialog.askopenfilename(
            title="选择 JLink.exe",
            filetypes=[("JLink", "JLink.exe"), ("Executable", "*.exe"), ("All files", "*.*")])
        if path:
            self.var_jlink.set(path)

    def _open_outdir(self) -> None:
        out = Path(self.var_outdir.get())
        out.mkdir(parents=True, exist_ok=True)
        try:
            os.startfile(out)  # noqa: S606
        except Exception as exc:  # noqa: BLE001
            messagebox.showwarning("打开失败", str(exc))

    def _on_detect_jlink(self) -> None:
        found = autodetect_jlink()
        if found:
            self.var_jlink.set(str(found))
            self.log(f"J-Link: {found}", "ok")
        else:
            self.log("未自动检测到 JLink.exe，请手动浏览选择", "warn")

    # ---------------- 自动检测固件 ----------------
    def _on_autodetect(self) -> None:
        boot = find_newest_bin(BOOT_PROJECT, "E1_CTU_BOOT")
        if boot:
            self.var_boot.set(str(boot))
        master = find_newest_bin(BOARDS["E1_MASTER"]["project"], "E1_MASTER_POWER_CTU")
        if master is None:
            fallback = ROOT / "ctu_host" / "ctu_sdk_c" / "build" / "E1_MASTER_POWER_CTU.bin"
            master = fallback if fallback.is_file() else None
        if master:
            self.var_master.set(str(master))
        slaver = find_newest_bin(BOARDS["E1_SLAVER"]["project"], "E1_SLAVER_POWER_CTU")
        if slaver is None:
            fallback = ROOT / "ctu_host" / "ctu_sdk_c" / "build" / "E1_SLAVER_POWER_CTU.bin"
            slaver = fallback if fallback.is_file() else None
        if slaver:
            self.var_slaver.set(str(slaver))

        missing = [t for t, p in (("Boot", boot), ("MASTER App", master),
                                  ("SLAVER App", slaver)) if p is None]
        self.log(f"自动检测完成；未找到: {'、'.join(missing) if missing else '无（全部命中）'}",
                 "warn" if missing else "ok")
        self._update_fw_summary()

    def _on_clear(self) -> None:
        self.var_boot.set("")
        self.var_master.set("")
        self.var_slaver.set("")
        self.lbl_fw.configure(text="")
        self.lbl_pack.configure(text="")

    def _update_fw_summary(self) -> None:
        parts = []
        for label, var, base, size in (
                ("Boot", self.var_boot, BOOT_ADDR, BOOT_SIZE),
                ("MASTER", self.var_master, APP_ADDR, APP_SIZE),
                ("SLAVER", self.var_slaver, APP_ADDR, APP_SIZE)):
            p = Path(var.get())
            if p.is_file():
                try:
                    info, _ = inspect_image(p, base, size)
                    parts.append(f"{label}={info['size'] / 1024:.1f}K")
                except ValueError:
                    parts.append(f"{label}=非法")
            else:
                parts.append(f"{label}=未选")
        self.lbl_fw.configure(text="  ".join(parts))

    # ---------------- 打包 ----------------
    def _pack_one(self, board: str) -> Path:
        meta = BOARDS[board]
        boot = Path(self.var_boot.get())
        app = Path(self.var_master.get() if board == "E1_MASTER" else self.var_slaver.get())
        if not boot.is_file():
            raise ValueError("请先选择 Boot 固件")
        if not app.is_file():
            raise ValueError(f"请先选择 {meta['label']} App 固件")

        out = Path(self.var_outdir.get()) / f"{meta['project']}_merged.bin"
        res = merge_firmware(boot, app, out)
        self.log(f"[{meta['label']}] 整包: {out.name}  "
                 f"{human_size(res['size'])}  CRC32=0x{res['crc32']:08X}  "
                 f"范围 0x{BOOT_ADDR:08X}~0x{BOOT_ADDR + res['size']:08X}", "ok")
        self.log(f"  Boot {res['boot']['size']}B (mtime {res['boot']['mtime']}) + "
                 f"App {res['app']['size']}B (mtime {res['app']['mtime']})")
        for w in res["warns"]:
            self.log(f"  ⚠ {w}", "warn")
        return out

    def _on_pack(self) -> None:
        try:
            outs = []
            for board in ("E1_MASTER", "E1_SLAVER"):
                var = self.var_master if board == "E1_MASTER" else self.var_slaver
                if Path(var.get()).is_file():
                    outs.append(self._pack_one(board))
                else:
                    self.log(f"跳过 {BOARDS[board]['label']}：未选择 App 固件", "warn")
            if not outs:
                raise ValueError("没有可打包的固件")
            self.lbl_pack.configure(text=f"已生成 {len(outs)} 个整包")
            self.var_status.set(f"打包完成：{len(outs)} 个整包")
        except Exception as exc:  # noqa: BLE001
            self.log(f"打包失败: {exc}", "error")
            messagebox.showerror("打包失败", str(exc))

    # ---------------- 烧录 ----------------
    def _sync_mode(self) -> None:
        if self.var_mode.get() == "app":
            self.chk_erase.state(["disabled"])
            self.var_erase.set(False)
        else:
            self.chk_erase.state(["!disabled"])

    def _build_script(self) -> tuple[Path, Path, int]:
        """生成 J-Link Commander 脚本；返回 (脚本路径, 烧录文件, 起始地址)"""
        board = self.var_board.get()
        meta = BOARDS[board]
        if self.var_mode.get() == "app":
            app = Path(self.var_master.get() if board == "E1_MASTER" else self.var_slaver.get())
            if not app.is_file():
                raise ValueError(f"请先选择 {meta['label']} App 固件")
            info, warns = inspect_image(app, APP_ADDR, APP_SIZE)
            for w in warns:
                self.log(f"⚠ {w}", "warn")
            target, addr = app, APP_ADDR
            self.log(f"[仅 App] {app.name} {human_size(info['size'])} "
                     f"CRC32=0x{info['crc32']:08X} → 0x{addr:08X}")
        else:
            target = self._pack_one(board)
            addr = BOOT_ADDR

        outdir = Path(self.var_outdir.get())
        outdir.mkdir(parents=True, exist_ok=True)
        script = outdir / f"{meta['project']}_flash.jlink"
        posix = str(target.resolve()).replace("\\", "/")

        lines = [
            f"device {self.var_device.get()}",
            f"si {self.var_interface.get()}",
            f"speed {int(self.var_speed.get())}",
            "connect",
            "r",
            "h",
        ]
        # 仅整包模式允许全片擦除；仅 App 模式绝不擦全片，避免擦掉 Boot
        if self.var_mode.get() == "pack" and self.var_erase.get():
            lines.append("erase")
        lines.append(f'loadbin "{posix}", 0x{addr:08X}')
        if self.var_verify.get():
            lines.append(f'verifybin "{posix}", 0x{addr:08X}')
        if self.var_reset.get():
            lines += ["r", "g"]
        lines.append("qc")
        script.write_text("\n".join(lines) + "\n", encoding="ascii")

        self.log(f"J-Link 脚本: {script.name}", "cmd")
        for ln in lines:
            self.log(f"  > {ln}", "cmd")
        return script, target, addr

    def _on_flash(self) -> None:
        if self._worker and self._worker.is_alive():
            return
        jlink = self._require_jlink()
        if jlink is None:
            return
        try:
            script, target, addr = self._build_script()
        except Exception as exc:  # noqa: BLE001
            self.log(f"准备烧录失败: {exc}", "error")
            messagebox.showerror("烧录失败", str(exc))
            return

        board = BOARDS[self.var_board.get()]["label"]
        self._start_jlink(jlink, script, "烧录",
                          desc=f"{board} @0x{addr:08X}",
                          target_name=target.name)

    def _require_jlink(self) -> Path | None:
        jlink = Path(self.var_jlink.get())
        if not jlink.is_file():
            messagebox.showerror("缺少 J-Link", "请选择有效的 JLink.exe")
            return None
        return jlink

    def _build_erase_script(self) -> Path:
        """仅擦除：连接目标后全片擦除，不写任何固件"""
        outdir = Path(self.var_outdir.get())
        outdir.mkdir(parents=True, exist_ok=True)
        script = outdir / "E1_CTU_erase_all.jlink"
        lines = [
            f"device {self.var_device.get()}",
            f"si {self.var_interface.get()}",
            f"speed {int(self.var_speed.get())}",
            "connect",
            "r",
            "h",
            "erase",
            "qc",
        ]
        script.write_text("\n".join(lines) + "\n", encoding="ascii")
        self.log(f"J-Link 脚本: {script.name}", "cmd")
        for ln in lines:
            self.log(f"  > {ln}", "cmd")
        return script

    def _on_erase(self) -> None:
        if self._worker and self._worker.is_alive():
            return
        if not messagebox.askyesno(
                "确认全片擦除",
                "将擦除目标板整片 Flash（Boot / App A / App B / Metadata 全部清空），\n"
                "擦除后板上无固件可运行，需重新烧录。\n\n确定继续？",
                icon="warning", default="no"):
            return
        jlink = self._require_jlink()
        if jlink is None:
            return
        try:
            script = self._build_erase_script()
        except Exception as exc:  # noqa: BLE001
            self.log(f"准备擦除失败: {exc}", "error")
            messagebox.showerror("擦除失败", str(exc))
            return

        self._start_jlink(jlink, script, "擦除",
                          desc=BOARDS[self.var_board.get()]["label"])

    def _start_jlink(self, jlink: Path, script: Path, op: str,
                     desc: str = "", target_name: str = "") -> None:
        """启动 J-Link Commander（烧录 / 擦除共用）"""
        args = [str(jlink), "-CommanderScript", str(script.resolve())]
        self._op_name = op
        self._stopping = False
        self.btn_flash.configure(state="disabled")
        self.btn_erase.configure(state="disabled")
        self.btn_stop.configure(state="normal")
        self.var_status.set(f"{op}中… {desc}".strip())
        extra = f"  ({target_name})" if target_name else ""
        self.log(f"启动: {' '.join(args)}{extra}", "cmd")

        self._worker = threading.Thread(target=self._flash_worker,
                                        args=(args,), daemon=True)
        self._worker.start()

    def _flash_worker(self, args: list[str]) -> None:
        creationflags = 0
        if os.name == "nt":
            creationflags = getattr(subprocess, "CREATE_NO_WINDOW", 0)
        try:
            proc = subprocess.Popen(
                args, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                stdin=subprocess.DEVNULL, creationflags=creationflags)
        except Exception as exc:  # noqa: BLE001
            self._q.put(("error", f"无法启动 JLink.exe: {exc}"))
            self._q.put(("@done", False))
            return

        self._proc = proc
        problems: list[str] = []
        buf = b""
        try:
            while True:
                chunk = proc.stdout.read(256) if proc.stdout else b""
                if not chunk:
                    break
                buf += chunk
                while True:
                    m = re.search(rb"[\r\n]", buf)
                    if not m:
                        break
                    line = buf[:m.start()].decode("utf-8", "replace").strip()
                    buf = buf[m.end():]
                    if line:
                        self._q.put(("info", f"  {line}"))
                        low = line.lower()
                        if any(mk in low for mk in ERROR_MARKERS):
                            problems.append(line)
                        else:
                            m2 = re.search(r"errors?:\s*(\d+)", low)
                            if m2 and int(m2.group(1)) > 0:
                                problems.append(line)
            tail = buf.decode("utf-8", "replace").strip()
            if tail:
                self._q.put(("info", f"  {tail}"))
            rc = proc.wait()
        except Exception as exc:  # noqa: BLE001
            self._q.put(("error", f"读取 J-Link 输出异常: {exc}"))
            rc = -1
        finally:
            self._proc = None

        ok = (rc == 0) and not problems
        if problems and not self._stopping:
            for p in problems[:10]:
                self._q.put(("error", f"  ✘ {p}"))
        if self._stopping:
            summary = "已中止"
            level = "warn"
        else:
            summary = (f"J-Link 退出码 {rc}，"
                       + (f"{self._op_name}成功" if ok else f"{self._op_name}失败"))
            level = "ok" if ok else "error"
        self._q.put((level, summary))
        self._q.put(("@done", ok))

    def _finish_ui(self, ok: bool) -> None:
        self.btn_flash.configure(state="normal")
        self.btn_erase.configure(state="normal")
        self.btn_stop.configure(state="disabled")
        if self._stopping:
            self.var_status.set("已中止")
            self.log(f"{self._op_name}已中止", "warn")
            return
        self.var_status.set(f"{self._op_name}完成 ✔" if ok else f"{self._op_name}失败 ✘")
        self.log(f"{self._op_name}完成 ✔" if ok
                 else f"{self._op_name}失败 ✘（详见上方日志）",
                 "ok" if ok else "error")

    def _on_stop(self) -> None:
        self._stopping = True
        proc = self._proc
        if proc and proc.poll() is None:
            self.log("正在中止 J-Link…", "warn")
            try:
                proc.terminate()
            except Exception:  # noqa: BLE001
                pass

    # ---------------- 退出 ----------------
    def _on_close(self) -> None:
        self._on_stop()
        for timer in (self._save_timer, self._poll_timer, self._startup_timer):
            if timer is not None:
                try:
                    self.after_cancel(timer)
                except Exception:  # noqa: BLE001
                    pass
        self._save_timer = None
        self._poll_timer = None
        self._startup_timer = None
        save_config(self._collect_config())
        self.destroy()


def main() -> int:
    app = FlasherApp()
    app.mainloop()
    return 0


if __name__ == "__main__":
    sys.exit(main())
