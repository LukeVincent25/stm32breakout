#!/usr/bin/env python3
"""Generate STM32F103C8T6 USB-C breakout schematic + PCB (KiCad 10)."""
from __future__ import annotations

import json
import math
import re
import shutil
import subprocess
import uuid
from collections import defaultdict
from pathlib import Path

KICAD = Path(r"E:\Kicad")
SYM = KICAD / "share" / "kicad" / "symbols"
FP = KICAD / "share" / "kicad" / "footprints"
CLI = KICAD / "bin" / "kicad-cli.exe"
ROOT = Path(__file__).resolve().parents[1]
PROJ = "stm32breakout"
SHEET_UUID = "7c9e6679-7425-40de-944b-e07fc1f90ae7"

# PCB size (mm)
BOARD_W = 60.0
BOARD_H = 80.0


def uid() -> str:
    return str(uuid.uuid4())


def extract_symbol(lib_path: Path, name: str) -> str:
    text = lib_path.read_text(encoding="utf-8")
    needle = f'(symbol "{name}"'
    start = None
    for m in re.finditer(re.escape(needle), text):
        after = text[m.end() : m.end() + 1]
        if after in ("", "\n", "\r", "\t", " "):
            start = m.start()
            break
    if start is None:
        raise KeyError(f"{name} not in {lib_path.name}")
    depth = 0
    i = start
    while i < len(text):
        ch = text[i]
        if ch == "(":
            depth += 1
        elif ch == ")":
            depth -= 1
            if depth == 0:
                return text[start : i + 1]
        i += 1
    raise RuntimeError(f"unbalanced symbol {name}")


def resolve_symbol(lib_path: Path, name: str) -> str:
    body = extract_symbol(lib_path, name)
    ext = re.search(r'\(extends "([^"]+)"\)', body)
    if not ext:
        return body
    parent_name = ext.group(1)
    parent = resolve_symbol(lib_path, parent_name)
    child_props = re.findall(
        r'(\(property "[^"]+"[\s\S]*?\n\t\t\))', body
    )
    out = parent
    out = out.replace(f'(symbol "{parent_name}"', f'(symbol "{name}"', 1)
    out = out.replace(f'(symbol "{parent_name}_', f'(symbol "{name}_')
    out = re.sub(r'\(extends "[^"]+"\)\n', "", out)
    for prop in child_props:
        pname = re.search(r'\(property "([^"]+)"', prop).group(1)
        out = re.sub(
            rf'\(property "{re.escape(pname)}"[\s\S]*?\n\t\t\)',
            prop,
            out,
            count=1,
        )
    return out


def embed_symbol(lib: str, name: str, lib_file: Path) -> str:
    body = resolve_symbol(lib_file, name)
    lib_id = f"{lib}:{name}"
    # Only the top-level lib_symbols entry uses "Lib:Name".
    # Nested unit drawings keep the short "Name_0_1" / "Name_1_1" identifiers.
    body = body.replace(f'(symbol "{name}"', f'(symbol "{lib_id}"', 1)
    lines = body.splitlines()
    return "\n".join("\t" + ln if ln else ln for ln in lines)


def parse_pins(symbol_body: str) -> list[dict]:
    pins = []
    for m in re.finditer(
        r'\(pin\s+(\S+)\s+(\S+)\s+'
        r'\(at\s+([-\d.]+)\s+([-\d.]+)\s+(\d+)\)'
        r'([\s\S]*?)(?=\n\t\t\t\(pin |\n\t\t\t\(alternate |\n\t\t\))',
        symbol_body,
    ):
        tail = m.group(6)
        num = re.search(r'\(number "([^"]+)"', tail)
        nam = re.search(r'\(name "([^"]+)"', tail)
        hidden = "(hide yes)" in tail[:80] or "(hide yes)" in m.group(0)[:200]
        if not num:
            continue
        pins.append(
            {
                "type": m.group(1),
                "x": float(m.group(3)),
                "y": float(m.group(4)),
                "rot": int(m.group(5)),
                "number": num.group(1),
                "name": nam.group(1) if nam else "",
                "hidden": "(hide yes)" in tail.split("(number")[0],
            }
        )
    # fallback simpler parse if nothing found
    if not pins:
        for m in re.finditer(
            r'\(pin\s+(\S+)\s+\S+\s*\n\s*\(at\s+([-\d.]+)\s+([-\d.]+)\s+(\d+)\)'
            r'([\s\S]*?)\(number "([^"]+)"',
            symbol_body,
        ):
            pins.append(
                {
                    "type": m.group(1),
                    "x": float(m.group(2)),
                    "y": float(m.group(3)),
                    "rot": int(m.group(4)),
                    "number": m.group(6),
                    "name": "",
                    "hidden": "(hide yes)" in m.group(5),
                }
            )
    # unique by pin number, first occurrence
    seen = set()
    uniq = []
    for p in pins:
        if p["number"] in seen:
            continue
        seen.add(p["number"])
        uniq.append(p)
    return uniq


def snap(v: float, grid: float = 1.27) -> float:
    return round(v / grid) * grid


def rot_xy(x: float, y: float, rot: int) -> tuple[float, float]:
    """Symbol coords are Y-up; schematic sheet coords are Y-down."""
    sx, sy = x, -y
    rot %= 360
    if rot == 0:
        return sx, sy
    if rot == 90:
        return sy, -sx
    if rot == 180:
        return -sx, -sy
    if rot == 270:
        return -sy, sx
    ang = math.radians(rot)
    return sx * math.cos(ang) - sy * math.sin(ang), sx * math.sin(ang) + sy * math.cos(ang)


def pin_world(inst_at, inst_rot, pin) -> tuple[float, float, int]:
    x, y = rot_xy(pin["x"], pin["y"], inst_rot)
    return inst_at[0] + x, inst_at[1] + y, 0


# ---------------------------------------------------------------------------
# Design data
# ---------------------------------------------------------------------------

LIBS = {
    "MCU_ST_STM32F1": SYM / "MCU_ST_STM32F1.kicad_sym",
    "Regulator_Linear": SYM / "Regulator_Linear.kicad_sym",
    "Connector": SYM / "Connector.kicad_sym",
    "Connector_Generic": SYM / "Connector_Generic.kicad_sym",
    "Device": SYM / "Device.kicad_sym",
    "Switch": SYM / "Switch.kicad_sym",
    "power": SYM / "power.kicad_sym",
    "Mechanical": SYM / "Mechanical.kicad_sym",
}

# Schematic coords (mm). Y down.
# PCB coords (mm). Y up. Origin bottom-left of board.

COMPONENTS = [
    # MCU
    dict(ref="U1", lib="MCU_ST_STM32F1", name="STM32F103C8Tx", value="STM32F103C8Tx",
         fp="Package_QFP:LQFP-48_7x7mm_P0.5mm", sch=(165.1, 114.3, 0), pcb=(30.0, 40.0, 0),
         pins={
             "1": "+3V3", "2": "PC13", "3": "PC14", "4": "PC15", "5": "PD0", "6": "PD1",
             "7": "NRST", "8": "GND", "9": "+3V3A", "10": "PA0", "11": "PA1", "12": "PA2",
             "13": "PA3", "14": "PA4", "15": "PA5", "16": "PA6", "17": "PA7", "18": "PB0",
             "19": "PB1", "20": "PB2", "21": "PB10", "22": "PB11", "23": "GND", "24": "+3V3",
             "25": "PB12", "26": "PB13", "27": "PB14", "28": "PB15", "29": "PA8", "30": "PA9",
             "31": "PA10", "32": "PA11", "33": "PA12", "34": "PA13", "35": "GND", "36": "+3V3",
             "37": "PA14", "38": "PA15", "39": "PB3", "40": "PB4", "41": "PB5", "42": "PB6",
             "43": "PB7", "44": "BOOT0", "45": "PB8", "46": "PB9", "47": "GND", "48": "+3V3",
         }),
    dict(ref="U2", lib="Regulator_Linear", name="AP2112K-3.3", value="AP2112K-3.3",
         fp="Package_TO_SOT_SMD:SOT-23-5", sch=(63.5, 43.18, 0), pcb=(16.0, 66.0, 90),
         pins={"1": "+5V", "2": "GND", "3": "+5V", "4": None, "5": "+3V3"}),
    dict(ref="J1", lib="Connector", name="USB_C_Receptacle_USB2.0_16P", value="USB_C_Receptacle_USB2.0_16P",
         fp="Connector_USB:USB_C_Receptacle_HRO_TYPE-C-31-M-12", sch=(40.64, 43.18, 0), pcb=(30.0, 77.2, 0),
         pins={
             "A1": "GND", "A4": "+5V", "A5": "CC1", "A6": "USB_DP", "A7": "USB_DN", "A8": None,
             "A9": "+5V", "A12": "GND", "B1": "GND", "B4": "+5V", "B5": "CC2", "B6": "USB_DP",
             "B7": "USB_DN", "B8": None, "B9": "+5V", "B12": "GND", "SH": "GND",
         }),
    dict(ref="J2", lib="Connector_Generic", name="Conn_01x20", value="GPIO_LEFT",
         fp="Connector_PinHeader_2.54mm:PinHeader_1x20_P2.54mm_Vertical",
         sch=(266.70, 50.8, 0), pcb=(8.0, 58.0, 0),
         pins={
             "1": "+5V", "2": "+3V3", "3": "GND", "4": "NRST", "5": "PA0", "6": "PA1",
             "7": "PA2", "8": "PA3", "9": "PA4", "10": "PA5", "11": "PA6", "12": "PA7",
             "13": "PB0", "14": "PB1", "15": "PB2", "16": "PB10", "17": "PB11", "18": "PC13",
             "19": "PC14", "20": "PC15",
         }),
    dict(ref="J3", lib="Connector_Generic", name="Conn_01x20", value="GPIO_RIGHT",
         fp="Connector_PinHeader_2.54mm:PinHeader_1x20_P2.54mm_Vertical",
         sch=(320.04, 50.8, 0), pcb=(52.0, 58.0, 0),
         pins={
             "1": "PA8", "2": "PA9", "3": "PA10", "4": "PA11", "5": "PA12", "6": "PA15",
             "7": "PB3", "8": "PB4", "9": "PB5", "10": "PB6", "11": "PB7", "12": "PB8",
             "13": "PB9", "14": "PB12", "15": "PB13", "16": "PB14", "17": "PB15", "18": "BOOT0",
             "19": "+3V3", "20": "GND",
         }),
    dict(ref="J4", lib="Connector_Generic", name="Conn_01x05", value="SWD",
         fp="Connector_PinHeader_2.54mm:PinHeader_1x05_P2.54mm_Vertical",
         sch=(63.5, 165.1, 0), pcb=(24.92, 6.0, 90),
         pins={"1": "+3V3", "2": "PA13", "3": "PA14", "4": "NRST", "5": "GND"}),
    dict(ref="Y1", lib="Device", name="Crystal_GND24", value="8MHz",
         fp="Crystal:Crystal_SMD_3225-4Pin_3.2x2.5mm", sch=(114.3, 63.5, 0), pcb=(18.5, 40.0, 90),
         pins={"1": "PD0", "2": "GND", "3": "PD1", "4": "GND"}),
    dict(ref="Y2", lib="Device", name="Crystal", value="32.768kHz",
         fp="Crystal:Crystal_SMD_3215-2Pin_3.2x1.5mm", sch=(114.3, 88.9, 0), pcb=(18.5, 48.0, 90),
         pins={"1": "PC14", "2": "PC15"}),
    dict(ref="FB1", lib="Device", name="FerriteBead", value="600R@100MHz",
         fp="Inductor_SMD:L_0603_1608Metric", sch=(165.1, 48.26, 90), pcb=(30.0, 48.5, 0),
         pins={"1": "+3V3", "2": "+3V3A"}),
    dict(ref="C1", lib="Device", name="C", value="10uF",
         fp="Capacitor_SMD:C_0805_2012Metric", sch=(50.8, 71.12, 0), pcb=(22.0, 72.0, 0),
         pins={"1": "+5V", "2": "GND"}),
    dict(ref="C2", lib="Device", name="C", value="1uF",
         fp="Capacitor_SMD:C_0603_1608Metric", sch=(63.5, 71.12, 0), pcb=(12.0, 72.0, 0),
         pins={"1": "+5V", "2": "GND"}),
    dict(ref="C3", lib="Device", name="C", value="1uF",
         fp="Capacitor_SMD:C_0603_1608Metric", sch=(81.28, 71.12, 0), pcb=(20.0, 64.0, 0),
         pins={"1": "+3V3", "2": "GND"}),
    dict(ref="C4", lib="Device", name="C", value="10uF",
         fp="Capacitor_SMD:C_0805_2012Metric", sch=(93.98, 71.12, 0), pcb=(24.0, 64.0, 0),
         pins={"1": "+3V3", "2": "GND"}),
    dict(ref="C5", lib="Device", name="C", value="100nF",
         fp="Capacitor_SMD:C_0603_1608Metric", sch=(139.7, 165.1, 0), pcb=(30.0, 33.5, 0),
         pins={"1": "+3V3", "2": "GND"}),
    dict(ref="C6", lib="Device", name="C", value="100nF",
         fp="Capacitor_SMD:C_0603_1608Metric", sch=(149.86, 165.1, 0), pcb=(36.5, 40.0, 90),
         pins={"1": "+3V3", "2": "GND"}),
    dict(ref="C7", lib="Device", name="C", value="100nF",
         fp="Capacitor_SMD:C_0603_1608Metric", sch=(160.02, 165.1, 0), pcb=(30.0, 46.5, 0),
         pins={"1": "+3V3", "2": "GND"}),
    dict(ref="C8", lib="Device", name="C", value="100nF",
         fp="Capacitor_SMD:C_0603_1608Metric", sch=(170.18, 165.1, 0), pcb=(33.5, 48.5, 0),
         pins={"1": "+3V3A", "2": "GND"}),
    dict(ref="C9", lib="Device", name="C", value="1uF",
         fp="Capacitor_SMD:C_0603_1608Metric", sch=(180.34, 165.1, 0), pcb=(26.5, 48.5, 0),
         pins={"1": "+3V3A", "2": "GND"}),
    dict(ref="C10", lib="Device", name="C", value="100nF",
         fp="Capacitor_SMD:C_0603_1608Metric", sch=(190.5, 165.1, 0), pcb=(26.5, 46.5, 0),
         pins={"1": "+3V3", "2": "GND"}),
    dict(ref="C11", lib="Device", name="C", value="20pF",
         fp="Capacitor_SMD:C_0603_1608Metric", sch=(114.3, 76.2, 0), pcb=(18.5, 36.5, 0),
         pins={"1": "PD0", "2": "GND"}),
    dict(ref="C12", lib="Device", name="C", value="20pF",
         fp="Capacitor_SMD:C_0603_1608Metric", sch=(127.0, 76.2, 0), pcb=(18.5, 43.5, 0),
         pins={"1": "PD1", "2": "GND"}),
    dict(ref="C13", lib="Device", name="C", value="12pF",
         fp="Capacitor_SMD:C_0603_1608Metric", sch=(114.3, 101.6, 0), pcb=(18.5, 51.5, 0),
         pins={"1": "PC14", "2": "GND"}),
    dict(ref="C14", lib="Device", name="C", value="12pF",
         fp="Capacitor_SMD:C_0603_1608Metric", sch=(127.0, 101.6, 0), pcb=(18.5, 53.5, 0),
         pins={"1": "PC15", "2": "GND"}),
    dict(ref="C15", lib="Device", name="C", value="100nF",
         fp="Capacitor_SMD:C_0603_1608Metric", sch=(88.9, 114.3, 0), pcb=(40.0, 52.0, 0),
         pins={"1": "NRST", "2": "GND"}),
    dict(ref="R1", lib="Device", name="R", value="5.1k",
         fp="Resistor_SMD:R_0603_1608Metric", sch=(63.5, 88.9, 0), pcb=(22.0, 75.5, 90),
         pins={"1": "CC1", "2": "GND"}),
    dict(ref="R2", lib="Device", name="R", value="5.1k",
         fp="Resistor_SMD:R_0603_1608Metric", sch=(73.66, 88.9, 0), pcb=(38.0, 75.5, 90),
         pins={"1": "CC2", "2": "GND"}),
    dict(ref="R3", lib="Device", name="R", value="22R",
         fp="Resistor_SMD:R_0603_1608Metric", sch=(63.5, 101.6, 0), pcb=(27.0, 72.0, 90),
         pins={"1": "PA11", "2": "USB_DN"}),
    dict(ref="R4", lib="Device", name="R", value="22R",
         fp="Resistor_SMD:R_0603_1608Metric", sch=(73.66, 101.6, 0), pcb=(33.0, 72.0, 90),
         pins={"1": "PA12", "2": "USB_DP"}),
    dict(ref="R5", lib="Device", name="R", value="10k",
         fp="Resistor_SMD:R_0603_1608Metric", sch=(88.9, 101.6, 0), pcb=(40.0, 55.0, 0),
         pins={"1": "+3V3", "2": "NRST"}),
    dict(ref="R6", lib="Device", name="R", value="10k",
         fp="Resistor_SMD:R_0603_1608Metric", sch=(101.6, 101.6, 0), pcb=(44.0, 55.0, 0),
         pins={"1": "BOOT0", "2": "GND"}),
    dict(ref="R7", lib="Device", name="R", value="10k",
         fp="Resistor_SMD:R_0603_1608Metric", sch=(101.6, 114.3, 0), pcb=(22.0, 32.0, 0),
         pins={"1": "PB2", "2": "GND"}),
    dict(ref="R8", lib="Device", name="R", value="1k",
         fp="Resistor_SMD:R_0603_1608Metric", sch=(88.9, 139.7, 0), pcb=(42.0, 68.0, 0),
         pins={"1": "+3V3", "2": "LED_PWR"}),
    dict(ref="R9", lib="Device", name="R", value="1k",
         fp="Resistor_SMD:R_0603_1608Metric", sch=(101.6, 139.7, 0), pcb=(42.0, 64.0, 0),
         pins={"1": "PC13", "2": "LED_USER"}),
    dict(ref="R10", lib="Device", name="R", value="1M",
         fp="Resistor_SMD:R_0603_1608Metric", sch=(139.7, 63.5, 0), pcb=(15.0, 40.0, 90),
         pins={"1": "PD0", "2": "PD1"}),
    dict(ref="D1", lib="Device", name="LED", value="GREEN",
         fp="LED_SMD:LED_0603_1608Metric", sch=(88.9, 152.4, 0), pcb=(46.0, 68.0, 0),
         pins={"1": "GND", "2": "LED_PWR"}),
    dict(ref="D2", lib="Device", name="LED", value="BLUE",
         fp="LED_SMD:LED_0603_1608Metric", sch=(101.6, 152.4, 0), pcb=(46.0, 64.0, 0),
         pins={"1": "LED_USER", "2": "+3V3"}),
    dict(ref="SW1", lib="Switch", name="SW_Push", value="NRST",
         fp="Button_Switch_SMD:SW_SPST_TL3342", sch=(50.8, 139.7, 0), pcb=(42.0, 66.0, 0),
         pins={"1": "NRST", "2": "GND"}),
    dict(ref="SW2", lib="Switch", name="SW_Push", value="BOOT0",
         fp="Button_Switch_SMD:SW_SPST_TL3342", sch=(63.5, 139.7, 0), pcb=(48.0, 66.0, 0),
         pins={"1": "BOOT0", "2": "+3V3"}),
]

for i, (x, y) in enumerate([(3.5, 3.5), (56.5, 3.5), (3.5, 76.5), (56.5, 76.5)], 1):
    COMPONENTS.append(dict(
        ref=f"H{i}", lib="Mechanical", name="MountingHole", value="M2.5",
        fp="MountingHole:MountingHole_2.7mm_M2.5",
        sch=(30.48 + i * 15.24, 241.3, 0), pcb=(x, y, 0), pins={},
    ))

# Keep symbols from overlapping: USB pins extend +15.24 mm, LDO -7.62 mm, MCU ±17.78/±40.64.
SCH_POS = {
    "J1": (40.64, 76.20, 0),
    "U2": (114.30, 50.80, 0),
    "C1": (63.50, 114.30, 0),
    "C2": (73.66, 114.30, 0),
    "C3": (129.54, 76.20, 0),
    "C4": (142.24, 76.20, 0),
    "R1": (63.50, 127.00, 0),
    "R2": (73.66, 127.00, 0),
    "R3": (88.90, 114.30, 0),
    "R4": (99.06, 114.30, 0),
    "Y1": (129.54, 101.60, 0),
    "Y2": (129.54, 127.00, 0),
    "C11": (142.24, 101.60, 0),
    "C12": (154.94, 101.60, 0),
    "C13": (142.24, 127.00, 0),
    "C14": (154.94, 127.00, 0),
    "R10": (154.94, 114.30, 90),
    "U1": (203.20, 127.00, 0),
    "FB1": (203.20, 63.50, 90),
    "C5": (177.80, 215.90, 0),
    "C6": (190.50, 215.90, 0),
    "C7": (203.20, 215.90, 0),
    "C8": (215.90, 215.90, 0),
    "C9": (228.60, 215.90, 0),
    "C10": (241.30, 215.90, 0),
    "C15": (88.90, 152.40, 0),
    "R5": (101.60, 152.40, 0),
    "R6": (114.30, 152.40, 0),
    "R7": (127.00, 152.40, 0),
    "R8": (88.90, 177.80, 0),
    "R9": (101.60, 177.80, 0),
    "D1": (88.90, 190.50, 0),
    "D2": (101.60, 190.50, 0),
    "SW1": (50.80, 152.40, 0),
    "SW2": (63.50, 152.40, 0),
    "J4": (50.80, 203.20, 0),
    "J2": (292.10, 101.60, 0),
    "J3": (355.60, 101.60, 0),
}
for c in COMPONENTS:
    if c["ref"] in SCH_POS:
        c["sch"] = SCH_POS[c["ref"]]

PCB_POS = {
    "U1": (30.0, 38.0, 0),
    "U2": (14.0, 63.0, 90),
    "J1": (30.0, 77.0, 0),
    "J2": (8.0, 54.0, 180),
    "J3": (52.0, 54.0, 180),
    "J4": (24.0, 12.0, 90),
    "Y1": (19.0, 38.0, 90),
    "Y2": (19.0, 45.5, 90),
    "FB1": (30.0, 46.5, 0),
    "C1": (20.0, 68.0, 0),
    "C2": (10.0, 68.0, 0),
    "C3": (18.0, 60.0, 0),
    "C4": (22.5, 60.0, 0),
    "C5": (30.0, 31.5, 0),
    "C6": (37.0, 38.0, 90),
    "C7": (30.0, 44.5, 0),
    "C8": (34.0, 46.5, 0),
    "C9": (26.0, 46.5, 0),
    "C10": (26.0, 44.5, 0),
    "C11": (19.0, 34.0, 0),
    "C12": (19.0, 41.5, 0),
    "C13": (19.0, 49.0, 0),
    "C14": (19.0, 51.5, 0),
    "C15": (38.0, 50.0, 0),
    "R1": (22.0, 71.5, 90),
    "R2": (38.0, 71.5, 90),
    "R3": (24.0, 68.0, 90),
    "R4": (36.0, 68.0, 90),
    "R5": (38.0, 53.0, 0),
    "R6": (42.5, 53.0, 0),
    "R7": (22.0, 30.0, 0),
    "R8": (38.0, 16.0, 0),
    "R9": (38.0, 19.0, 0),
    "R10": (15.5, 38.0, 90),
    "D1": (43.0, 16.0, 0),
    "D2": (43.0, 19.0, 0),
    "SW1": (18.0, 18.0, 0),
    "SW2": (18.0, 26.0, 0),
    "H1": (20.0, 4.0, 0),
    "H2": (40.0, 4.0, 0),
    "H3": (12.0, 76.0, 0),
    "H4": (48.0, 76.0, 0),
}
for c in COMPONENTS:
    if c["ref"] in PCB_POS:
        c["pcb"] = PCB_POS[c["ref"]]

POWER_SYMS = [
    ("#PWR01", "+5V", (38.10, 15.24)),
    ("#PWR02", "+3V3", (63.50, 15.24)),
    ("#PWR04", "GND", (88.90, 15.24)),
]

# USB VBUS is passive, so +5V needs a PWR_FLAG. LDO VOUT already drives +3V3.
# GND pins on the MCU/LDO are power_in, so GND also needs a flag.
PWR_FLAGS = [
    ("#FLG01", (38.10, 15.24), "+5V"),
    ("#FLG04", (88.90, 15.24), "GND"),
]
PWR_FLAGS_LABELED = [
    ("#FLG03", (114.30, 15.24), "+3V3A"),
]


def fmt_num(n: float) -> str:
    s = f"{n:.4f}".rstrip("0").rstrip(".")
    return s if s else "0"


def emit_property(name, value, x, y, hide=False, justify=None) -> str:
    hide_s = "\n\t\t\t\t\t(hide yes)" if hide else ""
    just = f"\n\t\t\t\t\t(justify {justify})" if justify else ""
    return f'''		(property "{name}" "{value}"
			(at {fmt_num(x)} {fmt_num(y)} 0)
			(effects
				(font
					(size 1.27 1.27)
				){just}{hide_s}
			)
		)'''


def emit_instance(comp, pin_info, project_name, sheet_uuid) -> str:
    x, y, rot = comp["sch"]
    lib_id = f"{comp['lib']}:{comp['name']}"
    props = [
        emit_property("Reference", comp["ref"], x, y - 2.54),
        emit_property("Value", comp["value"], x, y + 2.54),
        emit_property("Footprint", comp["fp"], x, y, hide=True),
        emit_property("Datasheet", "", x, y, hide=True),
        emit_property("Description", "", x, y, hide=True),
    ]
    pin_block = []
    for p in pin_info:
        pin_block.append(f'''		(pin "{p['number']}"
			(uuid "{uid()}")
		)''')
    in_bom = "no" if comp["ref"].startswith("H") else "yes"
    return f'''	(symbol
		(lib_id "{lib_id}")
		(at {fmt_num(x)} {fmt_num(y)} {rot})
		(unit 1)
		(exclude_from_sim no)
		(in_bom {in_bom})
		(on_board yes)
		(dnp no)
		(uuid "{uid()}")
{chr(10).join(props)}
{chr(10).join(pin_block)}
		(instances
			(project "{project_name}"
				(path "/{sheet_uuid}"
					(reference "{comp['ref']}")
					(unit 1)
				)
			)
		)
	)'''


def emit_power(ref, name, x, y, pin_info, project_name, sheet_uuid) -> str:
    lib_id = f"power:{name}"
    pin_block = "\n".join(
        f'''		(pin "{p['number']}"
			(uuid "{uid()}")
		)'''
        for p in pin_info
    )
    return f'''	(symbol
		(lib_id "{lib_id}")
		(at {fmt_num(x)} {fmt_num(y)} 0)
		(unit 1)
		(exclude_from_sim no)
		(in_bom no)
		(on_board yes)
		(dnp no)
		(uuid "{uid()}")
{emit_property("Reference", ref, x, y + 3.81, hide=True)}
{emit_property("Value", name, x, y - 3.556)}
{emit_property("Footprint", "", x, y, hide=True)}
{emit_property("Datasheet", "", x, y, hide=True)}
{emit_property("Description", "", x, y, hide=True)}
{pin_block}
		(instances
			(project "{project_name}"
				(path "/{sheet_uuid}"
					(reference "{ref}")
					(unit 1)
				)
			)
		)
	)'''


def emit_flag(ref, x, y, net, pin_info, project_name, sheet_uuid, labeled=False) -> str:
    pin_block = "\n".join(
        f'''		(pin "{p['number']}"
			(uuid "{uid()}")
		)'''
        for p in pin_info
    )
    extra = ""
    if labeled:
        extra = f'''
	(global_label "{net}"
		(shape passive)
		(at {fmt_num(x)} {fmt_num(y)} 0)
		(effects
			(font
				(size 1.27 1.27)
			)
			(justify left)
		)
		(uuid "{uid()}")
	)'''
    return f'''	(symbol
		(lib_id "power:PWR_FLAG")
		(at {fmt_num(x)} {fmt_num(y)} 0)
		(unit 1)
		(exclude_from_sim no)
		(in_bom no)
		(on_board yes)
		(dnp no)
		(uuid "{uid()}")
{emit_property("Reference", ref, x, y - 3.81, hide=True)}
{emit_property("Value", "PWR_FLAG", x, y - 5.08)}
{emit_property("Footprint", "", x, y, hide=True)}
{emit_property("Datasheet", "", x, y, hide=True)}
{emit_property("Description", "", x, y, hide=True)}
{pin_block}
		(instances
			(project "{project_name}"
				(path "/{sheet_uuid}"
					(reference "{ref}")
					(unit 1)
				)
			)
		)
	){extra}'''


def emit_label(net, x, y, rot, shape="bidirectional") -> str:
    # justify so text sits away from the symbol
    if rot % 360 == 0:
        just = "right"
    elif rot % 360 == 180:
        just = "left"
    else:
        just = "left"
    return f'''	(global_label "{net}"
		(shape {shape})
		(at {fmt_num(x)} {fmt_num(y)} {rot % 360})
		(effects
			(font
				(size 1.27 1.27)
			)
			(justify {just})
		)
		(uuid "{uid()}")
	)'''


def emit_nc(x, y) -> str:
    return f'''	(no_connect
		(at {fmt_num(x)} {fmt_num(y)})
		(uuid "{uid()}")
	)'''


def emit_text(txt, x, y, size=2.54) -> str:
    return f'''	(text "{txt}"
		(exclude_from_sim no)
		(at {fmt_num(x)} {fmt_num(y)} 0)
		(effects
			(font
				(size {size} {size})
			)
			(justify left bottom)
		)
		(uuid "{uid()}")
	)'''


def build_schematic(pin_cache: dict) -> str:
    used = {}
    lib_blocks = []
    for c in COMPONENTS:
        key = (c["lib"], c["name"])
        if key in used:
            continue
        used[key] = True
        lib_blocks.append(embed_symbol(c["lib"], c["name"], LIBS[c["lib"]]))
    for _ref, name, _xy in POWER_SYMS:
        key = ("power", name)
        if key in used:
            continue
        used[key] = True
        lib_blocks.append(embed_symbol("power", name, LIBS["power"]))
    lib_blocks.append(embed_symbol("power", "PWR_FLAG", LIBS["power"]))

    items = [
        emit_text("STM32F103C8T6 Breakout", 20, 12, 3.175),
        emit_text("USB-C 5V in, AP2112K-3.3, SWD, 2x20 GPIO", 20, 16.5, 1.27),
        emit_text("USB / 3V3", 30, 18, 1.8),
        emit_text("MCU", 150, 70, 1.8),
        emit_text("GPIO LEFT (J2)", 250, 28, 1.8),
        emit_text("GPIO RIGHT (J3)", 305, 28, 1.8),
    ]

    for c in COMPONENTS:
        key = f"{c['lib']}:{c['name']}"
        pins = pin_cache[key]
        items.append(emit_instance(c, pins, PROJ, SHEET_UUID))
        netmap = c.get("pins") or {}
        for p in pins:
            wx, wy, prot = pin_world(c["sch"][:2], c["sch"][2], p)
            net = netmap.get(p["number"], None)
            if p.get("hidden"):
                continue
            if p["number"] not in netmap:
                if p["type"] != "no_connect":
                    items.append(emit_nc(wx, wy))
                continue
            if net is None:
                items.append(emit_nc(wx, wy))
            else:
                items.append(emit_label(net, wx, wy, prot, "passive"))

    for ref, name, (x, y) in POWER_SYMS:
        items.append(emit_power(ref, name, x, y, pin_cache[f"power:{name}"], PROJ, SHEET_UUID))
        items.append(emit_label(name, x, y, 0, "passive"))

    flag_pins = pin_cache["power:PWR_FLAG"]
    for ref, (x, y), net in PWR_FLAGS:
        items.append(emit_flag(ref, x, y, net, flag_pins, PROJ, SHEET_UUID, labeled=False))
    for ref, (x, y), net in PWR_FLAGS_LABELED:
        items.append(emit_flag(ref, x, y, net, flag_pins, PROJ, SHEET_UUID, labeled=True))

    return f'''(kicad_sch
	(version 20260306)
	(generator "eeschema")
	(generator_version "10.0")
	(uuid "{SHEET_UUID}")
	(paper "A3")
	(title_block
		(title "STM32F103C8T6 Breakout")
		(date "2026-09-07")
		(rev "1")
		(company "")
		(comment 1 "USB-C PD sink (5.1k CC), 3.3V LDO, 8MHz HSE, 32.768kHz LSE")
		(comment 2 "SWD 1x5, dual 1x20 GPIO headers, BOOT0/NRST, user LED on PC13")
	)
	(lib_symbols
{chr(10).join(lib_blocks)}
	)
{chr(10).join(items)}
	(sheet_instances
		(path "/"
			(page "1")
		)
	)
	(embedded_fonts no)
)
'''


def write_project(path: Path) -> None:
    src = Path(r"E:\Kicad\share\kicad\demos\stickhub\StickHub.kicad_pro")
    data = json.loads(src.read_text(encoding="utf-8"))
    data["meta"]["filename"] = f"{PROJ}.kicad_pro"
    data["sheets"] = [[SHEET_UUID, "Root"]]
    data["text_variables"] = {
        "PROJECTNAME": "stm32breakout",
        "BOARD": "STM32F103C8T6 Breakout",
    }
    if "pcbnew" in data and "last_paths" in data["pcbnew"]:
        data["pcbnew"]["last_paths"]["step"] = ""
    data["net_settings"]["classes"] = [
        {
            "bus_width": 12,
            "clearance": 0.15,
            "diff_pair_gap": 0.15,
            "diff_pair_via_gap": 0.25,
            "diff_pair_width": 0.2,
            "line_style": 0,
            "microvia_diameter": 0.3,
            "microvia_drill": 0.1,
            "name": "Default",
            "pcb_color": "rgba(0, 0, 0, 0.000)",
            "priority": 2147483647,
            "schematic_color": "rgba(0, 0, 0, 0.000)",
            "track_width": 0.2,
            "tuning_profile": "",
            "via_diameter": 0.6,
            "via_drill": 0.3,
            "wire_width": 6,
        },
        {
            "bus_width": 12,
            "clearance": 0.2,
            "diff_pair_gap": 0.2,
            "diff_pair_via_gap": 0.25,
            "diff_pair_width": 0.2,
            "line_style": 0,
            "microvia_diameter": 0.3,
            "microvia_drill": 0.1,
            "name": "Power",
            "pcb_color": "rgba(0, 0, 0, 0.000)",
            "priority": 0,
            "schematic_color": "rgba(0, 0, 0, 0.000)",
            "track_width": 0.4,
            "tuning_profile": "",
            "via_diameter": 0.6,
            "via_drill": 0.3,
            "wire_width": 6,
        },
    ]
    data["net_settings"]["netclass_patterns"] = [
        {"netclass": "Power", "pattern": "+3V3*"},
        {"netclass": "Power", "pattern": "+5V"},
        {"netclass": "Power", "pattern": "GND"},
    ]
    if "board" in data and "design_settings" in data["board"]:
        sev = data["board"]["design_settings"].get("rule_severities") or {}
        # USB-C 16P stacked pads share copper; this is a library footprint property.
        sev["solder_mask_bridge"] = "warning"
        data["board"]["design_settings"]["rule_severities"] = sev
    # Keep ERC matrix from template; allow pin_not_driven as warning
    if "erc" in data and "rule_severities" in data["erc"]:
        data["erc"]["rule_severities"]["pin_not_driven"] = "warning"
        data["erc"]["rule_severities"]["missing_input_pin"] = "warning"
        data["erc"]["rule_severities"]["single_global_label"] = "ignore"
        data["erc"]["rule_severities"]["stacked_pin_name"] = "ignore"
    path.write_text(json.dumps(data, indent=2), encoding="utf-8")


def parse_netlist(path: Path) -> dict:
    text = path.read_text(encoding="utf-8")
    nets = defaultdict(list)
    chunks = re.split(r"\n\t\t\(net\n", text)
    for chunk in chunks[1:]:
        nm = re.search(r'\(name "([^"]+)"\)', chunk)
        if not nm:
            continue
        name = nm.group(1).replace("&quot;", '"')
        refs = re.findall(r'\(ref "([^"]+)"\)\s*\n\s*\(pin "([^"]+)"\)', chunk)
        nets[name].extend(refs)
    return nets


def fp_load(lib_fp: str):
    import pcbnew

    lib, name = lib_fp.split(":", 1)
    pretty = FP / f"{lib}.pretty"
    fp = pcbnew.FootprintLoad(str(pretty), name)
    if fp is None:
        raise RuntimeError(f"cannot load {lib_fp}")
    return fp


def mm(v: float):
    import pcbnew

    return pcbnew.FromMM(v)


def xy(x: float, y: float):
    import pcbnew

    return pcbnew.VECTOR2I(mm(x), mm(y))


def add_edge(board):
    import pcbnew

    pts = [(0, 0), (BOARD_W, 0), (BOARD_W, BOARD_H), (0, BOARD_H)]
    for i, (x0, y0) in enumerate(pts):
        x1, y1 = pts[(i + 1) % 4]
        s = pcbnew.PCB_SHAPE(board)
        s.SetShape(pcbnew.SHAPE_T_SEGMENT)
        s.SetLayer(pcbnew.Edge_Cuts)
        s.SetStart(xy(x0, y0))
        s.SetEnd(xy(x1, y1))
        s.SetWidth(mm(0.1))
        board.Add(s)


def add_silk_text(board, text, x, y, size=1.0, layer=None):
    import pcbnew

    t = pcbnew.PCB_TEXT(board)
    t.SetText(text)
    t.SetPosition(xy(x, y))
    t.SetLayer(layer or pcbnew.F_SilkS)
    t.SetTextSize(pcbnew.VECTOR2I(mm(size), mm(size)))
    t.SetTextThickness(mm(size * 0.15))
    board.Add(t)


def add_track(board, x0, y0, x1, y1, width, layer, net):
    import pcbnew

    tr = pcbnew.PCB_TRACK(board)
    tr.SetStart(xy(x0, y0))
    tr.SetEnd(xy(x1, y1))
    tr.SetWidth(mm(width))
    tr.SetLayer(layer)
    tr.SetNet(net)
    board.Add(tr)


def add_via(board, x, y, net):
    import pcbnew

    via = pcbnew.PCB_VIA(board)
    via.SetPosition(xy(x, y))
    via.SetViaType(pcbnew.VIATYPE_THROUGH)
    via.SetWidth(mm(0.6))
    via.SetDrill(mm(0.3))
    via.SetNet(net)
    board.Add(via)


def pad_xy(pad) -> tuple[float, float]:
    import pcbnew

    p = pad.GetPosition()
    return pcbnew.ToMM(p.x), pcbnew.ToMM(p.y)


def add_zone(board, net, layer, clearance=0.2):
    import pcbnew

    zone = pcbnew.ZONE(board)
    zone.SetLayer(layer)
    zone.SetNet(net)
    zone.SetLocalClearance(mm(clearance))
    zone.SetMinThickness(mm(0.2))
    zone.SetPadConnection(pcbnew.ZONE_CONNECTION_FULL)
    zone.SetThermalReliefGap(mm(0.2))
    zone.SetThermalReliefSpokeWidth(mm(0.25))
    zone.SetIsFilled(False)
    zone.SetAssignedPriority(0)
    outline = zone.Outline()
    outline.NewOutline()
    margin = 0.4
    for x, y in (
        (margin, margin),
        (BOARD_W - margin, margin),
        (BOARD_W - margin, BOARD_H - margin),
        (margin, BOARD_H - margin),
    ):
        outline.Append(mm(x), mm(y))
    board.Add(zone)
    return zone


def route_manhattan(board, a, b, width, layer, net, via_layer_switch=False):
    """L-shaped route between two (x,y) points."""
    import pcbnew

    x0, y0 = a
    x1, y1 = b
    if abs(x0 - x1) < 0.05:
        add_track(board, x0, y0, x1, y1, width, layer, net)
        return
    if abs(y0 - y1) < 0.05:
        add_track(board, x0, y0, x1, y1, width, layer, net)
        return
    # dogleg at the destination's x first (escape then run)
    add_track(board, x0, y0, x0, y1, width, layer, net)
    add_track(board, x0, y1, x1, y1, width, layer, net)


def build_pcb(netlist: dict) -> None:
    import pcbnew

    pcb_path = str(ROOT / f"{PROJ}.kicad_pcb")
    print("  create board", flush=True)
    board = pcbnew.CreateEmptyBoard()
    board.SetCopperLayerCount(2)
    ds = board.GetDesignSettings()
    ds.m_TrackMinWidth = mm(0.15)
    ds.m_ViasMinSize = mm(0.5)
    ds.m_ViasMinDrill = mm(0.3)
    ds.m_MinClearance = mm(0.15)
    ds.m_CopperEdgeClearance = mm(0.25)
    ds.m_HoleToHoleMin = mm(0.25)

    # nets
    net_items = {}
    for name in sorted(netlist.keys()):
        ni = pcbnew.NETINFO_ITEM(board, name)
        board.Add(ni)
        net_items[name] = ni
    # ensure power nets exist even if named oddly
    for extra in ("GND", "+3V3", "+5V", "+3V3A"):
        if extra not in net_items:
            ni = pcbnew.NETINFO_ITEM(board, extra)
            board.Add(ni)
            net_items[extra] = ni

    pin_to_net = {}
    for net, nodes in netlist.items():
        for ref, pin in nodes:
            pin_to_net[(ref, pin)] = net

    print("  placing footprints", flush=True)
    footprints = {}
    for c in COMPONENTS:
        fp = fp_load(c["fp"])
        fp.SetReference(c["ref"])
        fp.SetValue(c["value"])
        x, y, rot = c["pcb"]
        fp.SetPosition(xy(x, y))
        fp.SetOrientationDegrees(rot)
        for pad in fp.Pads():
            pname = pad.GetNumber()
            n = pin_to_net.get((c["ref"], pname))
            if n and n in net_items:
                pad.SetNet(net_items[n])
        board.Add(fp)
        footprints[c["ref"]] = fp

    print("  outline + silk", flush=True)
    add_edge(board)
    add_silk_text(board, "STM32F103C8T6", 30, 22, 1.2)
    add_silk_text(board, "BREAKOUT", 30, 20, 1.0)
    add_silk_text(board, "USB-C 5V", 30, 61.5, 0.8)
    add_silk_text(board, "SWD", 30, 10.5, 0.8)
    add_silk_text(board, "RST", 42, 74.8, 0.7)
    add_silk_text(board, "BOOT", 48, 74.8, 0.7)

    # Header pin names on silk
    j2_names = ["5V","3V3","GND","NRST","PA0","PA1","PA2","PA3","PA4","PA5",
                "PA6","PA7","PB0","PB1","PB2","PB10","PB11","PC13","PC14","PC15"]
    j3_names = ["PA8","PA9","PA10","PA11","PA12","PA15","PB3","PB4","PB5","PB6",
                "PB7","PB8","PB9","PB12","PB13","PB14","PB15","BOOT0","3V3","GND"]
    j2 = footprints["J2"]
    for pad in j2.Pads():
        if not pad.GetNumber().isdigit():
            continue
        i = int(pad.GetNumber()) - 1
        px, py = pad_xy(pad)
        add_silk_text(board, j2_names[i], px + 3.4, py, 0.6)
    j3 = footprints["J3"]
    for pad in j3.Pads():
        if not pad.GetNumber().isdigit():
            continue
        i = int(pad.GetNumber()) - 1
        px, py = pad_xy(pad)
        add_silk_text(board, j3_names[i], px - 3.4, py, 0.6)

    # Collect pads by net
    pads_by_net = defaultdict(list)
    for fp in footprints.values():
        for pad in fp.Pads():
            n = pad.GetNetname()
            if n:
                pads_by_net[n].append(pad)

    import pcbnew as pn

    gnd = net_items["GND"]
    print("  routing", flush=True)
    routed = 0

    def fp_of(pad):
        parent = pad.GetParent()
        # KiCad 10 pad parent may be footprint or padstack
        try:
            return pad.GetParentFootprint().GetReference()
        except Exception:
            return parent.GetReference() if hasattr(parent, "GetReference") else ""

    def short_route(pads, net, w, max_dist=6.0):
        nonlocal routed
        pts = [(pad, *pad_xy(pad)) for pad in pads]
        used = set()
        for i, (p1, x1, y1) in enumerate(pts):
            best = None
            for j, (p2, x2, y2) in enumerate(pts):
                if i == j or (min(i, j), max(i, j)) in used:
                    continue
                d = math.hypot(x2 - x1, y2 - y1)
                if d < 0.2 or d > max_dist:
                    continue
                if best is None or d < best[0]:
                    best = (d, j, x2, y2)
            if best:
                used.add((min(i, best[1]), max(i, best[1])))
                route_manhattan(board, (x1, y1), (best[2], best[3]), w, pn.F_Cu, net)
                routed += 1

    # Leave copper as ratsnest + GND planes. Geometric L-routes short 0.5 mm
    # LQFP pitch and overlapping passives; finish traces in Pcbnew.
    routed = 0

    add_zone(board, gnd, pn.F_Cu, clearance=0.2)
    add_zone(board, gnd, pn.B_Cu, clearance=0.2)

    # ZONE_FILLER.Fill() crashes in KiCad 10.0.3's Python bindings.
    # kicad-cli pcb drc --refill-zones --save-board fills them instead.
    pn.SaveBoard(pcb_path, board)
    print(f"PCB saved {pcb_path} ({routed} net segments chained)")


def main():
    import os

    sch_only = os.environ.get("SCH_ONLY") == "1"
    print("Extracting symbols / pins...")
    pin_cache = {}
    for c in COMPONENTS:
        key = f"{c['lib']}:{c['name']}"
        if key in pin_cache:
            continue
        body = resolve_symbol(LIBS[c["lib"]], c["name"])
        pin_cache[key] = parse_pins(body)
        print(f"  {key}: {len(pin_cache[key])} pins")
    for name in {n for _r, n, _xy in POWER_SYMS} | {"PWR_FLAG"}:
        key = f"power:{name}"
        if key not in pin_cache:
            body = resolve_symbol(LIBS["power"], name)
            pin_cache[key] = parse_pins(body)

    sch_path = ROOT / f"{PROJ}.kicad_sch"
    pro_path = ROOT / f"{PROJ}.kicad_pro"
    print("Writing schematic...")
    sch_path.write_text(build_schematic(pin_cache), encoding="utf-8")
    write_project(pro_path)

    print("Running schematic upgrade + ERC...")
    up = subprocess.run([str(CLI), "sch", "upgrade", str(sch_path)], capture_output=True, text=True)
    print("upgrade", up.returncode, up.stdout, up.stderr)
    erc_out = ROOT / "erc.rpt"
    er = subprocess.run(
        [str(CLI), "sch", "erc", "-o", str(erc_out), "--format", "report", "--severity-all", str(sch_path)],
        capture_output=True,
        text=True,
    )
    print("erc", er.returncode, er.stdout, er.stderr)
    if erc_out.exists():
        print(erc_out.read_text(encoding="utf-8", errors="replace")[-4000:])
    if er.returncode != 0 and not erc_out.exists():
        raise SystemExit("schematic failed to load")

    net_path = ROOT / f"{PROJ}.net"
    print("Exporting netlist...")
    r = subprocess.run(
        [str(CLI), "sch", "export", "netlist", "-o", str(net_path), str(sch_path)],
        check=False,
        capture_output=True,
        text=True,
    )
    print(r.stdout)
    print(r.stderr)
    if not net_path.exists():
        raise SystemExit("netlist export failed")
    nets = parse_netlist(net_path)
    print(f"Nets: {len(nets)}")

    if sch_only:
        print("SCH_ONLY=1: skipping PCB")
        return
    print("Building PCB...")
    build_pcb(nets)
    print("Done generating.")


if __name__ == "__main__":
    main()
