# -*- coding: utf-8 -*-
"""检查电机相关引脚附近的核心板外设(冲突排查)"""
import re

path = r"g:\STM32\7_AutomatedSortingRobot_26\F407VET6主控板和拓展板信息\勇气核心板\原理图-勇气核心板_STM32F407V-Ver5.pdf"
data = open(path, "rb").read()
raw = data.decode("latin1")

texts = []
for m in re.finditer(r"([\d.\-]+)\s+([\d.\-]+)\s+Td(?:\s|[\x00-\x20])+\(([^()]*)\)\s*Tj", raw):
    x, y, t = float(m.group(1)), float(m.group(2)), m.group(3)
    texts.append((x, y, t.strip()))

# 我们的电机引脚
pins = ["PA3", "PA4", "PC4", "PC5", "PB0", "PB1", "PB2", "PE7", "PE8", "PE9", "PE11", "PE13", "PE14"]

for p in pins:
    hits = [(x, y, t) for (x, y, t) in texts if t == p]
    if not hits:
        print(f"--- {p}: (未在PDF中找到该引脚标签) ---")
        continue
    for (px, py, _) in hits:
        near = []
        for (x, y, t) in texts:
            if t in ("", p):
                continue
            if abs(x - px) < 25 and abs(y - py) < 25:
                near.append((x, y, t))
        if not near:
            continue
        near.sort(key=lambda z: (abs(z[0]-px), abs(z[1]-py)))
        print(f"--- {p} @({px:.0f},{py:.0f}) 附近 ---")
        for (x, y, t) in near[:8]:
            print(f"    ({x:.0f},{y:.0f}) {t}")


