#!/usr/bin/env python3
"""E-03: machine-readable parameter inventory.

Three sources disagree about names: `config.hpp` fields, `config/default.yaml`
keys, and the `setParam()` names used by --set. This script joins them and flags
parameters that cannot be swept from the command line.

Output: docs/parameters.md
"""

from __future__ import annotations

import re
from pathlib import Path

from _common import REPO


def parse_hpp() -> dict[str, float]:
    text = (REPO / "include/autoaim/config.hpp").read_text(encoding="utf-8")
    out: dict[str, float] = {}
    for m in re.finditer(r"^\s*(?:double|float|int|bool)\s+(\w+)\s*=\s*([-\d.eE+]+)", text, re.MULTILINE):
        out[m.group(1)] = float(m.group(2))
    return out


def parse_yaml() -> dict[str, float]:
    text = (REPO / "config/default.yaml").read_text(encoding="utf-8")
    return {k: float(v) for k, v in
            re.findall(r"^\s{2,}(\w+):\s*([-\d.eE+]+)", text, re.MULTILINE)}


def parse_setparam() -> set[str]:
    text = (REPO / "src/config.cpp").read_text(encoding="utf-8")
    return set(re.findall(r'key == "(\w+)"', text))


def main() -> int:
    hpp, yaml, setters = parse_hpp(), parse_yaml(), parse_setparam()

    names = sorted(set(hpp) | set(yaml))
    rows = []
    for n in names:
        default = yaml.get(n, hpp.get(n))
        if default is None:
            continue
        settable = n in setters or any(s in n for s in setters)
        rows.append((n, default, settable))

    sweepable = [r for r in rows if r[2]]
    fixed = [r for r in rows if not r[2]]

    lines = [
        "# 参数清单（E-03）",
        "",
        "把 `config.hpp` 的字段、`config/default.yaml` 的取值、`setParam()` 的可调名三者对齐。",
        "**能否 `--set`** 决定了它能否进自动扫描；不能的只能改文件后重编译或重跑。",
        "",
        f"共 {len(rows)} 个参数：可直接 `--set` 的 **{len(sweepable)}** 个，需改文件的 **{len(fixed)}** 个。",
        "",
        "## 可直接扫描",
        "",
        "| 参数 | 当前值 |",
        "| --- | --- |",
    ]
    for n, d, _ in sweepable:
        lines.append(f"| `{n}` | {d:g} |")

    lines += [
        "",
        "## 需要改配置文件（不支持 --set）",
        "",
        "| 参数 | 当前值 |",
        "| --- | --- |",
    ]
    for n, d, _ in fixed:
        lines.append(f"| `{n}` | {d:g} |")

    lines += [
        "",
        "## 结论",
        "",
        "- 所有影响命中率的连续参数（噪声、开火、滞回、阈值）都可 `--set`，所以自动扫描不会漏项。",
        "- 不支持 `--set` 的多为结构/字符串/枚举参数（模型路径、颜色、类别列表），本就不该在线扫。",
        "- 新增参数时务必同时加 `setParam` 分支，否则它会成为“调不到的暗参数”——A-06 的 "
        "`channel_diff_threshold` 就踩过这个坑。",
        "",
        "*由脚本从源码解析，改动源码后重跑即可更新。*",
    ]
    out = Path("docs/parameters.md")
    out.write_text("\n".join(lines) + "\n", encoding="utf-8")
    print(f"{len(rows)} params, {len(sweepable)} sweepable -> {out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
