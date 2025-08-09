#!/usr/bin/env python3
"""
Convert clang-tidy textual output to a minimal SARIF v2.1.0 report.

Expected input lines (typical):
  path/to/file.cpp:12:34: warning: message... [check-name]

Notes:
- Only warnings and errors are emitted as SARIF results; notes are ignored.
- File paths are emitted as-is; GitHub interprets them relative to the repo root.
"""
import json
import os
import re
import sys
from typing import Dict, Any, List


LINE_RE = re.compile(
    r"^(?P<file>[^:\n]+):(?P<line>\d+):(?P<col>\d+):\s+(?P<severity>warning|error|note):\s+(?P<message>.*?)(?:\s*\[(?P<rule>[^\]]+)\])?\s*$"
)


def parse(lines: List[str]) -> Dict[str, Any]:
    results: List[Dict[str, Any]] = []
    rules: Dict[str, Dict[str, Any]] = {}

    for ln in lines:
        m = LINE_RE.match(ln)
        if not m:
            continue
        sev = m.group("severity")
        if sev == "note":
            # Skip notes to reduce noise
            continue
        rule = m.group("rule") or "clang-tidy"
        message = m.group("message").strip()
        path = m.group("file").strip()
        try:
            line = int(m.group("line"))
        except ValueError:
            line = 1
        level = "error" if sev == "error" else "warning"

        # Record rule once
        if rule not in rules:
            rules[rule] = {
                "id": rule,
                "name": rule,
                "shortDescription": {"text": rule},
                "fullDescription": {"text": rule},
                "defaultConfiguration": {"level": level},
            }

        results.append(
            {
                "ruleId": rule,
                "level": level,
                "message": {"text": message},
                "locations": [
                    {
                        "physicalLocation": {
                            "artifactLocation": {"uri": path},
                            "region": {"startLine": line},
                        }
                    }
                ],
            }
        )

    run = {
        "tool": {
            "driver": {
                "name": "clang-tidy",
                "informationUri": "https://clang.llvm.org/extra/clang-tidy/",
                "rules": list(rules.values()),
            }
        },
        "results": results,
    }
    sarif = {"$schema": "https://json.schemastore.org/sarif-2.1.0.json", "version": "2.1.0", "runs": [run]}
    return sarif


def main() -> int:
    if len(sys.argv) < 2:
        print("Usage: clang_tidy_to_sarif.py <clang-tidy.log> [out.sarif]", file=sys.stderr)
        return 2
    in_path = sys.argv[1]
    out_path = sys.argv[2] if len(sys.argv) > 2 else "clang-tidy.sarif"
    try:
        with open(in_path, "r", encoding="utf-8", errors="ignore") as f:
            lines = f.readlines()
    except FileNotFoundError:
        print(f"Input log not found: {in_path}", file=sys.stderr)
        return 1
    sarif = parse(lines)
    with open(out_path, "w", encoding="utf-8") as f:
        json.dump(sarif, f, indent=2)
    print(f"Wrote SARIF: {out_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

