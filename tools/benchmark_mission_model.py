#!/usr/bin/env python3
"""Read-only Russian mission/JSON smoke benchmark against a llama.cpp server.

Synthetic fixtures test catalog selection, not competition task accuracy.
No ROS, CAN or navigation interfaces are used. Results belong outside Git.
"""
import argparse
import json
import time
import urllib.request
from pathlib import Path


CATALOG = [
    {"id": "house", "description": "Разрушенный жилой дом с обвалившейся стеной"},
    {"id": "tower", "description": "Круглая высокая башня"},
    {"id": "bridge_north", "description": "Северный мост над рекой"},
    {"id": "bridge_south", "description": "Южный мост над рекой рядом с башней"},
    {"id": "tanker", "description": "Жёлтая автоцистерна для топлива"},
]
CASES = [
    ("Ищи пострадавшего возле разрушенного дома.", "house", "selected"),
    ("Человек ждёт помощи рядом с круглой высоткой.", "tower", "selected"),
    ("Пострадавший у бензовоза.", "tanker", "selected"),
    ("Ищи у моста, который ближе к башне.", "bridge_south", "selected"),
    ("Осмотри северный мост, не южный.", "bridge_north", "selected"),
    ("Ищи у моста.", None, "ambiguous"),
    ("Пострадавший у железнодорожного вокзала.", None, "unknown"),
    ("Найди человека у дома с обвалившейся стеной, а не у башни.", "house", "selected"),
]
SCHEMA = {
    "type": "object", "additionalProperties": False,
    "properties": {
        "object_id": {"enum": [item["id"] for item in CATALOG] + [None]},
        "status": {"enum": ["selected", "ambiguous", "unknown"]},
    },
    "required": ["object_id", "status"],
}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--url", default="http://127.0.0.1:8088")
    parser.add_argument("--model", default="robot-model")
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--timeout", type=float, default=180)
    parser.add_argument("--catalog", type=Path, help="JSON array of catalog objects")
    parser.add_argument("--cases", type=Path, help="JSON array of [mission, expected_id, status]")
    args = parser.parse_args()
    if bool(args.catalog) != bool(args.cases):
        parser.error("--catalog and --cases must be provided together")
    catalog = json.loads(args.catalog.read_text()) if args.catalog else CATALOG
    cases = json.loads(args.cases.read_text()) if args.cases else CASES
    schema = json.loads(json.dumps(SCHEMA))
    schema["properties"]["object_id"]["enum"] = [item["id"] for item in catalog] + [None]
    rows = []
    for mission, expected_id, expected_status in cases:
        payload = {
            "model": args.model,
            "messages": [
                {"role": "system", "content":
                 "Выбери объект только из каталога. Верни JSON с object_id и status. "
                 "Если однозначного объекта нет: object_id=null, status=ambiguous "
                 "при нескольких подходящих или unknown при отсутствии. "
                 "Не придумывай факты или координаты. Каталог: " +
                 json.dumps(catalog, ensure_ascii=False)},
                {"role": "user", "content": mission},
            ],
            "chat_template_kwargs": {"enable_thinking": False},
            "temperature": 0, "max_tokens": 96, "cache_prompt": True,
            "response_format": {"type": "json_schema", "json_schema": {
                "name": "mission", "strict": True, "schema": schema}},
        }
        request = urllib.request.Request(
            args.url.rstrip("/") + "/v1/chat/completions",
            data=json.dumps(payload).encode(),
            headers={"Content-Type": "application/json"})
        start = time.monotonic()
        try:
            with urllib.request.urlopen(request, timeout=args.timeout) as response:
                result = json.load(response)
            content = result["choices"][0]["message"]["content"]
            parsed = json.loads(content)
            correct = (parsed == {"object_id": expected_id, "status": expected_status})
            row = {"mission": mission, "correct": correct, "answer": parsed,
                   "usage": result.get("usage"), "timings": result.get("timings")}
        except Exception as exc:
            row = {"mission": mission, "correct": False, "error": str(exc)}
        row["wall_seconds"] = round(time.monotonic() - start, 3)
        rows.append(row)
        print(json.dumps(row, ensure_ascii=False), flush=True)
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps({"model": args.model,
            "catalog_source": str(args.catalog) if args.catalog else "synthetic_fixture",
            "cases_source": str(args.cases) if args.cases else "synthetic_fixture",
            "passed": sum(r["correct"] for r in rows), "cases": rows},
            ensure_ascii=False, indent=2) + "\n")


if __name__ == "__main__":
    main()
