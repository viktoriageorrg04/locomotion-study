# logging_utils.py
import csv, json, time, pathlib
from dataclasses import asdict

def ensure_run_dir(root="runs"):
    p = pathlib.Path(root) / time.strftime("%Y%m%d_%H%M%S")
    p.mkdir(parents=True, exist_ok=True)
    return p

def write_jsonl(path, record: dict):
    with open(path, "a") as f:
        f.write(json.dumps(record) + "\n")

def write_csv_header_if_needed(path, fieldnames):
    exists = pathlib.Path(path).exists()
    f = open(path, "a", newline="")
    w = csv.DictWriter(f, fieldnames=fieldnames)
    if not exists:
        w.writeheader()
    return f, w
