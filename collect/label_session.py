#!/usr/bin/env python3
"""Send build_clips.py requests to the local Teacher pool; resumable, failures logged instead of aborting.

Runs on the host (standard library only):
  python3 collect/label_session.py --session-dir data/student/sessions/<id> [--url http://127.0.0.1:8766 ...]
Without --url every healthy Teacher on 8766/8768/8769/8770 is used in parallel (one worker per Teacher).
Reads  <session-dir>/requests.jsonl, appends to responses.jsonl, records failures in failures.jsonl.
Re-running skips (sample_id, mode) pairs already present in responses.jsonl.
"""
import argparse
import json
import queue
import sys
import threading
import time
import urllib.error
import urllib.request
from pathlib import Path

DEFAULT_PORTS = (8766, 8768, 8769, 8770)


def healthy(url):
    try:
        with urllib.request.urlopen(url.rstrip("/") + "/health", timeout=2) as r:
            return json.load(r).get("status") == "ready"
    except Exception:
        return False


def submit(url, payload, busy_retries=60):
    body = json.dumps(payload).encode()
    for _ in range(busy_retries):
        req = urllib.request.Request(url.rstrip("/") + "/v1/infer", body, {"Content-Type": "application/json"})
        try:
            with urllib.request.urlopen(req, timeout=1800) as r:
                return json.load(r), None
        except urllib.error.HTTPError as e:
            if e.code == 429:
                time.sleep(1)
                continue
            return None, f"HTTP {e.code}: {e.read().decode(errors='replace')[:500]}"
        except (urllib.error.URLError, TimeoutError, ConnectionError) as e:
            return None, f"connection: {e}"
    return None, "teacher stayed busy"


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--session-dir", required=True, type=Path)
    p.add_argument("--url", action="append", help="Teacher base URL (repeatable). Default: auto-discover 8766/8768/8769/8770")
    p.add_argument("--limit", type=int, default=0, help="stop after this many new responses (0 = all)")
    p.add_argument("--workers-per-teacher", type=int, default=int(__import__("os").environ.get("TEACHER_WORKERS", "1")), help="concurrent requests per Teacher (server allows TEACHER_CONCURRENCY, default 1; 2 measured no faster)")
    args = p.parse_args()
    urls = args.url or [f"http://127.0.0.1:{port}" for port in DEFAULT_PORTS if healthy(f"http://127.0.0.1:{port}")]
    if not urls:
        sys.exit("살아있는 Teacher 없음 (8766/8768/8769/8770): ./start-teacher.sh 또는 ./start-teacher-pool.sh")
    d = args.session_dir
    requests = [json.loads(l) for l in filter(str.strip, (d / "requests.jsonl").read_text().splitlines())]
    done = set()
    resp_path = d / "responses.jsonl"
    if resp_path.is_file():
        for l in filter(str.strip, resp_path.read_text().splitlines()):
            r = json.loads(l)
            done.add((r["sample_id"], r["mode"]))
    todo = [r for r in requests if (r["sample_id"], r["mode"]) not in done]
    if args.limit:
        todo = todo[:args.limit]
    print(f"teachers={len(urls)} x{max(1, args.workers_per_teacher)} workers {urls} requests={len(requests)} done={len(done)} todo={len(todo)}", flush=True)
    q = queue.Queue()
    for r in todo:
        q.put(r)
    lock = threading.Lock()
    stats = {"ok": 0, "fail": 0, "dead": 0}
    t0 = time.monotonic()
    out = resp_path.open("a")
    bad = (d / "failures.jsonl").open("a")

    def worker(url):
        while True:
            try:
                payload = q.get_nowait()
            except queue.Empty:
                return
            result, err = submit(url, payload)
            with lock:
                if err:
                    stats["fail"] += 1
                    bad.write(json.dumps({**payload, "error": err, "teacher": url, "at": time.time()}) + "\n")
                    bad.flush()
                    print(f"FAIL {payload['sample_id']} {payload['mode']} @{url}: {err[:120]}", flush=True)
                    if err.startswith("connection"):
                        q.put(payload)  # hand it to another teacher
                        stats["dead"] += 1
                        return
                else:
                    stats["ok"] += 1
                    result["teacher_url"] = url
                    out.write(json.dumps(result, ensure_ascii=False) + "\n")
                    out.flush()
                n = stats["ok"] + stats["fail"]
                if n % 10 == 0 or n == len(todo):
                    el = time.monotonic() - t0
                    print(f"[{n}/{len(todo)}] ok={stats['ok']} fail={stats['fail']} {el / max(n, 1):.2f}s/req eta={(len(todo) - n) * el / max(n, 1) / 60:.1f}min", flush=True)

    threads = [threading.Thread(target=worker, args=(u,), daemon=True) for u in urls for _ in range(max(1, args.workers_per_teacher))]
    for t in threads:
        t.start()
    for t in threads:
        t.join()
    out.close()
    bad.close()
    if not q.empty():
        print(f"경고: 남은 요청 {q.qsize()}개 (모든 Teacher 연결 실패). 다시 실행하면 이어서 처리", file=sys.stderr)
        sys.exit(1)
    print(json.dumps({**stats, "teachers": len(urls), "responses": str(resp_path)}))


if __name__ == "__main__":
    main()
