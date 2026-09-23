#!/usr/bin/env python3
import sys
import time
import json
import urllib.request

if len(sys.argv) < 2:
    print("Usage: python3 benchmark_pick.py '<natural language command>'")
    sys.exit(1)

cmd = sys.argv[1]
print(f"============================================================")
print(f"  BENCHMARK TEST: '{cmd}'")
print(f"============================================================")

req = urllib.request.Request(
    'http://localhost:8000/api/command',
    data=json.dumps({'command': cmd}).encode('utf-8'),
    headers={'Content-Type': 'application/json'}
)
try:
    res = urllib.request.urlopen(req)
    res_data = json.loads(res.read())
    print("Command submission response:", res_data)
except Exception as e:
    print("Error submitting command:", e)
    sys.exit(1)

seen_cot = set()
for t in range(50):
    time.sleep(1.5)
    try:
        s_res = urllib.request.urlopen('http://localhost:8000/api/state')
        state = json.loads(s_res.read())
    except Exception as e:
        print(f"[{t*1.5:.1f}s] Connection error: {e}")
        continue

    task = state.get('task', {})
    status = task.get('task_status')
    cot = state.get('cot', [])
    for entry in cot:
        text = entry.get('text', '')
        if text and text not in seen_cot:
            seen_cot.add(text)
            print(f"[{t*1.5:.1f}s] [COT] {text}")

    if status in ['COMPLETE', 'FAILED']:
        print(f"============================================================")
        print(f"  FINAL RESULT: {status}")
        print(f"============================================================")
        sys.exit(0 if status == 'COMPLETE' else 1)

print("TIMEOUT: Task did not complete within 75 seconds.")
sys.exit(2)
