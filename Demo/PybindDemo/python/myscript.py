# myscript.py
def add(a, b):
    return a + b

def greet(name):
    return f"Hello, {name}!"

import math, time, threading

data = {"value": 0.0}

def worker():
    while True:
        data["value"] = math.sin(time.time())
        time.sleep(0.5)

# 启动线程
t = threading.Thread(target=worker, daemon=True)
t.start()

