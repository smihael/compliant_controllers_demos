"""Check the authenticated API and a Lab asset, not just the process."""
import json
import os
import re
from urllib.request import Request, urlopen

base = "http://127.0.0.1:" + os.environ.get("JUPYTER_PORT", "8889")
headers = {"Authorization": "token " + os.environ.get("JUPYTER_TOKEN", "surface")}


def get(path):
    with urlopen(Request(base + path, headers=headers), timeout=2) as response:
        return response.read()


json.loads(get("/api/status"))
page = get("/lab").decode()
assets = re.findall(r'src="([^"]*static/lab/[^"]+\.js[^"]*)"', page)
if not assets:
    raise RuntimeError("Lab page contains no JavaScript assets")
get(assets[0])
