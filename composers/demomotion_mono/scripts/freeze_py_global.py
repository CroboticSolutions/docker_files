#!/usr/bin/env python3
"""Ispiše efektivni skup pip paketa interpretera koji ga pokreće.

Pokreće se u razvojnom kontejneru, sa sourcanim ROS-om (kao u gui_stacku):

    /root/py_global/bin/python freeze_py_global.py > ../requirements/py_global.lock
    /root/arms_ws/src/megapose6d/venv/bin/python freeze_py_global.py --local > ../requirements/megapose.lock

Za svaki paket uzima prvu distribuciju na sys.path, tj. onu koju Python stvarno
učita. py_global vidi i /usr/local i ~/.local, a običan `pip freeze --local`
bi njih izostavio. Preskače apt/ROS pakete, pakete iz workspacea, torch/nvidia-*
(oni su u torch.txt) i pakete iz gita/lokalnih direktorija (ispisani na stderr,
idu ručno u py_global.git.txt).
"""
import importlib.metadata as md
import json
import re
import sys

SKIP = re.compile(r"^(torch|torchvision|torchaudio|triton|nvidia-.*|pip)$")
# Ostaci starijih eksperimenata (GraspNet/AnyGrasp, HPE) koje DemoMotion ne importa, a nemaju
# wheel za cp312: MinkowskiEngine je CUDA build koji ne prolazi s novim torchom, pointnet2 nije na PyPI.
EXCLUDE = {"minkowskiengine", "pointnet2", "graspnetapi", "grasp-nms", "xtcocotools"}
only_local = "--local" in sys.argv

seen = {}
for dist in md.distributions():
    name = dist.metadata.get("Name")
    if not name:
        continue
    key = re.sub(r"[-_.]+", "-", name).lower()
    seen.setdefault(key, dist)

for key, dist in sorted(seen.items()):
    path = str(dist._path)
    if only_local and sys.prefix not in path:
        continue
    if path.startswith(("/opt/ros", "/usr/lib/python3")) or SKIP.match(key) or key in EXCLUDE:
        continue
    if "/arms_ws/" in path and sys.prefix not in path:
        continue
    direct = dist.read_text("direct_url.json")
    if direct:
        info = json.loads(direct)
        if "dir_info" in info or "vcs_info" in info:
            commit = info.get("vcs_info", {}).get("commit_id", "")
            print(f"# git/lokalno: {dist.metadata['Name']} {info.get('url')} {commit}", file=sys.stderr)
            continue
    print(f"{dist.metadata['Name']}=={dist.version}")
