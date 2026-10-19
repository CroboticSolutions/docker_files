#!/usr/bin/env python3
"""Skine modele za DemoMotion u DEMOMOTION_MODELS i provjeri SHA256.

    python3 -I scripts/download_models.py /opt/demomotion/models

Samo standardna biblioteka. Postojeća datoteka s točnim SHA256 se preskače,
pa se skripta može pokretati više puta (i nastaviti nakon prekida).
Hashevi su iz razvojnog kontejnera (2026-10-19): skinuto = ono s čim radimo.

MANO se ne skida odavde: treba MPI račun (mano.is.tue.mpg.de) i licenca
zabranjuje redistribuciju. Vidi README.md, "Modeli".
"""
import hashlib
import shutil
import sys
import urllib.request
from pathlib import Path

GDRIVE = "https://drive.usercontent.google.com/download?export=download&confirm=t&id="
MEGAPOSE = "https://www.paris.inria.fr/archive_ylabbeprojectsdata/megapose/megapose-models/"

# (putanja u DEMOMOTION_MODELS, URL, sha256)
MODELS = [
    ("sam/sam2.1_hiera_large.pt",
     "https://dl.fbaipublicfiles.com/segment_anything_2/092824/sam2.1_hiera_large.pt",
     "2647878d5dfa5098f2f8649825738a9345572bae2d4350a2468587ece47dd318"),
    ("wilor/detector.pt",
     "https://huggingface.co/spaces/rolpotamias/WiLoR/resolve/main/pretrained_models/detector.pt",
     "5ef3df44e42d2db52d4ffe91f83a22ce9925e2acc9abebf453f2c5d22e380033"),
    ("wilor/wilor_final.ckpt",
     "https://huggingface.co/spaces/rolpotamias/WiLoR/resolve/main/pretrained_models/wilor_final.ckpt",
     "3e97aafc7dd08d883a4cc5a027df61fdb6fda6136dbd1319405413862ada6bb2"),
    ("megapose/coarse-rgb-906902141/checkpoint.pth.tar",
     MEGAPOSE + "coarse-rgb-906902141/checkpoint.pth.tar",
     "30fa2ba97d1bb1b27f6443b6671f557baebf3dff93fcc8605a6c7924ead759a4"),
    ("megapose/coarse-rgb-906902141/config.yaml",
     MEGAPOSE + "coarse-rgb-906902141/config.yaml",
     "24de955519e9d65c82a237b29844576989504e956448ecae07b0e2f2b6675bd9"),
    ("megapose/refiner-rgb-653307694/checkpoint.pth.tar",
     MEGAPOSE + "refiner-rgb-653307694/checkpoint.pth.tar",
     "52dd473fc719488e2f45a98aac2f83960ebe1534b2845cb292c0a287ba70adb7"),
    ("megapose/refiner-rgb-653307694/config.yaml",
     MEGAPOSE + "refiner-rgb-653307694/config.yaml",
     "29e02244d9b674fd5e374c9902b567cfe5ef4e2e2e3fef01dd1d6188df7b7d84"),
    ("megapose/refiner-rgbd-288182519/checkpoint.pth.tar",
     MEGAPOSE + "refiner-rgbd-288182519/checkpoint.pth.tar",
     "6e8c833f8f0937492d024f81c533f4cb860570f0195b235c4b71356e78b79235"),
    ("megapose/refiner-rgbd-288182519/config.yaml",
     MEGAPOSE + "refiner-rgbd-288182519/config.yaml",
     "78731baa4059a8b3502ff7c7144dee6eab4989269b7c595c8db156b72e4af44e"),
    # NVIDIA Google Drive folder 1DFezOAD0oD1BblsXVxqDsl8fj0qzB82i (FoundationPose README)
    ("foundationpose/2023-10-28-18-33-37/config.yml",
     GDRIVE + "1477-st1s1TxXN6oqfM5ZnsQwd8BCzVg1",
     "28a6ba94a33230ee5fc3c51939486281578b0972542bd9e38ca6123e75605686"),
    ("foundationpose/2023-10-28-18-33-37/model_best.pth",
     GDRIVE + "1E9FPB5WFIBMLrOJqZLpoVOK4Mjzrrxhv",
     "774700586ddc435d408fc01c9809c43e151232936369dfbea0f0f964ba471d60"),
    ("foundationpose/2024-01-11-20-02-45/config.yml",
     GDRIVE + "1kQkQG-q_VvLRozv30hyeLB7P_jiEEqiE",
     "a79db4de3b95885dd5ae86833b37b8698a75dad81e87d1086cd50b2fcd8dda3f"),
    ("foundationpose/2024-01-11-20-02-45/model_best.pth",
     GDRIVE + "1Zdjnkn4EHOI5_k08apofwRgTjWpai4E4",
     "81924d384bf5c26c646ee4783104982ae3d1e049c181c36641b6a7aeae494c26"),
]


def sha256(path):
    h = hashlib.sha256()
    with open(path, "rb") as f:
        for chunk in iter(lambda: f.read(1 << 20), b""):
            h.update(chunk)
    return h.hexdigest()


def download(url, dest):
    tmp = dest.with_name(dest.name + ".part")
    req = urllib.request.Request(url, headers={"User-Agent": "demomotion-setup"})
    with urllib.request.urlopen(req, timeout=60) as resp, open(tmp, "wb") as out:
        total = int(resp.headers.get("Content-Length") or 0)
        done = 0
        while chunk := resp.read(1 << 20):
            out.write(chunk)
            done += len(chunk)
            if total:
                print(f"\r    {done >> 20}/{total >> 20} MB", end="", flush=True)
    print()
    return tmp


def main():
    if len(sys.argv) != 2:
        sys.exit(__doc__)
    root = Path(sys.argv[1])
    failed = []
    for rel, url, digest in MODELS:
        dest = root / rel
        if dest.is_file() and sha256(dest) == digest:
            print(f"  OK   {rel}")
            continue
        print(f"  ...  {rel}")
        dest.parent.mkdir(parents=True, exist_ok=True)
        try:
            tmp = download(url, dest)
        except OSError as exc:
            print(f"  NE   {rel}: {exc}")
            failed.append(rel)
            continue
        if sha256(tmp) != digest:
            tmp.unlink()
            print(f"  NE   {rel}: SHA256 se ne poklapa (izvor se promijenio?)")
            failed.append(rel)
            continue
        shutil.move(tmp, dest)
        print(f"  OK   {rel}")
    if failed:
        sys.exit(f"Nije skinuto: {', '.join(failed)}")


if __name__ == "__main__":
    main()
