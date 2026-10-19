# DemoMotion: cijeli stack u jednom kontejneru

Za računala s Piperom, Orbbec Femto Boltom i NVIDIA GPU-om (RTX 50xx radi, torch je cu128).
U kontejneru rade GUI (:8080), debug konzola (:8091), dash bridge (:9094) i WebRTC (:8181, :8182).
GUI sam pokreće driver, MoveIt, arm_api2, kameru, SAM2, WiLoR i MegaPose.

```bash
./setup_host.sh            # jednom po računalu: provjere, udev, .env, mape
./demomotion.sh build      # povuče vrhove grana iz repos.yaml i builda image
./demomotion.sh up         # http://localhost:8080
./demomotion.sh attach     # tmux s logovima servisa (Ctrl-b d za izlaz)
./demomotion.sh down
```

## Preduvjeti

- Ubuntu, Docker s compose v2 i buildx, NVIDIA driver i NVIDIA Container Toolkit
- SSH ključ s pristupom `CroboticSolutions` repoima, učitan u agent (`ssh-add -l`)
- Modeli (ispod), oko 3.6 GB
- Prvi build traje dugo (torch, pytorch3d za FoundationPose: 20-40 min po arhitekturi, depthai,
  Orbbec) i image je velik (~25 GB). Kasniji buildovi ponovno grade samo kod, i to samo ako se
  neka grana pomakla. pytorch3d i nvdiffrast ostaju u cacheu.

## Modeli

`DEMOMOTION_MODELS` u `.env`, montira se read-only na `/models`:

```
models/
├── sam/sam2.1_hiera_large.pt          857 MB
├── wilor/wilor_final.ckpt             2.4 GB
├── wilor/detector.pt                   52 MB
├── mano/MANO_RIGHT.pkl                  4 MB   MANO licenca: ne dijeliti javno
├── mano/MANO_LEFT.pkl
├── megapose/                          248 MB   coarse-rgb-*, refiner-rgb-*, refiner-rgbd-*
└── foundationpose/                    247 MB   2023-10-28-18-33-37/, 2024-01-11-20-02-45/
```

Iz postojećeg razvojnog kontejnera, na hostu (`<dev>` je ime tog kontejnera):

```bash
M=/opt/demomotion/models; mkdir -p $M/{sam,wilor,mano}
docker cp <dev>:/root/arm_perception/models/sam/sam2.1_hiera_large.pt $M/sam/
docker cp <dev>:/root/arms_ws/src/perception/WiLoR/pretrained_models/wilor_final.ckpt $M/wilor/
docker cp <dev>:/root/arms_ws/src/perception/WiLoR/pretrained_models/detector.pt $M/wilor/
docker cp <dev>:/root/arms_ws/src/perception/WiLoR/mano_data/mano/. $M/mano/
docker cp <dev>:/root/arms_ws/src/megapose6d/local_data/megapose-models/. $M/megapose/
docker cp <dev>:/root/foundationpose/FoundationPose/weights/. $M/foundationpose/
```

FoundationPose težine su iz NVIDIA Google Drive foldera (link u README-u
[NVlabs/FoundationPose](https://github.com/NVlabs/FoundationPose)).

## Kalibracija i podaci

`DEMOMOTION_DATA` u `.env`. Sve što mora preživjeti novi image je tu:

| Mapa | U kontejneru | Što |
|---|---|---|
| `ros_home/` | `/root/.ros` | `calibration/` (hand-eye, TCP, kamera) i ROS logovi |
| `bags/` | `/root/bags` | snimljeni bagovi |
| `diagnostics/` | `/root/arms_ws/diagnostics` | dijagnostika iz GUI-ja |
| `logs/` | `/var/log/arm_api2_gui` | launch logovi GUI-ja |
| `block_groups/`, `gui_config/`, `cache/`, `megapose_cache/` | | ostalo |

Kalibracija je vezana uz konkretan robot i kameru. Na novom računalu napravi novu kroz GUI.
Ako isti robot prelazi iz razvojnog kontejnera:

```bash
docker cp <dev>:/root/.ros/calibration/. /opt/demomotion/data/ros_home/calibration/
```

## Koja verzija koda

`repos.yaml` prati **grane**. Svaki `./demomotion.sh build` uzme njihov vrh, a točni commitovi
koji su ušli u image su u `/root/arms_ws/VERSIONS.yaml` (`./demomotion.sh versions`).
Upstream SAM2 je fiksiran na commit (`SAM2_COMMIT` u Dockerfileu).

Python paketi su zaključani u `requirements/` (izvučeni iz razvojnog kontejnera skriptom
`scripts/freeze_py_global.py`). Novi paket: dodati ga u razvojni kontejner, regenerirati lock
i commitati.

## Ograničenja prve verzije

- FoundationPose CUDA ekstenzije se kompajliraju za `FP_CUDA_ARCHS` iz `.env` (zadano RTX 30xx/40xx/50xx).
  GPU izvan tog popisa javi "no kernel image is available": dodaj njegovu arhitekturu i ponovno buildaj.
- Kalibracija welding guna koja se upisuje u `piper_description` (xacro u `src`) nestaje s
  novim imageom. Treba je prebaciti u `~/.ros/calibration` ili u git.
- `privileged: true` i finalni image = build stage. Sužavanje ovlasti i smanjivanje imagea dolaze kasnije.
- Kontejner koristi CycloneDDS samo na loopbacku: ROS topici nisu vidljivi s drugih računala
  (GUI u browseru jest).
