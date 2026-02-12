# Docker setup for arm_api2 (ROS 2 Jazzy)

Build and run the arm_api2 workspace with Piper, MoveIt2, arm_api2_gui, ros2_dash_gui, aiortc, SAM, etc.

## Build

Specify the build target you want.

**Build with GUI:**
```bash
DOCKER_BUILDKIT=1 docker build --no-cache -t arm_api2_img:jazzy_izzy_twist_new_new --target add_gui --ssh default .
```

## Run (interactive shell)

```bash
./run_docker.sh
```

Then start an existing container:
```bash
docker start -i arm_api2_cont
```

Attach to a running container:
```bash
docker exec -it arm_api2_cont bash
```

Tmuxinator to start the workspace:
```bash
tmuxinator start robot_workspace
```

Stop all tmuxinator proceses:
```bash
tmux kill-server 
```

## Docker Compose (arm_api2 + Ollama)

Run arm_api2 container together with an Ollama LLM server. The compose file builds the image with `LLM_BACKEND=ollama` and starts tmuxinator inside the arm_api container.

**Before first run:** ensure SSH agent is available (e.g. `eval $(ssh-agent)` and `ssh-add`) and optionally:
```bash
ln -sf $SSH_AUTH_SOCK ~/.ssh/ssh_auth_sock
```

**Start services in the background:**
```bash
docker-compose -f compose.dash_ollama.yml up -d
```

Ollama listens on `http://localhost:11434`. The arm_api container runs with tmuxinator (robot_workspace). Open the Dash GUI in your browser at `http://localhost:8050` (or the URL your bridge/launch uses). On first run, Ollama may need to pull the model; refresh until the GUI is ready.

**Stop services:**
```bash
docker-compose -f compose.dash_ollama.yml stop
```
Do not use `down` if you want to keep Ollama data (models); `down` removes volumes and you would need to re-download models.


## File layout

- `Dockerfile` – multi-stage build (ros2_base, add_gui)
- `compose.dash_ollama.yml` – Compose stack: Ollama + arm_api2 (add_gui, Ollama LLM)
- `run_docker.sh` – run image interactively with host network, X11, SSH agent, GPU
- `robot_workspace.yml` – tmuxinator config (copied into image)
- `to_copy/` – optional files to copy into the image or at runtime
