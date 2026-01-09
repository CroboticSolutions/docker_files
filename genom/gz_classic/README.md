# GenoM - Gazebo 

Initial docker to set up GenoM components for the easier development. 

Run build command as: 
```bash
# Build base stage only (ROS/Genom/Gazebo without GUI)
docker build --target base --ssh default -t genom_img:gz_classic .

# Build REACT_UI stage (includes React GUI, WebRTC, Node.js)
docker build --target REACT_UI --ssh default -t genom_img:gz_classic_react .
```

### Multistage Build

This Dockerfile uses Docker multistage builds with two stages:

- **`base`**: Core ROS/GenoM/Gazebo Classic setup with all development tools
- **`react_ui`**: Extends base stage with:
  - Node.js 18 (via nvm)
  - minithex_react_gui (cloned from GitHub)
  - webrtc_test (cloned from GitHub)
  - open3d and websockets Python packages
  - All npm dependencies for React UI

Build with `--target base` for a lighter image with just the core stack, or `--target REACT_UI` for the full setup with GUI support.

After that you can enter container following `autopilot_ndt` instructions from composers. 

When you enter container, start the whole sim using: 
```
tmuxinator start minithex_demo
```

## TODO: 
- [x] build example telekyb3 with all reqs
- [x] Set up init quadrotor simulation with gz-sim 
- [x] Add aerial physical interaction package with it 
- [ ] Wait for the ROS 2 support check

## Useful documentation: 
- [Genom-ROS-Gazebo](https://homepages.laas.fr/felix/files/tp.pdf)
- [Quick setup guid for robotpkg](http://robotpkg.openrobots.org/install.html) 
- [mrsim-gazebo](https://git.openrobots.org/projects/mrsim-gazebo) 
- [mrsim-gazebo-docs](https://git.openrobots.org/projects/mrsim-gazebo/pages/README)
- [robotpkg](https://git.openrobots.org/projects/telekyb3/pages/software/install/robotpkg)
- [Running a quadrotor in simulation](https://git.openrobots.org/projects/telekyb3/pages/software/run/quadrotor-simulation)
