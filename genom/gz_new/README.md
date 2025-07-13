# GenoM - Gazebo 

Initial docker to set up GenoM components for the easier development. 

# Env variables for gz: 
```
export GZ_SIM_PLUGIN_PATH=$HOME/openrobots/lib/gazebo:${GAZEBO_PLUGIN_PATH}
export GZ_SIM_MODEL_PATH=$HOME/openrobots/share/gazebo/models
```

# How to run init simulation: 

In one terminal you can run: 
```
./tk3-quadrotor-simulation.sh 
```

In the other terminal you can run: 
```
python3 -i quadrotor_ctl.py
```

After pressing play in the gazebo simulation you can run 
quadrotor in the python interpreter as: 
```
setup()
start()
... 
nhfc.set_position()
...
```

## TODO: 
- [x] build example telekyb3 with all reqs
- [x] Set up init quadrotor simulation with gz-sim 
- [ ] Test ctl 
- [ ] Add aerial physical interaction package with it 
- [ ] Wait for the ROS 2 support check

## Useful documentation: 
- [Genom-ROS-Gazebo](https://homepages.laas.fr/felix/files/tp.pdf)
- [Quick setup guid for robotpkg](http://robotpkg.openrobots.org/install.html) 
- [mrsim-gazebo](https://git.openrobots.org/projects/mrsim-gazebo) 
- [mrsim-gazebo-docs](https://git.openrobots.org/projects/mrsim-gazebo/pages/README)
- [robotpkg](https://git.openrobots.org/projects/telekyb3/pages/software/install/robotpkg)
- [Running a quadrotor in simulation](https://git.openrobots.org/projects/telekyb3/pages/software/run/quadrotor-simulation)
