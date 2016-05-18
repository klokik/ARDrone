#AR Drone 

The aim of this project is to use optical markers to precizely determine the pose of Unmanned aerial vehicle (UAV).

ArUco markers are used as reference points to locate UAV.

---

### Dependencies
* `gazebo`
* `ignition_math2`
* `opencv3`
* `opencv3_contrib`

### Building
```
mkdir build
cd build
cmake ..
make
```

### Running examples
When you build the project a bunch of examples would be created in the `bin` folder. To run these examples you first need to start `gazebo` with appropriate world.

### Docs
This project is the objective of the therm paper for my senior year at the uni. The paper text is located in the `docs` directory.