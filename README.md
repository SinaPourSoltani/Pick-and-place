# Pick-and-place
An implementation of a pick and place robot using techniques from robotics and computer vision.

## Usage:
Navigate to the build folder and CMake with the compiler flags that determine the use of vision method.
The SS flag toggles the Sparse Stereo. The DS flag toggles the Depth Sensor.

The executable takes a Scene as an argument, as well as the choice of a motion planning method.
Choose between: P2P P2PB RRT. P2P by default. 

```bash
cd PickNPlace/build/
cmake .. -DSS=OFF -DDS=ON
make
./PickNPlace ../../Project_WorkCell/Scene.wc.xml RRT
```
