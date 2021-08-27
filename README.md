# inrol_sim_peginhole
A ROS demonstration package for our article in 2021, *"Fast and Accurate Data-Driven Simulation Framework for Contact-Intensive Tight-Tolerance Robotic Assembly Tasks"*, IJRR (to be submitted).

## Reference
``` bash
Yoon, J., Lee. M., Son. D., and Lee, D. J. "Fast and Accurate Data-Driven
Simulation Framework for Contact-Intensive Tight-Tolerance Robotic Assembly
Tasks", International Journal of Robotics Research (To be submitted)
```

Please visit https://www.inrol.snu.ac.kr/journal for more information.

## Installation
### Environment
This program is built and tested in ubuntu 18.04 with ROS melodic. [cudnn 11.0](https://developer.nvidia.com/compute/machine-learning/cudnn/secure/8.0.5/11.0_20201106/cudnn-11.0-linux-x64-v8.0.5.39.tgz) and [cuda 11.0](https://developer.nvidia.com/cuda-11.0-download-archive) is required to be installed. Please refer to the linked pages.

From the above described environment, run following commands to build the package.
``` bash
$ cd <your_catkin_ws>/src
$ git clone https://github.com/JM-SNU/inrol_sim_peginhole
$ cd ..
$ catkin_make -DCMAKE_BUILD_TYPE=Release
```

## Running example launch file
``` bash
$ roslaunch inrol_sim_peginhole example.launch
```

## Changing the trajectory
The asset directory has the following structure.
```
./
|-  des/
    |-  pd.txt
    |-  Rd.txt
|-  obj/
|-  shader/
|-  txt/
```

In the above, `des` directory contains a space-separated data text for the setpoint of the peg tip.
Users can modify them if they want to change the simulation trajectory.

Other directories are intended to be not mutated, and should be left unmodified.
