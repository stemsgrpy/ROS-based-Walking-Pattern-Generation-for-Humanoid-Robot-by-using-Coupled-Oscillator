# ROS-based-Walking-Pattern-Generation-for-Humanoid-Robot-by-using-Coupled-Oscillator

## Walking Pattern Generation <code adjusting!!!>
- Generate three-dimensional trajectories of **waist** and **feet** for humanoid robot
- Download this project
```
  git clone https://github.com/stemsgrpy/ROS-based-Walking-Pattern-Generation-for-Humanoid-Robot-by-using-Coupled-Oscillator.git
```
- Source setup.*sh file of this project
```
  source <your workspace>/devel/setup.bash
```
 - Bulid the project
```
  cd <your workspace>
  catkin_make
```
 - Run core (master)
```
  roscore
```

## Node
- There are **three nodes** in this **walkinggait** project
  - walkingcycle
  - walkingtrajectory
  - inversekinematics
```
    # Running nodes in separate terminals
    rosrun walkinggait walkingcycle
    rosrun walkinggait walkingtrajectory
    rosrun walkinggait inversekinematics
```   

### Topic
```
    rostopic list
```   
```
    # Showing in the terminal
    /SendBodyAuto_Topic
    ...
```   

### Service 
```
    rosservice list
```   
```
    # Showing in the terminal
    /ik_client
    /test_client
    ...
```   

## Usage
- See message of **SendBodyAuto_Topic**
```
  rostopic type SendBodyAuto_Topic
  rosmsg show walkinggait/Interface
```
- Setting walking distance, such as 1cm
```
  # Forward
  rostopic pub -1 /SendBodyAuto_Topic walkinggait/Interface -- '1000' '0' '0' '0' '0' '0'
  # Backward
  rostopic pub -1 /SendBodyAuto_Topic walkinggait/Interface -- '-1000' '0' '0' '0' '0' '0'
  # Leftward
  rostopic pub -1 /SendBodyAuto_Topic walkinggait/Interface -- '0' '1000' '0' '0' '0' '0'
  # Rightward
  rostopic pub -1 /SendBodyAuto_Topic walkinggait/Interface -- '0' '-1000' '0' '0' '0' '0'
```
- Trajectory Dataset
  - **Trajectory_Record.ods** will be generated in **Parameter folder**

## Result

<p align="center">
  <img width="500" src="/README/OverallTrajectory.jpg">
</p>
<p align="center">
  Figure 1: Overall Trajectory (Forward Trajectory)
</p>

<p align="center">
  <img src="/README/ForwardTrajectory.jpg" alt="Description" width="320" height="180" border="0" />
  <img src="/README/BackwardTrajectory.jpg" alt="Description" width="320" height="180" border="0" />
  <img src="/README/LeftwardTrajectory.jpg" alt="Description" width="320" height="180" border="0" />
  <img src="/README/RightwardTrajectory.jpg" alt="Description" width="320" height="180" border="0" />
</p>
<p align="center">
  Figure 2: Forward, Backward, Leftward, Rightward Trajectory 
</p>

## Reference
[A Biologically Inspired Biped Locomotion Strategy for Humanoid Robots: Modulation of Sinusoidal Patterns by a Coupled Oscillator Model](https://ieeexplore.ieee.org/document/4456756)  
[Gait pattern generation and stabilization for humanoid robot based on coupled oscillators](https://ieeexplore.ieee.org/document/6095061)  