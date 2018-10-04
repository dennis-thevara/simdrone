# Simdrone
ROS Packages for the simulation of an ARDrone in Gazebo

* ROS Version: Kinetic Kame
* Gazebo version: 7.0

These packages are built on top of the tum_simulator package that was ported to Kinetic by @angelsantamaria, available [here](https://github.com/angelsantamaria/tum_simulator).

## Initialization

1. Create a ROS workspace if you don't have one already
   
   ```bash
   cd

   mkdir -p catkin_ws/src && cd catkin_ws/src && catkin_init_workspace
   ```
2. Install dependencies
   
   * Hector Suite:
   ```bash
   sudo apt-get install ros-kinetic-hector-*
   ```
   * Ardrone Autonomy:
   ```bash
   sudo apt-get install ros-kinetic-ardrone-autonomy
   ```

3. Clone this repository and catkin_make

   ```bash
   git clone https://github.com/dionysius07/simdrone.git

   cd ..

   catkin_make
   ```

4. Source the workspace setup.bash
   
   ```bash
   source devel/setup.bash
   ```

NOTE: If you encounter a parse error during catkin_make that reports:

```bash
/usr/include/boost/type_traits/detail/has_binary_operator.hp:50: Parse error at "BOOST_JOIN"
```

Execute the following:

```bash
sudo gedit /usr/include/boost/type_traits/detail/has_binary_operator.hpp
```

This will open a script file in gedit. Find the following line:

```C++
namespace BOOST_JOIN(BOOST_TT_TRAIT_NAME,_impl) {
```

Add in the following additional lines before and after it:

```c++
#ifndef Q_MOC_RUN
namespace BOOST_JOIN(BOOST_TT_TRAIT_NAME,_impl) {
#endif
```

Similarly, find the closing bracket lower down:

```C++
} // namespace impl
```
Add in the same additional lines:
```C++
#ifndef Q_MOC_RUN
} // namespace impl
#endif
```
Now return to your workspace and run catkin_make again, and the error should not reappear.

## Launch

1. Launch your Gazebo world:
   
   ```bash
   roslaunch cvg_sim_gazebo ardrone_testworld.launch
   ```

2. Launch the joystick teleop:
   
   ```bash
   roslaunch simdrone_control simdrone_teleop.launch
   ```
   NOTE: Currently supports only XBox 360 controllers

   Control | Function
   --- | ---
   X | Takeoff
   Y | Land
   Left Analog stick | Yaw (Rot Z)
   Right Analog stick | Linear movement in X and Y
   L1 | Decrease altitude
   R1 | Increase altitude