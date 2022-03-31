# UAV_search_and_rescue
This is the copy of cource project Autonomous System (LRG6300), 2021WS, Technical University of Munich

## Introduction

Imagine we are in an avalanche scenario, and victims are randomly separated in the different places. Number of victims is unknown. The goal of our UAV is to fly over the avalanche and locate all the victims in a short time.  

For details, please refer to our [report](/Report/Search_and_Rescue_Simulation_in_Avalanche.pdf) or [presentation slides](/Slides/Autonomous_Systems_Terminus_Presentation.pdf).

## How to Set up and Run the Project


### 1. Set up:

1. in folder /src, add Submodule

   ```
   git submodule add https://github.com/ethz-asl/mav_trajectory_generation.git
   git submodule add https://github.com/ethz-asl/mav_comm.git
   git submodule add https://github.com/ethz-asl/eigen_catkin.git
   git submodule add https://github.com/ethz-asl/eigen_checks.git
   git submodule add https://github.com/ethz-asl/glog_catkin.git
   git submodule add https://github.com/ethz-asl/nlopt.git
   ```

2. in folder /src, load Submodule

   ```
   git submodule init
   git submodule update
   ```

3. in folder /project, build

   ```
   # only in first time build need to specify the Python version
   catkin build -DPYTHON_EXECUTABLE=/usr/bin/python3
   ```

### 2. Run the Project
If you want to run in Docker, please read [here](/Docker/README.md)  
If you want to run in your own Linux environment(with ROS melodic or noetic installed), please read below:  
We have two search models

- An intuitive idea, Inch-by-Inch Search 

  Read the [How to run inch-by-inch search](/src/README.md)

- An optimized Planning Method

  1. In Terminal 1, run unity

     ```
     roslaunch unity_bridge unity_sim.launch 
     ```

  2. In Terminal 2, run victim signal generation

     ```
     roslaunch victim_signal_gen victim.launch
     ```



  3. In Terminal 3, run rviz to see the trajectory in coordinate system

     ```
     roslaunch trajectory_visualization traj_visualize.launch 
     ```

  4. In Terminal 4, run trajectory planning (Please make sure it's the last step)

     ```
     rosrun planning planning_node
     ```

## Contributors


- **Code**

| Package                  | Function                    | Coder                                | Debugger                       |
| ------------------------ | --------------------------- | ------------------------------------ | ------------------------------ |
| planning                 | plan the trajectory         | **Dian Yuan**, Yinglei Song          | **Dian Yuan**, Yinglei Song        |
| next_waypoint            | generate the waypoints      | Yang Xu                              | Yang Xu                        |
| victim_signal_gen        | generate the victim signal  | **Dian Yuan**                        | **Dian Yuan**, Yinglei Song        |
| trajectory_visualization | show the trajectory         | **Dian Yuan**                        | **Dian Yuan**                      |
| Inch-by-inch Searcher    | A simple way to search      | Yang Xu                              | Yang Xu                        |

- **Build**

| Files                   | Coder                                | Debugger                       |
| ------------------------  | ------------------------------------ | ------------------------------ |
| Each Package's CMakeLists.txt and Package.xml | Yang Xu, **Dian Yuan** | Yang Xu        |
| Dockerfile           | Yang Xu                              | Yang Xu                        |


- **Algorithm of planning**

  **Dian Yuan**, Yinglei Song,  Jianyu Tang, Yubo Min


- **Report**

  Jianyu Tang

- **Slides for Presentation**

  Yinglei Song, Yubo Min
