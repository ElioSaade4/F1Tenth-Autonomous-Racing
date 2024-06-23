# F1Tenth Lab 4: Follow The Gap

## Outline
In this lab, a node is developed to drive autonomously based on Follow The Gap algorithm. The algorithm consists of processing the lidar measurements to find the biggest and farthest gap that the car can drive towards without colliding with obstacles.

## Requirements
- Docker Desktop

## Running the Lab
To run this lab, perform the following steps:

1. Clone the repository to a local directory

2. Open **Docker Desktop**

3. Open a command prompt, and navigate to the cloned lab folder
```
cd C:\...\F1Tenth_Labs\LAB4-Follow_The_Gap
```

4. Run the following command in the command prompt to build the needed docker images and start the container
```
docker-compose up --build
```

5. After the container is up and running, open a new command prompt and open a bash session inside the container with this command:
```
docker exec -it lab4-follow_the_gap-sim-1 /bin/bash
```

7. In a web browser, open http://localhost:8080/vnc.html and click **Connect**

6. In the bash session, source the needed **ros** files, then build the project by running these commands in order:
```
cd .. 
source opt/ros/iron/setup.bash
cd sim_ws 
colcon build
source install/local_setup.bash
```

7. Start **tmux** with 
```
tmux
```
This will allow you to open mutliple bash sessions from the same window. Use **ctrl+B - C** to open a new session. Use **ctrl+B - #** to navigate between the sessions with their number. 

8. In the first bash session, launch the gym environment
```
ros2 launch f1tenth_gym_ros gym_bridge_launch.py
```
In the second bash session, run the keyboard teleoperations node to be able to drive the car by pressing keyboard keys (in case you need to stop the car)
```
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```
In the third bash session, run the wall following node that will start autonomous driving
```
ros2 run car_controller gap_follow_node
```

## Further Development
To modify the code, simply open the lab folder locally in **VSCode** and perform the changes. Then navigate to **sim_ws** in a bash session and run **colcon build**. This will re-build the ros packages using the updated code. Finally proceed to run the simulation environment as mentioned above. There is no need to stop the gym environment or the teleop keyboard to run **colcon build**. 