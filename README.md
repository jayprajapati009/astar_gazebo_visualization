# ENPM661  Project - 3

## Student Details

|Name|Jay Prajapati|Akash Parmar|
|---|:---:|:---:|
|UID|119208625|118737430|
|Directory ID|jayp|akasparm|
|Email ID|jayp@umd.edu|akasparm@umd.edu

## Dependencies used

|Module|Version|
|---|:---:|
|Python|3|
|Time|Default|
|Numpy|Default|
|heapq|Default|
|rospy|Default|
|OpenCV|4.7.0|

## Links

|Item|Link|
|---|:---:|
|Github Repository|[here](https://github.com/jayprajapati009/astar_gazebo_visualization.git)|
|Videos|[here](https://drive.google.com/drive/folders/1az9Xc2jjLH1F60jIFIt1WnjcL93X6fYJ?usp=sharing)|

## Run Code

### Part-1

Download the zip folder and navigate to the ```Part01``` folder.

```sh
cd projp2_akasparm_jayp/Part01
```

and run the python script.

```sh
python3 astar_akasparm_jayp_p1.py
```

#### Sample input

```
Enter your choice for mode of operation,

Type 1 for selecting the parameters manually
Type 2 for preset parameters

Your Choice: 1

Enter x coordinate of Start Point: -10
Enter y coordinate of Start Point: -40
Enter theta coordinate of Start Point: -160
Enter x coordinate of Goal Point: 510
Enter y coordinate of Goal Point: -28
Enter the clearance: 5
Enter the rpm1: 1
Enter the rpm2: 2
```

Note: After entering the input data, the optimized path will be displayed. The user can also select the preset option while entering the choices to get the sample output.


### Part-2

Download the zip folder and navigate to the ```Part02``` folder.

```sh
cd projp2_akasparm_jayp/Part02
```
Note: Assuming that the user has already created a catkin workspace and put the given ROS package in ```src``` folder.

Build the catkin workspace.

```sh
cd ~/catkin_ws && catkin_make
```
To simulate the turtlebot in Gazebo, first launch the launch file given in the ROS package and then run the python script.

```sh
roslaunch astar_turtlebot world.launch
```

In another terminal window


```sh
python3 astar_akasparm_jayp_p2.py
```

#### Sample input

```
Enter your choice for mode of operation,

Type 1 for selecting the parameters manually
Type 2 for preset parameters

Your Choice: 1

Enter x coordinate of Start Point: -10
Enter y coordinate of Start Point: -40
Enter theta coordinate of Start Point: -160
Enter x coordinate of Goal Point: 510
Enter y coordinate of Goal Point: -28
Enter the clearance: 5
Enter the rpm1: 1
Enter the rpm2: 2
```

Note: After entering the input data, the optimized path will be displayed and the turtlebot will be simulated in Gazebo. The user can also select the preset option while entering the choices to get the sample output.



Navigate to the ```Proj3p2_akash_jay``` folder after unzipping the folder,

```sh
python3 a_star_akash_jay.py
```

Enter the initial node and goal node as per the instruction in the terminal

- First enter the choice for entering the initial and the goal point
  - 1 for Manual entry
  - 2 for Preset Values
- Manual Input Parameters
  - Start Point x-coordinate
  - Start Point y-coordinate
  - Start Point theta (should be multiple of 30)
  - Goal Point x-coordinate
  - Goal Point y-coordinate
  - Goal Point theta (should be multiple of 30)
  - Clearance (should be 5)
  - Robot radius (should be 5)
  - Step size (should be between 1 to 10)

- Once the goal node is found, the simulation will be displayed in a window.
- Press the waitKey to end the code and get the mp4 file of the animation in the working directory