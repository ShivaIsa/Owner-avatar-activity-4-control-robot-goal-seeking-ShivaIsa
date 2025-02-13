# Control Robot - Goal Seeking

### Deadline : February 15th, 2025 11:59pm

***This class activity is to be done as individuals, not with partners nor with teams.***

### How to get started →

Begin by reading this entire writeup and making sure you have a good understanding of it. Next, spend some time planning how you’re going to approach the problem. 

# Introduction

---

In this class activity, you will implement the *goal seek* behavior on the TurtleBot3 robot in simulation. The goal-seeking task requires your robot to autonomously navigate towards a specific goal position from any starting point within the simulation environment.

TurtleBot3 is a holonomic robot, meaning it can move in any direction, allowing us to decouple rotation and translation. This feature enables you to control the robot’s rotation first, and then move it in the desired direction towards the goal. Your task will involve guiding the robot to move efficiently and accurately to a series of goal positions.

You’ll start by reading goal positions from a text file. Using these coordinates, the robot will calculate the required heading and distance to the goal. It will first rotate to face the goal, then move toward it. To ensure smooth movement, you’ll implement PID controllers for both rotation and translation. The rotation PID will manage smooth turning, while the translation PID will ensure the robot decelerates as it approaches the goal for a precise stop.

A PID controller (Proportional-Integral-Derivative) is a control system used to achieve stable movement by continuously adjusting a system’s output. It works by minimizing the error between a desired set-point and the actual output. The proportional term corrects based on the current error, the integral term accounts for accumulated past errors, and the derivative term anticipates future errors by considering the rate of change. 

Together, these three components ensure smooth, accurate control, reducing overshoot and helping systems like robots to reach their target positions with minimal oscillation or delay.

# Activity Information

---

### Objectives

- Learn to implement behaviors in robot control
- Understand each component of the PID controller
- Get familiar with Gazebo and TurtleBot3

### Resources

- http://wiki.ros.org/ROS/Introduction
- [TurtleBot3 Simulation Setup](https://www.notion.so/TurtleBot3-Simulation-Setup-e516c2ce05da48c991d2c683d72876d2?pvs=21)
- https://pidexplained.com/pid-controller-explained/

### Requirements

- Your package should build when simply dropped into a workspace and compiled using `catkin build`
- Your launch file should execute and launch the simulator and your nodes
- Your package, nodes and launch files should follow the naming convention, if your code does work due to the filenames being incorrect, you will receive zero points

### What we provide

- Detailed instructions to setup your simulation environment
- Several template code `.py` and launch files `.launch`
- A ROS package to compile your program

### What to submit

You must submit a compressed (zip) ROS package with the follow file structure

```bash
goak_seek
├── launch
│   └── goal_seek_part1.launch # Should launch simulator and your nodes (basic)
│   └── goal_seek_part2.launch # Should launch simulator and your nodes (with PID)
├── src
│   └── goal_seek_part1.py # Your ROS node for goal seek (basic)
│   └── goal_seek_part2.py # Your ROS node for goal seek (with PID)
├── goals.txt
├── package.xml
└── CMakeLists.txt
```

*Please make sure you adhere to the structure above, if your package doesn’t match it the grader will give you a **zero***

### Grading considerations

- **Late submissions:** Carefully review the course policies on submission and late assignments. Verify before the deadline that you have submitted the correct version.
- **Environment, names, and types:** You are required to adhere to the names and types of the functions and modules specified in the release code. Otherwise, your solution will receive minimal credit.

# Part 1 : Goal Seeker

---

In this part, you will write a ROS note that will read goal positions from a text file, subscribe to `/odom` (`nav_msgs/Odometry`) for current velocity and publish `geomtry_msgs/Twist` message to the `/cmd_vel` topic. The `goals.txt` file provided has several goal positions, you code must parse the text and seek all the goal positions in the text file. You robot must wait for a set amount of time (~5 seconds) after reaching a goal position before moving on to the next one. The release code includes a `goal_seek_part1.py` script that contains a code template. You may change anything in this script apart from its name and the shebang in the first line. 

### Plan of attack (Optional)

1. Write your node in `goal_seek_part1.py`
    1. Compute distance to goal
    2. Compute angle to goal
    *Hint : Use high-school geometry for angle between 2 points*
    3. Subscribe to `/odom` and get current velocity
    4. Set `max_linear` and `max_angular` velocities
    5. Write your goal seek logic and loop till you reach the goal (for each goal)
2. Set permissions for your python script using `chmod +x <script>.py`
3. Use `rosrun goal_seek goal_seek_part1.py` to run your publisher
4. Check your nodes and topics using `rqt_graph`
5. Write a launch file `goal_seek_part1.launch` and run it

### Sample `goals.txt`

```
4.5 6.4
7.6 2.4
8.7 2.3
```

# Part 2 : PID Controller

---

Now that you have your basic goal seeker working lets extend your node to include a PID controller. The PID controller will help smooth rotations and make sure your robot decelerates when close to the goal. You will need two PID controllers, one for linear velocity (translation) and another for angular velocity (rotation), each with its 3 gains.

$kp$, $ki$ and $kd$ are the proportional, integral and derivative gains for your PID controller. Each gain must be tuned based on expected behavior. You may start with only a P-controller and then add the derivative component and finally add the integral. This will help you turn each gain must faster and help you understand how they contribute to the final control output. In total you will have 6 gains to tune so isolating makes life easier!

**Please refer to lectures slides and resources above for a good understanding of the PID controller.** 

### Plan of attack (Optional)

1. Start by writing a P-controller, this goes in your `odom_callback` before you publish
2. Tune your p-controller, your robot’s velocity (linear and angular) should ramp up and down based on distance and angle to set-point
3. Include the derivative component and tune your gains, your robot’s velocity should be smoother and oscillations reduced
4. Finally, include the integral component and tune your gains
5. Put everything together and test your controller for a few goal points

# Submission and Assessment

---

Submit using the Github upload feature on [autolab](https://autolab.cse.buffalo.edu)

**Note: Make sure your code complies to all instructions, especially the naming conventions. Failure to comply will result in zero credit**

You will be graded on the following. Penalties are listed under each point, absolute values, w.r.t activity total.

1. Part 1 (Goal Seeker) [50%] 
    1. Rotation goal seek working [25%]
    2. Translation goal seek working [25%]
2. Part 2 (PID Controller) [50%]
    1. PID working and tuned for rotation [25%]
    2. PID working and tuned for translation [25%]

**Note: Starting from this activity, you will not receive partial credit for launch files working /not working. Please make sure your launch files execute**
