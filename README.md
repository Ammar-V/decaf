# Decaf

![Video of a mobile robot navigating through obstacles.](./assets/decaf-nav.gif)

Over the last couple years, I have been a part of the Autonomous Rover Team at the University of Toronto, where we build a (you guessed it) an autonomous rover for the Intelligent Ground Vehicles Competition (IGVC). I have also been a part of aUToronto, the Self-Driving Car team at UofT, where we develop a level 4 autonomous vehicle to compete at the SAE Autodrive Challenge Series. At both these teams, I have taken part in developing a slice of the perception stack, whether it is lane detection, 2D object detection, or 3D object detection. However, through my involvement with these teams, I have built a curiosity and interest in learning about other parts of the robotics stack, besides perception.

In this repo, I will outline how I developed a mini mobile robot that can autonomously navigate an environment (specifically, the course outlined in the IGVC rule book), using ROS2. This is a re-make of the robot that ART works on each year. Therefore, the goal of this project was not novelty or innovation; instead, it is for me to build up skill and general knowledge about an end-to-end robotics stack, so that I can leverage it in future projects.

Below, I will discuss some of the main challenges/learnings throughout this experience.

## Perception
For this mobile, robot, the perception stack was relatively straightforward. There are solid white lanes that need to be detected, a task highly suitable for classical approaches such as color thresholding. Once detected in the image plane, they are projected into 3D, and converted to a pointcloud. To use the lane lines when navigating, they needed to be converted to a LaserScan message and fused with the LiDAR data, since the slam_toolbox (the package used for SLAM in this project), strictly processes LiDAR data.

However, there is a problem that arises from using just thresholding for the lanes: any white parts in the frame will be assumed to be lane lines. The barrels in the environment have reflective stripes on them, which would be picked up as a lane line. Since the lane lines have to be projected into 3D, any false-positives are detrimental. Figure 2 shows what the effect of these _ghost_ objects on the mapping process.

![A picture showcasing false-positives from the lane detection algorithm]()

As such, I wrote a simple algorithm to overcome this as follows:

1. Perform thresholding to find the orange pixels on the barrel.
2. Assuming that the barrels are always upright, the orange pixels in the barrel can be used to _cancel_ out any white pixels from the stripe. As such, count the number of orange pixels in every column of the image.
3. Set a `pixel_intensity` for the number of acceptable orange pixels in any given column. If the number of orange pixels is greater than this, set the entire column to 0s in the lane line mask.
   - Currently, `pixel_intensity` is simply set to 1. However, the simulation environment is free of noise, and as such, this has been left as a tuning parameter that can be adjusted to work in the real world.

![A barrel detection algorithm, used to cancel out false positives in lane detection]() 


## State estimation
State estimation is crucial when it comes to robotics. Simply put, state estimation is the process of using sensor inputs to estimate the position and orientation of a robot in space. For a mobile robot, this is highly important so that the it can navigate from point A to B. 

Take a second to pause and think about how you navigate an environment. For example, if you want to go from your bedroom to your kitchen, you need to know how many steps you need to take (distance) in a certain direction (orientation). To do this, you need to know where the kitchen is relative to the bedroom. On top of that, you need even more fundamental information - knowing that you are in your bedroom and not already in the kitchen!


![A diagram of different sensors such as wheel encoders, IMU, and GPS.]()

To perform state estimation, one can use a variety of sensors. For example, a wheel encoder counts the number of rotations and combines information about the wheel's radius to calculate the odometry (how far the robot has moved from a previous known location). On the other hand, an IMU can provide tons of information about the robots state, such as velocity, acceleration, and orientation. Finally, a GPS can be used to pinpoint where the robot is in a global, world frame. 

However, in the real world, things are not as nice as they seem - sensors are highly noisy making any position/orientation estimation from pure sensors unreliable over time. A wheel can slip, leading to the encoder to believe the robot has traveled further than it actually has. IMU can have high variance and be super noisy. Similarly, a GPS may have an accuracy in meters, making it unreliable depending on your application (super expensive ones will have centimeter level accuracy). The solution? - an Extended Kalman Filter (EKF).

### Extended Kalman Filter

## SLAM
Simultaneous Localization and Mapping (SLAM) is a widely used technique used two solve two problems in robotics: mapping and localization (!). For this project, I heavily relied on the slam_toolbox, which takes in a LaserScan topic as input to map the environment, and localize within in. But why is SLAM needed in the first place?

Let's go back to the example of navigating your home. 

## Nav2






