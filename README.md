# Decaf

Over the last couple years, I have been a part of the Autonomous Rover Team at the University of Toronto, where we build a (you guessed it) an autonomous rover for the Intelligent Ground Vehicles Competition (IGVC). I have also been a part of aUToronto, the Self-Driving Car team at UofT, where we develop a level 4 autonomous vehicle to compete at the SAE Autodrive Challenge Series. At both these teams, I have taken part in developing a slice of the perception stack, whether it is lane detection, 2D object detection, or 3D object detection. However, through my involvement with these teams, I have built a curiosity and interest in learning about other parts of the robotics stack, besides perception.

In this repo, I will outline how I developed a mini mobile robot that can autonomously navigate an environment (specifically, the course outlined in the IGVC rule book), using ROS2. This is a re-make of the robot that ART works on each year. Therefore, the goal of this project was not novelty or innovation; instead, it is for me to build up skill and general knowledge about an end-to-end robotics stack, so that I can leverage it in future projects.

Throughout this file, I will use a :bulb: emoji to reference any research/articles that I used to build my understanding of a concept.

### Table of contents:

1. Simulating a mobile robot
    - Building the chassis
    - Adding sensors
    - Creating a simulation world
2. Building a perception stack
    - Detecting lane lines
    - Fusing perception data
3. Road to autonomy
    - State estimation
    - SLAM
    - Navigation


