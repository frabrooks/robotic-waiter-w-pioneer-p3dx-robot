# Beverly
Please could everyone put a little bit about their section in here?

It doesn't have to be too much but would help so that if we get asked a question in the demo, anyone can answer it and have a rough idea of how everything works.

# Localisation
- Using our code from assignment 2 with the particle filter.
- Not adaptive.

# Path-finding
- PathFinder is a ROS node that runs on the robot. 
- Takes pose estimate (from localisation), goal (from job planning), and obstacles on map (from obstacle avoidance). 
- Uses A* to path find on graph.
- Able to add obstacles to mark nodes as obstructed and path-find around them.

## Graph creation
- We used Blender to divide map (from previous exercises) into triangular regions.
- Exported as vertices and faces.
- Read in to system and generate traversable graph.

# Camera/QR code
- Two libraries: OpenCV and [the other one]
- OpenCV used to recognise shape of QR codes and format sensibly.
- [the other one] used to parse QR code and read data.

# Job planning/scheduling


# Obstacle avoidance
- (As above in path-finding, can replan around obstacles.


# Moving
