Collision avoidance is a broad field and an important technical problem in realizing the holodeck. Due to the breadth and complexity of this problem, considering that one semester is not enough for a very in-depth study, we cannot study it in such a large area. Instead, we will look at how to combine existing collision avoidance research results and make innovations, test the algorithm on Gazebo and evaluate the results. 

Therefore, we subdivided the goal of the entire project into three stages: flying to the target point, detecting and avoiding static obstacles, and detecting and avoiding dynamic obstacles. Among them, static obstacles refer to static obstacles whose positions are known in advance, and dynamic obstacles include sudden moving obstacles and other UAVs that cooperate with each other. The realization difficulty of these three stages increases layer by layer and progresses gradually.

This project will be able to use in airsim or other platform. This project implemented algorithm for drones to avoid collision in two types of environments.

For static obstacles environment.
Traditional algorithm BFS-UCS-A* is implemented as baseline algorithm.
MADDPG is the main algorithm used in static environment.

