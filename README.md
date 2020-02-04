This repository contains the code used to produce results in "IoT-enabled Autonomous System Collaboration forDisaster-Area Management" J Journal paper.
- AADL model 
  1. The folder contains the source code used to verify the framework and the developed components using AADL formal methods. 
- Ground_station
  1. calculate_cluster_center.py calculates the cluster center of the ground UGVs from their current location using K-mean clustering algorithm and send these information to the UAVs.(1)
  2. visualize_clustering.py creats a live visualization of the clustering scenario.(1)
- Jackal 1 and 2
  1. random_walk_ugv.py implements the random walk algorithm for the Jackal UGVs within the lab environment.(2)
  2. follow_square_trajectory.py implements the UGVs to follow a square trajectory.(2)
- UAV 1 and 2
  1. DroneClustering.py moves the drone to the cluster centers.(2)
  2. track_ugv_and_land.py tracks the UGVs and land on top of it at the end of the mission.(2)
  
  Demonstration Vedios 
  - https://youtu.be/oJjjpAHFQfg
  - https://youtu.be/b-oo2U6SeuM
  - https://youtu.be/HRDLazQLQ2g
  - https://www.youtube.com/watch?v=tAgj4TOMuH4
  - https://www.youtube.com/watch?v=p5oziiC8Wc8
  
