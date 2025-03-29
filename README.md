# Auto-Follower
**A person-following framework for urban Ackermann human-machine collaborative robotics. Auto-Follower comprises three main modules: Vision-LiDAR Servo Tracker (VLST), local mapping, and a navigation module based on the Iterative Radius Points Search(IRPS) method. Experiments in both urban environments and laboratories have been conducted to validate the effectiveness of our framework. The results demonstrate that our proposed framework, with its superior perception capabilities, achieves robust person-following performance and generalizes well across different robot types and application scenarios. A video of the demonstration of the method can be found on [YouTube](https://www.youtube.com/watch?v=xQaiPCszXCA).**

## Menu
- [**System Overview**](#system-overview)

## System Overview
<p align='center'>
    <img src="./visual_follow_move/config/doc/system_overview.png" alt="drawing" width="800"/>
</p>

The main contributions of our work can be summarized as follows:  
•	A novel person-following framework for urban human-robot collaboration, integrating VLST, local mapping, and IRPS-based navigation.   
•	A VLST module that fuses camera and LiDAR data to achieve 360-degree person identification and localization.  
•	An IRPS-based navigation method designed to dynamically determine obstacle-free navigation goals, ensuring safe and efficient path planning.  

