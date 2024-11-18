PROJECT #1
-------------------------------------------------------------------------------------------
**Unfinished**

* Evaluate performance of robot

**Finished**
* Import arm and trajectory files
* Compute Forward Kinematics for t=0, with 4 links
* Compute Forward Kinematics for arbitrary link count
* Compute Jacobian for 4R robot
* Compute Jacobian for arbitrary link count
* One DLS loop, no repetition for angle diff minimization 
* Full DLS loop for one point along trajectory
* Full DLS loop for arbitrary trajectory points

**Bugs/Concerns**
* The y value is being rounded to 0 for two points when taking the norm, meaning the boundary is being passed by the robot arm



**DLS Procedure**
1) Compute forward kinematics to find end-effector position
2) Calculate position differential: desired pos - curr pos
3) Compute the Jacobian
4) Solve for joint angle differential = Jtrans * (J * Jtrans + lamba^2 * I)^-1 * pos diff
5) Update configuration: new angle = curr angle + angle diff
6) Repeat until angle differential ~= 0 OR arm is unable to move any farther 
