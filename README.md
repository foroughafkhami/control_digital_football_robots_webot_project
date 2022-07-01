# Digital_LAB_Project

## Attacker

### Groups members duty
Miss. Afkhami: Optimizing Attackr code and final checks of the README.md
<br>
Miss. Saljoughi: Designing the concept and constraints for Attacker.
<br>
Mr. Rabiee: Editing README.md and coding the PID class and its usage in Attacker code.

### Code Discribtion
The robot follows the ball and as it gets to a certain distance of it, attacks it at full speed and attempts to follow it afterwards untill a goal is made.

### Strategy and Algorithm
1- We used a greedy algorithm (as the attacker gets closer to the ball, it moves faster).
<br>
2- Using PID controllers tuned for faster responce rather than minimum over shoot.
<br>
3- Avoiding contact with other robots so the not to get tangled up and miss the opportunity to goal.

### Controller and Discretization
We used a PI contorller for angle control and a P controller for distance. For disceteization of every section of the PID contoller we can use the ZOH method. this method consists of sampler with specific frequency.


### Innovations
We used the sonar sensor values to prevent the robots colliding with one another  or with the walls.


## Defender

### Groups members duty
Miss. Afkhami: Optimizing Defender code and final checks of the README.md
<br>
Miss. Saljoughi: Designing the concept and constraints for Defender.
<br>
Mr. Rabiee: Editing README.md and coding the PID class and its usage in Defender code.

### Code Discribtion
The robot moves to the small box in front of its own goal and rotates so as not to lack progress. if the ball gets close, defender attacks it and fends off the attack.

### Strategy and Algorithm
1- Using PID controllers tuned for faster responce rathers than minimum over shoot.
<br>
2- A greey aproach as the ball gets close the defenders goal.
<br>
3- Not caring about the contact with other robots as the priority of defending is higher.

### Controller and Discretization
We used a PI contorller for controlling the angle of the robot with the ball and its own goal and a P controller for distance in both cases. For disceteization of every section of the PID contoller we can use the ZOH method. this method consists of sampler with specific frequency.

### Innovations
Defining a small box in the form of the area which the robot should be inside, in order to start defeneding.
