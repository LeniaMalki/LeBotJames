# LeBotJames
 Source code for robot competition

## Project description:
The robot shall be the winner of a basket-ball game. This section includes a description of the game and then a specification of the robot.

### General description
The main objective of the game is to be the winner of a game with two rounds and two robots. The winner is the robot with the highest number of points. In the first round, robot #1 is the defender, and then the attacker. Robot #2 attacks first, and then defends. A round lasts 3 minutes.

Game as a defender: the robot tries to avoid the other robot to score points.

Game as an attacker: the robot scores points by successfully shooting balls in basket.

### Scoring
Game as a defender: the defender robot gets 1 point each time at least a part of the wheel of the attacking robot enters into the exclusive area of the defender, i.e., at least a part of the wheel of the attacking robot is over the boundary line (adhesive).

Game as an attacker: the attacker robot scores 1 point each time it successfully shoots a ball from its exclusive area. It scores 2 points when sucessfully shooting from the shared area. And it scores 3 points when successfully shooting one ball from the defender exclusive area. "Successfully" means that the ball enters the basket and remains in the basket (i.e., it does not bounce outside of the basket). To select the area of a robot, we take the one giving the most points to the robot, assuming that at least part of one of its wheel is inside (so, not on a line) of this area. Also, the attacker robot gets two points each time the defender robot has a part of one of its wheel entering into the attacker exclusive area, i.e. a part of one wheel is over the boundary line.

### General rules
The robot can be freely re-programmed between game rounds, but it is forbidden to remotely control your robot.

Robots have the right to release obstacles, e.g. throwing an obstacle into the opponent's area is encouraged.

Robot must start in their designated area.

The attacking robot can contain up to 2 balls when starting.