# Bludot
Robot Soccer AI for the LEGO MINDSTORMS robots used by TCC. 

## Robot Design
The assumed design of the robot, along with motor and direction designatons is shown below. Directions are relative to the wheels. its worth noting that the robot will move fastest on the diagonals between two wheels. The program is also flexible enough to handle different angles between the wheels, as long as opposite wheels are coaxial and parallell. the angle is measured between the right direction and wheel 3 in the diagram, and is assumed to be symetrical. the diagram below shows a wheel angle of 45°.
![image](diagrams/robot.png)


## Program
The following are high level diagrams of how the main parts of the AI work:

The main game loop is structured like this, where several movements are calculated and executed at once:
![image](diagrams/game_loop.png)

The arc function produces the turning factor that multiplies the angle detected of the ball to produce a desired path around the ball. Visualisations of several arc functions can be simulated [in this folder](ai-simulation/curve_sim) by running the index.html file. by default it compares the hybrid function the lerp arc. you can view the "bad area" where the ball will be knocked the wrong way by pressing space.

![image](diagrams/capture_ball.png)

Although the movements are summed in the diagram, in practice they are just added to the global sum of movements in the movement sum's class.

The sensor readings for the ball's direction are also interpolated with recursive use of the lerp function to help the bot maintain momentum, this helps avoid sharp turns that will make the bot fall over from interita.

![image](diagrams/allign_to_goal.png)

Its important to note that since the translational movement is scaled down to accomodate rotation, the higher the rotation is set the slower the bot will move translationally. in the extreme case when the rotation is 1, the bot will stop moving in order to spin at maximum speed.

![image](diagrams/movement_sum.png)
