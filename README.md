# apf-obstacle-avoidance

![Alt text](./Supplement/animated.gif)

This is a MATLAB implementation of obstacle avoidance with a 2DOF robot animation, using the [artificial potential field approach](https://cs.stanford.edu/group/manips/publications/pdfs/Khatib_1986_IJRR.pdf) introduced by [Oussama Khatib](https://cs.stanford.edu/groups/manips/ok.html) in 1986.

If you want to run the project just run potential_field_GUI.m and it will do the rest for you!

* You can introduce the coordinates for the initial end effector position, goal end effector position, obstacle position and robot base through the GUI.

* You can select the obstacle radius and link lengths of the 2DOF animated manipulator through the GUI.

### *Generate Workspace* will reinitialize the workspace with the new coordinates and dimensions you have entered.

![Alt text](./Figures/Workspace.png?raw=true "Initial Workspace")

### *Go to Config. Space* will generate a configuration space for the current workspace and display it.

![Alt text](./Figures/configspace.png?raw=true "Generated configuration space for the given initial workspace above")

### *Show Potential Field* will display how the curent potential field looks.

![Alt text](./Figures/potentials.png?raw=true "How the potential field looks like for the current configuration space")

* *Plan Motion* will plan the motion from initial position to goal position using the potential field approach.

![Alt text](./Figures/motion.png?raw=true "Whole GUI")

Be aware that the implementation is still not sturdy enough to handle some obstacle locations, dimensions and link lengths.