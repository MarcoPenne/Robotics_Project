# Robotics_Project

In this repository you will find our work about the dynamic identification of a 1R robot and a 3R robot, estimating the dynamic parameters/coefficients by solving an optimization problem.

As we know, it is very important to have an accurate dynamic model because it allows us to implement better control laws for achieving many tasks. In particular, we have dealt with the problem of torque sign lack, as it happens in some real robot (e.g., KUKA KR5). In fact, when we know only the absolute value of torque and we do not have the knowledge about the torque sign, we cannot use traditional methods for dynamic identification. For this reason, we have developed an algorithm able to estimate torque signs so that we could proceed with the identification using well known strategies.

The repository is organized as follows:

## Report
In this folder there is the report.pdf file.

## Code

The scripts Torque_generation_1dof.m and Torque_generation_3dof.m generate the data to be fed in the algorithm (regressor matrix and torques) by taking as input an exciting trajectory.

The scripts PBRP_1dof_tree.m and PBRP_3dof_tree.m run the algorithm for the estimation of the dynamic parameters, estimating the torque signs with the first version of our implemented algorithm.

The scripts PBRP_1dof_tree_classic.m, PBRP_3dof_tree_classic.m, PBRP_franka_tree_classic.m run the algorithm for the estimation of the dynamic parameters, estimating the torque signs with a fast version of our implemented algorithm.

The scripts validation_1dof.m and validation_3dof.m are used to validate our results on new trajectories.

dynamic_model_3R.m computes the dynamic model of the 3R robot.

The scripts Simulation_1dof.m and Simulation_3dof.m simulate the execution of a trajectory in V-REP.
