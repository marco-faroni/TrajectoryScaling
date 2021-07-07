# TrajectoryScaling

This repository contains Matlab files for testing trajectory scaling algorithms for robot manipulators. 

Developer:
* Marco Faroni

## How to use it

The main script is ``script_scaling.m``. Run it in Matlab.

## Configuration

Parameters can be change in ``script_scaling.m``.
You can configure your simulation by setting the following variables:

**method**
Choose the trajectory scaling method:
* ``thor`` : use a joint-space MPC trajectory scaling algorithm (see [1])
* ``cthor`` : use a Cartesian-space MPC trajectory scaling algorithm (see [2])
* ``local`` : use a non-look ahead joint-space trajectory scaling algorithm (see [3])
* ``clocal`` : use a non-look ahead Cartesian-space trajectory scaling algorithm (see [3])

_Important_: ``local`` and ``clocal`` can be used **with or without time-law adaptation module** (see [3] for details). If you wish to activate the timing-law adaptation module you should set the parameter **MODIFY_SREF** equal to 1 in the files _controllers/local/controllerdata.m_ and _controllers/clocal/controllerdata.m_


**task_space** 
Choose between ``cartesian`` and ``joint``

**time_vectors**
Choose the nominal execution times. All values in the vector are executed. Please notice that the values are not in seconds, the actual time will be printed during the execution as Ttot.

**horizon_vectors**
It is a vector containing the length of the predictive horizon you want to test.

**Np**
It is the number of prediction time instants used by MPC-based method (along the horizon window)

## References

If you wish to use this code please cite one of the following publications:

[1] Marco Faroni, Manuel Beschi, Corrado Guarino Lo Bianco, and Antonio Visioli. Predictive jointtrajectory scaling for manipulators with kinodynamic constraints. Control Engineering Practice, 95:104264, 2020. Available at https://www.sciencedirect.com/science/article/pii/S0967066119302151

[2] Marco Faroni, Manuel Beschi, Nicola Pedrocchi, and Antonio Visioli. Predictive inverse kinematics for redundant manipulators with task scaling and kinematic constraints. IEEE Transactions on Robotics, 35(1):278–285, 2019.   Available at https://ieeexplore.ieee.org/document/8477138

[3] Marco Faroni, Manuel Beschi, Antonio Visioli, and Nicola Pedrocchi. A real-time trajectory planning method for enhanced path-tracking performance of serial manipulators. Mechanism and Machine Theory, 156:104152, 2021. Available at https://www.sciencedirect.com/science/article/abs/pii/S0094114X20303694
