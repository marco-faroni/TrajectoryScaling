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

**task_space** 
Choose between ``cartesian`` and ``joint``

**time_vectors**
Choose the nominal execution times. All values in the vector are executed. Please notice that the values are not in seconds, the actual time will be printed during the execution as Ttot.

**horizon_vectors**
It is a vector containing the length of the predictive horizon you want to test.

**Np**
It is the number of prediction time instants used by MPC-based method (along the horizon window)
