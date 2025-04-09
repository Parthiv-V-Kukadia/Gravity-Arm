# Gravity-Arm: Controller Design for a Gravity-Assisted Underactuated Robot Arm

This repository contains the code for designing and simulating a controller for a gravity-assisted underactuated robot arm. The goal is to control the motion of the second joint of the arm to follow a desired trajectory, utilizing gravity to assist the movement.

## Repository Structure

The repository includes the following key files:

* **`Controller.m`:** This file defines the structure of the controller for the gravity arm. It includes an initialization function (`initControlSystem`) called once at the start of the simulation and a run function (`runControlSystem`) executed at each simulation time step. Currently, the provided code sets up the basic structure and loads equations of motion but does not implement any specific control logic.
* **`Simulator.m`:** This file contains the simulation environment for the gravity-assisted underactuated robot arm. It takes a controller function name as input and simulates the arm's dynamics, interacting with the provided controller. The simulator handles:
    * Initialization of the arm and controller.
    * Running the simulation loop.
    * Interfacing with the controller to obtain actuator commands (torque about the first joint).
    * Integrating the equations of motion of the gravity arm.
    * Providing sensor data (angle of the second joint) to the controller.
    * Optional features such as data logging, movie saving, and diagnostic plots.
* **`EOMs.mat` (Expected):** This file is expected to contain the symbolic (`symEOM`) and numeric (`numEOM`) representations of the equations of motion for the gravity arm. These equations are likely derived symbolically, considering the influence of gravity, and then converted into MATLAB functions for numerical evaluation during the simulation. The `initControlSystem` function in `Controller.m` attempts to load this file.

## Controller Interface (`Controller.m`)

The `Controller.m` file defines the interface between the simulator and the control algorithm. The controller function returns a structure with two function handles:

* **`func.init(parameters, data)`:** This function is called once at the beginning of the simulation. It receives:
    * `parameters`: A structure containing simulation parameters such as the time step (`tStep`), maximum applicable torque (`tauMax`), and the symbolic and numeric equations of motion (`symEOM`, `numEOM`).
    * `data`: A user-defined structure that can be used to store controller-specific data across simulation steps.
    * The `initControlSystem` function in the provided code loads the equations of motion into the `data` structure.

* **`func.run(sensors, references, parameters, data)`:** This function is called at each time step of the simulation. It receives:
    * `sensors`: A structure containing sensor measurements from the gravity arm, including:
        * `t`: Current time.
        * `q2`: Angle of the second joint relative to the first.
    * `references`: A structure containing desired reference values, including:
        * `q2`: Desired angle of the second joint.
    * `parameters`: The same parameters structure passed to the `init` function.
    * `data`: The user-defined data structure, potentially updated in previous `run` calls.
    * It should return a structure `actuators` containing the control command:
        * `tau1`: Torque to be applied about the first joint.
    * The `runControlSystem` function in the provided code currently outputs zero torque.

## Simulator Usage (`Simulator.m`)

The `Simulator.m` file provides a function `Simulator(controller, varargin)` to run the simulation.

* The first argument, `controller`, is a string specifying the name of the MATLAB function that defines the controller (e.g., `'MyGravityArmController'`).
* The `varargin` argument allows for optional parameter-value pairs to configure the simulation, including:
    * `'team'`: A string for a team name to be displayed on the figure.
    * `'datafile'`: A filename to log simulation data.
    * `'moviefile'`: A filename to save a movie of the simulation.
    * `'snapshotfile'`: A filename to save a snapshot of the final simulation frame.
    * `'controllerdatatolog'`: A cell array of field names from the controller's `data` structure to log.
    * `'diagnostics'`: A boolean flag to enable diagnostic plots of states and actuators.
    * `'tStop'`: The total simulation time.
    * `'disturbance'`: A boolean flag to add a disturbance torque.
    * `'reference'`: A function handle that specifies the desired reference trajectory for `q2` as a function of time.
    * `'initial'`: A 4x1 vector specifying the initial joint angles (`q1`, `q2`) and velocities (`v1`, `v2`).
    * `'display'`: A boolean flag to enable or disable the live animation of the simulation.

## Getting Started

1.  **Save the Code:** Save the provided `Controller.m` and `Simulator.m` files in your MATLAB working directory or in a directory on the MATLAB path.
2.  **Generate Equations of Motion:** You will need to generate the equations of motion for the gravity-assisted underactuated robot arm, considering the effect of gravity. This should be done both symbolically and numerically, and the results saved in a file named `EOMs.mat`. The `GetEOM` function within `Simulator.m` provides an example of how these equations are obtained and saved, taking into account physical parameters like link positions (`p1`, `p2`), masses (`m1`, `m2`), moments of inertia (`J1in1`, `J2in2`), friction coefficients (`b1`, `b2`), and gravity (`g`). You might need to use a symbolic computation tool (like MATLAB's Symbolic Math Toolbox) to derive these equations.
3.  **Implement Your Controller:** Modify the `runControlSystem` function in `Controller.m` to implement your desired control algorithm for the gravity arm. This function will receive the current angle of the second joint (`sensors.q2`) and the desired angle (`references.q2`) and should output a torque command (`actuators.tau1`) to control the first joint. Consider how gravity can be utilized in your control strategy.
4.  **Run the Simulation:** In the MATLAB command window, call the `Simulator` function with the name of your controller function (e.g., `'Controller'`) and any desired optional parameters. For example:
    ```matlab
    Simulator('Controller', 'tStop', 15, 'display', true, 'reference', @(t) 0.5*sin(0.8*t));
    ```
    This command will run the simulation for 15 seconds, display the animation, and set the desired angle of the second joint to `0.5*sin(0.8*t)`.

This repository provides a foundation for exploring control strategies for gravity-assisted underactuated robot arms. By implementing your control logic in `Controller.m` and using the `Simulator.m` to test it, you can analyze the arm's behavior and the effectiveness of your controller.
