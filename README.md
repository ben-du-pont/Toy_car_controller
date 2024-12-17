# Toy Car Controller

This repository provides a small simulation environment based a four wheel model of a car and a Pacejka tire model. It also contains the implementation of a PID controller for constant longitudinal speed tracking and a Pure Pursuit controller for the lateral control of the car, using the steering angle.

---

## Accessing the Configuration File

The configuration file for the PID and Pure Pursuit controllers can be found at:
src/CONTROLLER_PARAMS.py

### Understanding the Controllers

- **PID Controller**: A Proportional-Integral-Derivative (PID) controller uses three terms to adjust the control output: 
  - **Proportional**: Reacts to current error.
  - **Integral**: Reacts to the accumulation of past errors.
  - **Derivative**: Reacts to the rate of change of the error.

- **Pure Pursuit Controller**: This controller follows a path by calculating the steering angle needed to reach a target point on the path ahead of the car, allowing for smoother turns and better trajectory tracking.

---

## Getting Started

### Prerequisites

Before running the code, ensure you have the following:

1. **Clone the Repository**
   ```bash
   git clone https://github.com/your-repo/Duckietown-Activity.git
   cd Duckietown-Activity


2. **Install Requirements** Install the necessary packages from requirements.txt. You may want to verify compatibility based on your environment:

    ``` bash
    pip install -r requirements.txt

3. **Edit Controller Parameters** Modify the controller parameters in:

src/CONTROLLER_PARAMS.py

4. **Run the Simulation** Execute the simulation to visualize the controller's performance:

    ```bash
    python src/run.py


## Example of the controller performance

### Too fast - collision

![Controller Performance Collision](videos/collision.gif)

## Slower - No collision
![Controller Performance No Collision](videos/no-collision.gif)

