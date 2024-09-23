# Duckietown Activity: Toy Car Controller

## Welcome to the 2024 ETH Duckietown Class!

This activity is designed as a self-assessment tool to help you gauge your comfort level with the necessary tools for an engaging and successful semester in the Duckietown course. While your results will not be the primary factor in the selection process, they may serve as a tie-breaker if needed.

---

## Task Overview

### Objective

You will work with a simulated toy car and aim to achieve the best lap time possible on a designated track. To enhance the car's performance, you will be able to adjust the controller parameters, optimizing the tuning to reach faster speeds while ensuring the car stays on track.

---

## Accessing the Configuration File

The configuration file for the PID and Pure Pursuit controllers can be found at:
src/CONTROLLER_PARAMS.py

### TODO:

![Configuration File Screenshot](path/to/screenshot.png)

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



## Conclusion

This activity not only serves as a self-assessment but also provides valuable insights into your understanding of control systems and simulation environments. Take your time to experiment with different parameters and refine your strategies for optimal performance. Good luck!