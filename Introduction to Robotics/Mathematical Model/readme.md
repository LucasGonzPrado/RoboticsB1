# Mathematical Model for 6-DOF Robotic Arm Kinematics and Simulation

This section presents the kinematic and dynamic modeling, simulation, and analysis of a **6-DOF articulated robotic arm**.  
The robot is modeled using the **Modified Denavit–Hartenberg (MDH)** convention and validated through MATLAB-based simulations.  
Geometric offsets derived from the CAD model are incorporated to accurately represent the physical robot.

---

## 1. Forward Kinematics (MDH Model)

Forward kinematics defines the geometric structure of the robot and computes the pose of the end-effector from the joint variables.

- The robot is modeled using the **MDH convention**.
- Coordinate frames are assigned to each joint.
- Fixed offsets from the CAD design are included.
- The resulting kinematic model matches the real mechanical configuration.

![Kinematic Diagram](https://github.com/user-attachments/assets/391dd07a-20de-4128-8b0d-89c7e2d3ad51)
![Robot](https://github.com/user-attachments/assets/8952656a-d594-46d9-bea5-242ee4803029)

---

## 2. Inverse Kinematics

Inverse kinematics determines joint configurations that achieve a desired end-effector pose.

- Enables task-level planning in Cartesian space.
- Ensures solutions respect joint limits.
- Provides valid joint configurations for trajectory execution.

![Ik](https://github.com/user-attachments/assets/bc2c59f7-ba4c-4401-a708-54a417a82d90)


---

## 3. Trajectory Planning

A smooth trajectory is generated between an initial and a target configuration.

- Joint-space trajectories are time-parameterized.
- The resulting Cartesian motion is smooth and continuous.
- Start and end points are clearly defined.

![End - Effector Trajectory](https://github.com/user-attachments/assets/52a26c5d-f184-4ddf-88d8-cda45eae0156)

---

## 4. Jacobian Matrix and Singularity Analysis

The Jacobian matrix is computed to analyze velocity transmission and detect singular configurations.

- Relates joint velocities to end-effector linear and angular velocities.
- A scalar singularity index (σ) is evaluated.
- Regions with poor kinematic performance are identified.
- Color variation indicates proximity to singularities.

![Jacobian Matrix](https://github.com/user-attachments/assets/54acbdf2-da35-4db5-aae4-b6b43925e89b)

---

## 5. Workspace Generation

The reachable workspace of the robot is approximated by sampling joint configurations within their limits.

- Displays the volume reachable by the end-effector.
- Represents reachability independent of motion quality.
- Used to validate physical reach of the robot.

![93f65750-6879-42c5-9fbd-0d4c458362b2](https://github.com/user-attachments/assets/26f36799-1987-4e41-b425-dd8e63f62847)


---

## 6. Manipulability Analysis

The Yoshikawa manipulability index is evaluated along a planned trajectory.

- Quantifies the robot’s dexterity during motion.
- Highlights regions of good and poor manipulability.
- Helps identify optimal regions for task execution.

![Robot Animation Colored By Manipulability](https://github.com/user-attachments/assets/e51f0377-77e3-4a8b-8774-52e4afa136d9)

---

## 7. Joint Motion Profiles

Joint motion profiles are analyzed to ensure smooth and feasible motion.

- Joint positions, velocities, and accelerations are plotted over time.
- Confirms continuity and absence of abrupt changes.
- Validates trajectory feasibility for real actuators.

![Joint motion Profiles](https://github.com/user-attachments/assets/c188c240-b8e9-4cc3-86b4-50c458d580b1)
---

## 8. Dynamic Gravity Torque Evaluation

Gravity-induced torques are computed along the planned trajectory considering payload effects.

- Evaluates torque requirements for each joint.
- Identifies the most mechanically demanding joints.
- Useful for actuator selection and mechanical validation.

![Joint gravity torque along trayectory](https://github.com/user-attachments/assets/bc9ed4f8-0f4f-4485-beaf-59aba4b6a864)

---

## 9. Robot Animation (MDH)

A full 3D animation of the robot motion is generated using the MDH kinematic model.

- Confirms consistency between mathematical model and visualization.
- Displays coordinated motion of all six joints.
- Serves as a final validation of the simulation pipeline.

![Robot Animation](https://github.com/user-attachments/assets/48db4d3d-3364-49aa-8e33-085b7035b7f2)

---

## Applications

- **Robotics Education**  
  Visualization and understanding of advanced kinematic and dynamic concepts.

- **Simulation and Validation**  
  Verification of motion feasibility before physical implementation.

- **Trajectory and Motion Planning**  
  Design of smooth, singularity-aware trajectories.

- **Performance Evaluation**  
  Analysis of workspace quality, manipulability, and joint loading.

---

## Summary of Visualizations

- MDH-based robot model with CAD offsets  
- End-effector trajectories in Cartesian space  
- Workspace and singularity maps  
- Manipulability evaluation  
- Joint position, velocity, acceleration, and torque profiles  
- Full robot animation


