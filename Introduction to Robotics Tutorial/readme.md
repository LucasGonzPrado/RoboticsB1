# Task 1 - Pick & Place
<img width="1916" height="1076" alt="Screenshot 2025-12-08 215522" src="https://github.com/user-attachments/assets/1c627fe0-c162-4d68-bea5-b33ef7cc332b" />

## Objective ⚙️🛠️  

### Retrieving Tokens and Blocks from the Feeder
1. **Approach the feeder:** The robot moves to the pickup zone using predefined reference points for consistent accuracy.  
2. **Acquire the object:** Once positioned above the target, the vacuum nozzle activates to secure the block or token.

### Transfer to Fixture for Centering
1. **Navigate to the fixture:** The robot transports the item to the fixture plate while avoiding collisions through pre-taught paths.  
2. **Release:** The vacuum turns off to drop the piece.  
3. **Align:** The robot adjusts the part within the fixture to ensure proper centering.  
4. **Reacquire:** With the item centered, the nozzle grabs it again for the next step.

### Placement into Tray Slots
1. **Move to the tray:** The robot moves the calibrated piece to the tray position using predefined points.  
2. **Insertion:** The robot locates the correct hole and inserts the part gently to avoid impacts.  
3. **Finalize:** The nozzle releases the piece and the sequence repeats for all remaining components.

## Components 🕹️🔧
- 1 Feeder  
- 1 Fixture  
- 1 Tray  
- 3 Tokens  
- 3 Blocks  

# Task 2 – Stacking  
<img width="1897" height="995" alt="Screenshot 2025-12-08 221250" src="https://github.com/user-attachments/assets/d506c35e-54bf-46a6-b341-0b64081359d0" />

## Objective ⚙️🛠️  

### Retrieving Tokens and Blocks from the Feeder  
1. **Approach the feeder:** The robot moves toward the pickup area using predefined coordinates to guarantee repeatable positioning.  
2. **Grasp the item:** With the robot aligned above the token or block, the nozzle engages to secure the piece.

### Creating the Stacked Column  
1. **Navigate to the base:** The robot transports the item to the stacking base through a pre-established reference point.  
2. **Alternate stacking:** Blocks and tokens are placed one after another, forming a tower with alternating elements.

## Components  
- 1 Feeder  
- 1 Base  
- 10 Tokens  
- 10 Blocks  

# Task 3 – Checkers  
<img width="686" height="853" alt="Screenshot 2025-12-14 175033" src="https://github.com/user-attachments/assets/aedd50c7-248b-4db1-9b0e-f909af647b6b" />

## Objective   
Enable a user to play a checkers game against the robot using physical buttons, without directly touching the board or the pieces.

---

## Button-Based Control System — Checkers Game Robot

The robot control system uses a set of physical buttons to interact with and control the robot during the execution of a checkers game. These buttons allow the operator to select pieces, choose movement directions, and control the game flow in a simple and intuitive way. Each button is assigned a specific function to ensure smooth gameplay and accurate robot motion.

---

### 🔘 Button Functions

- 🟢 **Green Push Button**  
  Confirms the current selection and executes the selected action. This button validates the chosen move or capture and commands the robot to perform the operation on the board.

- 🔵 **Blue Push Button**  
  Moves the selection vertically when navigating the board. When a movement mode is active, this button commands the robot to move the selected piece along the left diagonal direction.

- ⚪ **White Push Button**  
  Moves the selection horizontally when navigating the board. When a movement mode is active, this button commands the robot to move the selected piece along the right diagonal direction.

- 🔴 **Red Push Button**  
  Activates **Move Mode**, allowing the robot to perform a standard diagonal movement with the selected checker piece.

- 🟠 **Orange Push Button**  
  Activates **Capture Mode**, enabling the robot to execute a capture by jumping over an opponent’s piece according to checkers rules.

---

###  Task Flow

1. **Select a piece:**  
   The user navigates across the board using the blue and white buttons to select a checker piece.

2. **Choose the operation mode:**  
   The user selects either *Move Mode* or *Capture Mode* depending on the intended action.

3. **Select movement direction:**  
   The movement direction is defined using the navigation buttons according to the active mode.

4. **Confirm action:**  
   The green button is pressed to confirm the selection and trigger the robot motion.

5. **Execute movement:**  
   The robot performs the selected move or capture accurately on the checkers board.

---

###  Modes of Operation

- **Move Mode:** Standard diagonal movement of a checker piece  
- **Capture Mode:** Jumping over and capturing an opponent’s piece  

---

## Components 

- 1 Six-axis robotic arm  
- 1 Checkers board  
- Checkers pieces (black and white)  
- Physical button-based HMI  
- Control electronics and wiring  

---

