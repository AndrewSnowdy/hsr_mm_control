# HSR Multi-Modal Control

## Requirements
- ROS2 Humble
- Ubuntu 22.04 (x86)

## Setup

### 1. Install HSR ROS2 Simulation
Clone the HSR ROS2 documentation repo and follow the simulation setup instructions:

```bash
git clone https://github.com/hsr-project/hsr_ros2_doc/tree/humble
```

Then navigate to `docs/setup_sim_en.md` and follow the instructions.

### 2. Clone This Repository
Clone into your ROS2 workspace `src` folder, then rebuild:

```bash
cd ~/ros2_ws/src
git clone https://github.com/AndrewSnowdy/hsr_mm_control
cd ..
colcon build
source install/setup.bash
```

## Usage

### Running Behaviors
In a terminal, launch one of the following modes:

**Behavior 1:**
```bash
ros2 launch hsr_mm_control hsr_mm_demo.launch.py mode:=behavior1
```

**Behavior 2:**
```bash
ros2 launch hsr_mm_control hsr_mm_demo.launch.py mode:=behavior2
```

### Running the Full Simulation
The full simulation requires launching the controllers separately.

**Terminal 1:**
```bash
ros2 launch hsr_mm_control hsr_mm_demo.launch.py mode:=full_sim
```

**Terminal 2:**
```bash
ros2 run hsr_mm_control mission_sequencer
```

**Terminal 3:**
```bash
ros2 run hsr_mm_control joint_trajectory_control
```

---

## Hardware Runs
### Scenario 1
<video src="https://github.com/user-attachments/assets/f41ea67a-a5d9-40f4-82c5-91c54d617275" controls autoplay loop muted></video>

### Scenario 2
<video src="https://github.com/user-attachments/assets/b96cae1d-0052-470d-8a3b-66a972b6a807" controls autoplay loop muted></video>

### Scenario 3
<video src="https://github.com/user-attachments/assets/c6cffd7e-8552-420a-a836-8cd0cc45d2a4" controls autoplay loop muted></video>

### Scenario 4
<video src="https://github.com/user-attachments/assets/73ccc976-24c6-4550-a567-39f9a98e6489" controls autoplay loop muted></video>

## Simulation Runs

### Scenario 5
<video src="https://github.com/user-attachments/assets/34c90a23-7a4f-45d5-8ca3-e4c5b512750e" controls autoplay loop muted></video>

### Scenario 6
<video src="https://github.com/user-attachments/assets/5634dcda-c5ca-43ef-bf30-0f33d35bb4f1" controls autoplay loop muted></video>

### Scenario 7
<video src="https://github.com/user-attachments/assets/4e86fdbd-f519-45a4-8c53-7f7cda5714ab" controls autoplay loop muted></video>
