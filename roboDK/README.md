PC Module (roboDK/pc_module.py)

Overview

This module listens for UDP JSON messages from the embedded modules (Gripper, Endowrist, Servomotors) and:
- Applies Gripper RPY to the simulated gripper in RoboDK
- Applies Endowrist RPY to the UR5e (sends URScript via TCP if a UR5e robot is reachable)
- Receives servo torques, maps them to a color-level (GREEN/YELLOW/RED), and logs numeric values to a CSV file

Message protocol (JSON over UDP)

Example Gripper message:
{
  "device": "G5_Gri",
  "roll": 5.0,
  "pitch": 0.0,
  "yaw": -15.0,
  "s1": 1,
  "s2": 0
}

Example Endowrist message:
{
  "device": "G5_Endo",
  "roll": 10.0,
  "pitch": -5.0,
  "yaw": 30.0,
  "s3": 1,
  "s4": 1
}

Example Servomotors message:
{
  "device": "G5_Ser",
  "torques": [0.12, 0.55, 0.9],
  "max_expected": 1.0
}

How to run

1. Start the PC module (RoboDK must be installed if using the RoboDK features):

   python3 roboDK/pc_module.py

2. From another process (or from the microcontroller), send JSON UDP messages to port 12345 on the PC IP.

3. Use the test sender to simulate messages locally:

   python3 roboDK/test_sender.py

What the PC module does with messages

- Gripper messages: update simulated Gripper orientation in RoboDK and handle S1/S2 buttons (needle parent).
- Endowrist messages: update tool pose in RoboDK and (if UR5e reachable) send movel commands to the real robot.
- Servomotors messages: build a compact color-coded status (e.g., s1:0.12(GREEN)), append numeric torques to roboDK/servo_torques.csv for analysis.

Notes and assumptions

- The script uses UDP port 12345 by default. Adjust constants in the script if required.
- Torque thresholds are heuristic (LOW<30%, MED 30-70%, HIGH>70% of `max_expected`). Adjust in the code if you prefer other thresholds.
- UR5e connection attempt uses TCP to the robot's default port 30002. If the robot is on a different IP or port, change `ROBOT_IP` / `ROBOT_PORT` in the script.

Next steps / improvements

- Provide a small web UI or richer GUI component to show color-coded bars per servo.
- Add authentication/ACL for UDP messages if operating over untrusted networks.
- Add unit tests and CI checks for message parsing.
