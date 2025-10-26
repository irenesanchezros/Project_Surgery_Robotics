#!/usr/bin/env python3
"""
PC module for SurgeryRobotics

Listens for UDP messages from the embedded modules (Gripper, Endowrist, Servomotors)
and:
 - applies Gripper RPY to the simulated gripper in RoboDK
 - applies Endowrist RPY to the UR5e (via socket commands to the robot when connected)
 - receives servo torques and maps to a color code and logs numeric values

Message format expected (JSON):
{
  "device": "G5_Endo" | "G5_Gri" | "G5_Ser",
  "roll": 0.0, "pitch": 0.0, "yaw": 0.0,    # for RPY messages
  "s1": 1, "s2": 1, ...                        # optional buttons
  "torques": [0.12, 0.34, 0.01]                # for servomotors
}

"""
from robodk.robolink import *
from robodk.robomath import *
import time
import math
import threading
import socket
import json
import tkinter as tk
import csv
import os

# Network constants
UDP_IP = "0.0.0.0"
UDP_PORT = 12345
BUFFER_SIZE = 2048

# Robot / RoboDK constants
ROBOT_NAME = 'UR5e'
ZERO_YAW_TOOL = 0
ZERO_YAW_GRIPPER = 0
READ_INTERVAL_S = 0.01

# Robot TCP connection for real UR5e (if used)
ROBOT_IP = '192.168.1.5'
ROBOT_PORT = 30002
accel_mss = 1.2
speed_ms = 0.75
timel = 0.1

# Global shared state
Endowrist_rpy = None
Gripper_rpy = None
Servo_torques = None
data_lock = threading.Lock()

# UDP socket
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind((UDP_IP, UDP_PORT))

# robot socket (TCP) to UR5e
robot_socket = None
robot_is_connected = False

# Log file
TORQUE_LOG = os.path.join(os.path.dirname(__file__), 'servo_torques.csv')


def initialize_robodk():
    """Connect to RoboDK and configure items used by the simulation."""
    RDK = Robolink()
    robot = RDK.Item(ROBOT_NAME)
    base = RDK.Item(f'{ROBOT_NAME} Base')
    endowrist = RDK.Item('Endowrist')
    gripper = RDK.Item('Gripper')
    needle = RDK.Item('Needle')
    try:
        Init_target = RDK.Item('Init')
        robot.MoveL(Init_target)
    except Exception:
        pass
    robot.setPoseFrame(base)
    robot.setPoseTool(endowrist)
    gripper_init = TxyzRxyz_2_Pose([0, 5, -105, 0, 0, 0])
    try:
        gripper.setParent(endowrist)
        gripper.setPose(gripper_init)
    except Exception:
        pass
    needle_init = TxyzRxyz_2_Pose([0, 0, 0, 0, 0, 0])
    try:
        needle.setParent(gripper)
        needle.setPose(needle_init)
    except Exception:
        pass
    try:
        robot.setSpeed(50)
    except Exception:
        pass
    return robot, base, gripper, needle


def check_robot_port(ROBOT_IP, ROBOT_PORT):
    global robot_socket
    try:
        robot_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        robot_socket.settimeout(1)
        robot_socket.connect((ROBOT_IP, ROBOT_PORT))
        return True
    except Exception:
        robot_socket = None
        return False


def send_ur_script(command):
    """Send a URScript command line to the connected UR5e (if connected)."""
    global robot_socket
    if robot_socket:
        try:
            robot_socket.send(("{}\n".format(command)).encode())
        except Exception as e:
            print(f"Error sending to robot: {e}")


def receive_response(t):
    time.sleep(t)


def endowrist2base_orientation(roll, pitch, yaw):
    # transformation used by existing project code
    roll2 = (roll + 90) % 360
    pitch2 = pitch % 360
    yaw2 = yaw % 360
    return roll2, pitch2, yaw2


def torque_to_level_color(torque, max_expected=1.0):
    """Map torque numeric value to a (level, color_name) tuple.
    - green for low, yellow for medium, red for high.
    The function assumes torque >= 0. If negative values are possible the mapping can be adjusted.
    """
    if max_expected <= 0:
        max_expected = 1.0
    pct = abs(torque) / max_expected
    if pct < 0.3:
        return 'LOW', 'GREEN'
    if pct < 0.7:
        return 'MED', 'YELLOW'
    return 'HIGH', 'RED'


def log_torques(torques):
    header = ['timestamp'] + [f's{i+1}' for i in range(len(torques))]
    write_header = not os.path.exists(TORQUE_LOG)
    with open(TORQUE_LOG, 'a', newline='') as fh:
        writer = csv.writer(fh)
        if write_header:
            writer.writerow(header)
        writer.writerow([time.time()] + torques)


def read_data_UDP():
    global Endowrist_rpy, Gripper_rpy, Servo_torques
    while True:
        try:
            data, addr = sock.recvfrom(BUFFER_SIZE)
            try:
                received_data = json.loads(data.decode())
            except json.JSONDecodeError:
                print('Invalid JSON received')
                continue

            device_id = received_data.get('device')
            if device_id == 'G5_Endo':
                with data_lock:
                    Endowrist_rpy = received_data
            elif device_id == 'G5_Gri':
                with data_lock:
                    Gripper_rpy = received_data
            elif device_id == 'G5_Ser' or device_id == 'SERVOS':
                # Accept either G5_Ser or SERVOS label just in case
                with data_lock:
                    Servo_torques = received_data
            else:
                # Unknown device - ignore or extend protocol
                pass
        except Exception as e:
            print(f"UDP reader error: {e}")
            break


def update_text_label(label, tool_orientation, gripper_orientation, status_message, torque_values):
    full_text = f"Tool: {tool_orientation}\nGripper: {gripper_orientation}\n{status_message}\nTorques: {torque_values}"
    label.after(0, lambda: label.config(text=full_text))


def move_robot(robot, gripper, needle, text_label):
    global ZERO_YAW_TOOL, ZERO_YAW_GRIPPER, Endowrist_rpy, Gripper_rpy, Servo_torques, robot_is_connected

    endow_msg = ''
    grip_msg = ''
    status_msg = ''
    torque_msg = ''

    while True:
        with data_lock:
            current_Endowrist_rpy = Endowrist_rpy
            current_Gripper_rpy = Gripper_rpy
            current_Servo = Servo_torques

        if current_Endowrist_rpy:
            e_roll = current_Endowrist_rpy.get('roll', 0)
            e_pitch = current_Endowrist_rpy.get('pitch', 0)
            e_yaw = current_Endowrist_rpy.get('yaw', 0)
            s3 = current_Endowrist_rpy.get('s3')
            s4 = current_Endowrist_rpy.get('s4')
            endo_roll, endo_pitch, endo_yaw = endowrist2base_orientation(e_roll, e_pitch, e_yaw)
            try:
                endowrist_pose = robot.Pose()
                Xr, Yr, Zr, rr, pr, yr = Pose_2_TxyzRxyz(endowrist_pose)
                endowrist_pose_new = transl(Xr, Yr, Zr) * rotz(math.radians(ZERO_YAW_TOOL)) * rotz(math.radians(endo_yaw)) * roty(math.radians(endo_pitch)) * rotx(math.radians(endo_roll))
                if robot.MoveL_Test(robot.Joints(), endowrist_pose_new) == 0:
                    robot.MoveL(endowrist_pose_new, True)
                    endow_msg = f"R={round(endo_roll)} P={round(endo_pitch)} W={round((endo_yaw+ZERO_YAW_TOOL)%360)}"
                    status_msg = ''
                    if robot_is_connected:
                        Xr, Yr, Zr, rr, pr, yr = Pose_2_TxyzRxyz(endowrist_pose_new)
                        endowrist_pose_msg = f"movel(p[{Xr}, {Yr}, {Zr}, {rr}, {pr}, {yr}], a={accel_mss}, v={speed_ms}, t={timel}, r=0.0000)"
                        send_ur_script(endowrist_pose_msg)
                        receive_response(timel)
                else:
                    endow_msg = f"R={round(endo_roll)} P={round(endo_pitch)} W={round((endo_yaw+ZERO_YAW_TOOL)%360)}"
                    status_msg = 'Robot cannot reach the position'
            except Exception as e:
                # If RoboDK is not available or robot item missing, just continue
                endow_msg = f"R={round(endo_roll)} P={round(endo_pitch)} W={round((endo_yaw+ZERO_YAW_TOOL)%360)}"

            # Buttons S3/S4 relative Z motion
            if s3 == 0 or s4 == 0:
                try:
                    current_pose = robot.Pose()
                    Tz = transl(0, 0, 5) if s3 == 0 else transl(0, 0, -5)
                    new_pose = current_pose * Tz
                    status_msg = 'S3 pressed: up' if s3 == 0 else 'S4 pressed: down'
                    if robot.MoveL_Test(robot.Joints(), new_pose) == 0:
                        robot.MoveL(new_pose, True)
                    else:
                        status_msg = 'Cannot move more in Z (relative)'
                except Exception:
                    pass

        if current_Gripper_rpy:
            g_roll = current_Gripper_rpy.get('roll', 0)
            g_pitch = current_Gripper_rpy.get('pitch', 0)
            g_yaw = current_Gripper_rpy.get('yaw', 0)
            s1 = current_Gripper_rpy.get('s1')
            s2 = current_Gripper_rpy.get('s2')
            try:
                gripper_pose = gripper.Pose()
                Xg, Yg, Zg, rg, pg, yg = Pose_2_TxyzRxyz(gripper_pose)
                gripper_pose_new = transl(Xg, Yg, Zg) * rotz(math.radians(ZERO_YAW_GRIPPER)) * rotz(math.radians(g_yaw)) * roty(math.radians(g_pitch)) * rotx(math.radians(g_roll))
                gripper.setPose(gripper_pose_new)
                grip_msg = f"R={round(g_roll)} P={round(g_pitch)} W={round((g_yaw+ZERO_YAW_GRIPPER)%360)}"
                if s1 == 0:
                    needle.setParentStatic(base)
                    status_msg = 'S1 pressed: needle released'
                elif s1 == 1:
                    needle.setParent(gripper)
                    needle.setPose(TxyzRxyz_2_Pose([0, 0, 0, 0, 0, 0]))
                    status_msg = 'S2 pressed: needle grasped'
            except Exception:
                pass

        if current_Servo:
            # Expect current_Servo to contain a 'torques' list
            torques = current_Servo.get('torques') if isinstance(current_Servo, dict) else None
            if torques:
                # log numeric values
                try:
                    log_torques(torques)
                except Exception:
                    pass
                # build terse color-coded status
                torque_parts = []
                for i, tval in enumerate(torques):
                    level, color = torque_to_level_color(tval, max_expected=current_Servo.get('max_expected', 1.0))
                    torque_parts.append(f"s{i+1}:{round(tval,3)}({color})")
                torque_msg = ' '.join(torque_parts)

        update_text_label(text_label, endow_msg, grip_msg, status_msg, torque_msg)
        time.sleep(READ_INTERVAL_S)


def on_closing(root, sock):
    print('Closing...')
    try:
        sock.close()
    except Exception:
        pass
    try:
        root.destroy()
    except Exception:
        pass


def set_zero_yaw_tool(value):
    global ZERO_YAW_TOOL
    ZERO_YAW_TOOL = float(value)


def set_zero_yaw_gripper(value):
    global ZERO_YAW_GRIPPER
    ZERO_YAW_GRIPPER = float(value)


def main():
    global robot_is_connected
    robot, base, gripper, needle = initialize_robodk()
    robot_is_connected = check_robot_port(ROBOT_IP, ROBOT_PORT)

    root = tk.Tk()
    root.title('PC Module - Surgery Robotics')
    root.protocol('WM_DELETE_WINDOW', lambda: on_closing(root, sock))

    text_label = tk.Label(root, text='', wraplength=600, justify='left', font=('Consolas', 10))
    text_label.pack(padx=10, pady=10)

    tool_yaw_slider = tk.Scale(root, from_=-180, to=180, orient=tk.HORIZONTAL, label='Tool Yaw', command=lambda v: set_zero_yaw_tool(float(v)), length=300)
    tool_yaw_slider.set(ZERO_YAW_TOOL)
    tool_yaw_slider.pack()

    gripper_yaw_slider = tk.Scale(root, from_=-180, to=180, orient=tk.HORIZONTAL, label='Gripper Yaw', command=lambda v: set_zero_yaw_gripper(float(v)), length=300)
    gripper_yaw_slider.set(ZERO_YAW_GRIPPER)
    gripper_yaw_slider.pack()

    # Start UDP reader thread
    udp_thread = threading.Thread(target=read_data_UDP, daemon=True)
    udp_thread.start()

    # Start robot movement thread
    robot_thread = threading.Thread(target=move_robot, args=(robot, gripper, needle, text_label), daemon=True)
    robot_thread.start()

    root.mainloop()


if __name__ == '__main__':
    main()
