#!/usr/bin/env python3
"""
Small test sender to exercise the PC module UDP listener.
Run this on the PC where pc_module.py is running (or point HOST to that PC).

This script sends three messages:
 - Endowrist RPY
 - Gripper RPY (corrected)
 - Servomotor torques
"""
import socket
import json
import time

HOST = '127.0.0.1'
PORT = 12345

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

# Example Endowrist message
endo = {
    'device': 'G5_Endo',
    'roll': 10.0,
    'pitch': -5.0,
    'yaw': 30.0,
    's3': 1,
    's4': 1
}

# Example Gripper corrected RPY
gri = {
    'device': 'G5_Gri',
    'roll': 5.0,
    'pitch': 0.0,
    'yaw': -15.0,
    's1': 1,
    's2': 0
}

# Example Servomotors torques
serv = {
    'device': 'G5_Ser',
    'torques': [0.12, 0.55, 0.9, 0.3],
    'max_expected': 1.0
}

for msg in (endo, gri, serv):
    data = json.dumps(msg).encode()
    sock.sendto(data, (HOST, PORT))
    print('Sent', msg['device'])
    time.sleep(0.5)

print('Done')
