#!/bin/bash

# 1. Terminalde ros2 run joy joy_node komutunu başlatır.
gnome-terminal -- bash -c "echo 'Starting joy_node...'; ros2 run joy joy_node; exec bash"
sleep 1

# 2. Terminalde sudo ile motor_server.py scriptini çalıştırır.
gnome-terminal -- bash -c "echo 'Starting motor_server.py...'; sudo /usr/bin/python3 /home/tatek/Documents/core/tatekbot/tatekbot_4ws_node/motor_server.py; exec bash"
sleep 10

# 3. Terminalde 4ws_node.py scriptini çalıştırır.
gnome-terminal -- bash -c "echo 'Starting 4ws_node.py...'; /usr/bin/python3 /home/tatek/Documents/core/tatekbot/tatekbot_4ws_node/4ws_node.py; exec bash"
