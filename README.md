# Blue-Line-Following-Robot

MATLAB script for connecting to AgileX LIMO. The computer running the script must be connected to the same Wi-Fi network as the robot, and the LIMO's IP address must be manually modified in the script (port remains unchanged), both of which allow the LIMO to be remotely controlled via TCP.

LIMO speed is capped by maxLinearVel at 0.15 m/s in order to prevent the robot from moving too quickly to detect lines.
