from __future__ import print_function
import os
import time

if os.name == 'nt':
    import msvcrt
    def getch():
        return msvcrt.getch().decode()
    def kbhit():
        return msvcrt.kbhit()
else:
    import termios, fcntl, sys, os
    from select import select
    fd = sys.stdin.fileno()
    old_term = termios.tcgetattr(fd)
    new_term = termios.tcgetattr(fd)

    def getch():
        new_term[3] = (new_term[3] & ~termios.ICANON & ~termios.ECHO)
        termios.tcsetattr(fd, termios.TCSANOW, new_term)
        try:
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_term)
        return ch

    def kbhit():
        new_term[3] = (new_term[3] & ~(termios.ICANON | termios.ECHO))
        termios.tcsetattr(fd, termios.TCSANOW, new_term)
        try:
            dr,dw,de = select([sys.stdin], [], [], 0)
            if dr != []:
                return 1
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_term)
            sys.stdout.flush()

        return 0

from dynamixel_sdk import *                    # Uses Dynamixel SDK library

#********* DYNAMIXEL Model definition *********
#***** (Use only one definition at a time) *****
MY_DXL = 'X_SERIES'       # X330 (5.0 V recommended), X430, X540, 2X430 (Note that XL320 does not support Extended Position Control Mode)

# Control table address
ADDR_TORQUE_ENABLE          = 64
ADDR_GOAL_POSITION          = 116
ADDR_GOAL_VELOCITY          = 104
ADDR_PRESENT_POSITION       = 132
ADDR_OPERATING_MODE         = 11

# Protocol version
PROTOCOL_VERSION            = 2.0            # See which protocol version is used in the Dynamixel

# Factory default ID of all DYNAMIXEL is 1
YAW_ID                      = 1             # XL330 is controlled for Yaw Moving
PITCH_ID                    = 2             # XL430 is controlled for Pitch Moving

# Use the actual port assigned to the U2D2.
# ex) Windows: "COM*", Linux: "/dev/ttyUSB*", Mac: "/dev/tty.usbserial-*"
DEVICENAME                  = '/dev/ttyUSB0'

BAUDRATE                    = 57600
VELOCITY_CONTROL_MODE       = 1                 # Value for Velocity control mode
EXT_POSITION_CONTROL_MODE   = 3                 # Value for Position control mode
EXT_POSITION_CONTROL_MODE   = 4                 # The value of Extended Position Control Mode that can be set through the Operating Mode (11)
TORQUE_ENABLE               = 1                 # Value for enabling the torque
TORQUE_DISABLE              = 0                 # Value for disabling the torque
MAX_POSITION_VALUE          = 1048575           # Of MX with firmware 2.0 and X-Series the revolution on Extended Position Control Mode is 256 rev
DXL_MOVING_STATUS_THRESHOLD = 20                # Dynamixel will rotate between this value

# ASCII for keyboard in python
ESC_ASCII_VALUE             = 0x1b
SPACE_ASCII_VALUE           = 0x20
BACKSPACE_ASCII_VALUE       = 0x08
W_ASCII_VALUE               = 0x57
A_ASCII_VALUE               = 0x41
S_ASCII_VALUE               = 0x53
D_ASCII_VALUE               = 0x44


# Initialize PortHandler instance
# Set the port path
# Get methods and members of PortHandlerLinux or PortHandlerWindows
portHandler = PortHandler(DEVICENAME)

# Initialize PacketHandler instance
# Set the protocol version
# Get methods and members of Protocol1PacketHandler or Protocol2PacketHandler
packetHandler = PacketHandler(PROTOCOL_VERSION)
########################################################################################
# Open port
if portHandler.openPort():
    print("Succeeded to open the port")
else:
    print("Failed to open the port")
    print("Press any key to terminate...")
    getch()
    quit()
########################################################################################
# Set port baudrate
if portHandler.setBaudRate(BAUDRATE):
    print("Succeeded to change the baudrate")
else:
    print("Failed to change the baudrate")
    print("Press any key to terminate...")
    getch()
    quit()
########################################################################################
# Set operating mode to extended position control mode
# Yaw motor
dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, YAW_ID, ADDR_OPERATING_MODE, VELOCITY_CONTROL_MODE)
if dxl_comm_result != COMM_SUCCESS:
    print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
elif dxl_error != 0:
    print("%s" % packetHandler.getRxPacketError(dxl_error))
else:
    print("Operating mode of XL330 changed to velocity control mode.")
# Pitch motor
dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, PITCH_ID, ADDR_OPERATING_MODE, VELOCITY_CONTROL_MODE)
if dxl_comm_result != COMM_SUCCESS:
    print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
elif dxl_error != 0:
    print("%s" % packetHandler.getRxPacketError(dxl_error))
else:
    print("Operating mode of XL430 changed to velocity control mode.")
########################################################################################
# Enable Dynamixel Torque
# Yaw motor
dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, YAW_ID, ADDR_TORQUE_ENABLE, TORQUE_ENABLE)
if dxl_comm_result != COMM_SUCCESS:
    print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
elif dxl_error != 0:
    print("%s" % packetHandler.getRxPacketError(dxl_error))
else:
    print("XL330 has been successfully connected")
# Pitch motor
dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, YAW_ID, ADDR_TORQUE_ENABLE, TORQUE_ENABLE)
if dxl_comm_result != COMM_SUCCESS:
    print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
elif dxl_error != 0:
    print("%s" % packetHandler.getRxPacketError(dxl_error))
else:
    print("XL430 has been successfully connected")
########################################################################################
# Main Fluke loop
while 1:
    print("\nPress any key to continue! (or press ESC to quit!)")
    if getch() == chr(ESC_ASCII_VALUE):
        break
    elif getch() == chr(W_ASCII_VALUE):
        print("W key detected")
    elif getch() == chr(A_ASCII_VALUE):
        print("A key detected")
    elif getch() == chr(S_ASCII_VALUE):
        print("S key detected")
    elif getch() == chr(D_ASCII_VALUE):
        print("D key detected")

    # Write goal position
    dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, YAW_ID, ADDR_GOAL_POSITION, MAX_POSITION_VALUE)
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error))

    while 1:
        # Read present position
        dxl_present_position, dxl_comm_result, dxl_error = packetHandler.read4ByteTxRx(portHandler, YAW_ID, ADDR_PRESENT_POSITION)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % packetHandler.getRxPacketError(dxl_error))

        print("   [ID:%03d] GoalPos:%03d  PresPos:%03d" %(YAW_ID, MAX_POSITION_VALUE, dxl_present_position), end = "\r")
        if kbhit():
            c = getch()
            if c == chr(SPACE_ASCII_VALUE):
                print("\n  Stop & Clear Multi-Turn Information! ")
                # Write the present position to the goal position to stop moving
                dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, YAW_ID, ADDR_GOAL_POSITION, dxl_present_position)
                if dxl_comm_result != COMM_SUCCESS:
                    print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
                elif dxl_error != 0:
                    print("%s" % packetHandler.getRxPacketError(dxl_error))

                time.sleep(0.3)

                # Clear Multi-Turn Information
                dxl_comm_result, dxl_error = packetHandler.clearMultiTurn(portHandler, YAW_ID)
                if dxl_comm_result != COMM_SUCCESS:
                    print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
                elif dxl_error != 0:
                    print("%s" % packetHandler.getRxPacketError(dxl_error))

                # Read present position
                dxl_present_position, dxl_comm_result, dxl_error = packetHandler.read4ByteTxRx(portHandler, YAW_ID, ADDR_PRESENT_POSITION)
                if dxl_comm_result != COMM_SUCCESS:
                    print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
                elif dxl_error != 0:
                    print("%s" % packetHandler.getRxPacketError(dxl_error))

                print("  Present Position has been reset. : %03d" % dxl_present_position)

                break

            elif c == chr(ESC_ASCII_VALUE):
                print("\n  Stopped!!")
                # Write the present position to the goal position to stop moving
                dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, YAW_ID, ADDR_GOAL_POSITION, dxl_present_position)
                if dxl_comm_result != COMM_SUCCESS:
                    print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
                elif dxl_error != 0:
                    print("%s" % packetHandler.getRxPacketError(dxl_error))
                break

        if not abs(MAX_POSITION_VALUE - dxl_present_position) > DXL_MOVING_STATUS_THRESHOLD:
            break

########################################################################################
# Disable Dynamixel Torque
dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, YAW_ID, ADDR_TORQUE_ENABLE, TORQUE_DISABLE)
if dxl_comm_result != COMM_SUCCESS:
    print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
elif dxl_error != 0:
    print("%s" % packetHandler.getRxPacketError(dxl_error))

# Close port
portHandler.closePort()