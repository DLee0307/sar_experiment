#!/usr/bin/env python3

import threading
import os
import sys
import rclpy
from rclpy.node import Node
import numpy as np

sys.path.append('/home/dlee/ros2_ws/src/sar_experiment')
from sar_env_exp.SAR_Exp_Interface import SAR_Exp_Interface

def cmd_send_exp(env):
    

    while True:
        # Converts input number into action name
        command_handlers = {
            0: env.handle_Ctrl_Reset,
            #1: env.handle_Pos_Cmd,
            #2: env.handle_Vel_Cmd,
            5: env.handle_Stop,
            #7: env.handle_Ang_Accel,
            #8: env.handle_Policy,
            9: env.handle_Plane_Pose,
            10: env.handle_P2P_traj,
            11: env.handle_Global_Vel_traj,
            #12: env.handle_Rel_Vel_traj,
            13: env.handle_Impact_traj,
            20: env.handle_Tumble_Detect,
            21: env.handle_Load_Params,
            #22: env.handle_Start_Logging,
            #23: env.handle_Cap_Logging,
            24: env.handle_Arm_Quad,
            #30: env.handle_Thrust_CMD,
            #31: env.handle_Motor_CMD,
        }
        
        try:
            print("========== Command Types ==========")
            print("0: Ctrl_Reset  7: Ang_Accel    10: P2P_traj          20: Tumble_Detect    24: Arm_Quad  ")
            print("1: Pos         8: Policy       11: Global_Vel_traj   21: Load_Params      30: Thrust_CMD")
            print("2: Vel         9: Plane_Pose   12: Rel_Vel_traj      22: Start_Logging    31: Motor_CMD ")
            print("5: Stop                        13: Impact_traj       23: Cap_Logging")

            cmd = env.userInput("\nCmd: ", int)
            if cmd in command_handlers:
                command_handlers[cmd]()
            else:
                print("Invalid Command: Try again")
        
        except Exception as e:
            print(f"Error: {e}")
            print('\033[93m' + "INVALID INPUT: Try again" + '\x1b[ 0m')
            continue

def main(args=None):
    rclpy.init(args=args)
    
    ## INIT GAZEBO ENVIRONMENT
    env = SAR_Exp_Interface()
    #env.sendCmd("Plane_Pose",cmd_vals=[env.Plane_Pos_x_init,env.Plane_Pos_y_init,env.Plane_Pos_z_init],cmd_flag=env.Plane_Angle_deg_init)
    
    # ## INITIALIZE CRAZYFLIE
    env.cf.setParam("kalman.resetEstimation", 1)
    # time.sleep(0.2)
    # now = datetime.datetime.now()
    # current_time = now.strftime("%m_%d-%H:%M")


    # ## INITIALIALIZE LOGGING DATA
    # Type = "IM"
    # Config = "B2"
    # Angle = int(0)

    # V_mag = env.userInput("Enter Velocity Magnitude: ", float)
    # V_angle = env.userInput("Enter Velocity Angle: ", int)
    # Trial_num = env.userInput("Enter Trial Number: ", int)
    # K_eff = env.userInput("Enter K_eff: ", float)

    # env.Log_Name = f"{Type}_{Config}_V{V_mag}_A{V_angle}_PA{Angle}_K{K_eff}_T{Trial_num}_{current_time}.csv"
    
    # str_input = input("Approve Log Name: " + env.Log_Name + "\nPress Enter to Continue")
    # if str_input == "q":
    #     print("Invalid Input: Exiting Program")
    #     sys.exit()
    # else:
    #     env.createCSV()

    cmd_thread = threading.Thread(target=cmd_send_exp, args=(env,))
    cmd_thread.start()    
    
    try:
        rclpy.spin(env.node)
    except KeyboardInterrupt:
        pass
    finally:
        cmd_thread.join()
        env.destroy()
        rclpy.shutdown()

if __name__ == '__main__':
    main()