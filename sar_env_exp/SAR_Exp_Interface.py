import threading
import os
import sys
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter # Added
import yaml
import numpy as np
from ament_index_python.packages import get_package_share_directory, PackageNotFoundError

sys.path.append('/home/dlee/ros2_ws/src/sar_simulation')
from sar_env.SAR_Base_Interface import SAR_Base_Interface

sys.path.append('/home/dlee/ros2_ws/src/sar_experiment')
crazyswarm_path = sys.path.append('/home/dlee/ros2_ws/src/sar_experiment/crazyswarm2')
from crazyswarm2.crazyflie_py.crazyflie_py import Crazyswarm

np.set_printoptions(suppress=True)

class SAR_Exp_Interface(SAR_Base_Interface):

    def __init__(self):
        super().__init__()
        self.node = self

        print("[STARTING] SAR_Exp is starting...")
        SAR_Base_Interface.__init__(self,Experiment_Setup=True)

        self.EXP_PATH = sys.path.append('/home/dlee/ros2_ws/src/sar_experiment/sar_env_exp')
        self.EXP_PATH = '/home/dlee/ros2_ws/src/sar_experiment'
        #self.EXP_PATH = os.path.dirname(get_package_share_directory('sar_env_exp'))
        self.loadBaseParams()
        self.loadExpParams()
        
        self.cf_swarm = Crazyswarm()
        self.cf = self.cf_swarm.allcfs.crazyflies[0]
        self.timeHelper = self.cf_swarm.timeHelper
        
        ## SAR PARAMETERS
        self.Done = False
        self.setParams()
        self.Log_Dir =  f"{self.EXP_PATH}/sar_logging_exp/local_logs"
        
        #print("self.SAR_Type",self.SAR_Type)
        #print("self.Policy_Type",self.Policy_Type)

    def setParams(self):
        ## SetParam function is in crazyflie.py
        #print(self.cf.paramTypeDict.keys())
        self.cf.setParam("stabilizer.controller", 5) # Set firmware controller to GTC

        ## SET SAR TYPE
        if self.SAR_Type == "Crazyflie":
            self.cf.setParam("P1.SAR_Type",1)
        elif self.SAR_Type == "Impulse_Micro":
            self.cf.setParam("P1.SAR_Type",2)
        elif self.SAR_Type == "Source_One_V5":
            self.cf.setParam("P1.SAR_Type",3)
        else:
            self.cf.setParam("P1.SAR_Type",0)


        ## SET EXP SETTINGS
        if self.Policy_Type == "PARAM_OPTIM":
            self.cf.setParam("P1.PolicyType",0)

        elif self.Policy_Type == "DEEP_RL_SB3":
            self.cf.setParam("P1.PolicyType",1)

        elif self.Policy_Type == "DEEP_RL_ONBOARD":
            self.cf.setParam("P1.PolicyType",2)

        ## SET CONTROLLER INERTIA VALUES
        self.cf.setParam("P1.Mass",self.Ref_Mass)
        self.cf.setParam("P1.I_xx",self.Ref_Ixx)
        self.cf.setParam("P1.I_yy",self.Ref_Iyy)
        self.cf.setParam("P1.Izz",self.Ref_Izz)
        self.cf.setParam("P1.L_eff",self.L_eff)

        self.cf.setParam("P1.Prop_14_x",self.Prop_Front[0])
        self.cf.setParam("P1.Prop_14_y",self.Prop_Front[1])
        self.cf.setParam("P1.Prop_23_x",self.Prop_Rear[0])
        self.cf.setParam("P1.Prop_23_y",self.Prop_Rear[1])
        self.cf.setParam("P1.C_tf",self.C_tf)
        self.cf.setParam("P1.Tust_max",self.Thrust_max)
        self.cf.setParam("P1.Fwd_Reach",self.Forward_Reach)


        # ## SET CONTROLLER INERTIA VALUES
        # InertiaParamDict = {
        #     "P1/Mass":               self.Ref_Mass,
        #     "P1/I_xx":                self.Ref_Ixx, 
        #     "P1/I_yy":                self.Ref_Iyy,
        #     "P1/Izz":                self.Ref_Izz,
        #     "P1/L_eff":              self.L_eff,
        # }
        # self.cf.setParams(InertiaParamDict)

        # SystemParamDict = {
        #     "P1/Prop_14_x":      self.Prop_Front[0], 
        #     "P1/Prop_14_y":      self.Prop_Front[1], 
        #     "P1/Prop_23_x":      self.Prop_Rear[0],
        #     "P1/Prop_23_y":      self.Prop_Rear[1],
        #     "P1/C_tf":           self.C_tf,
        #     "P1/Tust_max":     self.Thrust_max,
        #     "P1/Fwd_Reach":      self.Forward_Reach,

        # }
        # self.cf.setParams(SystemParamDict)
        
        self.cf.setParam("P2.P_kp_xy",self.P_kp_xy)
        self.cf.setParam("P2.P_kd_xy",self.P_kd_xy)
        self.cf.setParam("P2.P_ki_xy",self.P_ki_xy)
        self.cf.setParam("P2.i_range_xy",self.i_range_xy)

        self.cf.setParam("P2.P_kp_z",self.P_kp_z)
        self.cf.setParam("P2.P_Kd_z",self.P_kd_z)
        self.cf.setParam("P2.P_ki_z",self.P_ki_z)
        self.cf.setParam("P2.i_range_Z",self.i_range_z)

        self.cf.setParam("P2.R_kp_xy",self.R_kp_xy)
        self.cf.setParam("P2.R_kd_xy",self.R_kd_xy)
        self.cf.setParam("P2.R_ki_xy",self.R_ki_xy)
        self.cf.setParam("P2.i_range_xy",self.i_range_R_xy)

        self.cf.setParam("P2.R_kpz",self.R_kp_z)
        self.cf.setParam("P2.R_kdz",self.R_kd_z)
        self.cf.setParam("P2.R_ki_z",self.R_ki_z)
        self.cf.setParam("P2.i_range_R_z",self.i_range_R_z)



        # ## SET CONTROLLER GAIN VALUES
        # temp_str = f"/SAR_Type/{self.SAR_Type}/CtrlGains"
        # GainsDict = {
        #     "P2/P_kp_xy":      rospy.get_param(f"{temp_str}/P_kp_xy"),
        #     "P2/P_kd_xy":      rospy.get_param(f"{temp_str}/P_kd_xy"), 
        #     "P2/P_ki_xy":      rospy.get_param(f"{temp_str}/P_ki_xy"),
        #     "P2/i_range_xy":   rospy.get_param(f"{temp_str}/i_range_xy"),

        #     "P2/P_kp_z":       rospy.get_param(f"{temp_str}/P_kp_z"),        
        #     "P2/P_Kd_z":       rospy.get_param(f"{temp_str}/P_kd_z"),
        #     "P2/P_ki_z":       rospy.get_param(f"{temp_str}/P_ki_z"),
        #     "P2/i_range_z":    rospy.get_param(f"{temp_str}/i_range_z"),
        # }
        # self.cf.setParams(GainsDict)
    
        # GainsDict2 = {
        #     "P2/R_kp_xy":      rospy.get_param(f"{temp_str}/R_kp_xy"),
        #     "P2/R_kd_xy":      rospy.get_param(f"{temp_str}/R_kd_xy"),
        #     "P2/R_ki_xy":      rospy.get_param(f"{temp_str}/R_ki_xy"),
        #     "P2/i_range_xy":   rospy.get_param(f"{temp_str}/i_range_R_xy"),


        #     "P2/R_kpz":       rospy.get_param(f"{temp_str}/R_kp_z"),
        #     "P2/R_kdz":       rospy.get_param(f"{temp_str}/R_kd_z"),
        #     "P2/R_ki_z":       rospy.get_param(f"{temp_str}/R_ki_z"),
        #     "P2/i_range_R_z":  rospy.get_param(f"{temp_str}/i_range_R_z"),
        # }
        # self.cf.setParams(GainsDict2)
        
        ## SET SYSTEM GEOMETRY PARAMETERS INERTIA VALUES
        
        #print("SAR_Type", self.SAR_Type)
        #print("Plane_Pos_x_init", self.Plane_Pos_x_init)
        #print("Plane_Pos_y_init", self.Plane_Pos_y_init)
        #print("Plane_Pos_z_init", self.Plane_Pos_z_init)
        #print("Plane_Angle_deg_init", self.Plane_Angle_deg_init)

        # print("setParams in SAR_Exp_Interface.py is completed")
        
        #self.sendCmd("Plane_Pose",cmd_vals=[self.Plane_Pos_x_init,self.Plane_Pos_y_init,self.Plane_Pos_z_init],cmd_flag=self.Plane_Angle_deg_init)

        #self.sendCmd("Load_Params")
        #self.sendCmd("Ctrl_Reset")

    def loadExpParams(self):
        ## LOAD BASE PARAMETERS
        param_path = f"{self.EXP_PATH}/sar_config_exp/Exp_Settings.yaml"
                    
        with open(param_path, 'r') as file:
            self.exp_parameters = yaml.safe_load(file)

        parameters_to_set_SAR_SETTINGS = []
        for key, value in self.exp_parameters['SAR_SETTINGS'].items():
            if isinstance(value, bool):
                param_type = Parameter.Type.BOOL
            elif isinstance(value, int):
                param_type = Parameter.Type.INTEGER
            elif isinstance(value, float):
                param_type = Parameter.Type.DOUBLE
            elif isinstance(value, str):
                param_type = Parameter.Type.STRING
            else:
                continue  # Handle other types as needed

            parameters_to_set_SAR_SETTINGS.append(Parameter(f'SAR_SETTINGS.{key}', param_type, value))
        
        self.set_parameters(parameters_to_set_SAR_SETTINGS)

        parameters_to_set_CAM_SETTINGS = []
        for key, value in self.exp_parameters['CAM_SETTINGS'].items():
            if isinstance(value, bool):
                param_type = Parameter.Type.BOOL
            elif isinstance(value, int):
                param_type = Parameter.Type.INTEGER
            elif isinstance(value, float):
                param_type = Parameter.Type.DOUBLE
            elif isinstance(value, str):
                param_type = Parameter.Type.STRING
            else:
                continue  # Handle other types as needed

            parameters_to_set_CAM_SETTINGS.append(Parameter(f'CAM_SETTINGS.{key}', param_type, value))
        
        self.set_parameters(parameters_to_set_CAM_SETTINGS)

        parameters_to_set_PLANE_SETTINGS = []
        for key, value in self.exp_parameters['PLANE_SETTINGS'].items():
            if isinstance(value, bool):
                param_type = Parameter.Type.BOOL
            elif isinstance(value, int):
                param_type = Parameter.Type.INTEGER
            elif isinstance(value, float):
                param_type = Parameter.Type.DOUBLE
            elif isinstance(value, str):
                param_type = Parameter.Type.STRING
            else:
                continue  # Handle other types as needed

            parameters_to_set_PLANE_SETTINGS.append(Parameter(f'PLANE_SETTINGS.{key}', param_type, value))
        
        self.set_parameters(parameters_to_set_PLANE_SETTINGS)

        self.SAR_Type = self.get_parameter('SAR_SETTINGS.SAR_Type').get_parameter_value().string_value
        self.SAR_Config = self.get_parameter('SAR_SETTINGS.SAR_Config').get_parameter_value().string_value
        self.Policy_Type = self.get_parameter('SAR_SETTINGS.Policy_Type').get_parameter_value().string_value
        
        # self.get_logger().info(f'SAR_Type: {self.SAR_Type}')
        # self.get_logger().info(f'SAR_Config: {self.SAR_Config}')
        # self.get_logger().info(f'Policy_Type: {self.Policy_Type}')

        ## INERTIAL PARAMETERS
        self.Ref_Mass = self.get_parameter(f"SAR_Type.{self.SAR_Type}.Config.{self.SAR_Config}.Ref_Mass").get_parameter_value().double_value
        self.Ref_Ixx = self.get_parameter(f"SAR_Type.{self.SAR_Type}.Config.{self.SAR_Config}.Ref_Ixx").get_parameter_value().double_value
        self.Ref_Iyy = self.get_parameter(f"SAR_Type.{self.SAR_Type}.Config.{self.SAR_Config}.Ref_Iyy").get_parameter_value().double_value
        self.Ref_Izz = self.get_parameter(f"SAR_Type.{self.SAR_Type}.Config.{self.SAR_Config}.Ref_Izz").get_parameter_value().double_value
        
        # self.get_logger().info(f'Ref_Mass: {self.Ref_Mass}')
        # self.get_logger().info(f'Ref_Ixx: {self.Ref_Ixx}')
        # self.get_logger().info(f'Ref_Iyy: {self.Ref_Iyy}')
        # self.get_logger().info(f'Ref_Izz: {self.Ref_Izz}')

        self.Base_Mass = self.get_parameter(f"SAR_Type.{self.SAR_Type}.System_Params.Base_Mass").get_parameter_value().double_value
        self.Base_Ixx = self.get_parameter(f"SAR_Type.{self.SAR_Type}.System_Params.Base_Ixx").get_parameter_value().double_value
        self.Base_Iyy = self.get_parameter(f"SAR_Type.{self.SAR_Type}.System_Params.Base_Iyy").get_parameter_value().double_value
        self.Base_Izz = self.get_parameter(f"SAR_Type.{self.SAR_Type}.System_Params.Base_Izz").get_parameter_value().double_value
        
        # self.get_logger().info(f'Base_Mass: {self.Base_Mass}')
        # self.get_logger().info(f'Base_Ixx: {self.Base_Ixx}')
        # self.get_logger().info(f'Base_Iyy: {self.Base_Iyy}')
        # self.get_logger().info(f'Base_Izz: {self.Base_Izz}')

        ## GEOMETRIC PARAMETERS
        self.Forward_Reach = self.get_parameter(f"SAR_Type.{self.SAR_Type}.System_Params.Forward_Reach").get_parameter_value().double_value
        self.Leg_Length = self.get_parameter(f"SAR_Type.{self.SAR_Type}.Config.{self.SAR_Config}.Leg_Length").get_parameter_value().double_value
        self.Leg_Angle = self.get_parameter(f"SAR_Type.{self.SAR_Type}.Config.{self.SAR_Config}.Leg_Angle").get_parameter_value().double_value
        self.Prop_Front = self.get_parameter(f"SAR_Type.{self.SAR_Type}.System_Params.Prop_Front").get_parameter_value().double_array_value
        self.Prop_Rear = self.get_parameter(f"SAR_Type.{self.SAR_Type}.System_Params.Prop_Rear").get_parameter_value().double_array_value

        # self.get_logger().info(f'Forward_Reach: {self.Forward_Reach}')
        # self.get_logger().info(f'Leg_Length: {self.Leg_Length}')
        # self.get_logger().info(f'Leg_Angle: {self.Leg_Angle}')
        # self.get_logger().info(f'Prop_Front: {self.Prop_Front}')
        # self.get_logger().info(f'Prop_Rear: {self.Prop_Rear}')

        ## EFFECTIVE-GEOEMTRIC PARAMETERS
        self.L_eff = self.get_parameter(f"SAR_Type.{self.SAR_Type}.Config.{self.SAR_Config}.L_eff").get_parameter_value().double_value
        self.Gamma_eff = self.get_parameter(f"SAR_Type.{self.SAR_Type}.Config.{self.SAR_Config}.Gamma_eff").get_parameter_value().double_value
        self.Lx_eff = self.L_eff*np.sin(np.radians(self.Gamma_eff))
        self.Lz_eff = self.L_eff*np.cos(np.radians(self.Gamma_eff))
        self.Collision_Radius = self.L_eff

        # self.get_logger().info(f'L_eff: {self.L_eff}')
        # self.get_logger().info(f'Gamma_eff: {self.Gamma_eff}')

        ## SYSTEM AND FLIGHT PARAMETERS
        self.TrajAcc_Max = self.get_parameter(f"SAR_Type.{self.SAR_Type}.System_Params.TrajAcc_Max").get_parameter_value().double_array_value
        self.TrajJerk_Max = self.get_parameter(f"SAR_Type.{self.SAR_Type}.System_Params.TrajJerk_Max").get_parameter_value().double_array_value
        self.Tau_up = self.get_parameter(f"SAR_Type.{self.SAR_Type}.System_Params.Tau_up").get_parameter_value().double_value
        self.Tau_down = self.get_parameter(f"SAR_Type.{self.SAR_Type}.System_Params.Tau_down").get_parameter_value().double_value
        self.Thrust_max = self.get_parameter(f"SAR_Type.{self.SAR_Type}.System_Params.Thrust_max").get_parameter_value().double_value
        self.C_tf = self.get_parameter(f"SAR_Type.{self.SAR_Type}.System_Params.C_tf").get_parameter_value().double_value
        self.Ang_Acc_max = (9.81*self.Thrust_max*1e-3*self.Prop_Front[0])*2/self.Ref_Iyy
        self.setAngAcc_range([-self.Ang_Acc_max, self.Ang_Acc_max])
        
        self.Beta_Min_deg = -(self.Gamma_eff + np.degrees(np.arctan2(self.Forward_Reach-self.Lx_eff,self.Lz_eff)))
        self.Phi_P_B_impact_Min_deg = -self.Beta_Min_deg - self.Gamma_eff + 90

        # self.get_logger().info(f'TrajAcc_Max: {self.TrajAcc_Max}')
        # self.get_logger().info(f'TrajJerk_Max: {self.TrajJerk_Max}')
        # self.get_logger().info(f'Tau_up: {self.Tau_up}')
        # self.get_logger().info(f'Tau_down: {self.Tau_down}')
        # self.get_logger().info(f'Thrust_max: {self.Thrust_max}')
        # self.get_logger().info(f'C_tf: {self.C_tf}')

        ## CAM PARAMETERS
        self.Cam_Config = self.get_parameter(f"CAM_SETTINGS.Cam_Config").get_parameter_value().string_value
        self.Cam_Active = self.get_parameter(f"CAM_SETTINGS.Cam_Active").get_parameter_value().bool_value
        
        # self.get_logger().info(f'Cam_Config: {self.Cam_Config}')
        # self.get_logger().info(f'Cam_Active: {self.Cam_Active}')

        ## PLANE PARAMETERS
        self.Plane_Type = self.get_parameter(f"PLANE_SETTINGS.Plane_Type").get_parameter_value().string_value
        self.Plane_Config = self.get_parameter(f"PLANE_SETTINGS.Plane_Config").get_parameter_value().string_value
        self.Plane_Pos_x_init = self.get_parameter(f"PLANE_SETTINGS.Pos_X_init").get_parameter_value().double_value
        self.Plane_Pos_y_init = self.get_parameter(f"PLANE_SETTINGS.Pos_Y_init").get_parameter_value().double_value
        self.Plane_Pos_z_init = self.get_parameter(f"PLANE_SETTINGS.Pos_Z_init").get_parameter_value().double_value
        self.Plane_Angle_deg_init = self.get_parameter(f"PLANE_SETTINGS.Plane_Angle_init").get_parameter_value().integer_value

        # self.get_logger().info(f'Plane_Type: {self.Plane_Type}')
        # self.get_logger().info(f'Plane_Config: {self.Plane_Config}')
        # self.get_logger().info(f'Plane_Pos_x_init: {self.Plane_Pos_x_init}')
        # self.get_logger().info(f'Plane_Pos_y_init: {self.Plane_Pos_y_init}')
        # self.get_logger().info(f'Plane_Pos_z_init: {self.Plane_Pos_z_init}')
        # self.get_logger().info(f'Plane_Angle_deg_init: {self.Plane_Angle_deg_init}')

        ## CONTROLLER GAIN VALUES
        self.P_kp_xy = self.get_parameter(f"SAR_Type.{self.SAR_Type}.CtrlGains.P_kp_xy").get_parameter_value().double_value
        self.P_kd_xy = self.get_parameter(f"SAR_Type.{self.SAR_Type}.CtrlGains.P_kd_xy").get_parameter_value().double_value
        self.P_ki_xy = self.get_parameter(f"SAR_Type.{self.SAR_Type}.CtrlGains.P_ki_xy").get_parameter_value().double_value
        self.i_range_xy = self.get_parameter(f"SAR_Type.{self.SAR_Type}.CtrlGains.i_range_xy").get_parameter_value().double_value

        # self.get_logger().info(f'P_kp_xy: {self.P_kp_xy}')
        # self.get_logger().info(f'P_kd_xy: {self.P_kd_xy}')
        # self.get_logger().info(f'P_ki_xy: {self.P_ki_xy}')
        # self.get_logger().info(f'i_range_xy: {self.i_range_xy}')

        self.P_kp_z = self.get_parameter(f"SAR_Type.{self.SAR_Type}.CtrlGains.P_kp_z").get_parameter_value().double_value
        self.P_kd_z = self.get_parameter(f"SAR_Type.{self.SAR_Type}.CtrlGains.P_kd_z").get_parameter_value().double_value
        self.P_ki_z = self.get_parameter(f"SAR_Type.{self.SAR_Type}.CtrlGains.P_ki_z").get_parameter_value().double_value
        self.i_range_z = self.get_parameter(f"SAR_Type.{self.SAR_Type}.CtrlGains.i_range_z").get_parameter_value().double_value

        # self.get_logger().info(f'P_kp_z: {self.P_kp_z}')
        # self.get_logger().info(f'P_kd_z: {self.P_kd_z}')
        # self.get_logger().info(f'P_ki_z: {self.P_ki_z}')
        # self.get_logger().info(f'i_range_z: {self.i_range_z}')

        self.R_kp_xy = self.get_parameter(f"SAR_Type.{self.SAR_Type}.CtrlGains.R_kp_xy").get_parameter_value().double_value
        self.R_kd_xy = self.get_parameter(f"SAR_Type.{self.SAR_Type}.CtrlGains.R_kd_xy").get_parameter_value().double_value
        self.R_ki_xy = self.get_parameter(f"SAR_Type.{self.SAR_Type}.CtrlGains.R_ki_xy").get_parameter_value().double_value
        self.i_range_R_xy = self.get_parameter(f"SAR_Type.{self.SAR_Type}.CtrlGains.i_range_R_xy").get_parameter_value().double_value

        # self.get_logger().info(f'R_kp_xy: {self.R_kp_xy}')
        # self.get_logger().info(f'R_kd_xy: {self.R_kd_xy}')
        # self.get_logger().info(f'R_ki_xy: {self.R_ki_xy}')
        # self.get_logger().info(f'i_range_R_xy: {self.i_range_R_xy}')

        self.R_kp_z = self.get_parameter(f"SAR_Type.{self.SAR_Type}.CtrlGains.R_kp_z").get_parameter_value().double_value
        self.R_kd_z = self.get_parameter(f"SAR_Type.{self.SAR_Type}.CtrlGains.R_kd_z").get_parameter_value().double_value
        self.R_ki_z = self.get_parameter(f"SAR_Type.{self.SAR_Type}.CtrlGains.R_ki_z").get_parameter_value().double_value
        self.i_range_R_z = self.get_parameter(f"SAR_Type.{self.SAR_Type}.CtrlGains.i_range_R_z").get_parameter_value().double_value

        # self.get_logger().info(f'R_kp_z: {self.R_kp_z}')
        # self.get_logger().info(f'R_kd_z: {self.R_kd_z}')
        # self.get_logger().info(f'R_ki_z: {self.R_ki_z}')
        # self.get_logger().info(f'i_range_R_z: {self.i_range_R_z}')



    def spin(self):
        rclpy.spin(self.node)
    
    def destroy(self):
        self.node.destroy_node()

def main(args=None):
    rclpy.init(args=args)
    env = SAR_Exp_Interface()

    try:
        env.spin()
    except KeyboardInterrupt:
        pass
    finally:
        env.destroy()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
    