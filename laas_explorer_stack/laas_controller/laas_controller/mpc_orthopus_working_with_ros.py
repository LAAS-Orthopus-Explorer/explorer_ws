#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from rclpy.qos import QoSProfile, QoSDurabilityPolicy
import pinocchio as pin
import tempfile
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

import numpy as np
import yaml
from o2r_pi2_controllers.utils import (ca_rot2quat, 
                          ca_dist_quat, 
                          plot_3D_frame, 
                          rot_float, 
                          ca_quat2rot, 
                          ca_euler2rot, 
                          ca_quat2euler, 
                          ca_rot2euler, 
                          ca_euler2quat, 
                          M32M4,
                          ca_pose2M4x4,
                          list_deg2rad,
                          configclass,
                          hamilton_prod)
from o2r_pi2_controllers.utils_collision import ca_planar_distance_segment_to_segment, ca_planar_distance_point_to_segment, ca_cone_max_radius, ca_distance_cone_to_point, ca_distance_segment_to_segment, ca_distance_point_to_segment, ca_distance_cone_to_segment
import casadi as ca
from casadi import SX, Function, DM
from casadi import (
    vertcat,
    horzcat,
    diagcat,
    transpose,
    if_else,
    diag,
    inv,
    pinv,
    trace,
    norm_2,
    det,
    cos,
    sin,
    tan,
    tanh,
    acos,
    asin,
    atan,
    atan2,
    log10,
    exp,
    dot,
    sqrt,
    fabs,
    cross,
    fmin,
    fmax,
    pi,
    mtimes
)  # maths
try:
    from romea_mobile_base_msgs.msg import TwoAxleSteeringCommand, OneAxleSteeringCommand, SkidSteeringCommand
    from geometry_msgs.msg import Twist
    from std_msgs.msg import Float64MultiArray, Float32, Bool
    from gesture_command.msg import RobStateMM6
except:
    print('ROS msg not found')
from ament_index_python.packages import get_package_share_directory
import pinocchio as pin
import pinocchio.casadi as cpin

from o2r_pi2_controllers.managers.manager_mpc_acados import ManagerMpcAcados, CostTerm, HardConstTerm, SoftConstTerm

from o2r_pi2_controllers.managers import ManagerCasadiModel

@configclass
class ModelCfg:
    name_robot = "MPC_Orthupus_Kinematics"
    x_label = {'q1' : 0,
                'q2' : 1,
                'q3' : 2,
                'q4' : 3,
                'q5' : 4,
                'q6' : 5,    
                'q7' : 6,
                'q8' : 7,
                'q9' : 8,
                'q10' : 9,
                'q11' : 10,  
                'q12' : 11,  

    }
    u_label = {'dq1' : 0,
                'dq2' : 1,
                'dq3' : 2,
                'dq4' : 3,
                'dq5' : 4,
                'dq6' : 5,    
                'dq7' : 6,
                'dq8' : 7,
                'dq9' : 8,
                'dq10' : 9,
                'dq11' : 10,  
                'dq12' : 11,  

    }
   
   
    nq = 12 # dim q | arm angle n
    nx = nq # Only Kinematics
    nu = 12 # dim dq | vel arm angle n
    njoint = 12

    # pin model
    urdf_filename = "/root/my_robot/robot.urdf"
    pin_model = pin.buildModelFromUrdf(urdf_filename) 
    #pin_model = pin.buildModelFromUrdf(urdf_filename, root_joint=pin.JointModelFreeFlyer()) 
    pin_data = pin_model.createData()

    # Run forward kinematics with a random configuration
    q = pin.randomConfiguration(pin_model)
    q = np.zeros(12)
    pin.forwardKinematics(pin_model, pin_data, q)

    # Print the position of each joint frame
    for name, oMi in zip(pin_model.names, pin_data.oMi):
        print("{:<24} : {: .2f} {: .2f} {: .2f}".format(name, *oMi.translation.T.flat))

    pin_model.gravity = pin.Motion.Zero()
    ca_model = cpin.Model(pin_model)
    ca_data = ca_model.createData()

    d_min = pin_model.lowerPositionLimit
    d_max = pin_model.upperPositionLimit

    qd_min = -pin_model.velocityLimit
    qd_max =  pin_model.velocityLimit

    
    print("qd_min:",qd_min)
    print("d_min:",d_min)
    print("d_max:",d_max)

    def _T_effector(self, x):
        cpin.forwardKinematics(self.ca_model, self.ca_data, self.x_state2pin_state(x))
        cpin.updateFramePlacements(self.ca_model, self.ca_data)
        name = "joint_6"
        R = self.ca_data.oMf[self.ca_model.getFrameId(name)].rotation
        t = self.ca_data.oMf[self.ca_model.getFrameId(name)].translation
        T_arm = vertcat(horzcat(R, t), np.array([[0., 0., 0., 1.]]))
        return T_arm

    
    def x_state2pin_state(self,x):
        ca_q = ca.SX(self.ca_model.nq, 1)
        for i in range(self.ca_model.nq):
            ca_q[i] = x[self.x_label['q'+ str(i+1)]]
        return ca_q 
    
    
    def u_state2pin_vel(self,u):
        ca_v = ca.SX(self.ca_model.nv, 1)
        for i in range(self.ca_model.nv):
            ca_v[i] = u[self.x_label['q'+ str(i+1)]]
        return ca_v 
    
    def _derivativ_sym(self, x, u): # skid-steered model
        return vertcat(
            u[0],
            u[1],
            u[2],
            u[3],
            u[4],
            u[5],
            u[6],
            u[7],
            u[8],
            u[9],
            u[10],
            u[11]
            )
    
@configclass
class CostCfg:
    coefs_keys = {'wPos_arm':1, 'qVelBase':12} # 3 Means the dimension of the position
    #coefs_keys = {'wPos_arm':1}

    '''def cost_arm_traj(self, u, x, traj_base, traj_arm, target, obstacles):
        eff_pose = self.T_effector(x)
        e_ee_traj = eff_pose[:3,3] - traj_arm[:3]
        #e_ee_traj = eff_pose[:3,3] - [0.2,0.2,0.3]  #traj_arm[:3]
        e = ca.sumsqr(e_ee_traj)
        print("Trajectory_arm:",  traj_arm[:3])
        print("error:", e_ee_traj)
        print("costo:", ca.sumsqr(e_ee_traj))
        print("Posición del efector con x=0:", eff_pose[:3, 3])
        return e'''
    
    '''# Function implemented by Remi for task space
    def cost_arm_traj(self, u, x, traj_base, traj_arm, target, obstacles):
        eff_pose = self.T_effector(x)
        e_ee_traj = eff_pose[:3, 3] - traj_arm[:3]
        a = 1.  
        e = 1 - exp(-a * norm_2(e_ee_traj)**2)
        return e'''
    
    '''def cost_arm_traj(self, u, x, traj_base, traj_arm, target, obstacles):
        eff_pose = self.T_effector(x)
        e_ee_traj = eff_pose[:3, 3] - traj_arm[:3]
        error_norm = ca.norm_2(e_ee_traj)  
        a = 100000000.0  
        cost = 1 - ca.exp(-a * error_norm**2)
        return cost'''
        
    '''def cost_arm_traj(self, u, x, traj_base, traj_arm, target, obstacles):
        eff_pose = self.T_effector(x)
        e_ee_traj = eff_pose[:3,3] - traj_arm[:3]
        error_norm = ca.norm_2(e_ee_traj) 
        a = 10.0 
        return 1 - ca.exp(-a * error_norm**2)'''
        
    '''def cost_arm_traj(self, u, x, traj_base, traj_arm, target, obstacles):
        e_ee_traj = x[:6] - traj_arm[:6]
        e = ca.sumsqr(e_ee_traj)
        print("joint ref:",  traj_arm[:6])
        print("error:", e_ee_traj)
        print("costo:", ca.sumsqr(e_ee_traj))
        return e'''
    # Function implemented by Remi for joint space given from example folder
    def cost_arm_traj(self, u, x, traj_base, traj_arm, target, obstacles):
        e_ee_traj = x[:12] - traj_arm[:12]
        a = 1.  
        e = 1 - exp(-a * norm_2(e_ee_traj)**2)
        return e
    
    '''def cost_vel_base(self, u, x, traj_base, traj_arm, target, obstacles):
        e_ctrl = ca.sumsqr(u / self.ubu)
        return e_ctrl'''
    
    '''def cost_vel_base(self, u, x, traj_base, traj_arm, target, obstacles):
        a = 1.0 
        e_ctrl = 1 - ca.exp(-a * ca.sumsqr(u / self.ubu))
        return e_ctrl'''
    
    '''def cost_vel_base(self, u, x, traj_base, traj_arm, target, obstacles):
        a = 10.0  
        normalized_u = u / self.ubu
        e_ctrl = 1 - ca.exp(-a * ca.sumsqr(normalized_u))
        return e_ctrl'''
    
    # Function implemented by Remi for command u from example folder
    def cost_vel_base(self, u, x, traj_base, traj_arm, target, obstacles):
        return u / self.ubu
    
    cost_terms = [
        CostTerm(func=cost_arm_traj, weight_key='wPos_arm'),
        CostTerm(func=cost_vel_base, weight_key='qVelBase'),
    ]

@configclass
class ConstCfg:
    # Limite constrains of the joint velocity 
    #lbu = -np.array([10.,10.,10.,10.,10.,10.,2.5,2.5,2.5,2.5,2.5,2.5]) # dq
    #ubu =  np.array([10.,10.,10.,10.,10.,10.,2.5,2.5,2.5,2.5,2.5,2.5]) # dq
    '''lbu = -np.array([5.,5.,5.,5.,5.,5.,2.5,2.5,2.5,2.5,2.5,2.5]) # dq
    ubu =  np.array([5.,5.,5.,5.,5.,5.,2.5,2.5,2.5,2.5,2.5,2.5]) # dq'''

    lbu = -0.7*np.ones(12) # dq
    ubu =  0.7*np.ones(12)

    # Limite constrains of the joint position
    #lbx = -np.array([1000.,1000.,1000.,-1.34,2.2,2.71,3.14,3.57,2.6,3.14,3.57,2.6]) # q
    #ubx = np.array([1000.,1000.,1000.,-1.34,2.2,2.71,3.14,3.57,2.6,3.14,3.57,2.6]) # q

    lbx = np.array([-2.97, -2.14, -2.97, -2.97, -1.8, -2.97, -1.05, -1.05, -1.05, -1.05, -1.05,  0.]) # q
    ubx = np.array([ 2.97, 2.14, 2.97, 2.97, 1.8,  2.97, 0.,   0.,   0.,   0.,   0.,   1.05]) # q

    def collision_point_A(self, u, x, obstacles, target):
        eff_pose = self.T_effector(x)
        A = [2,5,0]
        const_point_A = sqrt((eff_pose[:3,3][0]-A[0])**2 + (eff_pose[:3,3][1]-A[1])**2 + (eff_pose[:3,3][2]-A[2])**2)
        return [const_point_A]
    
    '''const_terms = [
        HardConstTerm(func=collision_point_A, lh=0.5, uh=10e3, lh_e=0.5, uh_e=10e3),
        SoftConstTerm(func=collision_point_A, lh=1., uh=10e3, lh_e=1., uh_e=10e3),
    ]'''
    const_terms = []

@configclass
class OrthopusKinematicsCfg():
    robot_model = ModelCfg()
    #ros_msg = RosCfg()
    cost = CostCfg()
    const = ConstCfg()

class MpcControllerNode(Node):
    def __init__(self):
        super().__init__('mpc_controller_node')

        # Initialize configuration
        cfg = OrthopusKinematicsCfg()
        self.orthopus = ManagerMpcAcados(cfg)

        # MPC
        self.dt = 0.01
        self.horizon = 20

        # Envirement parameters
        nb_obstacles = 0
        self.orthopus.obstacles_len = 7 * nb_obstacles
        self.orthopus.traj_base_len = 7 * nb_obstacles
        self.orthopus.traj_arm_len = 12
        self.orthopus.update_ref_len() # update the length of the parameters vector after obstacles length is set
       
        wdq = 0.
        # Weight cost
        '''self.cost_coefs = {
            'wPos_arm': {'1': 1.0},
            'qVelBase': {'1': 1.0}
        }'''
        self.cost_coefs = {'wPos_arm': {'1': 1.}, 
        'qVelBase': {'1': wdq, '2': wdq, '3': wdq, '4': wdq,'5': wdq, '6': wdq, '7': wdq, '8': wdq,'9': wdq, '10': wdq, '11': wdq, '12': wdq}
        }
        self.publisher_u = self.create_publisher(Float64MultiArray, '/mpc_command', 10)
        self.subscriber = self.create_subscription(JointState, '/joint_states', self.joint_state_callback, 10)

        self.get_logger().info('MPC Node initialized and listening to /joint_states')

        #self.x = np.array([0.1, -0.2, 0.15, -0.1, 0.05, 0.0, -0.2, -0.1, -0.1,  0.0, -0.05, 0.05])  # 12 values
        self.x = np.zeros(self.orthopus.nx)
        self.u = np.zeros(self.orthopus.nu)

        # Integrator
        self.orthopus.create_integrator(self.dt, True)

         # MPC Solver
        self.orthopus.create_ocp_solver(self.x, self.horizon,
                                    np.array([self.dt for i in range(self.horizon)]), 
                                    sum(np.array([self.dt for i in range(self.horizon)])), 
                                    True, True)
        
        self.lates_joint_position = None 

        # Timer periods in seconds 
        self.controller_time_period = 0.005
        self.mpc_time_period = 0.02

        self.create_timer()

    def joint_state_callback(self, msg: JointState):
        if len(msg.position) < self.orthopus.nq:
            self.get_logger().warn('JointState has insufficient positions.')
            return

        #x = np.array(msg.position[:self.orthopus.nq])

        #########
     

        self.orthopus.integrator.set("x", self.x)
        self.orthopus.integrator.set("u", self.u)
        status = self.orthopus.integrator.solve()
        if status != 0:
            print(f"integrator error: status {status} =! 0")

        self.x = self.orthopus.integrator.get("x")
        print("integrated state:", self.x)

        #if new callback value:
        #    self.x = callback
       

        cost_coefs_refs = self.orthopus.extract_cost_coefs(self.cost_coefs)
        r1 = 0.2
        ref = np.hstack((
                cost_coefs_refs, 
                r1*np.zeros((self.orthopus.traj_base_len)), 
                r1*np.ones((self.orthopus.traj_arm_len)), 
                r1*np.zeros((self.orthopus.target_len)), 
                r1*np.zeros((self.orthopus.obstacles_len))
            ))
        print("Ref: ", ref )        
        refs = []
        for j in range(self.horizon+1):
            refs.append(ref)
        print(f'MPC solver reference : {refs}')
        x_mpc, u_mpc, status = self.orthopus.solve_ocp(self.x, self.horizon, refs)
        print("MPC states x_mpc:", x_mpc)
        print("MPC commands:", u_mpc) # list of commands for each step of the MPC solver
        print("MPC status:", status)

        self.u = u_mpc[0]

        # Publish command u
        cmd_msg = Float64MultiArray()
        cmd_msg.data = u_mpc[0].tolist()
        self.publisher_u.publish(cmd_msg)
        self.get_logger().info(f"Published MPC command u[0]: {u_mpc[0]}")

    ########

      


def main(args=None):
    rclpy.init(args=args)
    node = MpcControllerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
   
