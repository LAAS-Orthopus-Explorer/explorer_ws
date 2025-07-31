#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from rclpy.qos import QoSProfile, QoSDurabilityPolicy
import pinocchio as pin
import tempfile

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

'''class URDFPinocchioNode(Node):
    def __init__(self):
        super().__init__("urdf_pinocchio_node")
        self.get_logger().info("Waiting for URDF on topic /robot_description...")

        # Create QoS with transient local durability to receive already published messages
        qos_profile = QoSProfile(
            depth=1,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL
        )

        self.subscription = self.create_subscription(
            String,
            "/robot_description",
            self.urdf_callback,
            qos_profile
        )

    def urdf_callback(self, msg):
        self.get_logger().info("URDF received. Saving and loading with Pinocchio...")

        urdf_string = msg.data

        try:
            # Save the URDF to a temporary file
            with tempfile.NamedTemporaryFile(mode='w', delete=False, suffix=".urdf") as temp_file:
                temp_file.write(urdf_string)
                urdf_filename = temp_file.name

            self.get_logger().info(f"URDF saved to: {urdf_filename}")

            # Load the model with Pinocchio using free-flyer base
            model = pin.buildModelFromUrdf(urdf_filename, pin.JointModelFreeFlyer())
            data = model.createData()

            self.get_logger().info(f"Pinocchio model loaded: {model.name}")

            # Run forward kinematics with a random configuration
            q = pin.randomConfiguration(model)
            pin.forwardKinematics(model, data, q)

            # Print the position of each joint frame
            for name, oMi in zip(model.names, data.oMi):
                print("{:<24} : {: .2f} {: .2f} {: .2f}".format(name, *oMi.translation.T.flat))

            self.get_logger().info("Forward kinematics completed successfully.")
            # Optionally unsubscribe to stop processing more messages
            self.destroy_subscription(self.subscription)

        except Exception as e:
            self.get_logger().error(f"Failed to load URDF or compute kinematics: {e}")'''

@configclass
class ModelCfg:
    name = "campero_dynamic"
    x_label = {'x' : 0,
                'y' : 1,
                'yaw' : 2,
                'q1' : 3,
                'q2' : 4,
                'q3' : 5,
                'q4' : 6,
                'q5' : 7,
                'q6' : 8,

                'vx' : 9,
                'wz' : 10,
                'w1' : 11,
                'w2' : 12,
                'w3' : 13,
                'w4' : 14,
                'w5' : 15,
                'w6' : 16,
    }
    u_label = {'fx' : 0,
                'tauz' : 1,
                'tau1' : 2,
                'tau2' : 3,
                'tau3' : 4,
                'tau4' : 5,
                'tau5' : 6,
                'tau6' : 7,
    }
   
    '''nq = 9 # static |x,y, yaw, arm angle n
    nv = 8 # velocity |v,v_ang, v arm angle n
    nx = nq + nv
    nu = 8 # tau |F,tau_ang, tau arm angle n
    njoint = 6'''

    nq = 15 # static |x,y, yaw, arm angle n
    nv = 14 # velocity |v,v_ang, v arm angle n
    nx = nq + nv
    nu = 14 # tau |F,tau_ang, tau arm angle n
    njoint = 12

    # pin model
    urdf_filename = "/root/my_robot/robot.urdf"
    pin_model = pin.buildModelFromUrdf(urdf_filename, root_joint=pin.JointModelFreeFlyer()) # For continuous joints, Pinocchio uses the complex representation of SO(2). The 2 elements are thus cos(theta), sin(theta).
    pin_data = pin_model.createData()

    # Run forward kinematics with a random configuration
    q = pin.randomConfiguration(pin_model)
    pin.forwardKinematics(pin_model, pin_data, q)

    # Print the position of each joint frame
    for name, oMi in zip(pin_model.names, pin_data.oMi):
        print("{:<24} : {: .2f} {: .2f} {: .2f}".format(name, *oMi.translation.T.flat))

    pin_model.gravity = pin.Motion.Zero()
    ca_model = cpin.Model(pin_model)
    ca_data = ca_model.createData()

    

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
        ca_q[0] =  x[self.x_label['x']]
        ca_q[1] =  x[self.x_label['y']]
        quat = ca_euler2quat([0.,0., x[self.x_label['yaw']]])
        ca_q[3] =  quat[1]
        ca_q[4] =  quat[2]
        ca_q[5] =  quat[3]
        ca_q[6] =  quat[0]
        ca_q[15] = x[self.x_label['q1']]
        ca_q[16] = x[self.x_label['q2']]
        ca_q[17] = x[self.x_label['q3']]
        ca_q[18] = x[self.x_label['q4']]
        ca_q[18] = x[self.x_label['q5']]
        ca_q[18] = x[self.x_label['q6']]
        return ca_q 
    
    def x_state2pin_v(self,x):
        ca_vq = ca.SX(self.ca_model.nv, 1)
        ca_vq[0] =  x[self.x_label['vx']]
        ca_vq[5] =  x[self.x_label['wz']]
        ca_vq[10] = x[self.x_label['w1']]
        ca_vq[11] = x[self.x_label['w2']]
        ca_vq[12] = x[self.x_label['w3']]
        ca_vq[13] = x[self.x_label['w4']]
        ca_vq[14] = x[self.x_label['w5']]
        ca_vq[15] = x[self.x_label['w6']]
        return ca_vq
    
    def u2pin_tau(self,u):
        ca_tau = ca.SX(self.ca_model.nv, 1)
        ca_tau[0] =  u[self.u_label['fx']]
        ca_tau[5] =  u[self.u_label['tauz']]
        ca_tau[10] = u[self.u_label['tau1']]
        ca_tau[11] = u[self.u_label['tau2']]
        ca_tau[12] = u[self.u_label['tau3']]
        ca_tau[13] = u[self.u_label['tau4']]
        ca_tau[14] = u[self.u_label['tau5']]
        ca_tau[15] = u[self.u_label['tau6']]
        return ca_tau
    
    def _derivativ_sym(self, x, u):                                               # too slow
        k = SX.sym("k", x.shape[0])
        k[self.x_label['x']] =  x[self.x_label['vx']] * cos(x[self.x_label['yaw']])
        k[self.x_label['y']] =  x[self.x_label['vx']] * sin(x[self.x_label['yaw']])
        k[self.x_label['yaw']] = x[self.x_label['wz']]
        k[self.x_label['q1']] = x[self.x_label['w1']]
        k[self.x_label['q2']] = x[self.x_label['w2']]
        k[self.x_label['q3']] = x[self.x_label['w3']]
        k[self.x_label['q4']] = x[self.x_label['w4']]
        k[self.x_label['q5']] = x[self.x_label['w5']]
        k[self.x_label['q6']] = x[self.x_label['w6']]

        k[self.x_label['vx']] = u[self.u_label['fx']]
        k[self.x_label['wz']] = u[self.u_label['tauz']]
        k[self.x_label['w1']] = u[self.u_label['tau1']]
        k[self.x_label['w2']] = u[self.u_label['tau2']]
        k[self.x_label['w3']] = u[self.u_label['tau3']]
        k[self.x_label['w4']] = u[self.u_label['tau4']]
        k[self.x_label['w5']] = u[self.u_label['tau5']]
        k[self.x_label['w6']] = u[self.u_label['tau6']]
        return k

    def init_state(self):
        x = np.zeros(self.nx)
        x[self.nq-self.njoint:self.nq] = self.mm_arm_init_pose
        return x
'''
@configclass
class RosCfg:
    def cmd_base(self,x,u,simu):
        if simu == 'True' :
            cmd = SkidSteeringCommand()
            cmd.longitudinal_speed = x[self.x_label['vx']]
            cmd.angular_speed      = x[self.x_label['wz']]
        else:
            cmd = Twist()
            cmd.linear.x =  x[self.x_label['vx']]
            cmd.angular.x = x[self.x_label['wz']]
            cmd.angular.y = x[self.x_label['wz']]
            cmd.angular.z = x[self.x_label['wz']]
        return cmd

    def cmd_arm_pos(self,x,u,horizon_steps,simu):
        arm_command_msg = Float64MultiArray()
        start_idx_pos = self.nq - self.njoint
        start_idx_vel = self.nx - self.njoint

        arm_command_msg.data = (
            x[start_idx_pos:start_idx_pos + self.njoint] + 
            x[start_idx_vel:start_idx_vel + self.njoint] * horizon_steps
        ).tolist()

        return arm_command_msg
    
    def cmd_arm_vel(self,x,u,simu):
        arm_vel_msg = Float64MultiArray()
        start_idx_vel = self.nu - self.njoint
        arm_vel_msg.data = x[start_idx_vel:start_idx_vel + self.njoint].tolist()
        return arm_vel_msg
    
    def state_msg(self,x,u):
        msg = RobStateMM6()
        msg.x = x[self.x_label['x']]
        msg.y = x[self.x_label['y']]
        msg.z = 0.
        msg.orientation = x[self.x_label['yaw']]
        msg.front_wheel_angle = 0.
        msg.rear_wheel_angle = 0.
        msg.arm1 =            x[self.x_label['q1']]
        msg.arm2 =            x[self.x_label['q2']]
        msg.arm3 =            x[self.x_label['q3']]
        msg.arm4 =            x[self.x_label['q4']]
        msg.arm5 =            x[self.x_label['q5']]
        msg.arm6 =            x[self.x_label['q6']]
        msg.vx =              x[self.x_label['vx']]
        msg.vtheta_front =    x[self.x_label['wz']]
        msg.vtheta_rear =     x[self.x_label['wz']]
        msg.varm1 =           x[self.x_label['w1']]
        msg.varm2 =           x[self.x_label['w2']]
        msg.varm3 =           x[self.x_label['w3']]
        msg.varm4 =           x[self.x_label['w4']]
        msg.varm5 =           x[self.x_label['w5']]
        msg.varm6 =           x[self.x_label['w6']]
        msg.tau_x =           u[self.u_label['fx']]
        msg.tau_theta_front = u[self.u_label['tauz']]
        msg.tau_theta_rear =  u[self.u_label['tauz']]
        msg.tau_arm1 =        u[self.u_label['tau1']]
        msg.tau_arm2 =        u[self.u_label['tau2']]
        msg.tau_arm3 =        u[self.u_label['tau3']]
        msg.tau_arm4 =        u[self.u_label['tau4']]
        msg.tau_arm5 =        u[self.u_label['tau5']]
        msg.tau_arm6 =        u[self.u_label['tau6']]
        return msg
    
@configclass
class CostCfg:
    coefs_keys = {'wPos_base':3, 'wPos_arm':3, 'wRot':1, 'wKeepInFov':1, 'wOcc':1, 'wMan':1, 'qVelBase':2, 'qVelArm':6, 'qAccBase':2, 'qAccArm':6, 'wFollow_base':1, 'wFollow_arm':1}

    def cost_base_traj(self, u, x, traj_base, traj_arm, target, obstacles):
        T_front_base = self.T_base_front(x)
        e_base_traj = T_front_base[:3, 3] - traj_base[:3]
        return e_base_traj
    
    def cost_arm_traj(self, u, x, traj_base, traj_arm, target, obstacles):
        eff_pose_quat = self.quat_effector(x)
        e_ee_traj = eff_pose_quat[:3] - traj_arm[:3]
        return e_ee_traj
    
    def cost_manipulability(self, u, x, traj_base, traj_arm, target, obstacles):
        name = "wrist_3_joint"
        arm_jac = cpin.computeFrameJacobian(self.ca_model, self.ca_data, self.x_state2pin_state(x), self.ca_model.getFrameId(name), pin.LOCAL)[:3,-self.njoint:]
        a1, a2 = 10, 0.2
        w = exp(- a1*(sqrt(det(arm_jac@transpose(arm_jac))) - a2))
        return w
    
    def cost_vel_arm(self, u, x, traj_base, traj_arm, target, obstacles):
        return x[self.nx-self.njoint:]
    
    def cost_vel_base(self, u, x, traj_base, traj_arm, target, obstacles):
        return x[self.nq:self.nx-self.njoint]

    def cost_acc_arm(self, u, x, traj_base, traj_arm, target, obstacles):
        return u[self.nu-self.njoint:]
    
    def cost_acc_base(self, u, x, traj_base, traj_arm, target, obstacles):
        return u[:self.nu-self.njoint]
    
    def cost_keep_target_in_fov(self, u, x, traj_base, traj_arm, target, obstacles):
        T_arm = self.T_effector(x)
        sight_segment = [T_arm[:3, 3], (T_arm @ ca_pose2M4x4(np.array([0,0,50])))[:3,3]]
        dist_roi_2_sight_segment,roi_point = ca_distance_point_to_segment(target,sight_segment[0],sight_segment[1])
        return dist_roi_2_sight_segment
    
    def cost_occlusion(self, u, x, traj_base, traj_arm, target, obstacles):
        T_arm = self.T_effector(x)
        sight_segment = [T_arm[:3, 3], (T_arm @ ca_pose2M4x4(np.array([0,0,50])))[:3,3]]
        HFOV,VFOV = (69/2)*pi/180,(42/2)*pi/180
        OBSTACLE_SEGMENT_1_LIST = []
        OBSTACLE_SEGMENT_2_LIST = []
        OBSTACLE_RADIUS_LIST = []
        for i in range(int(obstacles.size1()/7)):
            OBSTACLE_SEGMENT_1_LIST.append(obstacles[i*7:i*7+3])
            OBSTACLE_SEGMENT_2_LIST.append(obstacles[i*7+3:i*7+6])
            OBSTACLE_RADIUS_LIST.append(obstacles[i*7+6])

        vy = mtimes(T_arm,np.array([[0,1,0,1]]).T)[:3]
        vz = mtimes(T_arm,np.array([[0,0,1,1]]).T)[:3]
        planar_dist_roi_2_sight_segment,_ = ca_planar_distance_point_to_segment(target,sight_segment[0],sight_segment[1],vy,vz)
        dist_roi_2_sight_segment,roi_point = ca_distance_point_to_segment(target,sight_segment[0],sight_segment[1])

        R_end = ca_cone_max_radius(sight_segment[0],sight_segment[1],HFOV,VFOV,sight_segment[1])
        R_roi = ca_cone_max_radius(sight_segment[0],sight_segment[1],HFOV,VFOV,target)
        R_max_roi = ca_cone_max_radius(sight_segment[0],sight_segment[1],HFOV,VFOV,roi_point)

        planar_d_occ_obs_min = R_end
        R_max_obs_min = R_end
        for i, (OBSTACLE_CENTER_0, OBSTACLE_CENTER_1, OBSTACLE_RADIUS) in enumerate(zip(OBSTACLE_SEGMENT_1_LIST, OBSTACLE_SEGMENT_2_LIST, OBSTACLE_RADIUS_LIST)):            
            dist_sight_segment_2_obstacle,sight_segment_point,obstacle_point = ca_distance_segment_to_segment(sight_segment[0],sight_segment[1], OBSTACLE_CENTER_0,OBSTACLE_CENTER_1)
            R_max_obs = ca_cone_max_radius(sight_segment[0],sight_segment[1],HFOV,VFOV,sight_segment_point)
            R_obs = ca_cone_max_radius(sight_segment[0],sight_segment[1],HFOV,VFOV,obstacle_point)

            planar_dist_sight_segment_2_obstacle,_,__ = ca_planar_distance_segment_to_segment(sight_segment[0],sight_segment[1], OBSTACLE_CENTER_0,OBSTACLE_CENTER_1,vy,vz)
            planar_d_occ_obs = norm_2(planar_dist_sight_segment_2_obstacle + planar_dist_roi_2_sight_segment)
            planar_d_occ_obs_beind = if_else(R_max_obs >= R_max_roi,planar_d_occ_obs_min, fmin(planar_d_occ_obs, planar_d_occ_obs_min)) # Do not compute obstacle beind the roi
            planar_d_occ_obs_min = planar_d_occ_obs_beind
            # planar_d_occ_obs_min = if_else(norm_2(planar_dist_sight_segment_2_obstacle)>R_obs, planar_d_occ_obs_min, planar_d_occ_obs_beind) # do not comput obs if outside of his ellipse
            R_max_obs_min = if_else(planar_d_occ_obs==planar_d_occ_obs_min,R_max_obs,R_max_obs_min)
        
        SMALL_NUM = 1e-8
        e_planar_docc_obs = 1/(planar_d_occ_obs_min+SMALL_NUM) - 1/R_end
        return e_planar_docc_obs

    cost_terms = [
        CostTerm(func=cost_base_traj, weight_key='wPos_base'),
        CostTerm(func=cost_arm_traj, weight_key='wPos_arm'),
        CostTerm(func=cost_manipulability, weight_key='wMan'),
        CostTerm(func=cost_vel_arm, weight_key='qVelArm'),
        CostTerm(func=cost_vel_base, weight_key='qVelBase'),
        CostTerm(func=cost_acc_arm, weight_key='qAccArm'),
        CostTerm(func=cost_acc_base, weight_key='qAccBase'),
        # CostTerm(func=cost_keep_target_in_fov, weight_key='wKeepInFov'),
        # CostTerm(func=cost_occlusion, weight_key='wOcc'),
    ]

@configclass
class ConstCfg:
    lbu = -np.array([20.,10.,5.,20.,20.,5.,5.,5.]) #tau
    ubu =  np.array([20.,10.,5.,20.,20.,5.,5.,5.])
    lbq = -np.array([1000.,1000.,1000.,-1.34,2.2,2.71,3.14,3.57,2.6]) # q
    ubq =  np.array([1000.,1000.,1000., 4.66,2.2,2.71,3.14,1.57,-2.2])
    lbv = -np.array([0.5,1.,0.5,0.5,0.5,0.3,0.5,0.3]) # v
    ubv =  np.array([0.5,1.,0.5,0.5,0.5,0.3,0.5,0.3])
    lbx = np.hstack((lbq,lbv))
    ubx = np.hstack((ubq,ubv))

    def collision_base_obstacle(self, u, x, obstacles, target):
        base = self.T_base(x)
        base2rearaxle = SX.sym("base2center", 4,4)
        base2rearaxle[:,:] = np.eye(4)
        base2rearaxle[0,3] = -(self.length + self.wheels_distance_length)/4 #x
        world2rearaxle = ca.mtimes(base, base2rearaxle)
        base2frontaxle = SX.sym("base2frontaxle", 4,4)
        base2frontaxle[:,:] = np.eye(4)
        base2frontaxle[0,3] = (self.length + self.wheels_distance_length)/4 #x
        world2frontaxle = ca.mtimes(base, base2frontaxle)

        collision_base_obstacle_const = []

        for i in range(int((obstacles.size1())/7)):
            OBSTACLE_CENTER_0 = obstacles[i*7:i*7+3]
            OBSTACLE_CENTER_1 = obstacles[i*7+3:i*7+6]
            OBSTACLE_RADIUS = obstacles[i*7+6]

            const_obstacle_base,__,___ = ca_distance_segment_to_segment(world2rearaxle[:3,3],world2frontaxle[:3,3],OBSTACLE_CENTER_0,OBSTACLE_CENTER_1)
            collision_base_obstacle_const.append(const_obstacle_base - (self.width/2 + OBSTACLE_RADIUS))
        return collision_base_obstacle_const

    def collision_effector_obstacle(self, u, x, obstacles, target):
        eff_pose = self.T_effector(x)

        collision_arm_obstacle_const = []

        for i in range(int((obstacles.size1())/7)):
            OBSTACLE_CENTER_0 = obstacles[i*7:i*7+3]
            OBSTACLE_CENTER_1 = obstacles[i*7+3:i*7+6]
            OBSTACLE_RADIUS = obstacles[i*7+6]

            const_obstacle_arm,__ = ca_distance_point_to_segment(eff_pose[:3,3],OBSTACLE_CENTER_0,OBSTACLE_CENTER_1)
            collision_arm_obstacle_const.append(const_obstacle_arm - OBSTACLE_RADIUS)
        return collision_arm_obstacle_const
    
    def auto_collision_base(self, u, x, obstacles, target):
        base = self.T_base(x)
        eff_pose = self.T_effector(x)
        base2rearaxle = SX.sym("base2center", 4,4)
        base2rearaxle[:,:] = np.eye(4)
        base2rearaxle[0,3] = -(self.length + self.wheels_distance_length)/4 #x
        world2rearaxle = ca.mtimes(base, base2rearaxle)
        base2frontaxle = SX.sym("base2frontaxle", 4,4)
        base2frontaxle[:,:] = np.eye(4)
        base2frontaxle[0,3] = (self.length + self.wheels_distance_length)/4 #x
        world2frontaxle = ca.mtimes(base, base2frontaxle)
        const_basecenter,__ = ca_distance_point_to_segment(eff_pose[:3,3],world2rearaxle[:3,3],world2frontaxle[:3,3])
        return const_basecenter
    
    def auto_collision_lidar(self, u, x, obstacles, target):
        base = self.T_base(x)
        eff_pose = self.T_effector(x)
        base2lidar = SX.sym("base2lidar", 4,4)
        base2lidar[:,:] = self.T_base2lidar
        worldlidar = ca.mtimes(base, base2lidar)
        const_lidar = sqrt((eff_pose[:3,3][0]-worldlidar[0,3])**2 + (eff_pose[:3,3][1]-worldlidar[1,3])**2 + (eff_pose[:3,3][2]-worldlidar[2,3])**2)
        return const_lidar

    def auto_collision_lidar_ray(self, u, x, obstacles, target):
        base = self.T_base(x)
        eff_pose = self.T_effector(x)
        base2lidar = SX.sym("base2lidar", 4,4)
        base2lidar[:,:] = self.T_base2lidar
        worldlidar = ca.mtimes(base, base2lidar)
        return eff_pose[:3,3][2] + worldlidar[2,3]

    const_terms = [
        HardConstTerm(func=auto_collision_base, lh=0.5, uh=10e3, lh_e=0.5, uh_e=10e3),
        HardConstTerm(func=auto_collision_lidar, lh=0.2, uh=10e3, lh_e=0.2, uh_e=10e3),
        HardConstTerm(func=auto_collision_lidar_ray, lh=0., uh=10e3, lh_e=0., uh_e=10e3),
        HardConstTerm(func=collision_base_obstacle, lh=0.1, uh=10e3, lh_e=0.1, uh_e=10e3),
        SoftConstTerm(func=collision_base_obstacle, lh=0.3, uh=10e3, lh_e=0.3, uh_e=10e3),
        HardConstTerm(func=collision_effector_obstacle, lh=0.2, uh=10e3, lh_e=0.2, uh_e=10e3),
        SoftConstTerm(func=collision_effector_obstacle, lh=0.6, uh=10e3, lh_e=0.6, uh_e=10e3),
    ]
'''
@configclass
class CamperoDynamicCfg():
    robot_model = ModelCfg()
    #ros_msg = RosCfg()
    #cost = CostCfg()
    #const = ConstCfg()

'''def main(args=None):
    rclpy.init(args=args)
    node = URDFPinocchioNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()'''

if __name__ == '__main__':
    #main()
    #campero = ManagerMpcAcados(CamperoDynamicCfg())
    campero = ManagerCasadiModel(CamperoDynamicCfg())
    #print(campero)
    #print(campero.name)
    x = np.zeros(campero.nx)
    u = np.zeros(campero.nu)
    print(campero.x_state2pin_state(x))
    print(campero._T_effector(x))
    '''print(campero.state_msg(x, u))
    print(campero.compute_cost(u, x,
                            np.zeros((campero.cost_coefs_len, 1)), 
                            np.zeros((campero.traj_base_len, 1)), 
                            np.zeros((campero.traj_arm_len, 1)), 
                            np.zeros((campero.target_len, 1)), 
                            np.zeros((campero.obstacles_len, 1))))'''
   

