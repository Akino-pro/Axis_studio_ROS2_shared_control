#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import socket
import threading
import numpy as np
import math
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import random
from builtin_interfaces.msg import Duration
from multiprocessing import shared_memory, Lock
import struct



method_choice = 1  # Default to method 1

def set_method_choice(choice):
    global method_choice
    if choice in [1, 2]:
        method_choice = choice
    else:
        raise ValueError("Invalid method choice. Please choose 1 or 2.")

def swivel_angle(Origin, Elbow, Wrist):
    AB = np.array(Wrist) - np.array(Origin)
    AD = np.array(Elbow) - np.array(Origin)
    AP = (np.dot(AD, AB) / np.dot(AB, AB)) * AB
    C = np.array(Origin) + AP
    radius = np.linalg.norm(C - Elbow)
    n = AB / np.linalg.norm(AB)

    # Generating a basis for the plane orthogonal to AB
    if all(n[:2] == 0):
        # Handle the edge case where n is aligned with the z-axis
        orthogonal_vector = np.array([1, 0, 0])
    else:
        orthogonal_vector = np.array([-n[1], n[0], 0])
        orthogonal_vector /= np.linalg.norm(orthogonal_vector)

    v1 = orthogonal_vector
    v2 = np.cross(n, v1)
    v2 /= np.linalg.norm(v2)

    # Generate points on the circle
    num_points = 100
    theta = np.linspace(0, 2 * np.pi, num_points)
    circle_points = np.zeros((num_points, 3))
    for i in range(num_points):
        circle_points[i] = C + radius * (np.cos(theta[i]) * v1 + np.sin(theta[i]) * v2)

    # Finding the new elbow position
    minIdx = np.argmin(circle_points[:, 2])
    lowestPoint = circle_points[minIdx, :]
    vector_C_to_lowest = lowestPoint - C
    u = vector_C_to_lowest / np.linalg.norm(vector_C_to_lowest)
    v = np.cross(u, n) / np.linalg.norm(np.cross(u, n))
    r = (Elbow - C) / np.linalg.norm(Elbow - C)
    phi = np.arctan2(np.dot(v, r), np.dot(u, r))
    # print(phi)
    AE = 0.4208
    EB = 0.3143
    AB_length = np.linalg.norm(AB)
    AF = (AE**2 - EB**2 + AB_length**2) / (2 * AB_length)
    F = Origin + (AF / AB_length) * AB
    EF = np.sqrt(AE**2 - AF**2)
    E = F + EF * (np.cos(phi) * u + np.sin(phi) * v)
    
    return E, phi
class SocketNode(Node):
    def __init__(self):
        super().__init__('socket_node')
        self.initialize_shared_memory()
        # Initialize ROS Publisher
        self.state_publisher_ = self.create_publisher(JointState, 'joint_states', 10)
        self.trajectory_publisher = self.create_publisher(JointTrajectory, '/joint_trajectory_controller/joint_trajectory', 10)

        # Initialize Socket Server
        self.ip = "192.168.30.134"
        self.port = 12345
        self.server_thread = threading.Thread(target=self.start_receiver)
        self.server_thread.start()

        # Initialize DH table and ik_results
        self.method2_info = [0.0] * 10
        self.ik_results = [0.0] * 7 
        self.ik_results_1 = [0.0] * 7 
        self.DH_table = np.array([
            [-np.pi/2, 0, 0.2848, 0],
            [np.pi/2, 0, -0.0118, 0],
            [-np.pi/2, 0, 0.4208, 0],
            [np.pi/2, 0, -0.0128, 0],
            [-np.pi/2, 0, 0.3143, 0],
            [np.pi/2, 0, 0, 0],
            [0, 0, 0.1673, 0],
            [0, 0, 0.088, 0]
        ])
        self.DH_table_1 = np.array([
            [-np.pi/2, 0, 0.2848, 0],
            [np.pi/2, 0, -0.0118, 0],
            [-np.pi/2, 0, 0.4208, 0],
            [np.pi/2, 0, -0.0128, 0],
            [-np.pi/2, 0, 0.3143, 0],
            [np.pi/2, 0, 0, 0],
            [0, 0, 0.1673, 0],
            [0, 0, 0.088, 0]
        ])

    def initialize_shared_memory(self):
        # Attempt to create a new shared memory segment, or attach to it if it already exists
        try:
            self.shm = shared_memory.SharedMemory(name='ik_results_shm', create=True, size=1024)
        except FileExistsError:
            self.shm = shared_memory.SharedMemory(name='ik_results_shm', create=False)
        self.lock = Lock()

    def write_to_shared_memory(self, data):
        """Serialize and write a list of floats to shared memory."""
        with self.lock:
            # Use struct to pack the data into bytes. Here 'f' stands for float and
            # '*' indicates the number of floats to pack.
            bytes_data = struct.pack('f'*len(data), *data)
            # Write the serialized bytes data to shared memory
            self.shm.buf[:len(bytes_data)] = bytes_data

    def start_receiver(self):
        server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        server.bind((self.ip, self.port))
        server.listen(1)
        self.get_logger().info(f"Receiver listening on {self.ip}:{self.port}")

        conn, addr = server.accept()
        self.get_logger().info(f"Connected by {addr}")

        try:
            while True:
                data = conn.recv(1024)
                if not data:
                    break

                message = data.decode()
                self.get_logger().info(f"Received: {message}")

                object_data = message.split('], ')
                position_list = []
                for obj in object_data:
                    numbers_only = obj[obj.find('[') + 1:obj.find(']')].split(', ')
                    position_list.extend([float(value) for value in numbers_only])

                if len(position_list) != 18:
                    continue

                position_array = np.array(position_list).reshape(6, 3)
                self.get_logger().info(f"Positions as NumPy array: {position_array}")

                self.ik_results, _ = self.calculate_inverse_kinematics(position_array, self.DH_table[:, 3],self.DH_table)
                self.get_logger().info(f"Inverse Kinematics Results: {self.ik_results}")
                
                Origin = position_array[0]
                Elbow = position_array[1]
                Wrist = position_array[2]
                newE, s_angle = swivel_angle(Origin, Elbow, Wrist)
                position_array[1] = newE
                self.method2_info = Origin.tolist() + Wrist.tolist() + position_array[4].tolist() + [s_angle]

                self.ik_results_1, _ = self.calculate_inverse_kinematics(position_array, self.DH_table_1[:, 3],self.DH_table_1)
                # Publish the joint states immediately after calculating
                self.publish_joint_state()

        finally:
            conn.close()
            server.close()

    # def publish_joint_state(self):
    #     global method_choice
    #     if method_choice == 1:
    #         ik_results_to_use = self.ik_results_1
    #     else:
    #         ik_results_to_use = self.ik_results

    #     # Ensure ik_results_to_use has the correct length and data types
    #     if len(ik_results_to_use) != 7:
    #         self.get_logger().error(f'Invalid ik_results data length. Expected length 7, got {len(ik_results_to_use)}.')
    #         return

    #     if not all(isinstance(x, float) for x in ik_results_to_use):
    #         self.get_logger().error('One or more elements in ik_results are not of type float.')
    #         return

    #     if any(np.isnan(x) or np.isinf(x) for x in ik_results_to_use):
    #         self.get_logger().error('One or more elements in ik_results are NaN or infinite.')
    #         return

    #     joint_state = JointState()
    #     joint_state.header.stamp = self.get_clock().now().to_msg()
    #     joint_state.name = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6', 'joint_7']  # Update with the correct joint names
    #     joint_state.position = [float(f"{x:.6f}") for x in ik_results_to_use[0:7]]  # Use selected ik_results
    #     joint_state.velocity = []
    #     joint_state.effort = []
        
    #     self.get_logger().info(f"Publishing Joint States: {joint_state.position}")

    #     # send joint angles to the real robot by writing into shared memory
    #     self.write_to_shared_memory([float(f"{x:.6f}") for x in ik_results_to_use[0:7]])

    #     self.state_publisher_.publish(joint_state)


    def publish_joint_state(self):
            # Ensure ik_results has the correct length and data types
        # Check if ik_results has the correct length
        # if len(self.ik_results) != 7:
        #     self.get_logger().error(f'Invalid ik_results data length. Expected length 7, got {len(self.ik_results)}.')
        #     return



        global method_choice
        if method_choice == 1:
            ik_results_to_use = self.ik_results

        else:
            ik_results_to_use = self.ik_results_1

        # Check if all elements in ik_results are of type float
        if not all(isinstance(x, float) for x in self.ik_results):
            self.get_logger().error('One or more elements in ik_results are not of type float.')
            return

        # Check for NaN or infinite values in ik_results
        if any(np.isnan(x) or np.isinf(x) for x in self.ik_results):
            self.get_logger().error('One or more elements in ik_results are NaN or infinite.')
            return

        joint_state = JointState()
        joint_state.header.stamp = self.get_clock().now().to_msg()
        
        # joint_state.name = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6', 'joint_7', 'end_effector']  # Update with the correct joint names
        # joint_state.position = [float(f"{x:.6f}") for x in ik_results_to_use[0:7]] + [0.0]*9  # Use selected ik_results
        joint_state.name = [
            'joint_1', 'joint_2', 'joint_3', 'joint_4', 
            'joint_5', 'joint_6', 'joint_7', 
            'robotiq_85_left_knuckle_joint', 
            'robotiq_85_right_knuckle_joint', 
            'robotiq_85_left_inner_knuckle_joint', 
            'robotiq_85_right_inner_knuckle_joint', 
            'robotiq_85_left_finger_tip_joint', 
            'robotiq_85_right_finger_tip_joint',
            'joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6', 'joint7'
        ]
        joint_state.position = (
            [float(f"{x:.6f}") for x in ik_results_to_use[0:7]] +  # Arm joints
            [0.0] * 6 +
            [float(f"{x:.6f}") for x in self.ik_results[0:7]] # Gripper joints
        )

        # joint_state.name = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6', 'joint_7','joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6', 'joint7']
        # # joint_state.name = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6', 'joint_7', 'end_effector','joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6', 'joint7']




        # joint_state.position = [float(f"{x:.6f}") for x in self.ik_results_1[0:7]]  + [float(f"{x:.6f}") for x in self.ik_results[0:7]]
        # joint_state.position = [random.uniform(-5, 5) for _ in range(7)]
        # joint_state.position = [float(f"{x:.6f}") for x in self.ik_results_1[0:7]] + [0.0] + [float(f"{x:.6f}") for x in self.ik_results[0:7]]



        joint_state.velocity = []
        joint_state.effort = []
        self.get_logger().info(f"JSP Results: {joint_state.position}")

        # send joint angles to the real robot by writing into shared memory
        self.write_to_shared_memory([float(f"{x:.6f}") for x in self.ik_results_1[0:7]]+
                                     [float(f"{x:.6f}") for x in self.method2_info]+
                                     [float(f"{x:.6f}") for x in self.ik_results[0:7]])   
        
        # self.get_logger().info(f"Inverse Kinematics Results: {self.ik_results}")
        # self.get_logger().info(f"Good fake  Results: {[random.uniform(-5, 5) for _ in range(7)]}")
        self.state_publisher_.publish(joint_state)

    # def publish_joint_trajectory(self, positions, time_from_start_sec):
    #     joint_trajectory = JointTrajectory()
    #     joint_trajectory.joint_names = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6', 'joint_7','robotiq_85_left_knuckle_link']
    #     point = JointTrajectoryPoint()
    #     point.positions = [float(f"{x:.6f}") for x in self.ik_results[0:7] ] + [0.0]
    #     point.time_from_start = Duration(sec=time_from_start_sec)
    #     joint_trajectory.points.append(point)
    #     self.get_logger().info('Publishing Joint Trajectory...')
    #     self.trajectory_publisher.publish(joint_trajectory)

    def calculate_inverse_kinematics(self, positions,theta_vec,DH_table):
        # Transformation and position setup
        # Rtrans = np.array([[0, -1, 0], [0, 0, 1], [-1, 0, 0]])
        Rtrans = np.array([[1, 0, 0], [0, 1, 0], [0, 0, 1]])
        Origin = np.dot(Rtrans, positions[0])
        Elbow = np.dot(Rtrans, positions[1])
        Wrist = np.dot(Rtrans, positions[2])
        Pump = np.dot(Rtrans, positions[4])
        R_Index = np.dot(Rtrans, positions[3])
        R_Middle = np.dot(Rtrans, positions[4])
        R_Ring = np.dot(Rtrans, positions[5])
        PumpN = np.cross(R_Ring - R_Middle, R_Index - R_Middle)

        
        T_all = self.getT_all(theta_vec,DH_table)

        # Calculating angles based on positions
        j = 0

        # Calculating theta for each joint
        theta_vec[j] = self.theta_func(Origin, Elbow, self.getT_all(theta_vec,DH_table)[:,4*j:4*(j+1)])
        j += 1
        theta_vec[j] = self.theta_func(Origin, Elbow, self.getT_all(theta_vec,DH_table)[:,4*j:4*(j+1)]) + np.pi / 2
        j += 1
        theta_vec[j] = self.theta_func(Elbow, Wrist, self.getT_all(theta_vec,DH_table)[:,4*j:4*(j+1)])
        j += 1
        theta_vec[j] = self.theta_func(Elbow, Wrist, self.getT_all(theta_vec,DH_table)[:,4*j:4*(j+1)]) + np.pi / 2
        j += 1
        theta_vec[j] = self.theta_func(Wrist, Pump, self.getT_all(theta_vec,DH_table)[:,4*j:4*(j+1)])
        j += 1
        theta_vec[j] = self.theta_func(Wrist, Pump, self.getT_all(theta_vec,DH_table)[:,4*j:4*(j+1)]) + np.pi / 2
        j += 1
        theta_vec[j] = self.theta_func(Pump, PumpN, self.getT_all(theta_vec,DH_table)[:,4*j:4*(j+1)])

        theta_real = theta_vec;
        # theta_real[0] = theta_real[0] + np.pi/2;
        # if theta_real[1] > np.pi*3/2 :
        #     theta_real[1] = theta_real[1] - np.pi;
        
        return theta_real,theta_vec

    def getT_all(self, theta_vec,DH_table):
        DH_row = DH_table
        DH_row[:,3] = theta_vec
        DH_size = DH_row.shape[0]
        T_end = np.eye(4)
        T_all = [T_end]

        for i in range(DH_size):
            T = self.homogeneous_matrix(DH_row[i, :])
            T_end = np.dot(T_end, T)
            T_all.append(T_end)

        return np.concatenate(T_all, axis=1)

    def homogeneous_matrix(self, DH_row):
        alpha,a,d, theta = DH_row

        return np.array([
            [math.cos(theta), -math.sin(theta) * math.cos(alpha), math.sin(theta) * math.sin(alpha), a * math.cos(theta)],
            [math.sin(theta),  math.cos(theta) * math.cos(alpha), -math.cos(theta) * math.sin(alpha), a * math.sin(theta)],
            [0,               math.sin(alpha),                  math.cos(alpha),                    d],
            [0,               0,                                0,                                  1]
        ])

    def theta_func(self, P1, P2, T):
        O = T[0:3, 3]
        nx, ny, nz = T[0:3, 0], T[0:3, 1], T[0:3, 2]

        P1p = P1 - np.dot(P1 - O, nz) * nz
        P2p = P2 - np.dot(P2 - O, nz) * nz

        Vecp = P2p - P1p
        x, y = np.dot(Vecp, nx), np.dot(Vecp, ny)

        return math.atan2(y, x)


def main(args=None):
    import sys
    method = int(input("Enter method choice (1 or 2): "))
    set_method_choice(method)

    rclpy.init(args=args)
    unity_receiver_node = SocketNode()
    rclpy.spin(unity_receiver_node)
    unity_receiver_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
