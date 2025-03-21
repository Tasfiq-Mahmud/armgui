import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import tkinter as tk
import threading
import pybullet as p
import pybullet_data
import time
import launch_ros
import os
from PIL import Image, ImageTk
import numpy as np

class robotArmGUI(Node):
    def __init__(self):
        super().__init__('arm_gui')
        self.pub = self.create_publisher(Float32MultiArray, '/joint_angles', 10)
        self.sub = self.create_subscription(Float32MultiArray,'/joint_angles',self.update_pybullet,10)
        self.root = tk.Tk()
        self.root.title("Robot Arm Gui")
        self.root.geometry("1000x800")
        self.root.minsize(300,400)
        self.joint_angles = [0.0, 0.0, 0.0, 0.0]

        self.canvas = tk.Label(self.root)
        self.canvas.grid(row=0,column=0,padx=50)
        # self.canvas.pack()

        ''' Creating Sliders'''
        self.joint_angles = [0.0, 0.0, 0.0, 0.0]
        self.slide=tk.Frame(self.root)
        self.slide.grid(row=0,column=1)
        # self.sliders=[]
        self.slider_names=['base block rotating block joint','rotating block vertical arm joint','vertical arm forearm joint','actuator forearm joint']
        for i in range(2,6):
            slider = tk.Scale(self.slide, from_=-90, to=90, orient="horizontal", length=300, label=self.slider_names[i-2], command=lambda val, idx=i: self.update_angle(idx, val))
            slider.set(0)
            # slider.pack()
            slider.grid(row=i-2,column=0,pady=10)
            # self.sliders.append(slider)
        
        '''Creating a thread for pybullet'''
        self.pbthread = threading.Thread(target=self.start_pb, daemon=True)
        self.pbthread.start()
 
        self.spin_ros()
        self.root.mainloop()

    def start_pb(self):
        """ Initializeing Pybullet simulation """
        p.connect(p.DIRECT)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
 
        # Load the URDF model
        p.loadURDF("plane.urdf")
        pkg_path=launch_ros.substitutions.FindPackageShare(package='armgui').find('armgui')
        # print(pkg_path)
        self.robot_id = p.loadURDF(os.path.join(pkg_path, 'urdf/arm.urdf'), basePosition=[0, 0, 0])

        # Camera settings
        self.view_matrix = p.computeViewMatrix(
            cameraEyePosition=[1, -1, 1], 
            cameraTargetPosition=[0, 0, 0.1], 
            cameraUpVector=[0, 0, 1] 
        )
        self.projection_matrix = p.computeProjectionMatrixFOV(
            fov=15, aspect=1, nearVal=0.1, farVal=3
        )

        self.update_pybullet_view()

    def update_pybullet(self,msg):
        self.get_logger().info(f'msg is - {msg.data}')
        for i, angle in enumerate(self.joint_angles):
            p.setJointMotorControl2(self.robot_id, i+2, p.POSITION_CONTROL, targetPosition=angle * 3.14 / 180)
        p.stepSimulation()

    def update_pybullet_view(self):
        """ Capture Pybullet rendering and update Tkinter Canvas """
        width, height, rgb, _, _ = p.getCameraImage(
            width=500, height=300, viewMatrix=self.view_matrix, projectionMatrix=self.projection_matrix
        )

        # Convert RGB to Image
        rgb_array = np.array(rgb, dtype=np.uint8)[:, :, :3] 
        img = Image.fromarray(np.array(rgb_array, dtype=np.uint8), 'RGB')
        img = ImageTk.PhotoImage(img)
        
        # Update Canvas
        self.canvas.config(image=img)
        self.canvas.image = img  

        self.root.after(50, self.update_pybullet_view)

    def update_angle(self,index,value):
        self.joint_angles[index-2]=float(value)
        msg = Float32MultiArray()
        msg.data = self.joint_angles
        self.pub.publish(msg)
    
    # def sub_clbk(self,msg):
    #     self.get_logger().info(f'msg is - {msg.data}')
    
    def spin_ros(self):
        """Periodically process ROS 2 events to avoid blocking."""
        rclpy.spin_once(self, timeout_sec=0.1)
        self.root.after(50, self.spin_ros)



def main(args=None):
    rclpy.init(args=args)
    node = robotArmGUI()
    node.destroy_node()
    rclpy.shutdown()
    

if __name__=='__main__':
    main()