#!/usr/bin/python3
# Drive the SCUTTLE while visualizing a simple LIDAR point cloud in NodeRED

import socket
import json
import numpy as np
import L1_lidar as lidar
import L2_vector as vec
import L2_inverse_kinematics as ik
import L2_speed_control as sc
from time import sleep
from threading import Thread
import time
import L2_Tracking_Config as config
import os

class SCUTTLE:

    def __init__(self):

        #Kinematics#
        self.wheelRadius = 0.04
        self.wheelBase = 0.1
        self.safe_distance= .25

        self.flag_path = "/home/pi/Birthday-Massacre/software/mxet300_labpantilt_has_target.txt"
        self.control_flag = self.get_control_flag()

        self.time_stuck = 2  # Time in seconds before considering the robot stuck
        self.last_time = time.time()
        self.last_lidar_data = None
        self.stuck = False

        self.A_matrix = np.array([[1/self.wheelRadius, -self.wheelBase/self.wheelRadius], [1/self.wheelRadius, self.wheelBase/self.wheelRadius]])
        self.max_xd = 0.4
        self.max_td = (self.max_xd/self.wheelBase)

        #UPD communication#
        self.IP = "127.0.0.1"
        self.port = 3554
        self.dashBoardDatasock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.dashBoardDatasock.bind((self.IP, self.port))
        self.dashBoardDatasock.settimeout(.25)

        #NodeRED data in#
        self.dashBoardData = None

        #LIDAR Thread#   
        lidarThread = Thread(target=self.scan_loop, daemon=True)
        lidarThread.start()

        #NodeRED Data Thread#
        self.dashBoardDataThread = Thread(target=self._dashBoardDataLoop, daemon=True)
        self.dashBoardDataThread.start()

        #Driving Thread#
        self.controlThread = Thread(target=self._controlLoop, daemon=True)
        self.controlThread.start()
    

    def scan_loop(self):
        while(True):
            data = self.cartesian_scan()
            data_msg = data.encode('utf-8')
            self.dashBoardDatasock.sendto(data_msg, ("127.0.0.1", 3555))
            sleep(.025)

    def cartesian_scan(self):
        rows = ''
        polar_data = lidar.polarScan(num_points=100)

        for d,t in polar_data:
            cartesian_point = vec.polar2cart(d,t)
            rows += self.format_row(cartesian_point)

        return rows[:-1]

    # Format the x,y lidar coordinates so that the bubble-chart can display them
    def format_row(self, point, r=3):
        x, y = point
        return '{x: ' + str(x) + ', y: ' + str(y) + ', r:' + str(r) + '},'

    def _dashBoardDataLoop(self):
        while True:
            try:
                dashBoardData,recvAddr = self.dashBoardDatasock.recvfrom(1024)
                self.dashBoardData = json.loads(dashBoardData)

            except socket.timeout:
                self.dashBoardData = None

    def detect_obstacle(self):
        polar_data = lidar.polarScan(num_points=100)  # More points = better angle resolution
        left = []
        center = []
        right = []

        for d, t in polar_data:
            if d > 0:  #adds the 100 readings to thier appropriate list
                if -60 < t <= -15:
                    right.append(d)
                elif -15 < t <= 15:
                    center.append(d)
                elif 15 < t <= 60:
                    left.append(d)
        # average the values in the list
        avg_right = np.mean(right) if right else float('inf') #if the list is not empty calculates mean
        avg_center = np.mean(center) if center else float('inf') #if the list is empty the object isn't within range so set to infity meaning verrrrry far away
        avg_left = np.mean(left) if left else float('inf')

        print(f"Avg Distances - L: {avg_left:.2f}, C: {avg_center:.2f}, R: {avg_right:.2f}") #testing

        return avg_left, avg_center, avg_right

    def check_stuck(self, avg_left, avg_center, avg_right):

        current_time = time.time()

        if self.last_lidar_data:
            # Compare current LIDAR data with the previous one
            if np.abs(self.last_lidar_data[1] - avg_center) < 0.05:  # Check if center distance hasn't changed much
                if current_time - self.last_time > self.time_stuck:
                    self.stuck = True
            else:  #nothing in the way for a period of time changes time
                self.last_time = current_time 

        self.last_lidar_data = (avg_left, avg_center, avg_right)

    def backup_and_turn(self):
       
        print("Getting stuck — backing up and turning")
        sc.driveOpenLoop(np.array([-4, -4]))  # Backup
        sleep(1)  

        if self.detect_obstacle()[0] > self.detect_obstacle()[2]: # left>right
            sc.driveOpenLoop(np.array([-3, 2]))  # Turn left
        else:
            sc.driveOpenLoop(np.array([4, -4]))  # Turn right
        sleep(1)  

    def _controlLoop(self):
        while True:  # Run continuously
            try:
                # Check control flag on each iteration
                self.control_flag = self.get_control_flag()

                if self.control_flag == 0:  # Only move if no target is detected
                    avg_left, avg_center, avg_right = self.detect_obstacle()

                    # Check if the robot is stuck
                    self.check_stuck(avg_left, avg_center, avg_right)

                    if self.stuck:
                        self.backup_and_turn()
                        self.stuck = False  # Reset stuck state 
                    elif avg_center < self.safe_distance + 0.1 or avg_left < self.safe_distance + 0.1 or avg_right < self.safe_distance + 0.1: #0.1 helps with the jittering
                        # Obstacle ahead → turn to side with more clearance
                        if avg_left > avg_right + 0.1:
                            print("Turning left")
                            self.backup_and_turn()
                        elif avg_right > avg_left + 0.1:
                            self.backup_and_turn()
                            print("Turning right")
                        else:
                            print("Centered obstacle — backing up")
                            sc.driveOpenLoop(np.array([-4, -4]))  # Optional: back up a bit hasn't been used
                    else:
                        print("Path is clear, moving forward")
                        sc.driveOpenLoop(np.array([4, 4]))  # go forward
                else:
                    print("Target detected - stopping movement")
                    sc.driveOpenLoop(np.array([0, 0]))  # Stop the robot
                    sleep(2)

                sleep(0.3)  # Control loop rate
            except Exception as e:
                print(f"Error in control loop: {e}") #lets me know about errors
                sleep(0.5)
            

    def _getWheelSpeed(self,userInputTarget):
        try:
            robotTarget = self._mapSpeeds(np.array([userInputTarget['y'],-1*userInputTarget['x']]))
            wheelSpeedTarget = self._calculateWheelSpeed(robotTarget)
            return wheelSpeedTarget
        except:
            pass
    
    def _mapSpeeds(self,original_B_matrix):
        B_matrix = np.zeros(2)
        B_matrix[0] = self.max_xd * original_B_matrix[0]
        B_matrix[1] = self.max_td * original_B_matrix[1]
        return B_matrix

    def _calculateWheelSpeed(self,B_matrix):
        C_matrix = np.matmul(self.A_matrix,B_matrix)
        C_matrix = np.round(C_matrix,decimals=3)
        return C_matrix


    def getdashBoardData(self):
        return self.dashBoardData


    # def get_control_flag(self):
    #     with open(self.flag_path, 'r') as f:
    #         control_flag = int(f.read().strip())
    #         print(control_flag)
    #     return control_flag

    def get_control_flag(self):
        """Read the control flag from the target file"""
        try:
            if os.path.exists('mxet300_labpantilt_has_target.txt'):
                with open('mxet300_labpantilt_has_target.txt', 'r') as f:
                    content = f.read().strip()
                    if content:  # Check if file is not empty
                        try:
                            control_flag = int(content)
                            print(f"Read control flag: {control_flag}")
                        except ValueError:
                            print("Invalid control flag value, defaulting to 0")
                            control_flag = 0
                    else:
                        print("Control flag file is empty, defaulting to 0")
                        control_flag = 0
            else:
                print("Control flag file not found, defaulting to 0")
                control_flag = 0
        except Exception as e:
            print(f"Error reading control flag: {e}, defaulting to 0")
            control_flag = 0
        return control_flag







if __name__ == "__main__":

    robot = SCUTTLE()
    try:
        while True:
            sleep(1)
    except KeyboardInterrupt:
        print("Stopping robot")