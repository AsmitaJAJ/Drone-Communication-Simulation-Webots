#python ver 3.10
""" This project aims to simulate a scenario where a UAV is being used in a rescue operation 
to detect a person in danger and communicates its position back to its base station. The main
focus is to tackle the problem of data loss and corruption caused in such cases due 
to dense vegetation by simulating long range communication (LoRa protocol)."""
import socket

import network_server
import numpy as np
from controller import Robot, Camera, Compass, GPS, Gyro, InertialUnit, Motor, LED
from simple_pid import PID
import socket
import time
import random
import threading
from queue import Queue
import cv2
import json
import matplotlib.pyplot as plt
import os
drone_data = {}



class DroneController(Robot):


    def __init__(self):
        super(DroneController, self).__init__()

        # Initialize timestep
        self.timestep = int(self.getBasicTimeStep())
        self.message_queue=Queue()

        # Initialize sensors
        self.camera = self.getDevice('camera')
        self.camera.enable(self.timestep)

        self.imu = self.getDevice('inertial unit')
        self.imu.enable(self.timestep)

        self.gps = self.getDevice('gps')
        self.gps.enable(self.timestep)

        self.compass = self.getDevice('compass')
        self.compass.enable(self.timestep)

        self.gyro = self.getDevice('gyro')
        self.gyro.enable(self.timestep)

        # Initialize camera stabilization motors
        self.camera_roll_motor = self.getDevice('camera roll')
        self.camera_pitch_motor = self.getDevice('camera pitch')

        # Initialize LEDs
        self.front_left_led = self.getDevice('front left led')
        self.front_right_led = self.getDevice('front right led')

        # Initialize motors
        self.front_left_motor = self.getDevice('front left propeller')
        self.front_right_motor = self.getDevice('front right propeller')
        self.rear_left_motor = self.getDevice('rear left propeller')
        self.rear_right_motor = self.getDevice('rear right propeller')
        #intitalize co=ordinates
        self.latitude = None
        self.longitude = None
        self.altitude = None
        
        self.latency_data = []  # To store latency (delay) data
        self.data_rate_data = []  # To store transmission rate (bytes/sec)
        self.packet_success_data = []  # To store packet delivery success (1 = success, 0 = failure)
        self.total_packets = 0
        self.successful_packets = 0
        
        self.motors = [
            self.front_left_motor,
            self.front_right_motor,
            self.rear_left_motor,
            self.rear_right_motor
        ]

        # Set motors to velocity mode
        for motor in self.motors:
            motor.setPosition(float('inf'))
            motor.setVelocity(0.0)  # Start with motors stopped

        # Initialize PID controllers
        self.__initialize_pid_controllers()


        # Start sender thread
        self.sender_thread = threading.Thread(target=self.__sender, daemon=True)
        self.sender_thread.start()

        # Control parameters
        self.k_vertical_thrust = 70
        self.k_vertical_offset = 0.6
        self.k_vertical_p = 5.0
        self.k_roll_p = 20.0
        self.k_pitch_p = 0.0

        self.target_altitude = 0.7  # Target altitude in meters

        print("DroneController initialized.")

    def __initialize_pid_controllers(self):
        """Initialize PID controllers for roll, pitch, yaw, and vertical stabilization."""

        # Roll PID
        K_u = 150.0
        T_u = 342.857 / 1000.0  # seconds
        params_roll = {
            'P': K_u / 5.0,
            'I': (2.0 / 5.0) * K_u / T_u,
            'D': K_u * T_u / 15.0,
            'sp': 0.0
        }
        self.roll_pid = PID(
            params_roll['P'],
            params_roll['I'],
            params_roll['D'],
            setpoint=params_roll['sp'],
            output_limits=(-2.0, 2.0),
            sample_time=self.timestep / 1000.0
        )

        # Pitch PID
        K_u = 150.0
        T_u = 682.66 / 1000.0  # seconds
        params_pitch = {
            'P': K_u / 5.0,
            'I': (2.0 / 5.0) * K_u / T_u,
            'D': K_u * T_u / 15.0,
            'sp': 0.0
        }
        self.pitch_pid = PID(
            params_pitch['P'],
            params_pitch['I'],
            params_pitch['D'],
            setpoint=params_pitch['sp'],
            output_limits=(-2.0, 2.0),
            sample_time=self.timestep / 1000.0
        )

        # Yaw PID
        K_u = 20.0
        T_u = 1621.33 / 1000.0  # seconds
        params_yaw = {
            'P': 0.8 * K_u,
            'I': 0.0,
            'D': K_u * T_u / 10.0,
            'sp': 0.0
        }
        self.yaw_pid = PID(
            params_yaw['P'],
            params_yaw['I'],
            params_yaw['D'],
            setpoint=params_yaw['sp'],
            output_limits=(-2.0, 2.0),
            sample_time=self.timestep / 1000.0
        )

        # Vertical PID
        K_u = 20.0
        T_u = 2668.8 / 1000.0  # seconds
        params_vert = {
            'P': 0.8 * K_u,
            'I': 0.0,
            'D': K_u * T_u / 10.0,
            'sp': 0.0
        }
        self.vert_pid = PID(
            params_vert['P'],
            params_vert['I'],
            params_vert['D'],
            setpoint=params_vert['sp'],
            output_limits=(-5.0, 5.0),
            sample_time=self.timestep / 1000.0
        )

        print("PID controllers initialized.")

    def __sender(self):
        """The drone is sending messages to the base station via Long range communication"""
        while True:
            if self.altitude is not None and self.latitude is not None and self.longitude is not None:
                
                sos_message=self.message_queue.get()
                network_server.forward_sos(sos_message)
                self.message_queue.task_done()
                drone_data={"latitude":self.latitude, "longitude":self.longitude, "altitude":self.altitude}
                print(self.altitude )
                with open('/Users/seema/Desktop/flask_app/drone_data.json', 'w') as file:
                    json.dump(drone_data, file)
            time.sleep(1) 
            

    def __simulate_data_loss(self, message, loss_probability=0):
        """Simulate random data loss in the message caused due to dense vegetation"""
        self.total_packets += 1 #to count total transmission
        message_list = list(message)
        if random.random() < loss_probability and len(message_list) > 0:
            loss_start = random.randint(0, len(message_list) - 1)
            loss_length = random.randint(1, len(message_list) - loss_start)
            print(f"Simulating data loss from index {loss_start} to {loss_start + loss_length - 1}")
            for i in range(loss_start, loss_start + loss_length):
                message_list[i] = ''
            self.packet_success_data.append(0) #how many packets were failed to be delivered
        else:
            self.successful_packets += 1
            self.packet_success_data.append(1) #packets that were successfully delivered
        return ''.join(message_list)

    def __simulate_delay(self):
        """Simulate random communication delay, again due to attenuation"""
        if random.random() < 0.3:
            delay_time = random.uniform(0.5, 2.0)
            print(f"Simulating communication delay of {delay_time:.2f} seconds...")
            self.latency_data.append(delay_time)
            time.sleep(delay_time)
        else:
            self.latency_data.append(0) #no delay

    def __stabilize_pose(self, pose_angles, pose_vel):
        """Compute PID corrections for roll, pitch, and yaw to stabilize the drone."""
        phi, theta, psi = pose_angles
        p, q, r = pose_vel

        roll_correction = self.roll_pid(phi) - p
        pitch_correction = self.pitch_pid(theta) - q
        yaw_correction = self.yaw_pid(r)

        return [roll_correction, pitch_correction, yaw_correction]

    def __compute_disturbances(self):
        """Compute vertical thrust based on altitude."""
        # Get current odometry
        orientation = self.imu.getRollPitchYaw()
        ang_velocity = self.gyro.getValues()
        position = self.gps.getValues()

        # Stabilize orientation
        roll_c, pitch_c, yaw_c = self.__stabilize_pose(orientation, ang_velocity)

        # Vertical control
        current_altitude = position[2]
        self.vert_pid.setpoint = self.target_altitude
        thrust_level = self.vert_pid(current_altitude)

        # Compute motor inputs based on PID corrections
        fl_motor = self.k_vertical_thrust + thrust_level + roll_c - pitch_c - yaw_c  # Front Left
        fr_motor = self.k_vertical_thrust + thrust_level - roll_c - pitch_c + yaw_c  # Front Right
        rl_motor = self.k_vertical_thrust + thrust_level + roll_c + pitch_c + yaw_c  # Rear Left
        rr_motor = self.k_vertical_thrust + thrust_level - roll_c + pitch_c - yaw_c  # Rear Right

        return fl_motor, fr_motor, rl_motor, rr_motor

    def __actuate_motors(self, motor_velocities):
        """Set the velocities for each motor."""
        fl, fr, rl, rr = motor_velocities
        self.front_left_motor.setVelocity(fl)
        self.front_right_motor.setVelocity(-fr)  # Note: Invert if necessary
        self.rear_left_motor.setVelocity(-rl)    # Note: Invert if necessary
        self.rear_right_motor.setVelocity(rr)

    def __send_sos_message(self, latitude, longitude, altitude):
        """Create and enqueue an SOS message."""
        sos_message = f"SOS! Coordinates: Latitude={latitude}, Longitude={longitude}, Altitude={altitude}"
        self.__simulate_delay() #delay due to attenuation
        sos_message_c=self.__simulate_data_loss(sos_message) #corrupted message- to simulate data corruption
        
         
        self.message_queue.put(sos_message_c)

    def run(self):
        """Main control loop for the drone."""
        print("Drone control is active")
        while self.step(self.timestep) != -1:
            current_time = self.getTime()

            # Retrieve sensor data
            orientation = self.imu.getRollPitchYaw()
            ang_velocity = self.gyro.getValues()
            position = self.gps.getValues()

            # Blink the front LEDs alternatively with a 1-second rate
            led_state = int(current_time) % 2
            self.front_left_led.set(led_state)
            self.front_right_led.set(1 - led_state)

            # Stabilize the camera based on IMU data
            roll_acc, pitch_acc, _ = ang_velocity
            self.camera_roll_motor.setPosition(-0.115 * roll_acc)
            self.camera_pitch_motor.setPosition(-0.1 * pitch_acc)

            # Compute motor velocities using PID controllers
            motor_velocities = self.__compute_disturbances()
            self.__actuate_motors(motor_velocities)
            
           
            
            #Video feed from drone using the built in camera
            
            image = self.camera.getImage()
            width = self.camera.getWidth()
            height = self.camera.getHeight()
            image_array = np.frombuffer(image, np.uint8).reshape((height, width, 4))
            image_bgr = cv2.cvtColor(image_array, cv2.COLOR_RGBA2BGR)
            
            
            cv2.imshow('Drone Camera Feed', image_bgr)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

            

            
            
            people_in_danger = self.__detect_people_in_danger()
            if people_in_danger:
                self.latitude = position[0]
                self.longitude = position[1]
                self.altitude = position[2]
                self.__send_sos_message(self.latitude, self.longitude, self.altitude)
        self.plot_graphs()
               
             
                
                    
                

    def __detect_people_in_danger(self):
        """Future work: To actually detect using open-cv/tensorflow. Not been implemented in this version
        Assume that a person has been detected in danger. Currently, this function will always return true"""
       
        return True
    def plot_graphs(self):
        """Plot the graphs for latency, data transmission rate, and packet delivery success"""
        # Plot Latency Graph
        #plt.clf()
        save_path="/Users/seema/Desktop/flask_app/dron_flight_analysis"
        directory = os.path.dirname(save_path)
        if directory and not os.path.exists(directory):
            os.makedirs(directory)
        plt.figure(figsize=(10, 5))
        plt.subplot(1, 3, 1)
        plt.plot(self.latency_data, label="Latency (s)", color='blue')
        plt.xlabel('Transmission Number')
        plt.ylabel('Latency (seconds)')
        plt.title('Latency over Time')
        plt.grid(True)
                # Plot Packet Delivery Success Rate Graph
        plt.subplot(1, 3, 3)
        plt.plot(self.packet_success_data, label="Packet Delivery Success (1=Success, 0=Failure)", color='red')
        plt.xlabel('Transmission Number')
        plt.ylabel('Success (1 or 0)')
        plt.title('Packet Delivery Success over Time')
        plt.grid(True)

        # Show the plot
        plt.tight_layout()
        
        plt.savefig(save_path, format='png')  # Save the graph as a PNG file (change format if needed)
        print(f"Graph saved to {save_path}")
        plt.show(block=True)
        



    
        

if __name__ == '__main__':
    # Instantiation
    controller = DroneController()
    try:
        controller.run()
    except KeyboardInterrupt:
        print("Controller interrupted by user.")
 
