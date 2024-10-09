#python ver 3.10
""" This project aims to simulate a scenario where a UAV is being used in a rescue operation 
to detect a person in danger and communicates its position back to its base station. The main
focus is to tackle the problem of data loss and corruption caused in such cases due 
to dense vegetation by simulating long range communication (LoRa protocol)."""


import numpy as np
from controller import Robot, Camera, Compass, GPS, Gyro, InertialUnit, Motor, LED
from simple_pid import PID
import socket
import time
import random
import threading
from queue import Queue
import cv2


class DroneController(Robot):


    def __init__(self):
        super(DroneController, self).__init__()

        # Initialize timestep
        self.timestep = int(self.getBasicTimeStep())

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

        # Initialize UDP communication to simulate long range communication
        self.udp_ip = "127.0.0.1"
        self.udp_port = 10200
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        # Initialize message queue
        self.message_queue = Queue()

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
            sos_message = self.message_queue.get() #message quwue at base station side
            if sos_message is None:
                break  
            self.__simulate_delay()
            corrupted_message = self.__simulate_data_loss(sos_message)
            self.sock.sendto(corrupted_message.encode(), (self.udp_ip, self.udp_port))
            print(f"Sent message: {corrupted_message}", flush=True)
            self.message_queue.task_done()

    def __simulate_data_loss(self, message, loss_probability=0.3):
        """Simulate random data loss in the message caused due to dense vegetation"""
        message_list = list(message)
        if random.random() < loss_probability and len(message_list) > 0:
            loss_start = random.randint(0, len(message_list) - 1)
            loss_length = random.randint(1, len(message_list) - loss_start)
            print(f"Simulating data loss from index {loss_start} to {loss_start + loss_length - 1}")
            for i in range(loss_start, loss_start + loss_length):
                message_list[i] = ''
        return ''.join(message_list)

    def __simulate_delay(self):
        """Simulate random communication delay, again due to attenuation"""
        if random.random() < 0.3:
            delay_time = random.uniform(0.5, 2.0)
            print(f"Simulating communication delay of {delay_time:.2f} seconds...")
            time.sleep(delay_time)

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
        print(f"Enqueueing SOS message: {sos_message}")
        self.message_queue.put(sos_message)

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

            

            # Example condition to send SOS message
            # Replace this with actual logic (e.g., based on sensor data)
            people_in_danger = self.__detect_people_in_danger()
            if people_in_danger:
                latitude = position[0]
                longitude = position[1]
                altitude = position[2]
                self.__send_sos_message(latitude, longitude, altitude)

    def __detect_people_in_danger(self):
        """Future work: To actually detect using open-cv/tensorflow. Not been implemented in this version
        Assume that a person has been detected in danger. Currently, this function will always return true"""
       
        return True

    def cleanup(self):
        
        # Signal the sender thread to exit
        self.message_queue.put(None)
        self.sender_thread.join()
        self.sock.close()
        print("DroneController cleaned up.")

if __name__ == '__main__':
    # Instantiation
    controller = DroneController()
    try:
        controller.run()
    except KeyboardInterrupt:
        print("Controller interrupted by user.")
    finally:
        controller.cleanup()
