from controller import Robot
import socket
import threading
#main thread
print("Starting the Webots receiver", flush=True)

# Initialize the robot
robot = Robot()
timestep = int(robot.getBasicTimeStep())  # How many times the simulation is run

udp_ip = "127.0.0.1"
udp_port = 10200
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind((udp_ip, udp_port))

# Buffer to keep track of received messages and avoid duplicates
received_messages = set()


#receiving thread
def receive_messages():
    while True:
        try:
            data, addr = sock.recvfrom(1024)  # Buffer size of 1024 bytes
            message = data.decode()
            
            # Check if the message was already received (to avoid duplicates)
            if message not in received_messages:
                # Process the new message
                print(f"Received message: {message} from {addr}", flush=True)
                # Add the message to the set of received messages
                received_messages.add(message)
            else:
                print(f"Duplicate message ignored: {message}", flush=True)
                
        except socket.error as e:
            print(f"Socket error: {e}", flush=True)
            break

# Start the message-receiving thread
receiver_thread = threading.Thread(target=receive_messages)
receiver_thread.start()

# Main loop for the robot
while robot.step(timestep) != -1:
    # You can add additional control logic here if needed
    pass

# Clean up
sock.close()