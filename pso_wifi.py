import numpy as np
from pyswarms.single.global_best import GlobalBestPSO
import serial
import time
import socket


ESP32_PORT = 80  # Same as the server port in the ESP32 code

# Target to be dynamically updated


target = []
xyz=list("xyz")
for i in range(0,3):

    coord=float(input(f"Enter the Co-ordinates of {xyz[i]}: "))
    target.append(coord)





'''
SETUP FOR FORWARD KINEMATICS
'''

# DH parameters (updated for starting at 90 degrees for Arm1)
dh_para = np.array([
    {"theta": 0, "a": 0, "d": 0, "alpha": 0},  # Base (fixed)
    {"theta": np.pi / 2, "a": 7.5, "d": 0, "alpha": np.pi / 2},  # Arm1 (90 degrees, 7.5 cm above base)
    {"theta": np.pi, "a": 9.45, "d": 0, "alpha": 0},  # Arm2 (9.45 cm radius)
    {"theta": -np.pi, "a": 20, "d": 0, "alpha": 0},  # Link 3
    {"theta": 0, "a": 13, "d": 6, "alpha": np.pi / 2}  # End-effector
])

# Transformation matrix
def Transform(theta, a, d, alpha):
    return np.array([
        [np.cos(theta), -np.sin(theta) * np.cos(alpha), np.sin(theta) * np.sin(alpha), a * np.cos(theta)],
        [np.sin(theta), np.cos(theta) * np.cos(alpha), -np.cos(theta) * np.sin(alpha), a * np.sin(theta)],
        [0, np.sin(alpha), np.cos(alpha), d],
        [0, 0, 0, 1]
    ])

# Forward kinematics
def forward_kinematics(dh_params):
    result = np.eye(4)  # Start with identity matrix
    for param in dh_params:
        result = np.dot(result, Transform(param["theta"], param["a"], param["d"], param["alpha"]))
    return result


'''
DEFINING AND INITIATING PARTICLE SWAMP OPTIMIZATION
'''


# Objective function for PSO
def objective_function(joint_angles):
    '''
    Finding cost (error) and regularization to stop extreme angles
    '''
    # calling targets
    
    costs = []
    for angles in joint_angles:
        try:
            # Update DH parameters with joint angles
            dh_para[1]["theta"] = np.pi / 2 + angles[0]  # Arm1 starts at 90 degrees
            dh_para[2]["theta"] = np.pi / 2 + angles[1]
            dh_para[3]["theta"] = -np.pi / 2 + angles[2]
            
            # Forward kinematics to calculate end-effector position
            result = forward_kinematics(dh_para)
            position = np.array([result[0, 3], result[1, 3], result[2, 3]])
            
            # Calculate error and add regularization
            error = np.linalg.norm(position - target) / np.linalg.norm(target)
            regularization = np.sum(np.square(angles))  # L2 regularization
            costs.append(error + 0.01 * regularization)
        except Exception as e:
            print(f"Error for angles {angles}: {e}")
            costs.append(float('inf'))  # Assign a large penalty for invalid configurations
    return np.array(costs)

# Optimizer setup
options = {"c1": 2.05, #Cognitive Coefficient 
 "c2": 2.05,           # social Coefficient  
  "w": 0.7}            # Inertial weight 


# Joint angle bounds
bounds = (np.radians([0, 0, 0]), #lower bounds
 np.radians([180, 175, 90]))  #upper bounds

# initiating optimizer
optimizer = GlobalBestPSO(n_particles=50, dimensions=3, options=options, bounds=bounds)


"""
PROCESSING COORDINATES GIVEN BY OPENCV 
"""


# Main function to perform optimization and send angles
def detect_and_move(x, y,z):
    """
    Perform a single detection and move the robotic arm to the detected coordinates.
    - x, y: Detected coordinates in the camera frame.
    """
    global best_angles
    

    # Optimize joint angles using PSO
    best_cost, best_angles = optimizer.optimize(objective_function, iters=200)
    print(f"Optimal Joint Angles (radians): {best_angles}")
    print(f"Optimal Joint Angles (degrees): {np.degrees(best_angles)}")
    print(f"Best Cost (Error): {best_cost}")
    
    return best_angles

ESP32_IP = input(str("Provide ESP32 IP address: "))  # Replace with the IP address of your ESP32

'''
INITIATING SERIAL COMMUNICATION (currently not working)
'''
# # Function to send angles to ESP32 over serial communication
# def send_angles_to_esp32(angles):
#     """
#     Sends angles to ESP32 over serial communication.
#     """
#     try:
#         angles_str = ",".join(map(str, angles)) + "\n"
#         ser.write(angles_str.encode())  # Send angles as bytes
#         ser.flush()
#         print(f"Sent angles to ESP32: {angles_str.strip()}")
#     except Exception as e:
#         print(f"Error while sending angles: {e}")

# # Function to receive response from ESP32
# def receive_response_from_esp32():
#     """
#     Receives and prints the response from ESP32.
#     """
#     try:
#         if ser.in_waiting > 0:
#             response = ser.readline().decode().strip()  # Read and decode the response
#             print(f"Response from ESP32: {response}")
#     except Exception as e:
#         print(f"Error while receiving response: {e}")


def connect_to_esp32():
    try:
        # Create a socket object and connect to ESP32 server
        client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        client_socket.connect((ESP32_IP, ESP32_PORT))
        print("Connected to ESP32")
        return client_socket
    except Exception as e:
        print(f"Error connecting to ESP32: {e}")
        return None

def send_data_to_esp32(client_socket, data):
    try:
        client_socket.sendall(data.encode())  # Send data as bytes
        print(f"Data sent to ESP32: {data}")
    except Exception as e:
        print(f"Error while sending data: {e}")

def receive_data_from_esp32(client_socket):
    try:
        response = client_socket.recv(1024).decode()  # Receive response from ESP32
        print(f"Response from ESP32: {response}")
    except Exception as e:
        print(f"Error while receiving data: {e}")

if __name__ == "__main__":
    client_socket = connect_to_esp32()
    if client_socket:
        # data_to_send = np.degrees(best_angles),10 
        angles_in_degrees = np.degrees(best_angles)  # Convert to degrees
        data_to_send = f"{angles_in_degrees[0]},{angles_in_degrees[1]},{angles_in_degrees[2]},{10}\n"
        send_data_to_esp32(client_socket, data_to_send)
 # Example data to send
        send_data_to_esp32(client_socket, data_to_send)
        receive_data_from_esp32(client_socket)
        client_socket.close()  # Close connection after communication

# Main execution
     
best_angles = detect_and_move(target[0],target[1],target[2])
    

    