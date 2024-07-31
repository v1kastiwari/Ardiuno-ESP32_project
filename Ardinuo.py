import numpy as np
import cv2
from cv2 import aruco
import math
import socket
import time
import json
import heapq

def detect_ArUco_details(image):
    ArUco_details_dict = {}
    ArUco_corners = {}
    # Define the ArUco dictionary and parameters
    aruco_dict = cv2.aruco.getPredefinedDictionary(aruco.DICT_4X4_1000)
    parameters = cv2.aruco.DetectorParameters()

    # Detect ArUco markers in the image
    corners, ids, _ = cv2.aruco.detectMarkers(image, aruco_dict, parameters=parameters)

    if ids is not None:
        for i in range(len(ids)):
            # Get the ID of the detected marker as an integer
            marker_id = int(ids[i][0])

            # Only process the marker with ID 100
            if marker_id == 100:
                # Calculate the center coordinates
                center = np.mean(corners[i][0], axis=0).astype(int)

                # Calculate the angle from the vertical
                angle = math.degrees(math.atan2(corners[i][0][1][1] - corners[i][0][0][1], corners[i][0][1][0] - corners[i][0][0][0]))

                # Store center coordinates as integers
                center_coordinates = [int(center[0]), int(center[1])]

                # Store corner coordinates as integers
                corner_coordinates = [list(map(int, coord)) for coord in corners[i][0]]

                # Populate the dictionaries
                ArUco_details_dict[marker_id] = [center_coordinates, int(angle)]
                ArUco_corners[marker_id] = corner_coordinates
    return ArUco_details_dict, ArUco_corners 

def calculate_angle(pointA, pointB, pointC):
    '''
    pointA is centre of aruco marker
    pointB is centre of top of aruco marker
    pointC is external point on map
    return angle between lines
    eg:
        angle = calculate_angle((118,372),
                                (151,321),
                                (299,313))
        angle is in degree
    '''
    # Convert the tuples to numpy arrays
    pointA = np.array(pointA)
    pointB = np.array(pointB)
    pointC = np.array(pointC)

    # Create vectors from point A to point B and point C
    vectorAB = pointB - pointA
    vectorAC = pointC - pointA

    # Calculate the unit vectors
    unitVectorAB = vectorAB / np.linalg.norm(vectorAB)
    unitVectorAC = vectorAC / np.linalg.norm(vectorAC)

    # Calculate the dot product of the unit vectors
    dotProduct = np.dot(unitVectorAB, unitVectorAC)

    # Calculate the angle in radians, and then convert to degrees
    angleInRadians = np.arccos(dotProduct)
    angleInDegrees = np.degrees(angleInRadians).astype(int)

    # Determine the sign of the angle
    crossProduct = np.cross(unitVectorAB, unitVectorAC)
    if crossProduct < 0:
        angleInDegrees = -angleInDegrees

    return angleInDegrees

def robo_command(degree_angle):
    if -60 <= degree_angle < -5:
        command = 'left'
    elif 5 <= degree_angle < 60:
        command = 'right'
    elif degree_angle >= 60:
        command = 'Sright'
    elif degree_angle <= -60:
        command = 'Sleft'
    else:
        command = 'forward'
    return command

 
def mark_ArUco_image(image, ArUco_details_dict, ArUco_corners, next_point):
    command = 'stop'  # Initialize with a default command
    center = [0,0]
    for ids, details in ArUco_details_dict.items():
        center = details[0]
        cv2.circle(image, center, 5, (0,0,255), -1)

        corner = ArUco_corners[int(ids)]
        cv2.circle(image, (int(corner[0][0]), int(corner[0][1])), 5, (50, 50, 50), -1)
        cv2.circle(image, (int(corner[1][0]), int(corner[1][1])), 5, (0, 255, 0), -1)
        cv2.circle(image, (int(corner[2][0]), int(corner[2][1])), 5, (128, 0, 255), -1)
        cv2.circle(image, (int(corner[3][0]), int(corner[3][1])), 5, (25, 255, 255), -1)

        tl_tr_center_x = int((corner[0][0] + corner[1][0]) / 2)
        tl_tr_center_y = int((corner[0][1] + corner[1][1]) / 2) 
        
        ##########################################################################
        #calculate the angle and give command
        degree_angle = calculate_angle(center,(tl_tr_center_x,tl_tr_center_y),next_point)
        command = robo_command(degree_angle)
        ###########################################################################

        cv2.line(image, center, (tl_tr_center_x, tl_tr_center_y), (255,0,0), 5)
        # cv2.line(image, center, next_point, (0,255,0), 2)
        cv2.circle(image, next_point, 5, (25, 255, 255), -1)
        
        display_offset = int(math.sqrt((tl_tr_center_x - center[0])**2 + (tl_tr_center_y - center[1])**2))
        cv2.putText(image, str(ids), (center[0]+int(display_offset/2), center[1]), cv2.FONT_HERSHEY_COMPLEX, 1, (0, 0, 255), 2)
        cv2.putText(image, str(degree_angle), (center[0]-display_offset, center[1]), cv2.FONT_HERSHEY_COMPLEX, 1, (0, 255, 0), 2)
        
    return image, command , center

ip = "172.20.10.4"
port = 8002

# Function to create a socket connection
def create_socket_connection():
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.bind((ip, port))
        s.listen()
        conn, addr = s.accept()
        print(f'connected to ip address : {addr}')
        return s, conn
    except Exception as e:
        print("Error creating socket connection:", e)
        return None, None
    
def calculate_distance(point1, point2):
  """
  Calculates the Euclidean distance between two points.

  Args:
    point1: A list representing a point [x, y].
    point2: A tuple representing a point (x, y).

  Returns:
    The Euclidean distance between the two points.
  """
  return math.sqrt((point1[0] - point2[0])**2 + (point1[1] - point2[1])**2)
#############################################################################
def a_star(graph, start, goal):
    frontier = []
    heapq.heappush(frontier, (0, start))
    came_from = {}
    cost_so_far = {}
    came_from[start] = None
    cost_so_far[start] = 0
    
    while frontier:
        _, current = heapq.heappop(frontier)
        
        if current == goal:
            break
        
        for next_node in graph.neighbors(current):
            new_cost = cost_so_far[current] + graph.cost(current, next_node)
            if next_node not in cost_so_far or new_cost < cost_so_far[next_node]:
                cost_so_far[next_node] = new_cost
                heapq.heappush(frontier, (new_cost, next_node))
                came_from[next_node] = current
    
    return came_from, cost_so_far

class Graph:
    def __init__(self):
        self.edges = {
            'A': ['B'],
            'B': ['F','D','C','A'],
            'C': ['B','G'],
            'D': ['B', 'E'],
            'E': ['J', 'D','F'],
            'F': ['E','B','G','I'],
            'G': ['H','C','F'],
            'H': ['I','G'],
            'I': ['H','J','F'],
            'J': ['I','E']
           
        }

    def neighbors(self, id):
        return self.edges[id]

    def cost(self, from_node, to_node):
        return 1  # assuming all edges have a cost of 1 for simplicity

def find_path(graph, path):
    all_paths = []
    for i in range(len(path) - 1):
        start = path[i]
        goal = path[i + 1]
        came_from, _ = a_star(graph, start, goal)
        current = goal
        sub_path = []
        while current != start:
            sub_path.append(current)
            current = came_from[current]
        sub_path.append(start)
        sub_path.reverse()
        all_paths.extend(sub_path)
    filtered_path = [all_paths[0]]
    for node in all_paths[1:]:
        if node != filtered_path[-1]:  # Filter out consecutive repeated nodes
            filtered_path.append(node)
    return filtered_path
###############################################################################################
key_value_list = ['H','F','A']
graph = Graph()
points_dict = {'A': (343, 405), 'B': (345, 314), 
               'C': (220, 306), 'D': (456, 303), 
               'E': (451, 215), 'F': (341, 207), 
               'G': (206, 213), 'H': (208, 114), 
               'I': (334, 108), 'J': (453, 102)}  

coordinates_list = []
# coordinates_list = [(300, 100), (450, 200), (150, 350), (500, 180)]  # Example coordinates
current_coordinate_index = 0  # Start with the first coordinate
###############################################################################################
if __name__ == "__main__":
    final_path = find_path(graph, key_value_list)
    for point in final_path:
        if point in points_dict:
            coordinates_list.append(points_dict[point])
    print(coordinates_list)
    cap = cv2.VideoCapture(0)
    screen_width = 1920  
    screen_height = 1080  

    video_width = int(screen_width / 2)
    video_height = int(screen_height/1.5)

    cv2.namedWindow("Camera Feed", cv2.WINDOW_NORMAL)
    cv2.resizeWindow("Camera Feed", video_width, video_height)
    cv2.moveWindow("Camera Feed", 0, 0)  

    s,conn = create_socket_connection()
    if s is None or conn is None:
        # Handle socket creation failure
        exit(1)
        
    last_command_sent = None
        
    while True:
        try:
            ret, frame = cap.read()
            if not ret:
                print("Failed to capture frame")
                break

            cv2.imshow("Camera Feed", frame)

            ArUco_details_dict, ArUco_corners = detect_ArUco_details(frame)

            next_point = coordinates_list[current_coordinate_index]

            marked_image, command ,center = mark_ArUco_image(frame, ArUco_details_dict, ArUco_corners, next_point)

            if last_command_sent is None or time.time() - last_command_sent >= 1:
                try:
                    conn.sendall(str.encode(command+'\n'))
                    last_command_sent = time.time()
                    print("Sent command:", command)
                except Exception as e:
                    print("Error:", e)
                    s,conn = create_socket_connection()

            # Check if robot has reached the current coordinate
            if calculate_distance(center, next_point) <= 5:  # Adjust threshold as needed
                current_coordinate_index = (current_coordinate_index + 1) % len(coordinates_list)
                print("Reached coordinate! Moving to next point.")
            cv2.imshow("Camera Feed", marked_image)
            
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
        except KeyboardInterrupt:
            break
        except Exception as e:
            print("Error:", e)

    s.close()  # Close the socket when done
    cap.release()
    cv2.destroyAllWindows()