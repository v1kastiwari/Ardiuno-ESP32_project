import numpy as np
import cv2
from cv2 import aruco
import math

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
    elif degree_angle < -60:
        command = 'Sleft'
    else:
        command = 'forward'
    return command

 
def mark_ArUco_image(image,ArUco_details_dict, ArUco_corners,next_point):

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
        # print((tl_tr_center_x,tl_tr_center_y))
        ##########################################################################
        #calculate the angle and give command
        degree_angle = calculate_angle(center,(tl_tr_center_x,tl_tr_center_y),next_point)
        command = robo_command(degree_angle)
        # print(command)
        # print(degree_angle)
        ###########################################################################
        cv2.line(image,center,(tl_tr_center_x, tl_tr_center_y),(255,0,0),5)
        ########################################################################
        # next_point is the external point
        cv2.line(image,center,next_point,(0,255,0),2)
        ########################################################################
        display_offset = int(math.sqrt((tl_tr_center_x - center[0])**2+(tl_tr_center_y - center[1])**2))
        cv2.putText(image,str(ids),(center[0]+int(display_offset/2),center[1]),cv2.FONT_HERSHEY_COMPLEX, 1, (0, 0, 255), 2)
        angle = details[1]
        # cv2.putText(image,str(angle),(center[0]-display_offset,center[1]),cv2.FONT_HERSHEY_COMPLEX, 1, (0, 255, 0), 2)
        cv2.putText(image,str(degree_angle),(center[0]-display_offset,center[1]),cv2.FONT_HERSHEY_COMPLEX, 1, (0, 255, 0), 2)
    return image , command , center

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


coordinates_list = [(300, 100), (450, 200), (150, 350), (500, 180)]  # Example coordinates
current_coordinate_index = 0  # Start with the first coordinate

if __name__ == "__main__":
    # Open the default camera (usually index 0)
    # cap = cv2.VideoCapture('aruco_final.mp4')
    cap = cv2.VideoCapture(0)
    # Get the screen resolution
    screen_width = 1920  # Change this according to your screen resolution
    screen_height = 1080  # Change this according to your screen resolution

    # Calculate the width and height for the video window (left half of the screen)
    # video_width = int(screen_width / 2)
    # video_height = int(screen_height/1.5)

    # Create a window to display the video feed
    cv2.namedWindow("Camera Feed", cv2.WINDOW_NORMAL)
    
    # Set the position and size of the window
    # cv2.resizeWindow("Camera Feed", video_width, video_height)
    cv2.moveWindow("Camera Feed", 0, 0)  # Move window to the top-left corner
    
    while True:
        try:

            # Capture frame-by-frame
            ret, frame = cap.read()

            # Check if the frame was successfully captured
            if not ret:
               print("Failed to capture frame")
               break
            cv2.imwrite("arena_image1.jpg", frame)
            # Display the captured frame
            cv2.imshow("Camera Feed", frame)

            # Perform ArUco detection and processing
            ArUco_details_dict, ArUco_corners = detect_ArUco_details(frame)
            # print("Detected details of ArUco: ", ArUco_details_dict)
            next_point = coordinates_list[current_coordinate_index]
            # Display the marked image
            marked_image, command , center = mark_ArUco_image(frame, ArUco_details_dict, ArUco_corners,next_point)
            # Check if robot has reached the current coordinate
            if calculate_distance(center, next_point) <= 5 :  # Adjust threshold as needed
                current_coordinate_index = (current_coordinate_index + 1) % len(coordinates_list)
                print("Reached coordinate! Moving to next point.")
            cv2.imshow("Camera Feed", marked_image)

            cv2.imshow("Camera Feed", marked_image)
            print(command)

            # Exit if 'q' is pressed
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
        except:
            pass
    cap.release()
    cv2.destroyAllWindows()
