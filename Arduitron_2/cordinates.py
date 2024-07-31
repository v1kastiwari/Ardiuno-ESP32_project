import cv2

# Colors for drawing circles
colors = [(255, 0, 0), (0, 255, 0), (0, 0, 255), 
          (255, 128, 0), (0, 128, 255),(128, 0, 255), 
          (255, 255, 0), (0, 255, 255), (255, 0, 255), 
          (128, 255, 0),(128, 0, 128), (0, 128, 128), 
          (255, 165, 0), (255, 0, 255),(255,180,0),
          (27,255,128),(56,255,265),(38,234,30),
          (30,255,128),(39,69,100),(10,127,130),
          (222,221,30),(35,67,220)
          ] 

# Global variables
drawing = False
click_count = 0  # Keep track of clicked points
points_dict = {}  # Store click coordinates in a dictionary

# Mouse callback function
def draw_circle(event, x, y, flags, param):
    global drawing, click_count, points_dict, colors

    if event == cv2.EVENT_LBUTTONDOWN:
        drawing = True

    elif event == cv2.EVENT_LBUTTONUP:
        drawing = False
        if click_count < len(colors):
            # Draw circle and store coordinates in the dictionary
            cv2.circle(img, (x, y), 5, colors[click_count], -1)
            key = chr(65 + click_count)  # Use ASCII char codes for A-Z
            points_dict[key] = (x, y)
            click_count += 1
            print(f"Point {key}: ({x}, {y})")

        if click_count == len(colors):
            cv2.imshow('image', img)
            print("All points drawn! Press Esc to exit.")

# Read an image from file (replace with your image path)
img = cv2.imread("arena_image1.jpg")

# Create a window and set the callback function
cv2.namedWindow('image')
cv2.setMouseCallback('image', draw_circle)

while click_count < len(colors):
    cv2.imshow('image', img)

    # Press 'esc' key to exit
    if cv2.waitKey(1) == 27:
        break

cv2.destroyAllWindows()

# Print and return final dictionary of points
print("Final list of points:", points_dict)

# You can access individual points using points_dict[key] (e.g., points_dict['A'] for the first point)