import cv2

# Colors for text
colors = [(0, 0, 255), (255, 0, 0), (0, 255, 0), (255, 128, 0), (0, 128, 255),
         (128, 0, 255), (255, 255, 0), (0, 255, 255), (255, 0, 255), (128, 255, 0),
         (128, 0, 128), (0, 128, 128)]  # 12 random colors

# Font for text
font = cv2.FONT_HERSHEY_SIMPLEX

points_dict = {'A': (343, 405), 'B': (345, 314), 'C': (220, 306), 'D': (456, 303), 
               'E': (451, 215), 'F': (341, 207), 'G': (206, 213), 'H': (208, 114), 
               'I': (334, 108), 'J': (453, 102)}  # Store click coordinates in a dictionary

# Load the arena image
img = cv2.imread("arena_image1.jpg")

# Iterate through the points dictionary
for key, (x, y) in points_dict.items():
    color_index = (ord(key) - 65) % len(colors)
    # Draw circle at the point
    cv2.circle(img, (x, y), 5, colors[color_index], -1)

    # Print key and coordinates next to the point
    text = f"{key}: ({x}, {y})"
    (text_width, text_height) = cv2.getTextSize(text, font, 0.7, 2)[0]
    text_offset_x = x + 5 if x + text_width < img.shape[1] else x - text_width - 5
    text_offset_y = y + text_height if y + text_height < img.shape[0] else y - text_height - 5
    cv2.putText(img, text, (text_offset_x, text_offset_y), font, 0.7, colors[color_index], 2)

# Display the image
cv2.imshow("Arena with Points", img)
cv2.waitKey(0)  # Wait for key press to close
cv2.destroyAllWindows()
