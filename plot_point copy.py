import cv2

# Colors for text
colors = [(0, 0, 255), (255, 0, 0), (0, 255, 0), (255, 128, 0), (0, 128, 255),
         (128, 0, 255), (255, 255, 0), (0, 255, 255), (255, 0, 255), (128, 255, 0),
         (128, 0, 128), (0, 128, 128)]  # 12 random colors

# Font for text
font = cv2.FONT_HERSHEY_SIMPLEX

# Your coordinate list (already adjusted)
coordinates = [(343, 405), (345, 314), (220, 306), (456, 303), 
               (451, 215), (341, 207), (206, 213), (208, 114), 
               (334, 108), (453, 102)]






# Specify the indices of coordinates to draw lines between
# line_indices = [0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22]
line_indices = [0,1,2,3,4,5,6,7,8,9]
# Load the arena image
img = cv2.imread("arena_image1.jpg")

# Draw circles and print coordinates
for i, (x, y) in enumerate(coordinates, 1):
    if i > len(colors):  # Handle more than 12 coordinates
        i %= len(colors)  # Cycle through colors
    cv2.circle(img, (x, y), 5, colors[i-1], -1)

    # Print coordinates next to the point
    text = f"({x}, {y})"
    (text_width, text_height) = cv2.getTextSize(text, font, 0.7, 2)[0]
    text_offset_x = x + 5 if x + text_width < img.shape[1] else x - text_width - 5
    text_offset_y = y + text_height if y + text_height < img.shape[0] else y - text_height - 5
    cv2.putText(img, text, (text_offset_x, text_offset_y), font, 0.7, colors[i-1], 2)

# Draw lines between specified coordinates
for idx in range(len(line_indices) - 1):
    start_idx = line_indices[idx]
    end_idx = line_indices[idx + 1]
    start_point = coordinates[start_idx]
    end_point = coordinates[end_idx]
    cv2.line(img, start_point, end_point, (0, 255, 255), 2)

# Display the image
cv2.imshow("Arena with Coordinates and Lines", img)
cv2.waitKey(0)  # Wait for key press to close
cv2.destroyAllWindows()
