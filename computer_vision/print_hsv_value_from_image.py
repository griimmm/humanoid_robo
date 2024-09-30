import cv2
import numpy as np

for i in range(4):
    img_path = f'styrofoam_imgcopy/img_{i+1}.jpg'
    # Load the image
    image = cv2.imread(img_path)

    # Convert BGR to HSV
    hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

    # Function to get HSV value on mouse click
    def get_hsv_value(event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            hsv_value = hsv_image[y, x]
            print(f'HSV Value at ({x}, {y}): {hsv_value}')

    # Set up mouse callback
    cv2.namedWindow('image')
    cv2.setMouseCallback('image', get_hsv_value)

    # Display the image
    while True:
        cv2.imshow('image', image)
        if cv2.waitKey(1) & 0xFF == 27:  # Press 'Esc' to exit
            break

    cv2.destroyAllWindows()
