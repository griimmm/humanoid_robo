import cv2
import numpy as np
import imutils

def nothing(x):
    pass

def find_foam(frame):
    '''
    1) finding the white colours
    2) select and draw the largest contour
    3) bitwise-and with the frame to get the contour in frame
    '''
    cv2.namedWindow('HSV Trackbars')
    
    # Create trackbars for HSV range
    cv2.createTrackbar('H_low', 'HSV Trackbars', 0, 179, nothing)
    cv2.createTrackbar('S_low', 'HSV Trackbars', 0, 255, nothing)
    cv2.createTrackbar('V_low', 'HSV Trackbars', 168, 255, nothing)

    cv2.createTrackbar('H_high', 'HSV Trackbars', 172, 179, nothing)
    cv2.createTrackbar('S_high', 'HSV Trackbars', 111, 255, nothing)
    cv2.createTrackbar('V_high', 'HSV Trackbars', 255, 255, nothing)

    while True:
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # Get trackbar positions
        h_low = cv2.getTrackbarPos('H_low', 'HSV Trackbars')
        s_low = cv2.getTrackbarPos('S_low', 'HSV Trackbars')
        v_low = cv2.getTrackbarPos('V_low', 'HSV Trackbars')
        
        h_high = cv2.getTrackbarPos('H_high', 'HSV Trackbars')
        s_high = cv2.getTrackbarPos('S_high', 'HSV Trackbars')
        v_high = cv2.getTrackbarPos('V_high', 'HSV Trackbars')

        lower_white = np.array([h_low, s_low, v_low], dtype=np.uint8)
        upper_white = np.array([h_high, s_high, v_high], dtype=np.uint8)
        
        color_mask = cv2.inRange(hsv, lower_white, upper_white)

        cnts = cv2.findContours(color_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        cnts = imutils.grab_contours(cnts)

        if cnts:
            c = max(cnts, key=cv2.contourArea)
            foam_mask = np.zeros(frame.shape[:2], dtype=np.uint8)
            cv2.drawContours(foam_mask, [c], 0, 255, -1)

            res = cv2.bitwise_and(frame, frame, mask=foam_mask)

            cv2.imshow('frame', frame)
            cv2.imshow('largest white contour', foam_mask)
            cv2.imshow('result', res)
        else:
            cv2.imshow('frame', frame)
            cv2.imshow('largest white contour', np.zeros_like(frame))
            cv2.imshow('result', np.zeros_like(frame))
            print("No foam")

        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break
        elif key == ord('n'):
            return  # Move to next image

    cv2.destroyAllWindows()

def main():
    images = []

    for i in range(4):
        img_path = f'styrofoam_imgcopy/img_{i+1}.jpg'
        w = 800
        h = 600
        img = cv2.imread(img_path)
        if img is None:
            print(f"Failed to load image: {img_path}")
            continue
        rs_img = cv2.resize(img, (w, h))
        images.append(rs_img)

    for i, img in enumerate(images):
        print(f"Processing image {i+1}")
        print("Adjust HSV values using trackbars. Press 'n' for next image or 'q' to quit.")
        find_foam(img)

if __name__ == "__main__":
    main()