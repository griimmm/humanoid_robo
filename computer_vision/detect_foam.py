import cv2
import numpy as np
import imutils

def find_foam(frame):
    '''
    1) finding the white colours
    2) select and draw the largest contour
    3) bitwise-and with the frame to get the contour in frame
    4) mark the top right corner of the contour
    5) Draw a line from top right to the left edge
    '''
    font = cv2.FONT_HERSHEY_COMPLEX 
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    lower_white = np.array([0,0,168], dtype=np.uint8)
    upper_white = np.array([172,111,255], dtype=np.uint8)
    
    color_mask = cv2.inRange(hsv,lower_white,upper_white)

    kernel = np.ones((3,3), np.uint8)
    color_mask = cv2.dilate(cv2.erode(color_mask, kernel, iterations=10), kernel, iterations=10)

    cnts = cv2.findContours(color_mask,cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    cnts = imutils.grab_contours(cnts)

    if cnts:
        c = max(cnts, key=cv2.contourArea)
        foam_mask = np.zeros(frame.shape[:2], dtype=np.uint8)
        cv2.drawContours(foam_mask, [c], 0, 255, -1)

        res = cv2.bitwise_and(frame, frame, mask=foam_mask)

        # draws a polygon over the largest contour
        epsilon = 0.02 * cv2.arcLength(c, True)
        approx = cv2.approxPolyDP(c, epsilon, True)
        
        frame_with_polygon = res.copy()
        cv2.drawContours(frame_with_polygon, [approx], 0, (0, 255, 0), 2)

        top_right = tuple(approx[approx[:, :, 0].argmax()][0])
        leftmost_point = tuple(approx[approx[:, :, 0].argmin()][0])
        num_points = 5 # number of circles to plot

        dist = top_right[0] - leftmost_point[0]
        dist_btw_pnts = dist / (num_points - 1)

        # draws circles from right to left
        points = []
        for i in range(num_points):
            y = top_right[1]
            x = int(top_right[0] - i * dist_btw_pnts)
            
            point = (x, y)
            points.append(point)
            coord_text = f"{x},{y}"

            cv2.circle(frame_with_polygon, point, 5, (0, 0, 255), -1)  
            cv2.putText(frame_with_polygon, coord_text, (x, y - 10), 
                font, 0.5, (0, 0, 255), 1)

        cv2.polylines(frame_with_polygon, [np.array(points)], False, (0, 0, 255 ), 2)

         
        cv2.imshow('frame', frame)
        cv2.imshow('largest white contour', foam_mask)
        cv2.imshow('result', res)
        cv2.imshow('polygon', frame_with_polygon)

        
    else:
        print("No foam")
        return

    return cv2.waitKey(0) & 0xFF

def main():
    images = []

    for i in range(5,21):
        img_path = f'/home/rishikesh/Desktop/hr/humanoid_robo/computer_vision/styrofoam_newv3/img_{i+1}.jpg'
        w = 800
        h = 600
        img = cv2.imread(img_path)
        if img is None:
            print(f"Failed to load image: {img_path}")
            continue
        rs_img = cv2.resize(img,(w,h))
        images.append(rs_img)

    for i, img in enumerate(images):
        print(f"Processing image {i+1}")
        key = find_foam(img)
        
        if key == ord('q'):
            break

    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()