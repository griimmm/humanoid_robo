import cv2
import numpy as np
import imutils

def find_foam(frame):
    '''
    1) finding the white colours
    2) select and draw the largest contour
    3) bitwise-and with the frame to get the contour in frame

    '''
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # these may need fine tuning based on the clour of the foam that we may use
    # lower_white = np.array([0,0,168], dtype=np.uint8)
    # upper_white = np.array([18,100,255], dtype=np.uint8)
    lower_white = np.array([20,20,120], dtype=np.uint8)
    upper_white = np.array([40,80,200], dtype=np.uint8)

    # lower_white = np.array([0,0,168], dtype=np.uint8)
    # upper_white = np.array([172,111,255], dtype=np.uint8)
    
    color_mask = cv2.inRange(hsv,lower_white,upper_white)

    # NEW:
    # kernel = np.ones((3,3), np.uint8)
    # color_mask = cv2.erode(cv2.dilate(color_mask, kernel, iterations=3), kernel, iterations=3)

    # kernel = np.ones((3,3), np.uint8)
    # color_mask = cv2.dilate(cv2.erode(color_mask, kernel, iterations=1), kernel, iterations=1)
    # color_mask = cv2.erode(cv2.dilate(color_mask, kernel, iterations=20), kernel, iterations=20)

    cnts = cv2.findContours(color_mask,cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    cnts = imutils.grab_contours(cnts)

    if cnts:
        c = max(cnts, key=cv2.contourArea)
        foam_mask = np.zeros(frame.shape[:2], dtype=np.uint8)
        cv2.drawContours(foam_mask, [c], 0, 255, -1)


        res = cv2.bitwise_and(frame, frame, mask=foam_mask)

        cv2.imshow('frame',frame)
        cv2.imshow('largest white contour', foam_mask)
        cv2.imshow('result', res)
    else:
        print("No foam")
        return
    

    cv2.waitKey(0)


# need to convert this to allow live feed
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
        rs_img = cv2.resize(img,(w,h))
        images.append(rs_img)

    for i, img in enumerate(images):
        print(f"Processing image {i+1}")
        find_foam(img)
        cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
    