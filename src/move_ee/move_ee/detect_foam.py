import cv2
import numpy as np
import imutils

def find_foam(frame,shape): #shape is the shape of contour to cut
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

    points_to_cut = []

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

        # points list
        points_list = []

        # draws circles from right to left. circles are yellow
        points = []
        if shape == "level": # levels the upper edge
            for i in range(num_points):
                y = top_right[1]
                x = int(top_right[0] - i * dist_btw_pnts)
                
                point = (x, y)
                
                points_list.append(point)
                points.append(point)
                coord_text = f"{x},{y}"

                cv2.circle(frame_with_polygon, point, 5, (0, 255, 255), -1)  
                cv2.putText(frame_with_polygon, coord_text, (x, y - 10), 
                    font, 0.5, (0, 255, 255), 1)
                
                # cv2.polylines(frame_with_polygon, [np.array(points)], False, (0, 0, 255 ), 2)

            points_to_cut = points_list

                # degmos

            return points

                
        elif shape == "circle": #contours of a circle to the foam
            M = cv2.moments(c) # calculates moments
            if M['m00'] != 0: 
                cx = int(M['m10'] / M['m00']) # centroid x
                cy = int(M['m01'] / M['m00']) # '' y
            
            fixed_distance = 20 # distance from top 
            fixed_radius = 80#100 #circle radius

            while True:
                # Get the topmost point of the circle 
                topmost_point = (cx, cy - fixed_radius)  # Adjust based on radius
                distance_from_top = topmost_point[1] - fixed_distance
                
                # Check if the circle's topmost point is at or below the fixed distance
                if distance_from_top <= 50:
                    break

                # Move the centroid by a fixed amount
                cy -= 10  
                                
            # Draw the circle at the centroid with the specified radius
            # cv2.circle(frame_with_polygon, (cx, cy), fixed_radius, (0, 0, 255), -1)

            #  draw the centroid point
            cv2.circle(frame_with_polygon, (cx, cy), 5, (0, 255, 0), -1)

            # Calculate and draw 20 points around the circle (adjustable)
            num_points = 20
            circle_points = []
            
            for i in range(num_points):
                angle = i * (360 / num_points)  # Calculate angle in degrees
                rad = np.radians(angle)  # Convert to radians
                
                # Calculate the coordinates of the points
                point_x = int(cx + fixed_radius * np.cos(rad))
                point_y = int(cy + fixed_radius * np.sin(rad))
                
                circle_points.append((point_x, point_y))
                cv2.circle(frame_with_polygon, (point_x, point_y), 5, (0, 255, 255), -1)  
                coord_text = f"{point_x},{point_y}"
                cv2.putText(frame_with_polygon, coord_text, (point_x + 5, point_y - 5),font, 0.5, (0, 255, 255), 1)

            points_to_cut = circle_points
        
            # return circle_points


        elif shape == "tree-like":
            M = cv2.moments(c)  # Calculate moments
            if M['m00'] != 0: 
                cx = int(M['m10'] / M['m00'])  # Centroid x
                cy = int(M['m01'] / M['m00'])  # Centroid y
            
            fixed_distance = 20  # Distance from top 
            fixed_radius = 80   # Circle radius

            cx1 = cx
            cy1 = cy
            while True:
                # Get the topmost point of the circle 
                topmost_point = (cx1, cy1 - fixed_radius)  
                distance_from_top = topmost_point[1] - fixed_distance
                
                # Check if the circle's topmost point is at or below the fixed distance
                if distance_from_top <= 50:
                    break

                # Move the centroid by a fixed amount
                cy1 -= 10  
                                    
            # Draw the circle at the centroid with the specified radius
            # cv2.circle(frame_with_polygon, (cx1, cy1), fixed_radius, (0, 0, 255), -1)

            # Calculate and draw upper semi-circle points around the circle
            num_points = 10  # More points for smoother appearance
            upper_points = []
            
            for i in range(num_points + 1):  # 0 to 10, so 11 points
                angle = i * (180 / num_points)  # Calculate angle in degrees for upper semi-circle
                rad = np.radians(angle)  # Convert to radians
                
                # Calculate the coordinates of the points for the upper semi-circle
                point_x = int(cx + fixed_radius * np.cos(rad))

                # Subtract to get upper semi-circle
                point_y = int(cy1 - fixed_radius * np.sin(rad))  

                upper_points.append((point_x, point_y))
                
                # Draw the points
                cv2.circle(frame_with_polygon, (point_x, point_y), 5, (0, 255, 255), -1)  
                
                # Add coordinate text in yellow
                coord_text = f"({point_x},{point_y})"
                cv2.putText(frame_with_polygon, coord_text, (point_x + 5, point_y - 5),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)

            # Define fixed offsets for leftmost and rightmost points
            offset = 130  # Change this value as needed

            leftmost_point = (cx - offset, cy)  # Left of the centroid
            rightmost_point = (cx + offset, cy)  # Right of the centroid

            # Draw the leftmost and rightmost points in yellow
            cv2.circle(frame_with_polygon, leftmost_point, 5, (0, 255, 255), -1)
            cv2.circle(frame_with_polygon, rightmost_point, 5, (0, 255, 255), -1)

            # Add coordinate text for leftmost and rightmost points in yellow
            cv2.putText(frame_with_polygon, f"({leftmost_point[0]},{leftmost_point[1]})", 
                        (leftmost_point[0] + 5, leftmost_point[1] - 5),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)
            cv2.putText(frame_with_polygon, f"({rightmost_point[0]},{rightmost_point[1]})", 
                        (rightmost_point[0] + 5, rightmost_point[1] - 5),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)

            # Add points below the ends of the upper semi-circle
            left_end = upper_points[0]
            right_end = upper_points[-1]

            # Calculate the y-coordinate for the new points (same as centroid)
            new_y = cy

            # Create new points below the ends of the semi-circle
            new_left_point = (left_end[0], new_y)
            new_right_point = (right_end[0], new_y)

            # Draw the new points in yellow
            cv2.circle(frame_with_polygon, new_left_point, 5, (0, 255, 255), -1)
            cv2.circle(frame_with_polygon, new_right_point, 5, (0, 255, 255), -1)

            # Add coordinate text for new points in yellow
            cv2.putText(frame_with_polygon, f"({new_left_point[0]},{new_left_point[1]})", 
                        (new_left_point[0] + 5, new_left_point[1] - 5),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)
            cv2.putText(frame_with_polygon, f"({new_right_point[0]},{new_right_point[1]})", 
                        (new_right_point[0] + 5, new_right_point[1] - 5),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)

            # # Draw lines connecting the upper points to the new lower points
            # cv2.line(frame_with_polygon, left_end, new_left_point, (0, 255, 0), 1)
            # cv2.line(frame_with_polygon, right_end, new_right_point, (0, 255, 0), 1)

            # # Connect the new lower points to the edge points
            # cv2.line(frame_with_polygon, new_left_point, leftmost_point, (0, 255, 0), 1)
            # cv2.line(frame_with_polygon, new_right_point, rightmost_point, (0, 255, 0), 1)

            cut_points = [leftmost_point,left_end] + upper_points + [right_end,rightmost_point]
            # print(cut_points)
            # return cut_points
            points_to_cut = cut_points
                    
        # cv2.imshow('frame', frame)
        # cv2.imshow('largest white contour', foam_mask)
        # cv2.imshow('result', res)
        # cv2.imshow('polygon', frame_with_polygon)

        
        # cv2.waitKey(0) & 0xFF 
    else:
        print("No foam")
        return

    return points_to_cut

def main():
    images = []

    for i in range(5,21):
        img_path = f'/home/rishikesh/Desktop/hr/humanoid_robo/computer_vision/styrofoam_newv4/img_{i+1}.jpg'
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
        key = find_foam(img,'tree-like')
        
        if key == ord('q'):
            break

    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()