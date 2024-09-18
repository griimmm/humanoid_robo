import cv2

cap = cv2.VideoCapture(1)
image_counter = 1

while True:
    success, img = cap.read()

    cv2.imshow("Webcam", img)
    key = cv2.waitKey(1)
    if key == ord('q'):
        break
    elif key == ord('s'):
        cv2.imwrite("image"+str(image_counter)+".jpg", img)
        image_counter += 1
        print("Saved image")

cv2.destroyAllWindows()