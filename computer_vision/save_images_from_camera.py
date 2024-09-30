import cv2

cap = cv2.VideoCapture(1)

while True:
    success, img = cap.read()
    print(f"{success} {cap.get(3)} {cap.get(4)}")
    cv2.imshow("Webcam", img)
    key = cv2.waitKey(1000)

    i = 0
    if key == ord('q'):
        break
    elif key == ord('s'):
        cv2.imwrite("img_"+str(i)+".jpg", img)
        i += 1

cv2.destroyAllWindows()