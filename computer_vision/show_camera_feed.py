import cv2

cap = cv2.VideoCapture(1)

while True:
    success, img = cap.read()
    print(f"{success} {cap.get(3)} {cap.get(4)}")
    cv2.imshow("Webcam", img)
    if cv2.waitKey(1000) == ord('q'):
        break

cv2.destroyAllWindows()