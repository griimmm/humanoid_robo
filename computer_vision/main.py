import cv2

def main():
    cap = cv2.VideoCapture(1)
    while True:
        success, img = cap.read()

        # TODO: write code here idk

        cv2.imshow("Webcam", img)
        if cv2.waitKey(1000) == ord('q'):
            break

    cv2.destroyAllWindows()

if __name__=='__main__':
    main()