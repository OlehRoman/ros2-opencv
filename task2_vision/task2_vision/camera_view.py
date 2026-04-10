import cv2

def main():
    cap = cv2.VideoCapture(0)

    if not cap.isOpened():
        print("ERROR: camera not opened")
        return

    while True:
        ret, frame = cap.read()

        if not ret:
            print("ERROR: failed to read frame")
            break

        cv2.imshow("camera_view", frame)

        key = cv2.waitKey(1)
        if key == 27:
            break

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
