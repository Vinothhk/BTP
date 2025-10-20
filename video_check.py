import cv2

v = cv2.VideoCapture(2)
while True:
    ret, frame = v.read()
    if not ret:
        break
    cv2.imshow("Video Feed", frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
v.release()
cv2.destroyAllWindows()
