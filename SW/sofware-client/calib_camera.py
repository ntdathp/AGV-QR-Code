import cv2
import numpy as np
from pyzbar.pyzbar import decode


def detectQRcode(image):
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    objectQRcode = decode(gray)
    for obDecoded in objectQRcode:
        x, y, w, h = obDecoded.rect
        cv2.rectangle(image, (x, y), (x + w, y + h), (0, 255, 0), 2)
        points = obDecoded.polygon
        if len(points) > 4:
            hull = cv2.convexHull(
                np.array([points for point in points], dtype=np.float32))
            hull = list(map(tuple, np.squeeze(hull)))
        else:
            hull = points
        return hull, (x + w // 2, y + h // 2)
    return None, None


cap = cv2.VideoCapture(0)  # M? k?t n?i v?i camera

while True:
    ret, frame = cap.read()
    if not ret:
        break

    # Hi?n th? khung h?nh
    cv2.imshow("Frame", frame)

    # Nh?n 's' �? ch?p ?nh t? camera
    if cv2.waitKey(1) & 0xFF == ord('s'):
        # L�u ?nh v?a ch?p
        cv2.imwrite("captured_image.jpg", frame)
        print("�? ch?p ?nh!")

        # �?c ?nh v?a ch?p v� ph�t hi?n QR code
        captured_image = cv2.imread("captured_image.jpg")
        hull_points, qr_center = detectQRcode(captured_image)

        # N?u ph�t hi?n ��?c QR code
        if hull_points:
            pt1, pt2, pt3, pt4 = hull_points

            # T�nh to�n kho?ng c�ch gi?a c�c c?nh c?a QR code
            edge_distance_1 = np.linalg.norm(np.array(pt1) - np.array(pt2))
            edge_distance_2 = np.linalg.norm(np.array(pt2) - np.array(pt3))
            edge_distance_3 = np.linalg.norm(np.array(pt3) - np.array(pt4))
            edge_distance_4 = np.linalg.norm(np.array(pt4) - np.array(pt1))

            print("Length 1:", edge_distance_1)
            print("Length 2:", edge_distance_2)
            print("Length 3:", edge_distance_3)
            print("Length 4:", edge_distance_4)
            print("----------")

            # V? khung v� c�c �i?m g�c QR code l�n ?nh
            cv2.fillPoly(captured_image, [np.array(hull_points)], (255, 0, 0))
            cv2.circle(captured_image, pt1, 3, (0, 255, 0), 3)
            cv2.circle(captured_image, pt2, 3, (255, 0, 0), 3)
            cv2.circle(captured_image, pt3, 3, (0, 0, 255), 3)
            cv2.circle(captured_image, pt4, 3, (255, 255, 0), 3)
            cv2.circle(captured_image, qr_center, 3, (0, 255, 0), -1)
            cv2.putText(captured_image, f'Detection: Pyzbar', (30, 80), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 255), 2,
                        cv2.LINE_AA)

            # Hi?n th? ?nh v?i c�c th�ng tin �? ph�t hi?n
            cv2.imshow("Captured Image", captured_image)

            # Ghi ?nh �? ch?p v� ��?c ph�t hi?n QR code v�o t?p
            cv2.imwrite("detected_qr_code.jpg", captured_image)
            print("�? l�u ?nh ch?a QR code v� th�ng tin ph�t hi?n!")

    # Nh?n 'q' �? tho�t
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Gi?i ph�ng t�i nguy�n v� ��ng c?a s?
cap.release()
cv2.destroyAllWindows()
