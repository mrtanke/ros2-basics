import face_recognition
import cv2
from ament_index_python.packages import get_package_share_directory # get package absolute path
import os

def main():
    # 1. Get the real path of the image
    default_image_path = os.path.join(get_package_share_directory('demo_python_service'), 'resource', 'default.jpg')

    # 2. Load the image using cv2
    image = cv2.imread(default_image_path)

    # 3. Detect faces in the image using face_recognition
    face_locations = face_recognition.face_locations(image, number_of_times_to_upsample=1, model="hog")

    # 4. Draw rectangles around detected faces
    for (top, right, bottom, left) in face_locations:
        cv2.rectangle(image, (left, top), (right, bottom), (0, 255, 0), 4)

    # 5. Display the image with detected faces
    cv2.imshow("Detected Faces", image)
    cv2.waitKey(0)
    cv2.destroyAllWindows()