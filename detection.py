import numpy as np
import cv2
import cv2.aruco as aruco # type: ignore
import math
import os
from picamera2 import Picamera2
import time

class Detection:
    def __init__(self):
        
        self._stop = False # for stop the program 
        self.true = True # continuing the detection loop

        self.marker_size = 5  # Marker size in centimeters

        # Camera matrix and distortion coefficients (obtained via calibration)
        self.camera_matrix = np.array(((1000.15867, 0, 700.59), (0, 933.1586, 400.36993), (0, 0, 1)))
        self.camera_distortion = np.array([0.0, 0.0, 0.0, 0.0, 0.0])
        
        # Flipping rotation matrix for alignment purposes
        self.R_flip = np.zeros((3, 3), dtype=np.float32)
        self.R_flip[0, 0] = 1.0
        self.R_flip[1, 1] = -1.0
        self.R_flip[2, 2] = -1.0

        # Markers importing 
        self.aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
        self.parameters = aruco.DetectorParameters()

        # Camera configuration
        self.cap = Picamera2()

        self.cap.preview_configuration.main.size =(800, 600)
        self.cap.preview_configuration.main.format = "RGB888"
        self.cap.configure("preview")
        self.cap.start()

        # Data for position and orientation [X, Y, Rotation, idNo, done{if done 0}]
        self.data = [0, 0, 0, 9] 

        pass

    def isRotationMatrix(self, R):
        Rt = np.transpose(R)
        shouldBeIdentity = np.dot(Rt, R)
        I = np.identity(3, dtype=R.dtype)
        n = np.linalg.norm(I - shouldBeIdentity)
        return n < 1e-6

    def rotationMatrixToEulerAngles(self, R):
        assert (self.isRotationMatrix(R))
        sy = math.sqrt(R[0, 0] * R[0, 0] + R[1, 0] * R[1, 0])
        singular = sy < 1e-6

        if not singular:
            z = math.atan2(R[1, 0], R[0, 0])
        else:
            z = 0

        return np.array([z])

    # Function to calculate Homography
    def calculate_homography(self, corners, marker_size):
        # Define real-world coordinates (marker size in cm)
        pts_dst = np.array([[0, 0], 
                            [marker_size, 0], 
                            [marker_size, marker_size], 
                            [0, marker_size]], dtype='float32')

        # Get the corners of the detected marker from the image
        pts_src = np.array(corners[0], dtype='float32')

        # Compute Homography matrix
        homography_matrix, _ = cv2.findHomography(pts_src, pts_dst)
        return homography_matrix

    def detect(self):

        def define_dt(distance):
            if distance > 24.5:
                return 10.0, -10.1
            if distance > 21.5:
                return 8.3, -9.9
            if distance > 18.5:
                return 8.1, -9.7
            if distance > 16.5:
                return 7.3, -8.5
            if distance > 14.5:
                return 7.4, -7.8
            if distance > 12.5:
                return 6.8, -6.1
            else:
                return 5.3, -6.1

        time.sleep(0.5)

        self.true = True  # Start the detection

        previous_x = None  # Keep track of the previous x value

        while self.true:
            if self._stop:
                self.cap.release()
                break

            frame = self.cap.capture_array()
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            corners, ids, _ = aruco.detectMarkers(image=gray, dictionary=self.aruco_dict, parameters=self.parameters)

            if ids is not None:
                ret = aruco.estimatePoseSingleMarkers(corners, self.marker_size, self.camera_matrix, self.camera_distortion)
                rvec, tvec = ret[0][0, 0, :], ret[1][0, 0, :]

                # Draw detected markers
                # aruco.drawDetectedMarkers(frame, corners)
                # cv2.drawFrameAxes(frame, self.camera_matrix, self.camera_distortion, rvec, tvec, 5)

                # Homography calculation
                homography_matrix = self.calculate_homography(corners, self.marker_size)

                # Compute rotation matrix and yaw marker
                R_ct = np.matrix(cv2.Rodrigues(rvec)[0])
                R_tc = R_ct.T
                yaw_marker = self.rotationMatrixToEulerAngles(self.R_flip * R_tc)

                y = round(-(tvec[1]  - 13.9), 1)
                dx, dangle = define_dt(y)

                # Calculate the corrected x position
                corrected_x = round(tvec[0] + dx, 1)

                # Introduce a threshold to avoid constant correction
                if previous_x is None or abs(corrected_x - previous_x) > 0.5:
                    x = corrected_x
                    previous_x = corrected_x  # Update previous x
                else:
                    x = previous_x  # Maintain the previous corrected value if change is small

                angle = round(math.degrees(yaw_marker) + dangle, 1)

                # Update data
                self.data = [x, y, angle, ids[0].item()]
                
                # Clear console and display information
            #     os.system('cls' if os.name == 'nt' else 'clear')
            #     print(f"id is :{ids[0]}")
            #     print(f"MARKER Position \nx Distance ={x}  y Distance = {y}  \nRotation = {angle}")
            #     print(dx, dangle)
            #     print(self.data)
            #     print()
            # cv2.imshow('frame', frame)

            # key = cv2.waitKey(1) & 0xFF
            # if key == ord('q'):
            #     self.cap.release()
            #     cv2.destroyAllWindows()
            #     break
            
    def stop_detection(self):
        self.true = False
        pass

    def stop_game(self):
        self._stop = True
        

    def reset(self):
        self.data = [0, 0, 0, -9, 0]
        print("done task!")

if __name__ == '__main__':
    det = Detection()
    det.detect() 
