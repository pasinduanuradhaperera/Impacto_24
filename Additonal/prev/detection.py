import numpy as np
import cv2
import cv2.aruco as aruco
import math
import os
from picamera2 import Picamera2
import time

class Detection:
    def __init__(self):
        self.marker_size = 5  # - [cm]
        
        self.camera_matrix = np.array(((1000.15867, 0, 700.59), (0, 933.1586, 400.36993), (0, 0, 0)))
        self.camera_distortion = np.array((-0.0, 0.0, 0, 0))
        
        # calculations 
        self.R_flip = np.zeros((3, 3), dtype=np.float32)
        self.R_flip[0, 0] = 1.0
        self.R_flip[1, 1] = -1.0
        self.R_flip[2, 2] = -1.0

        # markers importing 
        self.aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
        self.parameters = aruco.DetectorParameters()
        
        # camera configurations 
        self.cap = Picamera2()
        # self.cap.preview_configuration.main.size =(800, 600)
        self.cap.preview_configuration.main.format="RGB888"
        self.cap.configure("preview")
        self.cap.start()
        
        # data 
        self.data = [0,0,0,9] # [X, Y, Rotation, idNo]
        


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
   
    def detect(self):
        time.sleep(0.5)
        #os.system('cls' if os.name == 'nt' else 'clear')

        while True:
            frame = self.cap.capture_array()
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)            
            corners, ids, _ = aruco.detectMarkers(image=gray, dictionary=self.aruco_dict, parameters=self.parameters)                         

            if ids is not None:
                ret = aruco.estimatePoseSingleMarkers(corners, self.marker_size, self.camera_matrix, self.camera_distortion)
                rvec, tvec = ret[0][0, 0, :], ret[1][0, 0, :]

               # aruco.drawDetectedMarkers(frame, corners)
               # cv2.drawFrameAxes(frame, self.camera_matrix, self.camera_distortion, rvec, tvec, 5)

                R_ct = np.matrix(cv2.Rodrigues(rvec)[0])
                R_tc = R_ct.T
                yaw_marker = self.rotationMatrixToEulerAngles(self.R_flip * R_tc)
                relative_position = "MARKER Position \nx Distance =%4.0f  y Distance = %4.0f  \nRotation = %4.0f" % (tvec[0] + 13, -(tvec[1] - 20), math.degrees(yaw_marker) - 2)
                print()
                
                self.data = [round((tvec[0] + 20), 4), round(-(tvec[1] - 20)), round(math.degrees(yaw_marker) - 2), ids[0].item()]
                # os.system('cls' if os.name == 'nt' else 'clear')
                # print(f"id is :{ids[0]}")
                # print(relative_position)
                print(self.data)
                # print()
                
            
            # comment for performance 
                # comment for performance thease if want

           # cv2.imshow('frame', frame)


            
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                self.cap.release()
                cv2.destroyAllWindows()
                break
    
    def reset(self):
        self.data = [0,0,0,-9,0]
        print("done task!")

if __name__ == '__main__':
    det = Detection()
    det.detect()
