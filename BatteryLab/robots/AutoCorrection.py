
import numpy as np
import cv2
import time
import os
import threading
import json

from ..helper.Logger import Logger
from AssemblyRobot import Components, AssemblySteps

from pathlib import Path
from pydantic import BaseModel

class StepCorrectionConfig(BaseModel):
    name: str
    diam: float
    ksize: int
    minDist: int
    param1: int
    param2: int
    minR: int
    maxR: int

class AutoCorrectionConfig(BaseModel):
    CAM_PORT_BOTM:int = 1
    CAM_PORT_TOP:int = 2
    Anode_Drop=StepCorrectionConfig(name='Anode_Drop', diam=15, ksize=5, minDist=100, param1=100, param2=10, minR=80, maxR=85)
    Anode_Grab=StepCorrectionConfig(name='Anode_Grab', diam=15, ksize=5, minDist=300, param1=120, param2=15, minR=115, maxR=135)
    Cathode_Drop=StepCorrectionConfig(name='Cathode_Drop', diam=14, ksize=5, minDist=100, param1=100, param2=10, minR=75, maxR=80)
    Cathode_Grab=StepCorrectionConfig(name='Cathode_Grab', diam=14, ksize=5, minDist=300, param1=120, param2=15, minR=115, maxR=125)
    Separator_Drop=StepCorrectionConfig(name='Separator_Drop', diam=15.5, ksize=5, minDist=100, param1=120, param2=15, minR=85, maxR=93)
    Separator_Grab=StepCorrectionConfig(name='Separator_Grab', diam=15.5, ksize=5, minDist=500, param1=120, param2=15, minR=135, maxR=145)
    Anode_Spacer_Grab=StepCorrectionConfig(name='Anode_Spacer', diam=15.5, ksize=5, minDist=100, param1=120, param2=15, minR=135, maxR=140)
    Cathode_Spacer_Grab=StepCorrectionConfig(name='Cathode_Spacer', diam=15.5, ksize=5, minDist=100, param1=120, param2=15, minR=135, maxR=140)
    Cathode_Case_Grab=StepCorrectionConfig(name='Cathode_Case', diam=19.3, ksize=5, minDist=100, param1=120, param2=15, minR=187, maxR=195)
    Reference=StepCorrectionConfig(name='Reference', diam=2, ksize=5, minDist=100, param1=120, param2=20, minR=8, maxR=15)
    Suction_Cup=StepCorrectionConfig(name='Suction_Cup', diam=4, ksize=5, minDist=500, param1=120, param2=20, minR=50, maxR=60)
    Customize=StepCorrectionConfig(name='Customize', diam=2, ksize=5, minDist=100, param1=100, param2=10, minR=110, maxR=115)

class AutoCorrection():

    def __init__(self, logger: Logger):
        self.logger = logger
        self.correction_config = AutoCorrectionConfig()
        self.cam_port_bottom = self.correction_config.CAM_PORT_BOTM
        self.cam_port_top = self.correction_config.CAM_PORT_TOP
        pass

    def detect_object_center(self, img, object_config:dict):
        img_color = np.copy(img)
        img_gray = cv2.cvtColor(img_color, cv2.COLOR_BGR2GRAY)
        img_gray = cv2.medianBlur(img_gray, 5)

        circles = cv2.HoughCircles(img_gray, cv2.HOUGH_GRADIENT, 1, object_config['minDist'],
                                param1=object_config['param1'], param2=object_config['param2'],
                                minRadius=object_config['minR'], maxRadius=object_config['maxR'])
        h, w = img.shape[:2]
        found_circles = []
        if circles is not None:
            circles = np.uint16(np.around(circles))
            for c in circles[0, :]:
                x, y, r = int(c[0]), int(c[1]), int(c[2])
                if x-r <=0 or y-r <= 0 or x+r >= w or y+r >=h:
                    self.logger.info("Detected circle out of scope! Result discarded")
                else:
                    found_circles.append([[x, y], r])
            found_circles = sorted(found_circles, key=lambda x: x[1])
            self.logger.debug(f"sorted circles: {found_circles}")
            found_circles = found_circles[-1] if len(found_circles) > 0 else []
        return found_circles
    
    def project_to_3d(self, image_coordinates, H_mtx): 
        """
        This method takes the Homography matrix and the 2d image cartesian coordinates. It returns the (x, y)
        cartesian coordinates in 3d cartesian world coordinates on floor plane(at z=0). Notice that z coordinate is omitted
        here and added inside the tracking funtion. 
        
        Parameters
        ----------
        image_coordinates: 2d pixel coordinates (x,y)
        h: 3x3 Homography matrix np.array[3x3]
        Returns
        ----------
        floor_coordinates: List of x, y coordinates in 3d world of same pixel on floor plane i.e. (x,y,z) Considering z=0 and 
        ommitted here.
        """
        # if H_mtx == None:
        #     os.chdir(os.path.join(PATH, 'camera_data'))
        #     H_mtx = np.load('H_mtx.npy')
        #adding 1 for homogenous coordinate system
        x, y, w = H_mtx @ np.array([[*image_coordinates, 1]]).T
        X, Y = np.around(x/w, decimals=3), np.around(y/w, decimals=3) # Transform homogenous coordinates into cart coordinates
        return np.array([X, Y, 0, 0, 0, 0], dtype=np.float32)
    
    def draw_detection(self, img, found_circle, text:str=None):
        img_output = np.copy(img)
        h, w = img_output.shape[:2]
        img_center = (w//2, h//2)
        cv2.drawMarker(img_output, img_center, (0,255,0), cv2.MARKER_CROSS, 10, 1)
        self.logger.debug(f"found_circles:{found_circle}")
        center = found_circle[0]
        cv2.drawMarker(img_output, center, (0,0,255), cv2.MARKER_CROSS, 10, 1)
        cv2.circle(img_output, center, found_circle[1], (255, 0, 255), 1)
        if text:
            cv2.putText(img_output, text, (20, h-20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
        return img_output

    def get_offset(self, img, component:Components, state: AssemblySteps):
        calibration_file = Path(os.path.dirname(__file__)) / "data" / "calibration.json"
        with open(calibration_file, "r") as json_file:
            # TODO: what is H_mtx?
            H_mtx = np.array(json.load(json_file)[f"H_mtx_{state}"], dtype=np.float32)
        detectedObj = self.detect_object_center(img, getattr(self.correction_config, f"{component.name}_{state}"))
        if len(detectedObj) > 0:
            xy = self.project_to_3d(detectedObj[0], H_mtx)
            img_output = self.draw_detection(img, detectedObj, f"Offset: {xy[:2]}")
            if component == Components.Separator and state == AssemblySteps.Grab:
                correction = np.array([0,0,0,0,0,0], dtype=np.float32)
            else:
                correction = xy*np.array([-1,-1,0,0,0,0], dtype=np.float32)
            self.logger.info(f"{state.name} Offset detected: {xy}")
            return img_output, correction, True
        else:
            detectedObj = self.detect_object_center(img, self.correction_config.Suction_Cup)
            if len(detectedObj) > 0:
                self.logger.info(f"Object {component.name} failed being grabbed!")
                return img, np.array([0,0,0,0,0,0], dtype=np.float32), False
            else:
                self.logger.info(f"Object {component.name} failed being deteced!")
                # TODO: why return True here for grabbed?
                return img, np.array([0,0,0,0,0,0], dtype=np.float32), True

    def take_img(self, state: AssemblySteps, component: Components, nr:int=None):
        self.time_stamp = time.strftime("%Y_%m_%d_%Hh_%Mm_%Ss", time.localtime())
        self.dir_name = Path(os.path.dirname(__file__)) / "Alignments" / self.time_stamp[:10] / f"Cell{nr}"
        org_dir = Path(os.path.dirname(__file__)) / "Alignments" / self.time_stamp[:10] / "Origin" / f"{component.name}_{state}"
        org_filename = org_dir /  f"[No{nr}]_{component.name}_{state}_{self.time_stamp[:10]}.jpg"
        if state == AssemblySteps.Grab:
            alpha = 1.0
            beta = 5
            cam = cv2.VideoCapture(self.cam_port_bottom, cv2.CAP_DSHOW)
            ret, img = cam.read()
            cam.release()
        elif state == AssemblySteps.Drop:
            alpha = 2.0
            beta = 50
            cam = cv2.VideoCapture(self.cam_port_top, cv2.CAP_DSHOW)
            ret, img = cam.read()
            cam.release()
        if ret == True:
            img = cv2.convertScaleAbs(img, alpha=alpha, beta=beta)
            img_output = np.copy(img)
            img_output = cv2.normalize(img_output, None, 0, 255, cv2.NORM_MINMAX)
        else:
            self.logger.error(f"Camera {state} has lost its connection", exc_info=True)
            return
        if not self.dir_name.exists():
            os.makedirs(self.dir_name)
        if not org_dir.exists():
            os.makedirs(org_dir)
        # Write the img output
        cv2.imwrite(org_filename, img_output)
        return img
    
    def show_img(self, img, component: Components, nr: int):
        cv2.imshow(f"{component.name} No.[{nr}]", img)
        cv2.waitKey(2000)
        cv2.destroyAllWindows()

    def run_autocorrection(self, state: AssemblySteps, component: Components, nr:int=None, show_img:bool=False, save_img:bool=False):
        img = self.take_img(state, component, nr)
        img_output, correction, grabbed = self.get_offset(img, component, state)
        if save_img and grabbed:
            res_filename = Path(self.dir_name) / f"[No{nr}]_{component.name}_{state}_{self.time_stamp}.jpg"
            cv2.imwrite(res_filename, img_output)
        
        if show_img:
            threading.Thread(name="show_image", target=self.show_img, args=[img_output, component, nr], daemon=True).start()

        return correction, grabbed