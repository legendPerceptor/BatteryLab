from enum import Enum
from pydantic import BaseModel
from typing import List

class RobotTool(Enum):
    GRIPPER = 1
    SUCTION = 2

class Meca500RobotConstants(BaseModel):
    GRIP_F: int = 10
    GRIP_VEL: int = 20
    L_VEL: int = 10
    J_VEL: int = 10
    TCP_GP: List[float] = [0, 0, 0, 0, 0, 0]
    TCP_SK: List[float] = [0, 0, 0, 0, 0, 0]
    TCP_CA: List[float] = [0, 0, 0, 0, 0, 0]
    HOME_GP_J: List[float] = [0, 0, 0, 0, 0, 0]
    HOME_SK_J: List[float] = [0, 0, 0, 0, 0, 0]
    HOME_CA_J: List[float] = [0, 0, 0, 0, 0, 0]
    HOME_POST_J: List[float] = [0, 0, 0, 0, 0, 0]
    SNAP_SHOT_GRAB_PO: List[float] = [0, 0, 0, 0, 0, 0]

class Components(Enum):
    Anode_Case=1
    Anode_Spacer=2
    Anode=3
    Separator=4
    Cathode=5
    Cathode_Spacer=6
    Washer=7
    Cathode_Case=8

class AssemblySteps(Enum):
    Grab=1
    Drop=2
    Press=3
    Retrieve=4
    Store=5

class ComponentProperty():
    def __init__(self):
        self.railPo: List[float] = None
        self.dropPo: List[float] = None
        self.grabPo: List[List[float]] = None

class AssemblyRobotCameraConstants():
    def __init__(self):
        self.HOME_J: List[float] = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.TRF: List[float] = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.RobotPose: List[float] = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.AnodeCase: float = 0.0
        self.Anode: float = 0.0
        self.Separator: float = 0.0
        self.Cathode: float = 0.0
        self.Spacer: float = 0.0
        self.Washer: float = 0.0
        self.CathodeCase: float = 0.0

class CrimperRobotConstants(BaseModel):
        TRF : List[float] = [20, 0, 95,  0,  0,  0]
        Home : List[float] = [-45, 0, 0, 0, 45, 0]
        PostReadyPose: List[float] = [-26.08631, 332.29777, 100, -90.00002, 0, 90]
        PostDownPose: List[float] = [-26.08631, 332.29776, 70.09248, -90.00002, 0, 90]
        GrabReadyPose: List[float] = [-26.08631, 322.62557, 70.09251, -90.00002, 0, 90]
        GrabbedUpPose: List[float] = [-26.08631, 322.62556, 93.28549, -90.00002, 0, 90]
        CrimperReadyToOperatePose: List[float] = [1.40814, -193.09457, 340.27525, 90, 0, -90]
        CrimperDropPose: List[float] = [1.40814, -281.92951, 340.27525, 90, 0, -90]
        CrimperReadyToPickPose: List[float] = [1.40817, -292.28907, 339.06158, 90, 0, -90]
        CrimperPickPressPose: List[float] = [1.40817, -292.28907, 333.83704, 90, 0, -90]
        CrimperPickPose: List[float] = [1.40814, -281.92949, 331.8871, 90, 0, -90]
        CrimperPickedUpPose: List[float] = [1.40814, -281.92949, 338.49117, 90, 0, -90]
        StorageReadyPose: List[float] = [-88.25037, 258.84746, 138.26627, -90, -0.00001, 90]
        StorageDropPose: List[float] = [-88.25037, 258.84746, 105.13378, -90, -0.00002, 90]

class AssemblyRobotConstants():
    def __init__(self):
        self.HOME_J = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.TRF = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.POST_C_SK_PO: List[float] = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.POST_RAIL_LOCATION: float = 100.0
        self.LOOKUP_CAM_SK_PO: List[float] = [0, 0, 0, 0, 0, 0]
        self.LOOKUP_CAM_RAIL_LOCATION: float = 100.0
        self.AnodeCase: List[ComponentProperty] = []
        self.Anode: List[ComponentProperty] = []
        self.Separator: List[ComponentProperty] = []
        self.Cathode: List[ComponentProperty] = []
        self.Spacer: List[ComponentProperty] = []
        self.Washer: List[ComponentProperty] = []
        self.CathodeCase: List[ComponentProperty] = []

class StepCorrectionConfig(BaseModel):
    name: str
    diam: float
    ksize: int
    minDist: int
    param1: int
    param2: int
    minR: int
    maxR: int

class AutoCorrectionConfig():
    def __init__(self):
        self.CAM_PORT_BOTM:int = 1
        self.CAM_PORT_TOP:int = 2
        self.Anode_Drop=StepCorrectionConfig(name='Anode_Drop', diam=15, ksize=5, minDist=100, param1=100, param2=10, minR=80, maxR=85)
        self.Anode_Grab=StepCorrectionConfig(name='Anode_Grab', diam=15, ksize=5, minDist=300, param1=120, param2=15, minR=115, maxR=135)
        self.Cathode_Drop=StepCorrectionConfig(name='Cathode_Drop', diam=14, ksize=5, minDist=100, param1=100, param2=10, minR=75, maxR=80)
        self.Cathode_Grab=StepCorrectionConfig(name='Cathode_Grab', diam=14, ksize=5, minDist=300, param1=120, param2=15, minR=115, maxR=125)
        self.Separator_Drop=StepCorrectionConfig(name='Separator_Drop', diam=15.5, ksize=5, minDist=100, param1=120, param2=15, minR=85, maxR=93)
        self.Separator_Grab=StepCorrectionConfig(name='Separator_Grab', diam=15.5, ksize=5, minDist=500, param1=120, param2=15, minR=135, maxR=145)
        self.Anode_Spacer_Grab=StepCorrectionConfig(name='Anode_Spacer', diam=15.5, ksize=5, minDist=100, param1=120, param2=15, minR=135, maxR=140)
        self.Cathode_Spacer_Grab=StepCorrectionConfig(name='Cathode_Spacer', diam=15.5, ksize=5, minDist=100, param1=120, param2=15, minR=135, maxR=140)
        self.Cathode_Case_Grab=StepCorrectionConfig(name='Cathode_Case', diam=19.3, ksize=5, minDist=100, param1=120, param2=15, minR=187, maxR=195)
        self.Reference=StepCorrectionConfig(name='Reference', diam=2, ksize=5, minDist=100, param1=120, param2=20, minR=8, maxR=15)
        self.Suction_Cup=StepCorrectionConfig(name='Suction_Cup', diam=4, ksize=5, minDist=500, param1=120, param2=20, minR=50, maxR=60)
        self.Customize=StepCorrectionConfig(name='Customize', diam=2, ksize=5, minDist=100, param1=100, param2=10, minR=110, maxR=115)
