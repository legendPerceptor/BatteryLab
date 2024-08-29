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
        self.grabPo: dict[int, List[float]] = None

class AssemblyRobotConstants():
    def __init__(self):
        self.POST_C_SK_PO: List[float] = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.POST_RAIL_LOCATION: int = 100
        self.AnodeCase: ComponentProperty = None
        self.Anode: ComponentProperty = None
        self.Separator: ComponentProperty = None
        self.Cathode: ComponentProperty = None
        self.Spacer: ComponentProperty = None
        self.Washer: ComponentProperty = None
        self.CathodeCase: ComponentProperty = None

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
