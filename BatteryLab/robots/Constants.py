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
    TCP_GP: List[float] = [49.5, 0, 13.95, 45, 0, 0]
    TCP_SK: List[float] = [34.479, 11.2, 84.3116, 0, 30, 0]
    HOME_GP_J: List[float] = [0, 0, 0, 0, 0, 0]
    HOME_SK_J: List[float] = [-90, 0, 0, 0, 60, 0]
    HOME_POST_J: List[float] = [0, 0, 0, 0, 0, 0]
    HOME_SK_J: List[float] = [0, 0 ,0 ,0 ,0 ,0]
    SNAP_SHOT_GRAB_PO: List[float] = [ -29.527, 200.934, 91.81, -175.0, 0.0, 90.0]

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

class ComponentProperty(BaseModel):
    railPo: List[float]
    dropPo: List[float]
    grabPo: dict[int, List[float]]

class AssemblyRobotConstants(BaseModel):
    POST_C_SK_PO: List[float] = [91.516, 198.318, 57.244, 180.0, 0.0, 90.0]
    POST_RAIL_LOCATION: int = 100
    Anode_Case: ComponentProperty
    Anode_Spacer: ComponentProperty
    Anode: ComponentProperty
    Separator: ComponentProperty
    Cathode: ComponentProperty
    Cathode_Spacer: ComponentProperty
    Washer: ComponentProperty
    Cathode_Case: ComponentProperty

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
