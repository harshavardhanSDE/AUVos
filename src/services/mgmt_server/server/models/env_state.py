from pydantic import BaseModel, Field
from typing import List, Optional, Dict


class Motor(BaseModel):
    id: int
    type: str
    interface: str
    max_rpm: Optional[int] = None
    can_id: Optional[int] = None
    pwm_channel: Optional[int] = None


class Servo(BaseModel):
    id: int
    type: str
    interface: str
    pwm_channel: int
    min_angle: int
    max_angle: int


class Actuators(BaseModel):
    motors: List[Motor] = []
    servos: List[Servo] = []


class IMU(BaseModel):
    model: str
    interface: str
    i2c_address: str
    topic: str


class GPS(BaseModel):
    model: str
    interface: str
    port: str
    baudrate: int
    topic: str


class Lidar(BaseModel):
    model: str
    interface: str
    port: str
    baudrate: int
    topic: str


class Sensors(BaseModel):
    imu: IMU
    gps: GPS
    lidar: Lidar


class LocalizationNode(BaseModel):
    package: str
    node: str
    parameters: Dict[str, bool | str]


class MotionControlNode(BaseModel):
    package: str
    node: str
    parameters: Dict[str, float | int]
    remap: Dict[str, str]


class PerceptionNode(BaseModel):
    package: str
    node: str
    parameters: Dict[str, float]
    remap: Dict[str, str]


class ROSNodes(BaseModel):
    localization: LocalizationNode
    motion_control: MotionControlNode
    perception: PerceptionNode


class Network(BaseModel):
    ros_master_uri: str
    use_sim_time: bool


class Vehicle(BaseModel):
    name: str
    type: str


class Config(BaseModel):
    vehicle: Vehicle
    actuators: Actuators
    sensors: Sensors
    ros_nodes: ROSNodes
    network: Network
