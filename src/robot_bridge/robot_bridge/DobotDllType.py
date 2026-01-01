from ctypes import *
import platform

# Load the appropriate library based on OS
def load_dobot_dll():
    """Load the Dobot shared library based on platform"""
    system = platform.system()
    
    if system == 'Windows':
        if platform.architecture()[0] == '64bit':
            return CDLL('./DobotDll.dll', winmode=0)
        else:
            return CDLL('./DobotDll.dll')
    elif system == 'Darwin':  # macOS
        return CDLL('./libDobot.dylib')
    else:  # Linux
        return CDLL('./libDobot.so', mode=RTLD_GLOBAL)

# API Result Codes
DobotConnect_NoError = 0
DobotConnect_NotFound = 1
DobotConnect_Occupied = 2

DobotCommunicate_NoError = 0
DobotCommunicate_Timeout = 1
DobotCommunicate_InvalidParams = 2

# PTP Mode
PTPMode = c_byte

PTPMOVJXYZMode = 0
PTPMOVLXYZMode = 1
PTPJUMPXYZMode = 2
PTPMOVJANGLEMode = 3
PTPMOVLANGLEMode = 4
PTPJUMPANGLEMode = 5

PTPMOVJXYZINCMode = 6
PTPMOVLXYZINCMode = 7
PTPJUMPXYZINCMode = 8
PTPMOVJANGLEINCMode = 9
PTPMOVLANGLEINCMode = 10
PTPJUMPANGLEINCMode = 11

# CP Mode
CPMode = c_byte

CPRelativeMode = 0
CPAbsoluteMode = 1

# Arc Mode
ARCMode = c_byte

ARCCommonMode = 0
ARCFullCircleMode = 1

# Coordinate System
CoordinateSystem = c_byte

CoordinateDobot = 0
CoordinateSlider = 1

# Alarm State
AlarmsState = c_uint32

# End Effector Type
EndType = c_byte

EndTypeCustom = 0
EndTypeSuctionCup = 1
EndTypeGripper = 2
EndTypeLaser = 3
EndTypePen = 4
EndType3DPrinter = 5

# IO Function
IOFunction = c_byte

IOFunctionDummy = 0
IOFunctionDO = 1
IOFunctionPWM = 2
IOFunctionDI = 3
IOFunctionADC = 4

# IO Address
IOAddress = c_byte

# Structures
class Pose(Structure):
    """Robot pose structure"""
    pack = 1
    fields = [
        ("x", c_float),
        ("y", c_float),
        ("z", c_float),
        ("r", c_float),
        ("joint1Angle", c_float),
        ("joint2Angle", c_float),
        ("joint3Angle", c_float),
        ("joint4Angle", c_float)
    ]

class Kinematics(Structure):
    """Kinematics parameters"""
    pack = 1
    fields = [
        ("velocity", c_float),
        ("acceleration", c_float)
    ]

class AlarmsStateStruct(Structure):
    """Alarms state structure"""
    pack = 1
    fields = [
        ("alarmsState", c_uint32)
    ]

class HOMEParams(Structure):
    """HOME parameters"""
    pack = 1
    fields = [
        ("x", c_float),
        ("y", c_float),
        ("z", c_float),
        ("r", c_float)
    ]

class HOMECmd(Structure):
    """HOME command structure"""
    pack = 1
    fields = [
        ("reserved", c_uint32)
    ]

class EndEffectorParams(Structure):
    """End effector parameters"""
    pack = 1
    fields = [
        ("xBias", c_float),
        ("yBias", c_float),
        ("zBias", c_float)
    ]

class LaserStatus(Structure):
    """Laser status"""
    pack = 1
    fields = [
        ("isCtrlEnabled", c_bool),
        ("isOn", c_bool)
    ]

class SuctionCupStatus(Structure):
    """Suction cup status"""
    pack = 1
    fields = [
        ("isCtrlEnabled", c_bool),
        ("isSucked", c_bool)
    ]

class GripperStatus(Structure):
    """Gripper status"""
    pack = 1
    fields = [
        ("isCtrlEnabled", c_bool),
        ("isGripped", c_bool)
    ]

class JOGParams(Structure):
    """JOG parameters"""
    pack = 1
    fields = [
        ("velocity", c_float * 4),
        ("acceleration", c_float * 4)
    ]

class JOGCmd(Structure):
    """JOG command"""
    pack = 1
    fields = [
        ("isJoint", c_byte),
        ("cmd", c_byte)
    ]

class PTPParams(Structure):
    """PTP parameters"""
    pack = 1
    fields = [
        ("xyzVelocity", c_float),
        ("rVelocity", c_float),
        ("xyzAcceleration", c_float),
        ("rAcceleration", c_float)
    ]

class PTPJointParams(Structure):
    """PTP joint parameters"""
    pack = 1
    fields = [
        ("velocity", c_float * 4),
        ("acceleration", c_float * 4)
    ]

class PTPCoordinateParams(Structure):
    """PTP coordinate parameters"""
    pack = 1
    fields = [
        ("xyzVelocity", c_float),
        ("rVelocity", c_float),
        ("xyzAcceleration", c_float),
        ("rAcceleration", c_float)
    ]

class PTPJumpParams(Structure):
    """PTP jump parameters"""
    pack = 1
    fields = [
        ("jumpHeight", c_float),
        ("zLimit", c_float)
    ]

class PTPCommonParams(Structure):
    """PTP common parameters"""
    pack = 1
    fields = [
        ("velocityRatio", c_float),
        ("accelerationRatio", c_float)
    ]

class PTPCmd(Structure):
    """PTP command"""
    pack = 1
    fields = [
        ("ptpMode", c_byte),
        ("x", c_float),
        ("y", c_float),
        ("z", c_float),
        ("r", c_float)
    ]

class CPParams(Structure):
    """CP (Continuous Path) parameters"""
    pack = 1
    fields = [
        ("planAcc", c_float),
        ("juncitionVel", c_float),
        ("acc", c_float),
        ("realTimeTrack", c_byte)
    ]

class CPCmd(Structure):
    """CP command"""
    pack = 1
    fields = [
        ("cpMode", c_byte),
        ("x", c_float),
        ("y", c_float),
        ("z", c_float),
        ("velocity", c_float)
    ]

class ARCParams(Structure):
    """ARC parameters"""
    pack = 1
    fields = [
        ("xyzVelocity", c_float),
        ("rVelocity", c_float),
        ("xyzAcceleration", c_float),
        ("rAcceleration", c_float)
    ]

class ARCCmd(Structure):
    """ARC command"""
    pack = 1
    fields = [
        ("cirPoint", c_float * 4),
        ("toPoint", c_float * 4)
    ]

class WAITCmd(Structure):
    """WAIT command"""
    pack = 1
    fields = [
        ("waitTime", c_uint32)
    ]

class TRIGCmd(Structure):
    """TRIG command"""
    pack = 1
    fields = [
        ("address", c_byte),
        ("mode", c_byte),
        ("condition", c_byte),
        ("threshold", c_uint16)
    ]

class IOMultiplexing(Structure):
    """IO multiplexing"""
    pack = 1
    fields = [
        ("address", c_byte),
        ("multiplex", c_byte)
    ]

class IODO(Structure):
    """IO DO structure"""
    pack = 1
    fields = [
        ("address", c_byte),
        ("level", c_byte)
    ]

class IOPWM(Structure):
    """IO PWM structure"""
    pack = 1
    fields = [
        ("address", c_byte),
        ("frequency", c_float),
        ("dutyCycle", c_float)
    ]

class IODI(Structure):
    """IO DI structure"""
    pack = 1
    fields = [
        ("address", c_byte),
        ("level", c_byte)
    ]

class IOADC(Structure):
    """IO ADC structure"""
    pack = 1
    fields = [
        ("address", c_byte),
        ("value", c_int)
    ]

class EMotor(Structure):
    """E Motor structure"""
    pack = 1
    fields = [
        ("index", c_byte),
        ("isEnabled", c_byte),
        ("speed", c_int)
    ]

class EMotorS(Structure):
    """E Motor S structure"""
    pack = 1
    fields = [
        ("index", c_byte),
        ("isEnabled", c_byte),
        ("speed", c_int),
        ("distance", c_uint32)
    ]

class ColorSensor(Structure):
    """Color sensor structure"""
    pack = 1
    fields = [
        ("r", c_byte),
        ("g", c_byte),
        ("b", c_byte)
    ]

class InfraredSensor(Structure):
    """Infrared sensor structure"""
    pack = 1
    fields = [
        ("port", c_byte),
        ("value", c_byte)
    ]

class UserParams(Structure):
    """User parameters"""
    pack = 1
    fields = [
        ("params1", c_float),
        ("params2", c_float),
        ("params3", c_float),
        ("params4", c_float),
        ("params5", c_float),
        ("params6", c_float),
        ("params7", c_float),
        ("params8", c_float)
    ]

class DeviceVersion(Structure):
    """Device version structure"""
    pack = 1
    fields = [
        ("majorVersion", c_byte),
        ("minorVersion", c_byte),
        ("revision", c_byte)
    ]

class DeviceSN(Structure):
    """Device serial number"""
    pack = 1
    fields = [
        ("snNumber", c_char * 64)
    ]

class DeviceName(Structure):
    """Device name"""
    pack = 1
    fields = [
        ("deviceName", c_char * 64)
    ]

class QueuedCmdIndex(Structure):
    """Queued command index"""
    pack = 1
    fields = [
        ("queuedCmdIndex", c_uint64)
    ]

class WiFiStatus(Structure):
    """WiFi status"""
    pack = 1
    fields = [
        ("isConnected", c_bool)
    ]

class WiFiIPAddress(Structure):
    """WiFi IP address"""
    pack = 1
    fields = [
        ("dhcp", c_bool),
        ("addr1", c_byte),
        ("addr2", c_byte),
        ("addr3", c_byte),
        ("addr4", c_byte)
    ]

class WiFiNetmask(Structure):
    """WiFi netmask"""
    pack = 1
    fields = [
        ("addr1", c_byte),
        ("addr2", c_byte),
        ("addr3", c_byte),
        ("addr4", c_byte)
    ]

class WiFiGateway(Structure):
    """WiFi gateway"""
    pack = 1
    fields = [
        ("addr1", c_byte),
        ("addr2", c_byte),
        ("addr3", c_byte),
        ("addr4", c_byte)
    ]

class WiFiDNS(Structure):
    """WiFi DNS"""
    pack = 1
    fields = [
        ("addr1", c_byte),
        ("addr2", c_byte),
        ("addr3", c_byte),
        ("addr4", c_byte)
    ]

class WiFiSSID(Structure):
    """WiFi SSID"""
    pack = 1
    fields = [
        ("ssid", c_char * 64)
    ]

class WiFiPassword(Structure):
    """WiFi password"""
    pack = 1
    fields = [
        ("password", c_char * 64)
    ]

class SliderParams(Structure):
    """Slider parameters"""
    pack = 1
    fields = [
        ("velocity", c_float),
        ("acceleration", c_float)
    ]

# JOG Commands
JOGIdle = 0
JOGAP = 1
JOGAN = 2
JOGBP = 3
JOGBN = 4
JOGCP = 5
JOGCN = 6
JOGDP = 7
JOGDN = 8

# Constants
MM_PER_CIRCLE = 3.1415926535898

# Helper functions
def dType(queue_index=0):
    """Create a queued command index"""
    qci = QueuedCmdIndex()
    qci.queuedCmdIndex = queue_index
    return qci

# Load library
try:
    api = load_dobot_dll()
except Exception as e:
    print(f"Warning: Could not load Dobot library: {e}")
    api = None

# API Function Declarations (if library loaded successfully)
if api:
    # Connection functions
    api.SearchDobot.restype = c_int
    api.SearchDobot.argtypes = [POINTER(c_char), c_uint32]
    
    api.ConnectDobot.restype = c_int
    api.ConnectDobot.argtypes = [c_char_p, c_uint32, POINTER(c_char), POINTER(c_uint32)]
    
    api.DisconnectDobot.restype = c_int
    api.DisconnectDobot.argtypes = []
    
    # Device information
    api.GetDeviceSN.restype = c_int
    api.GetDeviceSN.argtypes = [POINTER(DeviceSN)]
    
    api.SetDeviceSN.restype = c_int
    api.SetDeviceSN.argtypes = [POINTER(DeviceSN)]
    
    api.GetDeviceName.restype = c_int
    api.GetDeviceName.argtypes = [POINTER(DeviceName)]
    
    api.SetDeviceName.restype = c_int
    api.SetDeviceName.argtypes = [POINTER(DeviceName)]
    
    api.GetDeviceVersion.restype = c_int
    api.GetDeviceVersion.argtypes = [POINTER(DeviceVersion)]
    
    # Pose and kinematics
    api.GetPose.restype = c_int
    api.GetPose.argtypes = [POINTER(Pose)]
    
    api.ResetPose.restype = c_int
    api.ResetPose.argtypes = [c_bool, c_float, c_float, c_float, c_float]
    
    # Alarms
    api.GetAlarmsState.restype = c_int
    api.GetAlarmsState.argtypes = [POINTER(c_ubyte), POINTER(c_uint32), c_uint32]
    
    api.ClearAllAlarmsState.restype = c_int
    api.ClearAllAlarmsState.argtypes = []
    
    # HOME functions
    api.SetHOMEParams.restype = c_int
    api.SetHOMEParams.argtypes = [POINTER(HOMEParams), c_bool, POINTER(c_uint64)]
    
    api.GetHOMEParams.restype = c_int
    api.GetHOMEParams.argtypes = [POINTER(HOMEParams)]
    
    api.SetHOMECmd.restype = c_int
    api.SetHOMECmd.argtypes = [POINTER(HOMECmd), c_bool, POINTER(c_uint64)]
    
    # End effector
    api.SetEndEffectorParams.restype = c_int
    api.SetEndEffectorParams.argtypes = [POINTER(EndEffectorParams), c_bool, POINTER(c_uint64)]
    
    api.GetEndEffectorParams.restype = c_int
    api.GetEndEffectorParams.argtypes = [POINTER(EndEffectorParams)]
    
    api.SetEndEffectorLaser.restype = c_int
    api.SetEndEffectorLaser.argtypes = [c_bool, c_bool, c_bool, POINTER(c_uint64)]
    
    api.GetEndEffectorLaser.restype = c_int
    api.GetEndEffectorLaser.argtypes = [POINTER(c_bool)]
    
    api.SetEndEffectorSuctionCup.restype = c_int
    api.SetEndEffectorSuctionCup.argtypes = [c_bool, c_bool, c_bool, POINTER(c_uint64)]
    
    api.GetEndEffectorSuctionCup.restype = c_int
    api.GetEndEffectorSuctionCup.argtypes = [POINTER(c_bool)]
    
    api.SetEndEffectorGripper.restype = c_int
    api.SetEndEffectorGripper.argtypes = [c_bool, c_bool, c_bool, POINTER(c_uint64)]
    
    api.GetEndEffectorGripper.restype = c_int
    api.GetEndEffectorGripper.argtypes = [POINTER(c_bool)]
    
    # JOG functions
    api.SetJOGParams.restype = c_int
    api.SetJOGParams.argtypes = [POINTER(JOGParams), c_bool, POINTER(c_uint64)]
    
    api.GetJOGParams.restype = c_int
    api.GetJOGParams.argtypes = [POINTER(JOGParams)]
    
    api.SetJOGCmd.restype = c_int
    api.SetJOGCmd.argtypes = [POINTER(JOGCmd), c_bool, POINTER(c_uint64)]
    
    # PTP functions
    api.SetPTPParams.restype = c_int
    api.SetPTPParams.argtypes = [POINTER(PTPParams), c_bool, POINTER(c_uint64)]
    
    api.GetPTPParams.restype = c_int
    api.GetPTPParams.argtypes = [POINTER(PTPParams)]
    
    api.SetPTPJointParams.restype = c_int
    api.SetPTPJointParams.argtypes = [POINTER(PTPJointParams), c_bool, POINTER(c_uint64)]
    
    api.GetPTPJointParams.restype = c_int
    api.GetPTPJointParams.argtypes = [POINTER(PTPJointParams)]
    
    api.SetPTPCoordinateParams.restype = c_int
    api.SetPTPCoordinateParams.argtypes = [POINTER(PTPCoordinateParams), c_bool, POINTER(c_uint64)]
    
    api.GetPTPCoordinateParams.restype = c_int
    api.GetPTPCoordinateParams.argtypes = [POINTER(PTPCoordinateParams)]
    
    api.SetPTPJumpParams.restype = c_int
    api.SetPTPJumpParams.argtypes = [POINTER(PTPJumpParams), c_bool, POINTER(c_uint64)]
    
    api.GetPTPJumpParams.restype = c_int
    api.GetPTPJumpParams.argtypes = [POINTER(PTPJumpParams)]
    
    api.SetPTPCommonParams.restype = c_int
    api.SetPTPCommonParams.argtypes = [POINTER(PTPCommonParams), c_bool, POINTER(c_uint64)]
    
    api.GetPTPCommonParams.restype = c_int
    api.GetPTPCommonParams.argtypes = [POINTER(PTPCommonParams)]
    
    api.SetPTPCmd.restype = c_int
    api.SetPTPCmd.argtypes = [POINTER(PTPCmd), c_bool, POINTER(c_uint64)]
    
    # CP functions
    api.SetCPParams.restype = c_int
    api.SetCPParams.argtypes = [POINTER(CPParams), c_bool, POINTER(c_uint64)]
    
    api.GetCPParams.restype = c_int
    api.GetCPParams.argtypes = [POINTER(CPParams)]
    
    api.SetCPCmd.restype = c_int
    api.SetCPCmd.argtypes = [POINTER(CPCmd), c_bool, POINTER(c_uint64)]
    
    # ARC functions
    api.SetARCParams.restype = c_int
    api.SetARCParams.argtypes = [POINTER(ARCParams), c_bool, POINTER(c_uint64)]
    
    api.GetARCParams.restype = c_int
    api.GetARCParams.argtypes = [POINTER(ARCParams)]
    
    api.SetARCCmd.restype = c_int
    api.SetARCCmd.argtypes = [POINTER(ARCCmd), c_bool, POINTER(c_uint64)]
    
    # WAIT function
    api.SetWAITCmd.restype = c_int
    api.SetWAITCmd.argtypes = [POINTER(WAITCmd), c_bool, POINTER(c_uint64)]
    
    # Queue functions
    api.SetQueuedCmdStartExec.restype = c_int
    api.SetQueuedCmdStartExec.argtypes = []
    
    api.SetQueuedCmdStopExec.restype = c_int
    api.SetQueuedCmdStopExec.argtypes = []
    
    api.SetQueuedCmdClear.restype = c_int
    api.SetQueuedCmdClear.argtypes = []
    
    api.GetQueuedCmdCurrentIndex.restype = c_int
    api.GetQueuedCmdCurrentIndex.argtypes = [POINTER(c_uint64)]

print("DobotDllType loaded successfully")
