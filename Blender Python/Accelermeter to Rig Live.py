bl_info = {
    "name": "KeeCap IMU Tools",
    "description": "Tools for live capture and read in of Motion Capture Suit",
    "author": "Nick Keeline",
    "version": (1, 4, 0),
    "blender": (2, 83, 0),
    "location": "3D View > Tools",
    "warning": "", # used for warning icon and text in addons panel
    "wiki_url": "",
    "tracker_url": "",
    "category": "Motion Capture"
}

import sys
#this must point to a pyserial install dir for the OS (makes it work with Blender version currently running)
sys.path.append("C:\\Users\\Nick\\AppData\\Local\\Programs\\Python\\Python35\\Lib\\site-packages")
import bpy
import math

from bpy.props import (StringProperty,
                       BoolProperty,
                       IntProperty,
                       FloatProperty,
                       EnumProperty,
                       PointerProperty,
                       )
from bpy.types import (Panel,
                       Operator,
                       PropertyGroup,
                       )
import mathutils
import serial
import time
import math

ser = serial.Serial()

class Sensor(object):
    """This is a sensor object that is used in Python:

    Attributes:
        name: the Name of the Sensor (also the bone name it get's mapped to
        TPoseAngle: A Quaternion representing the angular position of the sensor in the TPose
        TranspositionToMatchBone: A Quat Representing Rotation Necessary to cause motion of the sensor to move the bone inthe right dirention
    """

    def __init__(self, name):
        """Return a Customer object whose name is *name*.""" 
        self.TransposeMethod = "None"
        self.name = name
        self.bonename = name
        self.firstpoint = True
        self.invalidpoint = False
        self.hastwistbone = False
        self.twistbonename = ""
        #this is the world space edit position of the bone the sensor will be applied to.
        self.boneEditRotWS = mathutils.Quaternion((1,0,0,0))
        #this is the virtual sensor on the character
        self.boneSensorProxyPosWS = mathutils.Quaternion((1,0,0,0))
        self.bonecorrection = mathutils.Quaternion((1,0,0,0))
#        self.previous_point = mathutils.Euler((math.radians(0), math.radians(0.0), math.radians(0)), 'XYZ')

    def __GetFirstFloatFromString (self, string, prefix ):
        """Return Float found in the string after prefix and ending with ~.""" 
        stringAfterPrefix = string.split(prefix)[1]
        floatstring = stringAfterPrefix.split("~")[0]
        return float(floatstring)
    
    def set_TPoseAngle(self, TPose):
        """Set the TPose Position for the Sensor."""
        self.TPoseAngle = TPose.copy()
        
    def set_calibrated(self, calbool):
        """Boolean indicating Calibrated"""
        self.initialized = calbool
        
    def set_twistbonename(self, tbonename):
        """If Sensor applies to a bend and twist bone we set the name here."""
        self.hastwistbone = True
        self.twistbonename = tbonename
        
    def set_bonecorrection(self, bonecorrection):
        """Correction Quaternion to be applied to sensor."""
        self.bonecorrection = bonecorrection
    
    def set_Name(self, name):
        """Set the Sensor Name."""
        self.name = name
        
    def set_bonename(self, bonename):
        """Bone Name if other than Sensor name"""
        self.bonename = bonename
        
    def set_initialized(self, initbool):
        """Boolean indicating Initialized"""
        self.initialized = initbool
        
    def set_firstPoint(self, firstPointDone):
        """Boolean indicating First Point"""
        self.firstpoint = firstPointDone
        
    def set_PreviousPoint(self, QuatPrevious):
        """A Value to Store the Previous Point"""
        self.prevPoint = QuatPrevious
        
    def set_TranspositionMethod(self, Method):
        """Set the TPose Position for the Sensor."""
        self.TransposeMethod = Method

    def set_CurrentAngle(self, Angle):
        """Set the TPose Position for the Sensor."""
        self.CurrentAngle = Angle

    def set_OrientationAngle(self, TranspositionQuat):
        """Set the TPose Position for the Sensor."""
        self.OrientationAngle = TranspositionQuat.copy()

    def set_boneEditRotWS(self, boneEditRotWSQuat):
        """Set the TPose Position for the Sensor."""
        self.boneEditRotWS = boneEditRotWSQuat.copy()

    def set_boneSensorProxyPosWS(self, boneSensorProxyPosWSQuat):
        """Set the TPose Position for the Sensor."""
        self.boneSensorProxyPosWS = boneSensorProxyPosWSQuat.copy()

    def set_AngleFromLine(self, textLine):
        """Parses a Line String and stores to the object the Current Position"""
        TextSplitByName = textLine.split("~~" + self.name + "~")
        if(len(TextSplitByName) > 1):
            finalQuat = self.getQuatFromLine(TextSplitByName)
            self.set_CurrentAngle(finalQuat)
                    
            print("CurrentAngle for " + self.name + " set To:")
            print(self.CurrentAngle)
    
    def set_TPoseAngleFromLine(self, textLine):
        """Parses a Line String and stores to the object the TPose Position"""
        TextSplitByName = textLine.split("~~" + self.name + "~")
        if(len(TextSplitByName) > 1):
            finalQuat = self.getQuatFromLine(TextSplitByName)
            self.set_TPoseAngle(finalQuat)
                    
            print("TPoseAngle Set To:")
            print(self.TPoseAngle)

    def getQuatFromLine(self, TextSplitByName):
        """Parses a Line String and retieves the quaterion from the line"""
        TextAfterName = TextSplitByName[1]
        if ("qW:" in TextAfterName):
            print("Quat Found in File")
            #in Quat Mode so parse Quat
            qW = self.__GetFirstFloatFromString(TextAfterName, "qW:")
            qX = self.__GetFirstFloatFromString(TextAfterName, "qX:")
            qY = self.__GetFirstFloatFromString(TextAfterName, "qY:")
            qZ = self.__GetFirstFloatFromString(TextAfterName, "qZ:")
            
            if(((abs(qW) + abs(qX) + abs(qX) + abs(qX)) < .1) or math.isnan(qW) or math.isnan(qX) or math.isnan(qY) or math.isnan(qZ)):
                self.invalidpoint = True
            else:
                self.invalidpoint = False
            
            if self.TransposeMethod == "XY":#"SwitchXYNegXNegY":
                finalQuat = mathutils.Quaternion((qW,-qY,-qX,qZ))
            elif self.TransposeMethod == "XZ":
                finalQuat = mathutils.Quaternion((qW,-qZ,qY,-qX))
            elif self.TransposeMethod == "YZ":
                finalQuat = mathutils.Quaternion((qW,qX,-qZ,-qY))
            elif self.TransposeMethod == "ZXY":
                finalQuat = mathutils.Quaternion((qW,-qZ,-qX,-qY))
            elif self.TransposeMethod == "YZX":
                finalQuat = mathutils.Quaternion((qW,-qY,-qZ,-qX))
            else:
                finalQuat = mathutils.Quaternion((qW,qX,qY,qZ))
        else:
            print("Euler Mode Line Found in File")
            #in Euler Mode so parse euler
            xAngle = self.__GetFirstFloatFromString(TextAfterName, "X:")
            yAngle = self.__GetFirstFloatFromString(TextAfterName, "Y:")
            zAngle = self.__GetFirstFloatFromString(TextAfterName, "Z:")
            if self.TransposeMethod == "SwitchXYNegXNegY":
                CurrentEuler = mathutils.Euler((math.radians(zAngle), math.radians(yAngle), math.radians(-xAngle)), 'XYZ')
            else:
                CurrentEuler = mathutils.Euler((math.radians(xAngle), math.radians(yAngle), math.radians(zAngle)), 'XYZ')
            finalQuat = CurrentEuler.to_quaternion()
        return finalQuat @ self.bonecorrection
    
    def name(self):
        """Return the Name of the Sensor."""
        return self.name
    
    def invalidpoint(self):
        """Return the the state of the last point read by the sensor."""
        return self.invalidpoint

    def bonename(self):
        """Return the Name of the Bone."""
        return self.bonename

    def twistbonename(self):
        """Return the Name of the Twist Bone."""
        return self.twistbonename
    
    def CurrentAngle(self):
        """Return the Current Angle."""
        return self.CurrentAngle
    
    def OrientationAngle(self):
        """Return the Orientation Angle."""
        return self.OrientationAngle
    
    def BoneCorrection(self):
        """Return the Correction Quaternion for the Sensor."""
        return self.bonecorrection
    
    def BoneEditRotWS(self):
        """Return the Bone Edit Rotation in World Space."""
        return self.boneEditRotWS
    
    def BoneSensorProxyPosWS(self):
        """Return the of the Virtual Sensor Rotation in WS"""
        return self.boneSensorProxyPosWS
    
    def TPoseAngle(self):
        """Return the TPose Angle."""
        return self.TPoseAngle
    
    def firstPoint(self):
        """Return a bool telling whether this is the first point"""
        return self.firstpoint
    
    def prevPoint(self):
        """Return a Quat Angle that is the previous Point"""
        return self.prevPoint
    
#import sys
#import glob
#import serial

def serial_ports():
    """ Lists serial port names

        :raises EnvironmentError:
            On unsupported or unknown platforms
        :returns:
            A list of the serial ports available on the system
    """
    if sys.platform.startswith('win'):
        ports = ['COM%s' % (i + 1) for i in range(256)]
    elif sys.platform.startswith('linux') or sys.platform.startswith('cygwin'):
        # this excludes your current terminal "/dev/tty"
        ports = glob.glob('/dev/tty[A-Za-z]*')
    elif sys.platform.startswith('darwin'):
        ports = glob.glob('/dev/tty.*')
    else:
        raise EnvironmentError('Unsupported platform')

    result = []
    for port in ports:
        try:
            s = serial.Serial(port)
            s.close()
            result.append(port)
        except (OSError, serial.SerialException):
            pass
    return result
# ------------------------------------------------------------------------
#    store properties in the active scene
# ------------------------------------------------------------------------

class KeeCapSettings(bpy.types.PropertyGroup):

    my_bool: bpy.props.BoolProperty(
        name="KeyFrame to Rig during file or live capture",
        description="Determines whether we keyframe and record the keyframes to the rig when we read in a file.",
        default = False
        )
        
    save_to_txt: bpy.props.BoolProperty(
        name="Log Live Capture to Text File",
        description="When Capturing Data From the RS-232 do we want to save it to a text file as we do?",
        default = False
        )

    my_int: bpy.props.IntProperty(
        name = "Number of Samples",
        description="Number of Samples to Take",
        default = 200,
        min = 10,
        max = 10000
        )

    my_float: bpy.props.FloatProperty(
        name = "Float Value",
        description = "A float property",
        default = 23.7,
        min = 0.01,
        max = 30.0
        )

    my_string: bpy.props.StringProperty(
        name="User Input",
        description=":",
        default="",
        maxlen=1024,
        )

    savelogfile: bpy.props.StringProperty(
        name="Log File",
        description="Choose a file to log Capture to:",
        default="c:\log.txt",
        maxlen=1024,
        subtype='FILE_PATH'
        )

    my_file: bpy.props.StringProperty(
        name="Capture File",
        description="Choose a file to read in:",
        default="",
        maxlen=1024,
        subtype='FILE_PATH'
        )
        
    my_enum: bpy.props.EnumProperty(
        name="CaptureSpeed",
        description="Set the Capture Speed",
        items=[ ('ASFASTASPOSSIBLE', "As Fast as Possible", ""),
                ('SUPERFAST', "Super Fast", ""),
                ('FAST', "Fast", ""),
                ('MEDIUM', "Medium", ""),
                ('SLOW', "Slow", ""),
                ('SUPERSLOW', "Super Slow", ""),
               ]
        )
        
    mapping_mode: bpy.props.EnumProperty(
        name="MappingMode",
        description="Set the Mode to Apply to Model",
        items=[ ('ABSOLUTE', "Absolute Sensor Position", ""),
                ('TPOSEOFFSET', "TPose Adjusted Sense Position", ""),
               ]
        )
        
    keyframe_frequency: bpy.props.EnumProperty(
        name="KeyFrameFrequency",
        description="Every X number of points snap a keyframe",
        items=[ ('EVERYONE', "KeyFrame Every Point", ""),
                ('2X', "KeyFrame Every Other Point", ""),
                ('3X', "KeyFrame Every 3rd Point", ""),
                ('5X', "KeyFrame Every 4th Point", ""),
                ('6X', "KeyFrame Every 5th Point", ""),
                ('7X', "KeyFrame Every 6th Point", ""),
                ('8X', "KeyFrame Every 7th Point", ""),
                ('9X', "KeyFrame Every 9th Point", ""),
                ('10X', "KeyFrame Every 10th Point", ""),
               ]
        )
def PickOutTime(lineOfFile):        
    TextSplitByName = lineOfFile.split("~Time")
    TextAfterName = TextSplitByName[1]
    floatstring = TextAfterName.split("~")[0]
    return int(floatstring)
    
def SetTheTimeForTheFrame(scene, StartFrame,lineOfFile):
    #so here we get the number of frames to move forward if applying to rig.
    Time = PickOutTime(lineOfFile)
    print("time Is: " + str(Time))
    FrameRate = scene.render.fps
    print("Frame Rate Is: " + str(FrameRate))
    FrameCalculated = ((Time/1000)*FrameRate) + StartFrame
    print("Frame Calculated Is: " + str(FrameCalculated))
    bpy.context.scene.frame_set(FrameCalculated)
    
#I programmed and entire script to test the code below to correct for bad position sensors
#this inputs the tpose for the sensor the bones rest position and current sensor rotation and
#returns the final rotation corrected for the TPose offset from the bone rest possition.
def finalPositionTPoseCorrected(BoneRestRot,TPoseRot,SenseRot):
    #find the difference between the TPos Position and the current sensor poristion.
    TRotSenseRotDiff = SenseRot.inverted().rotation_difference(TPoseRot.inverted())
    #rotate the bone to it's rest position and apply the difference to it.
    rotation = TRotSenseRotDiff @ BoneRestRot
    #final rotation to apply to bone.
    return rotation

def GetBonePositionToSetInArmatureSpaceGivenWorldCoordQuat(Bone, WorldQuat, armature):
        
    #rotate the world quat first to armature rotation
    ArmatureSpaceWorldQuat = armature.rotation_quaternion.inverted()  @ WorldQuat
      
    #so we get the angle of the bone in armature space
    #this is the angle with all parent bones modifying it.
    ArmSpaceQuat = Bone.matrix.to_quaternion() #* armature.rotation_quaternion  
    
    #Next we get the pose position from the base of the bone and it's inverse
    PoseAngleQuat = Bone.rotation_quaternion
    PoseAngleQuatInv = PoseAngleQuat.inverted()
    
    #To get the angle of the rest (no angle applied) position of the pose bone in armature space 
    # we take the poses current position and move
    #it by the inverse of the pose angle
    PoseNoAngleBaseArmSpaceQuat = ArmSpaceQuat @ PoseAngleQuatInv
        
    #so now the rotation is the rest position in armature space rotated by our target world space rotation
    BoneRotationToSetInArmatureSpace = PoseNoAngleBaseArmSpaceQuat.inverted() @ ArmatureSpaceWorldQuat 
   
    #so now we need to rotate the bone position by the armature world space location
    #FinalOut = BoneRotationToSetInArmatureSpace @ armature.rotation_quaternion  
    
    #Ok, make this the final quat to use....
    return BoneRotationToSetInArmatureSpace
    
def SetBonePosition(scene, ThisSensor, RigArmature, WeShouldKeyframe,MapMode):
    
    mytool = scene.my_tool
    pose_bone = RigArmature.pose.bones[ThisSensor.bonename]
    
    GlobalSenseAdjust = mathutils.Euler((math.radians(0), math.radians(0.0), math.radians(180)), 'XYZ').to_quaternion()
    
    #get the difference between the two sensor positions....
    sensrot = ThisSensor.CurrentAngle
    tPoserot = ThisSensor.TPoseAngle
    proxyrot = ThisSensor.BoneSensorProxyPosWS() @ GlobalSenseAdjust
    editrot = ThisSensor.BoneEditRotWS() 
    
    #first we get the rotation from the virtual proxy sensor to the edit bone position delta in WS.
    EditToProxySensorDelta = proxyrot.rotation_difference(editrot)
    
    #The final absolute of the Sensor Position.
    finalAbsoluteSensorRotation = sensrot @ EditToProxySensorDelta 
    
    #We adjust the TPose to it's absolute position:
    TPoseRotSensorAdjustedAbsolute = tPoserot @ EditToProxySensorDelta
    
    #For the TPose adjustment we take the difference from the TPose Positino to the Edit Bone Position in WS
    EditToTPoseSensorDelta = TPoseRotSensorAdjustedAbsolute.rotation_difference(editrot)
    
    #For the final TPose adjusted we take the absolute and move it by the differnene of the TPose and the Edit Pos WS
    finalTPoseAdjustedRotation = finalAbsoluteSensorRotation @ EditToTPoseSensorDelta 
    
    #rotate bone to the rest sensor position
    if(MapMode == "ABSOLUTE"):
        NewBoneRotWS = finalAbsoluteSensorRotation
    elif (MapMode == "TPOSEOFFSET"):
        #NewBoneRotWS = finalTPoseAdjustedRotation 
        NewBoneRotWS = finalAbsoluteSensorRotation 
    else:
        NewBoneRotWS = finalAbsoluteSensorRotation
    
      
    
    #Get the rotation of the bone in edit mode
    #BoneEdit = RigArmature.data.bones[ThisSensor.bonename]
    #BoneEditRotation = BoneEdit.matrix_local.to_quaternion()
    #rotate the edit rotation quat first to armature rotation
    #ArmatureSpaceBoneEditPosition = RigArmature.rotation_quaternion * BoneEditRotation
    
    #old method doesn't use tpos to re-orient
    #SensorAdjusted = ThisSensor.CurrentAngle * ThisSensor.OrientationAngle
    #TPoseAdjusted = ThisSensor.TPoseAngle * ThisSensor.OrientationAngle
    #PoseAdjusted = ThisSensor.CurrentAngle * ThisSensor.OrientationAngle
    
    #this was back when I was testing mirroring, didn't end up needing it but it was enough work to get working that I leave it here for future generations.
    #MirrorMethod = 'XY'
    #MirrorMethod = 'YZ'
    #MirrorMethod = 'XZ'
    #TPoseAdjusted = MirrorQuat(TPoseAdjusted,MirrorMethod)
    #PoseAdjusted = MirrorQuat(PoseAdjusted,MirrorMethod)
    
    #New Method take differencebetween TPose and current position, then move the blender edit mode position by the difference.
    #so first we get the difference between the TPose Position and the current positiion.
    #TtoCurrPosDifference = TPoseAdjusted.inverted()*PoseAdjusted
    
    #if(MapMode == "ABSOLUTE"):
        #This sets the model to the absolute position of the sensor
        #SensorAdjusted = PoseAdjusted
    #elif (MapMode == "TPOSEOFFSET"):
        #Then we rotate the bones TPosition by the difference between the two quats above (rotate it by the change in the sensor from Tpos to now).
        #SensorAdjusted = ArmatureSpaceBoneEditPosition*TtoCurrPosDifference
    #else:
        #SensorAdjusted = PoseAdjusted
        
    #This was the old global edit position not adjusted for armature rotation... old don't use.   
    #SensorAdjusted = BoneEditRotation*TtoCurrPosDifference
    
    
    #Ok, make this the final quat to use....
    FinalQuat = GetBonePositionToSetInArmatureSpaceGivenWorldCoordQuat(pose_bone, NewBoneRotWS, RigArmature) 
    #FinalQuat = ThisSensor.bonecorrection * FinalQuat

    #Set Bone Position now that we've calculated it.
    pose_bone.rotation_mode = 'QUATERNION'

    #We only set the value of the bone and keyframe it if it's a valid point.
    if(ThisSensor.invalidpoint):  
        print("Error, This Sensor: " + ThisSensor.name + " had an illegal point, no keyframe.")
    else:
        #we have a problem in that the negative of a quaternion(all values multiplied by neg 1)
        # is the same angle as the original value.  Our Sensor and the math know this, but if 
        # the previous value is the negative of the current one the smoothing between goes crazy.
        # to fix this we find the magnitude of the difference to see if the current value is the
        # negative of the previous and if the magnitude of the difference is greater than one it is.
        if (ThisSensor.firstpoint):
            print("Initial Point Set")
            ThisSensor.set_CurrentAngle(FinalQuat)
            ThisSensor.firstpoint = False
            ThisSensor.set_PreviousPoint(FinalQuat)
        else:
            PrevToNowdifference = FinalQuat - ThisSensor.prevPoint
            if(PrevToNowdifference.magnitude < 1):
                print("Magnitude less than one:" + str(PrevToNowdifference.magnitude) + " keeping same.")
            else:
                print("Magnitude greater than one:" + str(PrevToNowdifference.magnitude) + " mult by neg 1.")
                FinalQuat = mathutils.Quaternion((-1*FinalQuat.w,-1*FinalQuat.x,-1*FinalQuat.y,-1*FinalQuat.z))

            
        #If the Bone has a twist bone we move the angle from the bend bone to it's twist counterpart
        if (ThisSensor.hastwistbone):
            pose_bone.rotation_quaternion = FinalQuat
            TwistBone = RigArmature.pose.bones[ThisSensor.twistbonename]
            TwistBone.rotation_mode = 'XYZ'
            pose_bone.rotation_mode = 'XYZ'
            yrotation = pose_bone.rotation_euler.y
            pose_bone.rotation_euler.y = 0
            TwistBone.rotation_euler.y = yrotation
            #print('Setting Twist Bone: ' + yrotation)
            TwistBone.rotation_mode = 'QUATERNION'
            pose_bone.rotation_mode = 'QUATERNION'
            #BendTwistAngle = FinalQuat.angle
            #print('Twist Transfer from bone to bone:')
            #print(BendTwistAngle)
            #FinalQuat.Angle = 0
            #FinalVector = FinalQuat.axis
            #FinalQautAxisQuat = FinalVector.to_track_quat('Y', 'X')
            #TwistFinalQuat = GetBonePositionToSetInArmatureSpaceGivenWorldCoordQuat(TwistBone, NewBoneRotWS, RigArmature)
            #TwistBone.rotation_quaternion = TwistFinalQuat
        else:
            pose_bone.rotation_quaternion = FinalQuat
            
        ThisSensor.set_PreviousPoint(FinalQuat)
        
        if (WeShouldKeyframe):
            pose_bone.rotation_mode = 'QUATERNION'
            pose_bone.keyframe_insert('rotation_quaternion')
            if (ThisSensor.hastwistbone):
                TwistBone.keyframe_insert('rotation_quaternion')
                
def SetAllSensorsToSelectedMode(Sensors, scene, armature):
    mytool = scene.my_tool
    for Sense in Sensors:
        #if (mytool.my_enum == 'EULER'):
            #armature.pose.bones[Sense.name].rotation_mode = 'XYZ'
        #else:
        armature.pose.bones[Sense.bonename].rotation_mode = 'QUATERNION'

def GetBoneEditRotationWorldSpace(arm, bonename):
    BoneEdit = arm.data.bones[bonename]
    BoneEditRotation = BoneEdit.matrix_local.to_quaternion()
    BoneEditWS = arm.rotation_quaternion @ BoneEditRotation
    return BoneEditWS

def GetInitialQuatsInitializedInSensors(scene, armature, Sensor, SenseObjectName):
    armature.pose.bones[Sensor.bonename].rotation_mode = 'QUATERNION'
    #Virtual Sensor on Bone WS Set
    BoneSensor = bpy.data.objects[SenseObjectName]
    BonesensWorldmatrix = BoneSensor.matrix_world
    BoneSensorRotWS = BonesensWorldmatrix.to_quaternion()
    Sensor.set_boneSensorProxyPosWS(BoneSensorRotWS.copy())
    #Edit Position of Bone in World Space
    Sensor.set_boneEditRotWS(GetBoneEditRotationWorldSpace(armature, Sensor.bonename).copy())
    return Sensor

    
def SetupSensors(scene, armature):
    mytool = scene.my_tool
    Sensors = []
    SwitchThighShin = True
    SwapMethod = "XY"
    
    
    Waist = Sensor("spine")
    Waist.set_bonename("hip")
    Waist.set_TranspositionMethod(SwapMethod)
    Waist = GetInitialQuatsInitializedInSensors(scene, armature, Waist, 'SensorProxyWaist')
    Sensors.append(Waist)

    Chest = Sensor("chest")
    Chest.set_bonename("abdomenUpper")
    Chest.set_TranspositionMethod(SwapMethod)
    Chest = GetInitialQuatsInitializedInSensors(scene, armature, Chest, 'SensorProxyChest')
    Sensors.append(Chest)
    
    Head = Sensor("head")
    Head.set_TranspositionMethod(SwapMethod)
    Head = GetInitialQuatsInitializedInSensors(scene, armature, Head, 'SensorProxyHead')
    Sensors.append(Head)
    
    #Right Arm
    UpperArmRight = Sensor("upper_arm.R")
    UpperArmRight.set_bonename("rShldrBend")
    UpperArmRight.set_twistbonename('rShldrTwist')
    UpperArmRight.set_TranspositionMethod(SwapMethod)
    UpperArmRight = GetInitialQuatsInitializedInSensors(scene, armature, UpperArmRight, 'SensorProxyBicep.R')
    Sensors.append(UpperArmRight)

    LowerArmRight = Sensor("forearm.R")
    LowerArmRight.set_bonename("rForearmBend")
    LowerArmRight.set_twistbonename('rForearmTwist')
    LowerArmRight.set_TranspositionMethod(SwapMethod)
    LowerArmRight = GetInitialQuatsInitializedInSensors(scene, armature, LowerArmRight, 'SensorProxyForeArm.R')
    Sensors.append(LowerArmRight)

    HandRight = Sensor("hand.R")
    HandRight.set_bonename("rHand")
    HandRight.set_TranspositionMethod(SwapMethod)
    HandRight = GetInitialQuatsInitializedInSensors(scene, armature, HandRight, 'SensorProxyHand.R')
    Sensors.append(HandRight)
        
    #Left Arm
    UpperArmLeft = Sensor("upper_arm.L")
    UpperArmLeft.set_bonename("lShldrBend")
    UpperArmLeft.set_twistbonename('lShldrTwist')
    UpperArmLeft.set_TranspositionMethod(SwapMethod)
    UpperArmLeft = GetInitialQuatsInitializedInSensors(scene, armature, UpperArmLeft, 'SensorProxyBicep.L')
    Sensors.append(UpperArmLeft)

    LowerArmLeft = Sensor("forearm.L")
    LowerArmLeft.set_bonename("lForearmBend")
    LowerArmLeft.set_twistbonename('lForearmTwist')
    LowerArmLeft.set_TranspositionMethod(SwapMethod)
    LowerArmLeft = GetInitialQuatsInitializedInSensors(scene, armature, LowerArmLeft, 'SensorProxyForeArm.L')
    Sensors.append(LowerArmLeft)

    HandLeft = Sensor("hand.L")
    HandLeft.set_bonename("lHand")
    HandLeft.set_TranspositionMethod(SwapMethod)
    HandLeft = GetInitialQuatsInitializedInSensors(scene, armature, HandLeft, 'SensorProxyHand.L')
    Sensors.append(HandLeft)
    

    #Right Leg
    UpperLegRight = Sensor("thigh.R")
    UpperLegRight.set_bonename("rThighBend")
    UpperLegRight.set_twistbonename('rThighTwist')
    if (SwitchThighShin):
        UpperLegRight.set_Name("shin.R")
    UpperLegRight.set_TranspositionMethod(SwapMethod)
    UpperLegRight = GetInitialQuatsInitializedInSensors(scene, armature, UpperLegRight, 'SensorProxyThigh.R')
    Sensors.append(UpperLegRight)

    LowerLegRight = Sensor("shin.R")
    LowerLegRight.set_bonename("rShin")
    if (SwitchThighShin):
        LowerLegRight.set_Name("thigh.R")
    LowerLegRight.set_TranspositionMethod(SwapMethod)
    LowerLegRight = GetInitialQuatsInitializedInSensors(scene, armature, LowerLegRight, 'SensorProxyKnee.R')
    Sensors.append(LowerLegRight)

    FootRight = Sensor("foot.R")
    FootRight.set_bonename("rFoot")
    FootRight.set_TranspositionMethod(SwapMethod)
    FootRight = GetInitialQuatsInitializedInSensors(scene, armature, FootRight, 'SensorProxyFoot.R')
    Sensors.append(FootRight)

    #Left Leg
    UpperLegLeft = Sensor("thigh.L")
    UpperLegLeft.set_bonename("lThighBend")
    UpperLegLeft.set_twistbonename('lThighTwist')
    if (SwitchThighShin):
        UpperLegLeft.set_Name("shin.L")
    UpperLegLeft.set_TranspositionMethod(SwapMethod)
    UpperLegLeft = GetInitialQuatsInitializedInSensors(scene, armature, UpperLegLeft, 'SensorProxyThigh.L')
    Sensors.append(UpperLegLeft)

    LowerLegLeft = Sensor("shin.L")
    LowerLegLeft.set_bonename("lShin")
    if (SwitchThighShin):
        LowerLegLeft.set_Name("thigh.L")
    LowerLegLeft.set_TranspositionMethod(SwapMethod)
    LowerLegLeft = GetInitialQuatsInitializedInSensors(scene, armature, LowerLegLeft, 'SensorProxyKnee.L')
    Sensors.append(LowerLegLeft)

    FootLeft = Sensor("foot.L")
    FootLeft.set_bonename("lFoot")
    FootLeft.set_TranspositionMethod(SwapMethod)
    FootLeft = GetInitialQuatsInitializedInSensors(scene, armature, FootLeft, 'SensorProxyFoot.L')
    Sensors.append(FootLeft)
    
    return Sensors

def GetArmature():
    arm = bpy.data.objects['Genesis3Male']
    arm.rotation_mode = 'QUATERNION'
    return arm

def UpdateScreenNoFrameChange(scene):
    bpy.ops.wm.redraw_timer(type='DRAW_WIN_SWAP', iterations=1)
    #scene.update()
    #scene.frame_set(scene.frame_current + 1)
    #scene.frame_set(scene.frame_current - 1)
def GetKeyFrameFrequency(SelectionString):
    if (SelectionString == "EVERYONE"):
        Selection = 1
    elif (SelectionString == "2X"):
        Selection = 2
    elif (SelectionString == "3X"):
        Selection = 3
    elif (SelectionString == "4X"):
        Selection = 4
    elif (SelectionString == "5X"):
        Selection = 5
    elif (SelectionString == "6X"):
        Selection = 6
    elif (SelectionString == "7X"):
        Selection = 7
    elif (SelectionString == "8X"):
        Selection = 8
    elif (SelectionString == "9X"):
        Selection = 9
    elif (SelectionString == "10X"):
        Selection = 10
    else:
        Selection = 1
    return Selection
# ------------------------------------------------------------------------
#    operators
# ------------------------------------------------------------------------

class IMUConnecOperator(bpy.types.Operator):
    bl_idname = "wm.connect"
    bl_label = "Connect To Rig Via 232"

    def execute(self, context):
        scene = context.scene
        mytool = scene.my_tool
        
        # print the values to the console
        print("Connecting to Rig...")
        #print("bool state:", mytool.my_bool)
        #print("int value:", mytool.my_int)
        #print("float value:", mytool.my_float)
        #print("string value:", mytool.my_string)
        #print("enum state:", mytool.my_enum)
        # Prevent unsupported Execution in Local View modes
        space_data = bpy.context.space_data
        
        # Make variable scene that is current scene
        scene = context.scene
        
        ser.baudrate = 115200
        #ser.port = '/dev/ttyACM0'
        ser.port = serial_ports()[0]
        ser.open()
        print("connecting...")
        a = ser.readline()
        print(a)
        while(b'DoneMenus' not in a):
            a = ser.readline()
            print(a)
        
        return {'FINISHED'}

class IMUDisconnectConnecOperator(bpy.types.Operator):
    bl_idname = "wm.disconnect"
    bl_label = "DisconnectConnect From Rig Via 232"

    def execute(self, context):
        scene = context.scene
        mytool = scene.my_tool
        ser.reset_input_buffer()
        
        print("Disconnecting From Rig...")
        space_data = bpy.context.space_data
        
        # Make variable scene that is current scene
        scene = context.scene
        
        ser.close()
        
        return {'FINISHED'}
    
    
class IMUInitializeOperator(bpy.types.Operator):
    bl_idname = "wm.initialize"
    bl_label = "Initialize Sensors"

    def execute(self, context):
        # Make variable scene that is current scene
        scene = context.scene
        mytool = scene.my_tool
        ser.reset_input_buffer()
        
        print("Initialize Sensors...")
        space_data = bpy.context.space_data
        
        
        ser.write(b'i')
        a = ser.readline().decode("ascii")
        print(a)
        while('DoneMenus' not in a):
            a = ser.readline().decode("ascii")
            print(a)
        
        return {'FINISHED'}
        
class IMUCalibrateOperator(bpy.types.Operator):
    bl_idname = "wm.calibrate"
    bl_label = "Calibrate Sensors"

    def execute(self, context):
        scene = context.scene
        mytool = scene.my_tool
        ser.reset_input_buffer()
        
        print("Calibrate Sensors...")
        space_data = bpy.context.space_data
        
        # Make variable scene that is current scene
        scene = context.scene
        
        ser.write(b'c')
        a = ser.readline().decode("ascii")
        print(a)
        while('DoneMenus' not in a):
            a = ser.readline().decode("ascii")
            print(a)
        
        return {'FINISHED'}
 #### This operator takes the user position and the keyframed position and creates a calibration quat
 #### That will be applied to the sensor as an offset for future file reading.
class CalibrateBoneOperator(bpy.types.Operator):
    bl_idname = "wm.calbone"
    bl_label = "Calibrate Bone"

    def execute(self, context):
        scene = context.scene
        mytool = scene.my_tool
        
        arm = GetArmature()
        print("Current Frame")
        CurrentFrame = scene.frame_current
        print(CurrentFrame)

        # Active object
        ob = bpy.context.object

        #if it's an armature go ahead
        if ob.type == 'ARMATURE':
            armature = ob.data

            #get currently selected bone
            numberselected = 0
            for bone in armature.bones:
                if bone.select:
                    print(bone.name)
                    SelectedBone = bone
                    pose_bone = arm.pose.bones[bone.name]
                    numberselected = numberselected + 1
            if numberselected == 1:
                #ok only one bone selected so go ahead
                currentRotMode = pose_bone.rotation_mode
                pose_bone.rotation_mode = 'QUATERNION'
                CurrentPosition = pose_bone.rotation_quaternion.copy()
                scene.frame_set(CurrentFrame)
                KeyPosition = pose_bone.rotation_quaternion.copy()
                print('Current Position')
                print(CurrentPosition)
                print('Key Position')
                print(KeyPosition)
                #setting the correction to the difference between the selected position and the keyframed position.
                Correction = KeyPosition.inverted() @ CurrentPosition
                print("Setting Bone Correction to ")
                print(Correction)
                result = 'FINISHED'
            else:
                self.report({'ERROR'},"incorrect number of bones selected.")
                #print(numberselected)
                result = 'CANCELLED'
        
        return {'FINISHED'}
               
class IMUCaptureOperator(bpy.types.Operator):
    bl_idname = "wm.capture"
    bl_label = "Capture and Map to Rig"

    def execute(self, context):
        scene = context.scene
        mytool = scene.my_tool

        armature = GetArmature()
    
        #change to quaternion mode  --disabled this because quat mode didn't work well (kept flipping and glitching)
        ser.reset_input_buffer()
        ser.write(b'p')
        a = ser.readline().decode("ascii")
        while('5' not in a):
            a = ser.readline().decode("ascii")
            print(a)
            
        if (mytool.my_enum == "ASFASTASPOSSIBLE"):
            ser.write(b'0')
        if (mytool.my_enum == "SUPERFAST"):
            ser.write(b'1')
        if (mytool.my_enum == "FAST"):
            ser.write(b'2')
        if (mytool.my_enum == "MEDIUM"):
            ser.write(b'3')
        if (mytool.my_enum == "SLOW"):
            ser.write(b'4')
        if (mytool.my_enum == "SUPERSLOW"):
            ser.write(b'5')
            
        KeyFrameFrequency = GetKeyFrameFrequency(mytool.keyframe_frequency)
        CurrentKeyFrameNumberForFreqTest = 100
        
        a = ser.readline().decode("ascii")
        while('DoneMenus' not in a):
            a = ser.readline().decode("ascii")
            print(a)
            
        ser.reset_input_buffer()    
        ser.write(b'q')
        #else:
            #ser.write(b'w')
        a = ser.readline().decode("ascii")
        print(a)
        while('DoneMenus' not in a):
            a = ser.readline().decode("ascii")
            print(a)

        Sensors = SetupSensors(scene, armature)
        
        if(mytool.save_to_txt):
            filepath = mytool.savelogfile
            f = open(filepath, 'w')
            
        print("Capturing Sensors...")
        space_data = bpy.context.space_data
        
        # Make variable scene that is current scene
        scene = context.scene
        
        ser.write(b'r')
                
        StartFrame = scene.frame_current
        i=0
        while (i < scene.my_tool.my_int):
            i = i + 1
            line = ser.readline().decode("ascii")
            #line = str(bytes,'utf-8')
            #line = bytes.decode('ascii')
            print(line)
            if(mytool.save_to_txt):
                f.write(line) 
            if (line.split('  ', 1)[0] == 'TPOS'):
                for Sense in Sensors:
                    if (Sense.name in line):
                        Sense.set_TPoseAngleFromLine(line)
            if (line.split('  ', 1)[0] == 'PNTSTART'):
                for Sense in Sensors:
                    if (Sense.name in line):
                        print(Sense.name)
                        Sense.set_AngleFromLine(line)
                        SetBonePosition(scene, Sense, armature, mytool.my_bool, mytool.mapping_mode)
                #UpdateScreenNoFrameChange(scene)
                
                #keyframe every KeyFrameFrequency
                if((CurrentKeyFrameNumberForFreqTest >= KeyFrameFrequency) and mytool.my_bool):
                    CurrentKeyFrameNumberForFreqTest = 1
                    SetTheTimeForTheFrame(scene,StartFrame,line)
                else:
                    CurrentKeyFrameNumberForFreqTest = CurrentKeyFrameNumberForFreqTest + 1
            UpdateScreenNoFrameChange(scene)
        
        #quit running
        ser.write(b'q')
        a = ser.readline().decode("ascii")
        print(a)
        
        if(mytool.save_to_txt):
            f.write(a) 
            f.close()
        while('DoneMenus' not in a):
            a = ser.readline().decode("ascii")
            print(a)
            
        return {'FINISHED'}
    
    
class IMUReadInFileOperator(bpy.types.Operator):
    bl_idname = "wm.readfile"
    bl_label = "Read In File and Apply to Model"

    def execute(self, context):
        scene = context.scene
        mytool = scene.my_tool

        armature = GetArmature()
        
        print("Disconnecting From Rig...")
        space_data = bpy.context.space_data
        filepath = bpy.path.abspath(mytool.my_file)
        # Make variable scene that is current scene
        scene = context.scene
        
        CurrentFrame = scene.frame_current
        Sensors = SetupSensors(scene, armature)
            
        KeyFrameFrequency = GetKeyFrameFrequency(mytool.keyframe_frequency)
        CurrentKeyFrameNumberForFreqTest = 100
        
        StartFrame = scene.frame_current
        print(filepath)
        file = open(filepath, "r")
        for line in file:
            #time.sleep(.02)
            print(line)
            if (line.split('  ', 1)[0] == 'TPOS'):
                for Sense in Sensors:
                    Sense.set_TPoseAngleFromLine(line)
            if (line.split('  ', 1)[0] == 'PNTSTART'):
                for Sense in Sensors:
                    if (Sense.name in line):
                        print(Sense.name)
                        print(Sense.bonename)
                        Sense.set_AngleFromLine(line)
                        SetBonePosition(scene, Sense, armature, mytool.my_bool, mytool.mapping_mode)
                        
                #keyframe every KeyFrameFrequency
                if((CurrentKeyFrameNumberForFreqTest >= KeyFrameFrequency) and mytool.my_bool):
                    CurrentKeyFrameNumberForFreqTest = 1
                    SetTheTimeForTheFrame(scene,StartFrame,line)
                    print("Set Frame to:" + str(scene.frame_current))
                else:
                    CurrentKeyFrameNumberForFreqTest = CurrentKeyFrameNumberForFreqTest + 1
                UpdateScreenNoFrameChange(scene)
        file.close()
        
        SetAllSensorsToSelectedMode(Sensors, scene, armature)
        
        return {'FINISHED'}
    
    
class IMUReadResetProxySensorsOperator(bpy.types.Operator):
    bl_idname = "wm.resetproxysensors"
    bl_label = "Reset all proxy sensors to default positions"

    def execute(self, context):
        scene = context.scene
        mytool = scene.my_tool

        armature = GetArmature()
        
        print("Resetting All Sensor Positions on Rig...")
        
        bpy.data.objects['SensorProxyForeArm.R'].rotation_euler = mathutils.Euler((math.radians(0), math.radians(-10.0), math.radians(0)), 'XYZ')
        bpy.data.objects['SensorProxyHand.R'].rotation_euler = mathutils.Euler((math.radians(0), math.radians(0.0), math.radians(0)), 'XYZ')
        bpy.data.objects['SensorProxyBicep.R'].rotation_euler = mathutils.Euler((math.radians(0), math.radians(0.0), math.radians(0)), 'XYZ')
        bpy.data.objects['SensorProxyHead'].rotation_euler = mathutils.Euler((math.radians(-20), math.radians(0.0), math.radians(180)), 'XYZ')
        bpy.data.objects['SensorProxyBicep.L'].rotation_euler = mathutils.Euler((math.radians(0), math.radians(-10.0), math.radians(180)), 'XYZ')
        bpy.data.objects['SensorProxyForeArm.L'].rotation_euler = mathutils.Euler((math.radians(0), math.radians(-10.0), math.radians(180)), 'XYZ')
        bpy.data.objects['SensorProxyHand.L'].rotation_euler = mathutils.Euler((math.radians(0), math.radians(-10.0), math.radians(180)), 'XYZ')
        bpy.data.objects['SensorProxyChest'].rotation_euler = mathutils.Euler((math.radians(-70), math.radians(0.0), math.radians(180)), 'XYZ')
        bpy.data.objects['SensorProxyWaist'].rotation_euler = mathutils.Euler((math.radians(90), math.radians(0.0), math.radians(0)), 'XYZ')
        bpy.data.objects['SensorProxyThigh.L'].rotation_euler = mathutils.Euler((math.radians(180), math.radians(-75.0), math.radians(-90)), 'XYZ')
        bpy.data.objects['SensorProxyKnee.L'].rotation_euler = mathutils.Euler((math.radians(180), math.radians(-75.0), math.radians(-90)), 'XYZ')
        bpy.data.objects['SensorProxyFoot.L'].rotation_euler = mathutils.Euler((math.radians(0), math.radians(-20.0), math.radians(90)), 'XYZ')
        bpy.data.objects['SensorProxyThigh.R'].rotation_euler = mathutils.Euler((math.radians(180), math.radians(-75.0), math.radians(-90)), 'XYZ')
        bpy.data.objects['SensorProxyKnee.R'].rotation_euler = mathutils.Euler((math.radians(180), math.radians(-75.0), math.radians(-90)), 'XYZ')
        bpy.data.objects['SensorProxyFoot.R'].rotation_euler = mathutils.Euler((math.radians(0), math.radians(-20.0), math.radians(90)), 'XYZ')
        
        return {'FINISHED'} 
# ------------------------------------------------------------------------
#    menus
# ------------------------------------------------------------------------

#class BasicMenu(bpy.types.Menu):
#    bl_idname = "OBJECT_MT_select_test"
#    bl_label = "Select"

#    def draw(self, context):
#        layout = self.layout

#        # built-in example operators
#        layout.operator("object.select_all", text="Select/Deselect All").action = 'TOGGLE'
#        layout.operator("object.select_all", text="Inverse").action = 'INVERT'
#        layout.operator("object.select_random", text="Random")

# ------------------------------------------------------------------------
#    my tool in objectmode
# ------------------------------------------------------------------------

class IMUToolsPanel(bpy.types.Panel):
    """Creates a Panel for the KeeCap IMU Tools Window"""
    bl_label = "KeeCap"
    bl_idname = "KEECAP_PT_MAINPANEL"
    bl_space_type = "VIEW_3D"   
    bl_region_type = "UI"    
    bl_category = 'KeeCap'
    bl_context = "objectmode"   

    @classmethod
    def poll(self,context):
        return context.object is not None

    def draw(self, context):
        layout = self.layout
        scene = context.scene
        mytool = bpy.context.scene.my_tool

        layout.label(text="Live Capture")
        #layout.prop(mytool, "my_float")
        #layout.prop(mytool, "my_string")
        layout.operator("wm.connect")
        layout.operator("wm.initialize")
        layout.operator("wm.calibrate")
        layout.prop(mytool, "my_int")
        layout.operator("wm.capture")
        if(mytool.save_to_txt):
            layout.prop(mytool, "savelogfile")
        layout.operator("wm.disconnect")
        layout.label(text="File Playback")
        layout.prop(mytool, "my_file")
        layout.operator("wm.readfile")
        
        layout.label(text="Settings")
        layout.prop(mytool, "my_bool")
        if(mytool.my_bool):
            layout.prop(mytool, "keyframe_frequency", text="") 
        layout.prop(mytool, "save_to_txt")
        layout.label(text="Select Record Speed during live capture:")
        layout.prop(mytool, "my_enum", text="") 
        layout.label(text="Set the Mode to Apply Sensor Positions to Model:")
        layout.prop(mytool, "mapping_mode", text="") 
        layout.operator("wm.resetproxysensors")
        #mytool.my_enum = 'MEDIUM'
        #layout.menu("OBJECT_MT_select_test", text="Presets", icon="SCENE")

# ------------------------------------------------------------------------
#    my tool in objectmode
# ------------------------------------------------------------------------

#class OBJECT_PT_pose_panel(Panel):
#    bl_idname = "OBJECT_PT_pose_panel"
#    bl_label = "Keeline IMU Tools"
#    bl_space_type = "VIEW_3D"   
#    bl_region_type = "TOOLS"    
#    bl_category = "Tools"
#    bl_context = "posemode"   

#    @classmethod
#    def poll(self,context):
#        return context.object is not None

#    def draw(self, context):
#        layout = self.layout
#        scene = context.scene
#        mytool = scene.my_tool

#        layout.label(text="Use To Adjust Bone Offsets")
#        layout.operator("wm.calbone")
# ------------------------------------------------------------------------
# register and unregister
# ------------------------------------------------------------------------

def register():
    bpy.utils.register_class(IMUToolsPanel)
    bpy.utils.register_class(IMUConnecOperator)
    bpy.utils.register_class(IMUDisconnectConnecOperator)
    bpy.utils.register_class(IMUCalibrateOperator)
    bpy.utils.register_class(IMUInitializeOperator)
    bpy.utils.register_class(CalibrateBoneOperator)
    bpy.utils.register_class(IMUCaptureOperator)
    bpy.utils.register_class(IMUReadInFileOperator)
    bpy.utils.register_class(IMUReadResetProxySensorsOperator)
    bpy.utils.register_class(KeeCapSettings)
    bpy.types.Scene.my_tool = bpy.props.PointerProperty(type=KeeCapSettings)

def unregister():
    bpy.utils.unregister_class(IMUToolsPanel)
    bpy.utils.unregister_class(IMUConnecOperator)
    bpy.utils.unregister_class(IMUDisconnectConnecOperator)
    bpy.utils.unregister_class(IMUCalibrateOperator)
    bpy.utils.unregister_class(IMUInitializeOperator)
    bpy.utils.unregister_class(CalibrateBoneOperator)
    bpy.utils.unregister_class(IMUCaptureOperator)
    bpy.utils.unregister_class(IMUReadInFileOperator)
    bpy.utils.unregister_class(IMUReadResetProxySensorsOperator)
    bpy.utils.unregister_class(KeeCapSettings)

#if __name__ == "__main__":
#    register()


if __name__ == "__main__":
    register()
