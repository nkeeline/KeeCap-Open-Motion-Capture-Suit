bl_info = {
    "name": "IMU Tools",
    "description": "Tools for live capture and read in of Motion Capture Suit",
    "author": "Nick Keeline",
    "version": (1, 0, 0),
    "blender": (2, 79, 0),
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
#        self.previous_point = mathutils.Euler((math.radians(0), math.radians(0.0), math.radians(0)), 'XYZ')

    def __GetFirstFloatFromString (self, string, prefix ):
        """Return Float found in the string after prefix and ending with ~.""" 
        stringAfterPrefix = string.split(prefix)[1]
        floatstring = stringAfterPrefix.split("~")[0]
        return float(floatstring)
    
    def set_TPoseAngle(self, TPose):
        """Set the TPose Position for the Sensor."""
        self.TPoseAngle = TPose
        
    def set_calibrated(self, calbool):
        """Boolean indicating Calibrated"""
        self.initialized = calbool
        
    def set_bonename(self, bonename):
        """Sensor Name if other than bone name"""
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
        self.OrientationAngle = TranspositionQuat

    def set_AngleFromLine(self, textLine):
        """Parses a Line String and retieves the Current Position"""
        TextSplitByName = textLine.split("~~" + self.name + "~")
        if(len(TextSplitByName) > 1):
            TextAfterName = TextSplitByName[1]
            if ("qW:" in TextAfterName):
                print("Quat Found in File")
                #in Quat Mode so parse Quat
                qW = self.__GetFirstFloatFromString(TextAfterName, "qW:")
                qX = self.__GetFirstFloatFromString(TextAfterName, "qX:")
                qY = self.__GetFirstFloatFromString(TextAfterName, "qY:")
                qZ = self.__GetFirstFloatFromString(TextAfterName, "qZ:")
                if self.TransposeMethod == "SwitchXYNegXNegY":
                    finalQuat = mathutils.Quaternion((qW,-qY,-qX,qZ))
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
            self.set_CurrentAngle(finalQuat)
                    
            print("CurrentAngle for " + self.name + " set To:")
            print(self.CurrentAngle)
        
    def set_TPoseAngleFromLine(self, textLine):
        """Parses a Line String and retieves the Current Position"""
        TextSplitByName = textLine.split("~~" + self.name + "~")
        if(len(TextSplitByName) > 1):
            TextAfterName = TextSplitByName[1]
            if ("qW:" in TextAfterName):
                #in Quat Mode so parse Quat
                qW = self.__GetFirstFloatFromString(TextAfterName, "qW:")
                qX = self.__GetFirstFloatFromString(TextAfterName, "qX:")
                qY = self.__GetFirstFloatFromString(TextAfterName, "qY:")
                qZ = self.__GetFirstFloatFromString(TextAfterName, "qZ:")
                self.set_TPoseAngle(mathutils.Quaternion((qW,qX,qY,qZ)))
            else:
                xAngle = self.__GetFirstFloatFromString(TextAfterName, "X:")
                yAngle = self.__GetFirstFloatFromString(TextAfterName, "Y:")
                zAngle = self.__GetFirstFloatFromString(TextAfterName, "Z:")
                CurrentEuler = mathutils.Euler((math.radians(xAngle), math.radians(yAngle), math.radians(zAngle)), 'XYZ')
                self.set_TPoseAngle(CurrentEuler.to_quaternion())
            print("TPoseAngle Set To:")
            print(self.TPoseAngle)

    def name(self):
        """Return the Name of the Bone."""
        return self.name

    def bonename(self):
        """Return the Name of the Sensor."""
        return self.bonename
    
    def CurrentAngle(self):
        """Return the Current Angle."""
        return self.CurrentAngle
    
    def OrientationAngle(self):
        """Return the Orientation Angle."""
        return self.OrientationAngle
    
    def TPoseAngle(self):
        """Return the Name of the Bone."""
        return self.TPoseAngle
    
    def firstPoint(self):
        """Return a bool telling whether this is the first point"""
        return self.firstpoint
    
    def prevPoint(self):
        """Return a Quat Angle that is the previous Point"""
        return self.prevPoint

# ------------------------------------------------------------------------
#    store properties in the active scene
# ------------------------------------------------------------------------

class MySettings(PropertyGroup):

    my_bool = BoolProperty(
        name="KeyFrame to Rig during file or live capture",
        description="Determines whether we keyframe and record the keyframes to the rig when we read in a file.",
        default = False
        )
        
    save_to_txt = BoolProperty(
        name="Log Live Capture to Text File",
        description="When Capturing Data From the RS-232 do we want to save it to a text file as we do?",
        default = False
        )

    my_int = IntProperty(
        name = "Number of Samples",
        description="Number of Samples to Take",
        default = 200,
        min = 10,
        max = 10000
        )

    my_float = FloatProperty(
        name = "Float Value",
        description = "A float property",
        default = 23.7,
        min = 0.01,
        max = 30.0
        )

    my_string = StringProperty(
        name="User Input",
        description=":",
        default="",
        maxlen=1024,
        )

    savelogfile = StringProperty(
        name="Log File",
        description="Choose a file to log Capture to:",
        default="c:\log.txt",
        maxlen=1024,
        subtype='FILE_PATH'
        )

    my_file = StringProperty(
        name="Capture File",
        description="Choose a file to read in:",
        default="",
        maxlen=1024,
        subtype='FILE_PATH'
        )
        
    my_enum = EnumProperty(
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

#I programmed and entire script to test the code below to correct for bad position sensors
#this inputs the tpose for the sensor the bones rest position and current sensor rotation and
#returns the final rotation corrected for the TPose offset from the bone rest possition.
def finalPositionTPoseCorrected(BoneRestRot,TPoseRot,SenseRot):
    #find the difference between the TPos Position and the current sensor poristion.
    TRotSenseRotDiff = SenseRot.inverted().rotation_difference(TPoseRot.inverted())
    #rotate the bone to it's rest position and apply the difference to it.
    rotation = TRotSenseRotDiff * BoneRestRot
    #final rotation to apply to bone.
    return rotation

def GetBonePositionToSetInArmatureSpaceGivenWorldCoordQuat(Bone, WorldQuat):    
    #so we get the angle of the bone in armature space
    #this is the angle with all parent bones modifying it.
    ArmSpaceQuat = Bone.matrix.to_quaternion()
    
    #Next we get the pose position from the base of the bone and it's inverse
    PoseAngleQuat = Bone.rotation_quaternion
    PoseAngleQuatInv = PoseAngleQuat.inverted()
    
    #To get the angle of the rest (no angle applied) position of the pose bone in armature space 
    # we take the poses current position and move
    #it by the inverse of the pose angle
    PoseNoAngleBaseArmSpaceQuat = ArmSpaceQuat*PoseAngleQuatInv
        
    #so now the rotation is the rest position in armature space rotated by our target world space rotation
    BoneRotationToSetInArmatureSpace = PoseNoAngleBaseArmSpaceQuat.inverted() * WorldQuat
   
    #Ok, make this the final quat to use....
    return BoneRotationToSetInArmatureSpace.normalized()
    
def SetBonePosition(scene, ThisSensor, RigArmature, WeShouldKeyframe):
    
    mytool = scene.my_tool
    pose_bone = RigArmature.pose.bones[ThisSensor.bonename]
    
    #armature_to_world = armature.matrix_world
    #armature_to_world_quat = armature_to_world.to_quaternion()
    
    #Get the rotation of the bone in edit mode
    BoneEdit = RigArmature.data.bones[ThisSensor.name]
    BoneEditRotation = BoneEdit.matrix_local.to_quaternion()
    
    #This rotates the measuremnt by 90 degree increments to adjust for the sensors orientation compared ot the bone.
    #old method doesn't use tpos to re-orient
    SensorAdjusted = ThisSensor.CurrentAngle * ThisSensor.OrientationAngle
    #SensorAdjusted = finalPositionTPoseCorrected(BoneEditRotation,ThisSensor.TPoseAngle * ThisSensor.OrientationAngle,ThisSensor.CurrentAngle* ThisSensor.OrientationAngle)
    
    #Ok, make this the final quat to use....
    FinalQuat = GetBonePositionToSetInArmatureSpaceGivenWorldCoordQuat(pose_bone, SensorAdjusted)
    #FinalQuat = BonePositionInWorldSpace.normalized()

    #Set Bone Position now that we've calculated it.
    pose_bone.rotation_mode = 'QUATERNION'
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

    pose_bone.rotation_quaternion = FinalQuat
    ThisSensor.set_PreviousPoint(FinalQuat)
        
    if (WeShouldKeyframe):
        #bpy.ops.objects['metarig']
        #if (mytool.my_enum == 'EULER'):
            #pose_bone.rotation_mode = 'XYZ'
            #pose_bone.keyframe_insert('rotation_euler')
        #else:
        pose_bone.rotation_mode = 'QUATERNION'
        pose_bone.keyframe_insert('rotation_quaternion')

def SetAllSensorsToSelectedMode(Sensors, scene, armature):
    mytool = scene.my_tool
    for Sense in Sensors:
        #if (mytool.my_enum == 'EULER'):
            #armature.pose.bones[Sense.name].rotation_mode = 'XYZ'
        #else:
        armature.pose.bones[Sense.name].rotation_mode = 'QUATERNION'
    
def SetupSensors(scene, armature):
    mytool = scene.my_tool
    Sensors = []
    SwitchThighShin = True;
    #Arm/Flat Oriented sensors
    #if (mytool.my_enum == "QUATERNION"):
    adjustmentfinal1 = mathutils.Euler((math.radians(0), math.radians(0.0), math.radians(0)), 'XYZ')
    #else:
        #adjustmentfinal = mathutils.Euler((math.radians(0), math.radians(0.0), math.radians(0)), 'XYZ')

    #CHest/vertical orented sensors
    #if (mytool.my_enum == "QUATERNION"):
    adjustmentfinal2 = mathutils.Euler((math.radians(0), math.radians(180.0), math.radians(-90)), 'XYZ')
    #else:
        #adjustmentfinal = mathutils.Euler((math.radians(180), math.radians(0), math.radians(90)), 'XYZ')
        
    LeftArmAdjust = mathutils.Euler((math.radians(0), math.radians(0.0), math.radians(-90)), 'XYZ')
    headadjustment = mathutils.Euler((math.radians(90), math.radians(0.0), math.radians(0)), 'XYZ')
    chestadjustment = mathutils.Euler((math.radians(20), math.radians(0.0), math.radians(0)), 'XYZ')
    waistadjustment = mathutils.Euler((math.radians(180), math.radians(180.0), math.radians(0)), 'XYZ')
    legAdjustment = mathutils.Euler((math.radians(-25), math.radians(180.0), math.radians(-90)), 'XYZ')
    thighAdjustment = mathutils.Euler((math.radians(0), math.radians(180.0), math.radians(-90)), 'XYZ')
    footAdjustment = mathutils.Euler((math.radians(0), math.radians(180.0), math.radians(-90)), 'XYZ')
    
    Waist = Sensor("spine")
    Waist.set_bonename("hips")
    Waist.set_TranspositionMethod("SwitchXYNegXNegY")
    Waist.set_OrientationAngle(waistadjustment.to_quaternion())
    armature.pose.bones[Waist.name].rotation_mode = 'QUATERNION'
    Sensors.append(Waist)

    Chest = Sensor("chest")
    Chest.set_TranspositionMethod("SwitchXYNegXNegY")
    Chest.set_OrientationAngle(chestadjustment.to_quaternion())
    armature.pose.bones[Chest.name].rotation_mode = 'QUATERNION'
    Sensors.append(Chest)
    
    Head = Sensor("head")
    Head.set_TranspositionMethod("SwitchXYNegXNegY")
    Head.set_OrientationAngle(headadjustment.to_quaternion())
    armature.pose.bones[Head.name].rotation_mode = 'QUATERNION'
    Sensors.append(Head)
    

    #Right Arm
    UpperArmRight = Sensor("upper_arm.R")
    UpperArmRight.set_TranspositionMethod("SwitchXYNegXNegY")
    UpperArmRight.set_OrientationAngle(adjustmentfinal2.to_quaternion())
    armature.pose.bones[UpperArmRight.name].rotation_mode = 'QUATERNION'
    Sensors.append(UpperArmRight)

    LowerArmRight = Sensor("forearm.R")
    LowerArmRight.set_TranspositionMethod("SwitchXYNegXNegY")
    LowerArmRight.set_OrientationAngle(adjustmentfinal2.to_quaternion())
    armature.pose.bones[LowerArmRight.name].rotation_mode = 'QUATERNION'
    Sensors.append(LowerArmRight)

    HandRight = Sensor("hand.R")
    HandRight.set_TranspositionMethod("SwitchXYNegXNegY")
    HandRight.set_OrientationAngle(adjustmentfinal2.to_quaternion())
    armature.pose.bones[HandRight.name].rotation_mode = 'QUATERNION'
    Sensors.append(HandRight)
        
    #Left Arm
    UpperArmLeft = Sensor("upper_arm.L")
    UpperArmLeft.set_TranspositionMethod("SwitchXYNegXNegY")
    UpperArmLeft.set_OrientationAngle(LeftArmAdjust.to_quaternion())
    armature.pose.bones[UpperArmLeft.name].rotation_mode = 'QUATERNION'
    Sensors.append(UpperArmLeft)

    LowerArmLeft = Sensor("forearm.L")
    LowerArmLeft.set_TranspositionMethod("SwitchXYNegXNegY")
    LowerArmLeft.set_OrientationAngle(LeftArmAdjust.to_quaternion())
    armature.pose.bones[LowerArmLeft.name].rotation_mode = 'QUATERNION'
    Sensors.append(LowerArmLeft)

    HandLeft = Sensor("hand.L")
    HandLeft.set_TranspositionMethod("SwitchXYNegXNegY")
    HandLeft.set_OrientationAngle(LeftArmAdjust.to_quaternion())
    armature.pose.bones[HandLeft.name].rotation_mode = 'QUATERNION'
    Sensors.append(HandLeft)
    

    #Right Leg
    UpperLegRight = Sensor("thigh.R")
    if (SwitchThighShin):
        UpperLegRight.set_bonename("shin.R")
    UpperLegRight.set_TranspositionMethod("SwitchXYNegXNegY")
    UpperLegRight.set_OrientationAngle(thighAdjustment.to_quaternion())
    armature.pose.bones[UpperLegRight.name].rotation_mode = 'QUATERNION'
    Sensors.append(UpperLegRight)

    LowerLegRight = Sensor("shin.R")
    if (SwitchThighShin):
        LowerLegRight.set_bonename("thigh.R")
    LowerLegRight.set_TranspositionMethod("SwitchXYNegXNegY")
    LowerLegRight.set_OrientationAngle(legAdjustment.to_quaternion())
    armature.pose.bones[LowerLegRight.name].rotation_mode = 'QUATERNION'
    Sensors.append(LowerLegRight)

    FootRight = Sensor("foot.R")
    FootRight.set_TranspositionMethod("SwitchXYNegXNegY")
    FootRight.set_OrientationAngle(footAdjustment.to_quaternion())
    armature.pose.bones[FootRight.name].rotation_mode = 'QUATERNION'
    Sensors.append(FootRight)
    

    #Left Leg
    UpperLegLeft = Sensor("thigh.L")
    if (SwitchThighShin):
        UpperLegLeft.set_bonename("shin.L")
    UpperLegLeft.set_TranspositionMethod("SwitchXYNegXNegY")
    UpperLegLeft.set_OrientationAngle(thighAdjustment.to_quaternion())
    armature.pose.bones[UpperLegLeft.name].rotation_mode = 'QUATERNION'
    Sensors.append(UpperLegLeft)

    LowerLegLeft = Sensor("shin.L")
    if (SwitchThighShin):
        LowerLegLeft.set_bonename("thigh.L")
    LowerLegLeft.set_TranspositionMethod("SwitchXYNegXNegY")
    LowerLegLeft.set_OrientationAngle(legAdjustment.to_quaternion())
    armature.pose.bones[LowerLegLeft.name].rotation_mode = 'QUATERNION'
    Sensors.append(LowerLegLeft)

    FootLeft = Sensor("foot.L")
    FootLeft.set_TranspositionMethod("SwitchXYNegXNegY")
    FootLeft.set_OrientationAngle(footAdjustment.to_quaternion())
    armature.pose.bones[FootLeft.name].rotation_mode = 'QUATERNION'
    Sensors.append(FootLeft)
    
    return Sensors

def GetArmature():
    return bpy.data.objects['metarig']
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
        ser.port = '/dev/ttyACM0'
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
        scene = context.scene
        mytool = scene.my_tool
        ser.reset_input_buffer()
        
        print("Initialize Sensors...")
        space_data = bpy.context.space_data
        
        # Make variable scene that is current scene
        scene = context.scene
        
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
                    Sense.set_TPoseAngleFromLine(line)
            if (line.split('  ', 1)[0] == 'PNTSTART'):
                for Sense in Sensors:
                    if (Sense.name in line):
                        print(Sense.name)
                        Sense.set_AngleFromLine(line)
                        SetBonePosition(scene, Sense, armature, mytool.my_bool)
                bpy.ops.wm.redraw_timer(type='DRAW_WIN_SWAP', iterations=1)
                if(mytool.my_bool):
                    scene.frame_current = scene.frame_current + 1
        
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
        
        Sensors = SetupSensors(scene, armature)
        
        print(filepath)
        file = open(filepath, "r")
        for line in file:
            time.sleep(.02)
            print(line)
            if (line.split('  ', 1)[0] == 'TPOSE'):
                for Sense in Sensors:
                    Sense.set_TPoseAngleFromLine(line)
            if (line.split('  ', 1)[0] == 'PNTSTART'):
                for Sense in Sensors:
                    if (Sense.name in line):
                        print(Sense.name)
                        print(Sense.bonename)
                        Sense.set_AngleFromLine(line)
                        SetBonePosition(scene, Sense, armature, mytool.my_bool)
                if(mytool.my_bool):
                    scene.frame_current = scene.frame_current + 2
                    print("Set Frame to:" + str(scene.frame_current))
                bpy.ops.wm.redraw_timer(type='DRAW_WIN_SWAP', iterations=1)
        file.close()
        
        SetAllSensorsToSelectedMode(Sensors, scene, armature)
        
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

class OBJECT_PT_my_panel(Panel):
    bl_idname = "OBJECT_PT_my_panel"
    bl_label = "Keeline IMU Tools"
    bl_space_type = "VIEW_3D"   
    bl_region_type = "TOOLS"    
    bl_category = "Tools"
    bl_context = "objectmode"   

    @classmethod
    def poll(self,context):
        return context.object is not None

    def draw(self, context):
        layout = self.layout
        scene = context.scene
        mytool = scene.my_tool

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
        layout.prop(mytool, "save_to_txt")
        layout.prop(mytool, "my_enum", text="") 
        #mytool.my_enum = 'MEDIUM'
        #layout.menu("OBJECT_MT_select_test", text="Presets", icon="SCENE")

# ------------------------------------------------------------------------
# register and unregister
# ------------------------------------------------------------------------

def register():
    bpy.utils.register_module(__name__)
    bpy.types.Scene.my_tool = PointerProperty(type=MySettings)

def unregister():
    bpy.utils.unregister_module(__name__)
    del bpy.types.Scene.my_tool

if __name__ == "__main__":
    register()