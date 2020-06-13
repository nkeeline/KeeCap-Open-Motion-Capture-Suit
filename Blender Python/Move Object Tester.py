

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

def GetFirstFloatFromString (string, prefix ):
    stringAfterPrefix = string.split(prefix)[1]
    floatstring = stringAfterPrefix.split("~")[0]
    return float(floatstring)


    
#def getValueFromPositionArrayOutlierRejected(arr):
#    i=0
#    for val in arr:
        
obj = bpy.context.selected_objects[0]

ser = serial.Serial()

scaleFactor = .005
NumAverages = 5
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
scene = bpy.context.scene

ser.baudrate = 115200
ser.port = '/dev/ttyUSB0'
ser.open()
print("connecting...")
iteration = 0
#read serial ten times to get into real data:
while iteration < 10:
    a = ser.readline().decode("ascii")
    print(a)
    iteration = iteration+1

sensorFirstPosx = GetFirstFloatFromString(a,"Xmm:")*scaleFactor
sensorFirstPosy = GetFirstFloatFromString(a,"Ymm:")*scaleFactor
sensorFirstPosz = GetFirstFloatFromString(a,"Zmm:")*scaleFactor

iteration = 0
xpos = []
ypos = []
zpos = []
startposx = obj.location.x
startposy = obj.location.y
startposz = obj.location.z
while iteration < 300:
    a = ser.readline().decode("ascii")
    print(a)
    iteration = iteration + 1
    xpos.append(sensorFirstPosx - GetFirstFloatFromString(a,"Xmm:")*scaleFactor)
    ypos.append(sensorFirstPosy - GetFirstFloatFromString(a,"Ymm:")*scaleFactor)
    zpos.append(sensorFirstPosz - GetFirstFloatFromString(a,"Zmm:")*scaleFactor)
    
    
    if len(xpos) > NumAverages:
        obj.location.x = sum(xpos) / len(xpos)
        obj.location.y = sum(ypos) / len(ypos)
        obj.location.z = sum(zpos) / len(zpos)
        bpy.ops.wm.redraw_timer(type='DRAW_WIN_SWAP', iterations=1)
        xpos.clear()
        ypos.clear()
        zpos.clear()
        
ser.reset_input_buffer()

print("Disconnecting From Rig...")

ser.close()

obj.location = ((0.0, 0.0, 0.0))