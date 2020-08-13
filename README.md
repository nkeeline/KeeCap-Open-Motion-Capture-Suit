# KeeCap-Open-Motion-Capture-Suit
This is an arduino based open motion capture suit and all files necessary to build it.

If you are interested in this suit please watch these video tutorials:
Tutorial on how to use this code:

https://youtu.be/FGNmMZX0g_M

Tutorial on how the suit talks to Blender:

https://youtu.be/CypV9pPTCXo

If you want to test the PC only side without a suit you can use the latest python script from blender python dir in blender 2.83 and open the Move rig tester from the blender dir.

For the script to work in blender you MUST install PySerial into blender watch the second tutorial above to see how.

# This folder...

The suit bom.ods in the root contains all part numbers and prices for the components to buy the suit.  Open the file in libre office calc.  Most of the parts listed are from digikey.com, for example the imu board is this:
https://www.digikey.com/product-detail/en/microchip-technology/ATBNO055-XPRO/ATBNO055-XPRO-ND/5230918

# Arduino Code

This directory contains all of the code to put on the Arduino Mega 2560 in the suit.  It also contains the libraries the code uses you will need to place in your aruino libraries folder for the .ino file to compile.

# Blender Files

This contains the blender 2.83 files to export stl files necessary to 3d print the suit and a Move Rig.blend file with a character in it to map the motion captures to.

# Blender Python 

Contains a copy of the .py files for the addon to run in blender to import files from the suit.

# Build Procedures

This is the work instructions step by step for building the suit.


# Captures

Here are some sample captures taken during development and on set with sample videos of the capture process.

# Schematics Diagrams

These are the fritzing schematics and diagrams for the wiring of the suite to get an overview of how it's wired.
