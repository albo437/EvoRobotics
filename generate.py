import pyrosim.pyrosim as pyrosim

pyrosim.Start_SDF("boxes.sdf")

#Dimensions of the box
length = 1
width = 1
height = 1
#Position of the box
x = 0
y = 0
z = 0.5
size_modifier = 1
name = 0
for xpos in range(5):
    for ypos in range(5):
        for i in range(10):
            name += 1
            boxname = "box" + str(name)
            pyrosim.Send_Cube(boxname, [x, y, z], [length*size_modifier, width*size_modifier, height*size_modifier])
            z = z + height
            size_modifier *= 0.9
        x = x + length
        z = 0.5
        size_modifier = 1
    x = 0
    y = y + width
pyrosim.End()