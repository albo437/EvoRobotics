import pyrosim.pyrosim as pyrosim

pyrosim.Start_SDF("box.sdf")

#Dimensions of the box
length = 1
width = 2
height = 3
#Position of the box
x = 0
y = 0
z = 1.5
pyrosim.Send_Cube(name="Box", pos=[x, y, z] , size=[length, width, height])
pyrosim.End()