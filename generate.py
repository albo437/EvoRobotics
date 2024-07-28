import pyrosim.pyrosim as pyrosim

def create_world():
    pyrosim.Start_SDF("world.sdf")

    #Dimensions of the box
    length = 1
    width = 1
    height = 1
    #Position of the box
    x = 0
    y = 0
    z = 0.5

    pyrosim.Send_Cube(name = "box", pos = [x, y, z], size = [length, width, height])
    pyrosim.End()

def create_robot():
    pyrosim.Start_URDF("body.urdf")
    #Dimensions of the box
    length = 1
    width = 1
    height = 1
    #Position of the box
    x = 0
    y = 0
    z = 0.5

    pyrosim.Send_Cube(name = "torso", pos = [x, y, z], size = [length, width, height])
    pyrosim.End()

create_world()
create_robot()