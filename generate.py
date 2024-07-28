import pyrosim.pyrosim as pyrosim

def create_world():
    pyrosim.Start_SDF("world.sdf")

    #Dimensions of the box
    length = 1
    width = 1
    height = 1
    #Position of the box
    x = -4
    y = -4
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
    x = 1.5
    y = 0
    z = 1.5

    pyrosim.Send_Cube(name = "Torso", pos = [x, y, z], size = [length, width, height])
    pyrosim.Send_Joint( name = "Torso_BackLeg" , parent= "Torso" , child = "BackLeg" , type = "revolute", position = [1, 0, 1])
    pyrosim.Send_Cube(name = "BackLeg", pos = [-0.5, 0, -0.5], size = [length, width, height])
    pyrosim.Send_Joint( name = "Torso_FrontLeg" , parent= "Torso" , child = "FrontLeg" , type = "revolute", position = [2, 0, 1])
    pyrosim.Send_Cube(name = "FrontLeg", pos = [0.5, 0, -0.5], size = [length, width, height])
    
    pyrosim.End()

create_world()
create_robot()