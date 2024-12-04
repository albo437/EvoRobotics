import pybullet as p

from pyrosim.nndf import NNDF

from pyrosim.linksdf  import LINK_SDF

from pyrosim.linkurdf import LINK_URDF

from pyrosim.model import MODEL

from pyrosim.sdf   import SDF

from pyrosim.urdf  import URDF

from pyrosim.joint import JOINT

import numpy as np

SDF_FILETYPE  = 0

URDF_FILETYPE = 1

NNDF_FILETYPE   = 2

# global availableLinkIndex

# global linkNamesToIndices

def End():

    if filetype == SDF_FILETYPE:

        sdf.Save_End_Tag(f)

    elif filetype == NNDF_FILETYPE:

        nndf.Save_End_Tag(f)
    else:
        urdf.Save_End_Tag(f)

    f.close()

def End_Model():

    model.Save_End_Tag(f)

def Get_Touch_Sensor_Value_For_Link(linkName):

    touchValue = -1.0

    desiredLinkIndex = linkNamesToIndices[linkName]

    pts = p.getContactPoints()

    for pt in pts:

        linkIndex = pt[4]

        if ( linkIndex == desiredLinkIndex ):

            touchValue = 1.0

    return touchValue

def Get_Ray_Sensor_Value_For_Link(bodyID, linkName):
    bodyID = 2

    # Get the link position and orientation
    linkIndex = linkNamesToIndices[linkName]
    linkPosition, linkOrientation = p.getLinkState(bodyID, linkIndex)[:2]

    # Convert quaternion to rotation matrix to determine orientation
    rotation_matrix = p.getMatrixFromQuaternion(linkOrientation)
    forward_vector = np.array([rotation_matrix[0], rotation_matrix[3], rotation_matrix[6]])

    # Offset `rayFrom` to the front face of the cube
    front_face_position = np.array(linkPosition) + forward_vector * 0.5

    # Parameters for the square projection grid
    square_size = 0.5  # Square side length for the grid of rays
    num_points = 5     # Number of points per row/column

    # Initialize lists for ray start (rayFrom) and end (rayTo) positions
    rayFrom_positions = []
    rayTo_positions = []

    # Set up the grid of rays
    half_size = square_size / 2
    spacing = square_size / (num_points - 1)

    for i in range(num_points):
        for j in range(num_points):
            # Calculate offset within the square for each point
            x_offset = (i * spacing) - half_size
            y_offset = (j * spacing) - half_size

            # `rayFrom`: starting at the cube's front face
            ray_from_position = front_face_position + np.dot(rotation_matrix[:3], [x_offset, y_offset, 0])
            rayFrom_positions.append(ray_from_position.tolist())

            # `rayTo`: end points further in the forward direction, creating the grid in front
            ray_to_position = ray_from_position + forward_vector * 20.0  # Adjust this distance as needed
            rayTo_positions.append(ray_to_position.tolist())

    # Cast rays in batch
    results = p.rayTestBatch(rayFrom_positions, rayTo_positions)

     # Calculate the number of hits to body 1
    hit_count = sum([1 for result in results if result[0] == 1])
    total_rays = len(results)

    # Calculate score between -1 and 1
    hit_fraction = hit_count / total_rays
    score = 2 * hit_fraction - 1  # Map [0, 1] to [-1, 1]

    return score

def Prepare_Link_Dictionary(bodyID):

    global linkNamesToIndices

    linkNamesToIndices = {}

    for jointIndex in range( 0 , p.getNumJoints(bodyID) ):

        jointInfo = p.getJointInfo( bodyID , jointIndex )

        jointName = jointInfo[1]

        jointName = jointName.decode("utf-8")

        jointName = jointName.split("_")

        linkName = jointName[1]

        linkNamesToIndices[linkName] = jointIndex

        if jointIndex==0:

           rootLinkName = jointName[0]

           linkNamesToIndices[rootLinkName] = -1 

def Prepare_Joint_Dictionary(bodyID):

    global jointNamesToIndices

    jointNamesToIndices = {}

    for jointIndex in range( 0 , p.getNumJoints(bodyID) ):

        jointInfo = p.getJointInfo( bodyID , jointIndex )

        jointName = jointInfo[1]

        jointNamesToIndices[jointName] = jointIndex

def Prepare_To_Simulate(bodyID):

    Prepare_Link_Dictionary(bodyID)

    Prepare_Joint_Dictionary(bodyID)

def Send_Cube(name="default",pos=[0,0,0],size=[1,1,1]):

    global availableLinkIndex

    global links

    if filetype == SDF_FILETYPE:

        Start_Model(name,pos)

        link = LINK_SDF(name,pos,size)

        links.append(link)
    else:
        link = LINK_URDF(name,pos,size)

        links.append(link)

    link.Save(f)

    if filetype == SDF_FILETYPE:

        End_Model()

    linkNamesToIndices[name] = availableLinkIndex

    availableLinkIndex = availableLinkIndex + 1

def Send_Joint(name,parent,child,type,position,jointAxis):

    joint = JOINT(name,parent,child,type,position)

    joint.Save(f, jointAxis)

def Send_Motor_Neuron(name,jointName):

    f.write('    <neuron name = "' + str(name) + '" type = "motor"  jointName = "' + jointName + '" />\n')

def Send_Sensor_Neuron(name,linkName):

    f.write('    <neuron name = "' + str(name) + '" type = "sensor" linkName = "' + linkName + '" />\n')

def Send_Synapse( sourceNeuronName , targetNeuronName , weight ):

    f.write('    <synapse sourceNeuronName = "' + str(sourceNeuronName) + '" targetNeuronName = "' + str(targetNeuronName) + '" weight = "' + str(weight) + '" />\n')

def Send_Hidden_Neuron(name):

    f.write('    <neuron name = "' + str(name) + '" type = "hidden" />\n')
 
def Set_Motor_For_Joint(bodyIndex,jointName,controlMode,targetPosition,maxForce):

    p.setJointMotorControl2(

        bodyIndex      = bodyIndex,

        jointIndex     = jointNamesToIndices[jointName],

        controlMode    = controlMode,

        targetPosition = targetPosition,

        force          = maxForce)

def Start_NeuralNetwork(filename):

    global filetype

    filetype = NNDF_FILETYPE

    global f

    f = open(filename,"w")

    global nndf

    nndf = NNDF()

    nndf.Save_Start_Tag(f)

def Start_SDF(filename):

    global availableLinkIndex

    availableLinkIndex = -1

    global linkNamesToIndices

    linkNamesToIndices = {}

    global filetype

    filetype = SDF_FILETYPE

    global f
 
    f = open(filename,"w")

    global sdf

    sdf = SDF()

    sdf.Save_Start_Tag(f)

    global links

    links = []

def Start_URDF(filename):

    global availableLinkIndex

    availableLinkIndex = -1

    global linkNamesToIndices

    linkNamesToIndices = {}

    global filetype

    filetype = URDF_FILETYPE

    global f

    f = open(filename,"w")

    global urdf 

    urdf = URDF()

    urdf.Save_Start_Tag(f)

    global links

    links = []

def Start_Model(modelName,pos):

    global model 

    model = MODEL(modelName,pos)

    model.Save_Start_Tag(f)
