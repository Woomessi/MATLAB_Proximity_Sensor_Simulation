clear
clc
addpath(genpath(pwd))

%Create a robotScenario object.
scenario = robotScenario(UpdateRate=1,StopTime=10);

%Create a rigidBodyTree object of the Franka Emika Panda manipulator using loadrobot.
robotRBT = loadrobot("frankaEmikaPanda");

%Create a rigidBodyTree-based robotPlatform object using the manipulator model.
robot = robotPlatform("Manipulator",scenario, ...
                      RigidBodyTree=robotRBT);

%Create a non-rigidBodyTree-based robotPlatform object of a box to manipulate. Specify the mesh type and size.
box = robotPlatform("Box",scenario,Collision="mesh", ...
                    InitialBasePosition=[0.5 0.15 0.278]);
updateMesh(box,"Cuboid",Collision="mesh",Size=[0.06 0.06 0.1])

%Visualize the scenario.
ax = show3D(scenario,Collisions="on");
view(79,36)
light

%Specify the initial and the pick-up joint configuration of the manipulator, to move the manipulator from its initial pose to close to the box.
initialConfig = homeConfiguration(robot.RigidBodyTree);
pickUpConfig = [0.2371 -0.0200 0.0542 -2.2272 0.0013 ...
                2.2072 -0.9670 0.0400 0.0400];

%Create an RRT path planner using the manipulatorRRT object, and specify the manipulator model.
planner = manipulatorRRT(robot.RigidBodyTree,scenario.CollisionMeshes);
planner.IgnoreSelfCollision = true;

%Plan the path between the initial and the pick-up joint configurations. Then, to visualize the entire path, interpolate the path into small steps.
rng("default")
path = plan(planner,initialConfig,pickUpConfig);
path = interpolate(planner,path,25);

%Set up the simulation.
setup(scenario)

%Check the collision before manipulator picks up the box.
checkCollision(robot,"Box", ...
               IgnoreSelfCollision="on")

%Move the joints of the manipulator along the path and visualize the scenario.
helperRobotMove(path,robot,scenario,ax)

%Check the collision after manipulator picks up the box.
checkCollision(robot,"Box", ...
               IgnoreSelfCollision="on")

%Use the attach function to attach the box to the gripper of the manipulator.
attach(robot,"Box","panda_hand", ...
       ChildToParentTransform=trvec2tform([0 0 0.1]))

%Specify the drop-off joint configuration of the manipulator to move the manipulator from its pick-up pose to the box drop-off pose.
dropOffConfig = [-0.6564 0.2885 -0.3187 -1.5941 0.1103 ...
                 1.8678 -0.2344 0.04 0.04];

%Plan the path between the pick-up and drop-off joint configurations.
path = plan(planner,pickUpConfig,dropOffConfig);
path = interpolate(planner,path,25);

%Move the joints of the manipulator along the path and visualize the scenario.
helperRobotMove(path,robot,scenario,ax)

%Use the detach function to detach the box from the manipulator gripper.
detach(robot)

%Plan the path between the drop-off and initial joint configurations to move the manipulator from its box drop-off pose to its initial pose.
path = plan(planner,dropOffConfig,initialConfig);
path = interpolate(planner,path,25);

%Move the joints of the manipulator along the path and visualize the scenario.
helperRobotMove(path,robot,scenario,ax)