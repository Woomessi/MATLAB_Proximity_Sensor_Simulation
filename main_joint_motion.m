clear
clc
addpath(genpath(pwd))
%Create a robotScenario object.
% scenario = robotScenario(UpdateRate=1,StopTime=10);
scenario = robotScenario(UpdateRate=10);

%Create a rigidBodyTree object of the Franka Emika Panda manipulator using loadrobot.
robotRBT = importrobot('model_sensor_compact_win.urdf');

%Create a rigidBodyTree-based robotPlatform object using the manipulator model.
robot = robotPlatform("Manipulator",scenario, ...
                      RigidBodyTree=robotRBT);

%Create a non-rigidBodyTree-based robotPlatform object of a box to manipulate. Specify the mesh type and size.

% box = robotPlatform("Box",scenario,Collision="mesh", ...
%                     InitialBasePosition=[0.5 0.15 0.278]);
% updateMesh(box,"Cuboid",Collision="mesh",Size=[0.06 0.06 0.1])

%测距对象生成
chargeStation = robotPlatform("chargeStation", scenario, InitialBasePosition=[0.4 0 0]);%同时影响mesh与实际检测物体。但mesh的高度沿中心对称，实际检测物体的高度为沿z轴正方向。
chargeStation.updateMesh("Cuboid", Size=[0.01 1 1], Color=[0 0.8 0]);
chargingStationProfile = struct("Length", 0.01, "Width", 1, "Height", 1, 'OriginOffset', [0 0 0]);%xyz**x,length为中心对称式

ultraSonicSensorModel = ultrasonicDetectionGenerator(MountingLocation=[0 0 0], ...
    DetectionRange=[0.009 0.01 0.5], ...
    FieldOfView=[25, 25], ...
    Profiles=chargingStationProfile);

ult = robotSensor("UltraSonic", robot, ...
    CustomUltrasonicSensor(ultraSonicSensorModel), ...
    MountingLocation=[0 0 0]);%传感器的实际高度

[~, ~, det, ~] = read(ult);
%Visualize the scenario.
% ax = show3D(scenario,Collisions="on");
ax = show3D(scenario);
% view(79,36)
view(-65,45)
light
grid on

%Specify the initial and the pick-up joint configuration of the manipulator, to move the manipulator from its initial pose to close to the box.
initialConfig = homeConfiguration(robot.RigidBodyTree);
pickUpConfig = [0.2371 -0.0200 0.0542 -2.2272 0.0013 ...
                2.2072 -0.9670];
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
pose = robot.read();
rotAngle = quat2eul(pose(10:13));
hold on

if ~isempty(det)
    % Distance to object
    distance = det{1}.Measurement;
    displayText = ['Distance = ',num2str(distance)];
else
    distance = inf;
    displayText = 'No object detected!';
end

% Plot a cone to represent the field of view and range of the ultrasonic sensor
plotFOVCylinder(pose, ultraSonicSensorModel.DetectionRange(3), ultraSonicSensorModel.FieldOfView(1));
hold off

% Display the distance to the charging station detected by the ultrasonic sensor
t = text(0, 0, displayText, "BackgroundColor",'yellow');
t(1).Color = 'black';
t(1).FontSize = 10;

advance(scenario);

updateSensors(scenario);
% checkCollision(robot,"Box", ...
%                IgnoreSelfCollision="on")

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
                 1.8678 -0.2344];
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