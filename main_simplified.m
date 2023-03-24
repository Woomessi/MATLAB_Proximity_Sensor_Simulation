clear
clc
close all
addpath(genpath(pwd))

sampleRate = 10;
scenario = robotScenario(UpdateRate=sampleRate);

%机器人生成
my_robot = importrobot('model_sensor_compact_win.urdf');
robot = robotPlatform("rst", scenario, RigidBodyTree=my_robot);

%测距对象生成
chargeStation = robotPlatform("chargeStation", scenario, InitialBasePosition=[0.4 0 0]);%同时影响mesh与实际检测物体。但mesh的高度沿中心对称，实际检测物体的高度为沿z轴正方向。
chargeStation.updateMesh("Cuboid", Size=[0.01 1 2.8], Color=[0 0.8 0]);
chargingStationProfile = struct("Length", 0.01, "Width", 1, "Height", 2.8, 'OriginOffset', [0 0 0]);%xyz**x,length为中心对称式

ultraSonicSensorModel = ultrasonicDetectionGenerator(MountingLocation=[0 0 0], ...
    DetectionRange=[0.009 0.01 0.5], ...
    FieldOfView=[25, 25], ...
    Profiles=chargingStationProfile);

ult = robotSensor("UltraSonic", robot, ...
    CustomUltrasonicSensor(ultraSonicSensorModel), ...
    MountingLocation=[0 0 0]);%传感器的实际高度

setup(scenario);

[~, ~, det, ~] = read(ult);

figure(1);
show3D(scenario);
view(-65,45)
light
grid on

% Read the motion vector of the robot from the platform ground truth
% This motion vector will be used only for plotting graphic elements
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