clear
clc
addpath(genpath(pwd))

sampleRate = 10;
scenario = robotScenario(UpdateRate=sampleRate); 

addMesh(scenario,"Plane",Position=[5 0 0],Size=[20 12],Color=[0.7 0.7 0.7]);

startPosition = [-3 -3];
chargingPosition = [13 0];

wPts = [[startPosition 0.1]; ...
    5 0 0.1; ...
    10 0 0.1; ...
    13.75 0 0.1]; %Charging station

toa = [0 4 7 10];
traj = waypointTrajectory(Waypoints=wPts,...
    TimeOfArrival=toa, ReferenceFrame='ENU', ...
    SampleRate=sampleRate);
[pos, orient, vel, acc, angvel] = traj.lookupPose(0:1/sampleRate:10);

robot = robotPlatform("rst", scenario,...
    RigidBodyTree=loadrobot("clearpathHusky"), ...
    InitialBasePosition=pos(1,:), InitialBaseOrientation=compact(orient(1)));

addMesh(scenario,"Box",Position=[3  5 2],Size=[4 2 4],Color=[1 0.5 0.25],IsBinaryOccupied=true);
addMesh(scenario,"Box",Position=[3 -5 2],Size=[4 2 4],Color=[1 0.5 0.25],IsBinaryOccupied=true);
addMesh(scenario,"Box",Position=[7  5 2],Size=[4 2 4],Color=[1 0.5 0.25],IsBinaryOccupied=true);
addMesh(scenario,"Box",Position=[7 -5 2],Size=[4 2 4],Color=[1 0.5 0.25],IsBinaryOccupied=true);
addMesh(scenario,"Box",Position=[-3  -5 0.5],Size=[1 1 1],Color=[0.1 0.1 0.1]);

% addMesh(scenario,"Box",Position=[0  0 0.9],Size=[1 5 1.8],Color=[0.1 0.1 0.1],IsBinaryOccupied=true);
% Plane to denote Charging station location
addMesh(scenario,"Plane",Position=[13 0 .05],Size=[1 1],Color=[0 1 0]);

% chargeStation = robotPlatform("chargeStation", scenario,InitialBasePosition=[13.75 0 0]);
% chargeStation.updateMesh("Cuboid",Size=[0.5 1 1], Color=[0 0.8 0]);
% 
% chargingStationProfile = struct("Length", 0.5, "Width", 1, "Height", 1, 'OriginOffset', [0 0 0]);

chargeStation = robotPlatform("chargeStation", scenario,InitialBasePosition=[0.4 0 0.9]);
chargeStation.updateMesh("Cuboid",Size=[0.01 0.2 1.8], Color=[0 0.8 0]);
chargingStationProfile = struct("Length", 0.01, "Width", 0.2, "Height", 1.8, 'OriginOffset', [0 0 0]);


ultraSonicSensorModel = ultrasonicDetectionGenerator(MountingLocation=[0 0 0], ...
    DetectionRange=[0.03 0.04 5], ...
    FieldOfView=[70, 35], ...
    Profiles=chargingStationProfile);

ult = robotSensor("UltraSonic", robot, ...
    CustomUltrasonicSensor(ultraSonicSensorModel), ...
    MountingLocation=[0.5 0 0.05]);

figure(1);
ax = show3D(scenario);
view(-65,45)
light
grid on

isCharging = false;
i = 1;

setup(scenario); 

while ~isCharging
    [isUpdated, ~, det, isValid] = read(ult);
    
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

    % Plot a red shpere where the ultrasonic sensor detects an object
    % plotDetectionPoint(scenario, ...
    %     det{1}.ObjectAttributes{1}.PointOnTarget, ...
    %     ult.Name, ...
    %     pose);
        
        displayText = ['Distance = ',num2str(distance)];
    else
        distance = inf;
        displayText = 'No object detected!';
    end

    % Plot a cone to represent the field of view and range of the ultrasonic sensor
    plotFOVCylinder(pose, ultraSonicSensorModel.DetectionRange(3), 70);
    hold off

    if distance <= 0.2
        % Advance in steps of 1cm when the robot is within 20cm of the charging station
        currentMotion = lastMotion;
        currentMotion(1) = currentMotion(1) + 0.01;

        move(robot,"base",currentMotion);
        lastMotion = currentMotion;
        displayText = ['Detected Charger! Distance = ',num2str(distance)];
        if distance <= 0.05
            % The robot is charging when it is within 5cm of the charging station
            displayText = ['Charging!! Distance = ',num2str(distance)];
            isCharging = true;
        end
    else
        % Follow the waypointTrajectory to the vicinity of the charging station
        if i<=length(pos)
            motion = [pos(i,:), vel(i,:), acc(i,:), ...
                compact(orient(i)), angvel(i,:)];
            move(robot,"base",motion);
            lastMotion = motion;
            i=i+1;
        end
    end

    % Display the distance to the charging station detected by the ultrasonic sensor
    t = text(15, 0, displayText, "BackgroundColor",'yellow');
    t(1).Color = 'black';
    t(1).FontSize = 10;

    advance(scenario);
    updateSensors(scenario);
end