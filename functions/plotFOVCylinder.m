function plotFOVCylinder(tform, ultraSonicSensorModel)
%EXAMPLEHELPERPLOTFOVCYLINDER Plot cylinder showing the field of view of
%the sensor
maxDetectionRange = ultraSonicSensorModel.DetectionRange(3);
fov = ultraSonicSensorModel.FieldOfView(1);

[X,Y,Z]=cylinder([0 maxDetectionRange*tan(deg2rad(fov/2))], 50);
h = maxDetectionRange;
Z = Z*h;



M1=makehgtform(yrotate=pi/2);
surf(X,Y,Z, Parent=hgtransform(Matrix=tform*M1), LineStyle='none', FaceAlpha=0.4)

% position = tform(1:3,4)';
% rpy_zyx = tform2eul(tform);
% M2=makehgtform(translate=position,...
%     xrotate=rpy_zyx(3), yrotate=rpy_zyx(2), zrotate=rpy_zyx(1));
% surf(X,Y,Z, Parent=hgtransform(Matrix=M2*M1), LineStyle='none', FaceAlpha=0.4)
end

