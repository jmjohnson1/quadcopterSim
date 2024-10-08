position = s(1:3, :); % [m]
quat = s(4:7, :);
velocity = s(8:10, :);  % Velocity [m/s]
angularRates = s(11:13, :)*rad2deg;  % Rotation rate of b rel to n [deg/s]
motorRotRate = s(14:17, :);  % Motor anuglar velocities [rad/s]
thrust = zeros(1, numPoints);
eulerAngles = zeros(3, numPoints);
setpoints.euler = setpoints.euler*180/pi;

for (LV1 = 1:numPoints)
  C_bn = Quaternion2DCM(s(4:7, LV1));
  eulerAngles(:, LV1) = DCM2Euler321(C_bn)*rad2deg;
	thrust(LV1) = sum(const.kt*motorRotRate(:, LV1).^2);
end