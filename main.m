clear; close all; clc;
% Add the helper functions to the Matlab search path
addpath('./helperFunctions/');

% profile on;

constants;

% Trajectory type
% trajType = 'testAttitudeStabilize';
% trajType = 'testStepX';
% trajType = 'testStepY';
% trajType = 'testStepZ';
% trajType = 'testStep';
% waypoints = [];

trajType = 'minsnap';
t_wp = linspace(0, 10, 5);
waypoints = [0, 0, -0.75;
						 -1, 0, -0.75;
						 -1, 1, -0.75;
						 0, 1, -0.75;
						 0, 0, -0.75]';
waypoints = [waypoints; t_wp];
traj = GenerateTrajectory(waypoints, trajType, false); % Import trajectory

flightVar.mode = 'takeoff';
flightVar.takeoffFlag = false;
flightVar.landingFlag = false;
flightVar.waypointArrived = false;
flightVar.waypointArrivedTime = [];
flightVar.okToStartMission = false;
flightVar.missionStartTime = [];

minW = const.minW;
%%%%%%%%%%%%%%%%%%%%%%%%
%  Initial conditions  %
%%%%%%%%%%%%%%%%%%%%%%%%
initPosition = [0, 0, 0]';  % initial position in local NED frame [m]
initVelocity = [0, 0, 0]';  % initial velocity in local NED frame [m/s]
initAttitude = [0, 0, 160]'*deg2rad;  % initial roll, pitch, yaw [rad]
initRates = [0, 0, 0]';  % Initial angular rates in body frame [rad/s]
hoverThrust = const.mB_ctrl*9.81/4*[1, 1, 1, 1]';
initW = sqrt(hoverThrust./const.kt);  % Initial motor angular velocities [rad/s]
initWdot = [0, 0, 0, 0]';  % Initial motor angular acceleration [rad/s^2]

% Time options
tStart = 0;  % Simulation end time [sec]
tEnd = 30;  % Simulation start time [sec]
looprateFC = 250;  % Flight controller loop rate [Hz]
numSnapshots = 1;  % Number of states to save between each FC update

initC_bn = Euler3212DCM(initAttitude);
initq_bn = DCM2Quaternion(initC_bn);  % Scalar first

% Figure out time steps and the sizes of matrices that need to be initialized
dt_flightControl = 1/looprateFC;  % Flight controller time step [sec]
numUpdates = floor((tEnd - tStart)/dt_flightControl) + 1;
dt_sim = dt_flightControl/numSnapshots;  % Saved states time step [sec]
numPoints = (numUpdates)*numSnapshots + 1;
tSim = linspace(tStart, tEnd, numPoints);

disp("Simulation starting...");
tic;

%%%%%%%%%%%%%%%%
%  Simulation  %
%%%%%%%%%%%%%%%%
ctrl = controller; % Create controller object
options = odeset('AbsTol',1e-11,'RelTol',1e-11); % Set integration tolerences
% Initial state vector and memory allocation
s0 = [initPosition; initq_bn; initVelocity; initRates; initW];
s = zeros(length(s0), numPoints);
sNoisy = zeros(13, numPoints);
s(:, 1) = s0;
% Noisy state vector used in the controller
sNoisy(:, 1) = s0(1:13);
sIndex = 1;  % This will track where we are in the sim
tFc = zeros(1, numUpdates);  % Will be useful to have the times that correspond to flight controller updates
setpoints.position = zeros(3, numUpdates);
setpoints.velocity = zeros(3, numUpdates);
setpoints.euler = zeros(3, numUpdates);
setpoints.rotRate = zeros(4, numUpdates);
setpoints.thrust = zeros(1, numUpdates);
wb = waitbar(0, "Terribly Slow Simulation", 'CreateCancelBtn','setappdata(gcbf,''canceling'',1)');
for (LV1 = 1:numUpdates) 

	if getappdata(wb,'canceling')
		break
  end

  % Controller update
  [sp.position, sp.velocity, flightVar] = GetSetpoints(tSim(sIndex), s(:, sIndex), traj, true, true, flightVar);
  [rotRate, sp, pid] = ctrl.update(sNoisy(:, sIndex), sp, const, dt_flightControl);

  % Save the setpoints for plotting later
	pidOutput(:, LV1) = pid;

  setpoints.position(:, LV1) = sp.position;
  setpoints.velocity(:, LV1) = sp.velocity;
  setpoints.euler(:, LV1) = [sp.roll, sp.pitch, sp.yaw]';
  setpoints.rotRate(:, LV1) = rotRate;
  setpoints.thrust(:, LV1) = sp.thrust;
  tFc(LV1) = tSim(sIndex);

  for (LV2 = 1:numSnapshots)
    [t_temp, s_temp] = ode45(@(t, s)ODEs(t, s, rotRate, const), [0, dt_sim], s(:, sIndex), options);
    s(:, sIndex + 1) = s_temp(end, :);
    posNoisy = positionNoise(s(1:3, sIndex + 1));
    velNoisy = velocityNoise(s(8:10, sIndex + 1));
    qNoisy = quaternionNoise(s(4:7, sIndex + 1));
    wNoisy = gyroNoise(s(11:13, sIndex + 1));
    % sNoisy(:, sIndex + 1) = [posNoisy; qNoisy; velNoisy; wNoisy];
    sNoisy(:, sIndex + 1) = s(1:13, sIndex + 1);
    sIndex = sIndex + 1;
	end

	waitbar(LV1/numUpdates, wb)
end
toc
delete(wb);

disp("Post processing...")
postprocessing;

% profile off
% profview

disp("Plotting...")
plotscript;

save simData;