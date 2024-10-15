clear; close all; clc;
% Add the helper functions to the Matlab search path
addpath('./helperFunctions/');
addpath('./EKF/');

%%%%%%%%%%%
% Options %
%%%%%%%%%%%
opt.makeAnimation = true;
opt.useEKF = true;
opt.makePlots = true;
opt.debugEKFFiles = false;
opt.makeEKFPlots = false;
opt.plotVelocity = true;
opt.plotPosition = true;
opt.plotAttitude = true;
opt.plotMotorRates = true;
opt.plotBodyRates = true;
opt.plotThrust = true;
opt.plot3dTraj = true;


% Time options
tStart = 0;  % Simulation end time [sec]
tEnd = 45;  % Simulation start time [sec]
looprateFC = 200;  % Flight controller loop rate [Hz]
numSnapshots = 1;  % Number of states to save between each FC update

% Define how often measurements are passed into the EKF
measUpdateRate = 5 ;  % Hz


%%%%%%%%%%%%%%%%%%%%%
% Trajectory Import %
%%%%%%%%%%%%%%%%%%%%%
% Run a trajectory generation function. It must return a trajectory with this format:
%   [x0 x1 ... xn 
%    y0 y1 ... yn
%    z0 z1 ... zn
%    u0 u1 ... un
%    v0 v1 ... vn
%    w0 w1 ... wn
%    t0 t1 ... tn]
% (x, y, z) are the waypoint position in the local North-East-Down frame [m].
% (u, v, w) are the velocities to target while passing through the waypoint (NED) [m/s].
% t is the time when the quad should be passing through the waypoint [seconds]

  % Trajectory type
  % trajType = 'testAttitudeStabilize';
  % trajType = 'testStepX';
  % trajType = 'testStepY';
  % trajType = 'testStepZ';
  % trajType = 'testStep';
  % waypoints = [];

% trajType = 'minsnap';
% t_wp = linspace(0, 20, 4);
% waypoints = [0.00,  0.00,  -0.40;
%              0.00,  1.00,  -0.75;
%              1.00,  1.00,  -1.50;
%              0.00,  0.00,  -0.75]';
% waypoints = [waypoints; t_wp];
% traj = GenerateTrajectory(waypoints, trajType, false); % Import trajectory
stepZ = [-0.5 -1.0 -1.5 -1.0 -0.5];
tZ = [0 6 12 18 24];
traj = MakeStep(stepZ, tZ);

try
  constants;
  minW = const.minW;

  %%%%%%%%%%%%%%%%%%%%%%%%%%%
  % Flight status variables %
  %%%%%%%%%%%%%%%%%%%%%%%%%%%
  flightVar.mode = 'takeoff';
  flightVar.takeoffFlag = false;
  flightVar.landingFlag = false;
  flightVar.waypointArrived = false;
  flightVar.waypointArrivedTime = [];
  flightVar.okToStartMission = false;
  flightVar.missionStartTime = [];


  %%%%%%%%%%%%%%%%%%%%%%%%
  %  Initial conditions  %
  %%%%%%%%%%%%%%%%%%%%%%%%
  initPosition = [0, 0, 0]';  % initial position in local NED frame [m]
  initVelocity = [0, 0, 0]';  % initial velocity in local NED frame [m/s]
  initAttitude = [2, -6, 145]'*deg2rad;  % initial roll, pitch, yaw [rad]
  initRates = [0.1, -0.2, 0]';  % Initial angular rates in body frame [rad/s]

  % Initial motor rates
  hoverThrust = const.mB_ctrl*9.81/4*[1, 1, 1, 1]';
  initW = sqrt(hoverThrust./const.kt);  % Initial motor angular velocities [rad/s]
  initWdot = [0, 0, 0, 0]';  % Initial motor angular acceleration [rad/s^2]

  % Initial DCM and quaternion
  initC_bn = Euler3212DCM(initAttitude);
  initq_bn = DCM2Quaternion(initC_bn);  % Scalar first

  %%%%%%%%%%%%%%%%%
  % Sim variables %
  %%%%%%%%%%%%%%%%%
  % Figure out time steps and the sizes of matrices that need to be initialized
  dt_flightControl = 1/looprateFC;  % Flight controller time step [sec]
  numUpdates = floor((tEnd - tStart)/dt_flightControl) + 1;
  dt_sim = dt_flightControl/numSnapshots;  % Saved states time step [sec]
  numPoints = (numUpdates)*numSnapshots + 1;
  tSim = linspace(tStart, tEnd, numPoints);

  % Initialize accelerometer and gyro objects
  gyroTriad = memsIMU(const.tau_g, const.sigma_gyro, const.sigma_gyro_gm);
  accelTriad = memsIMU(const.tau_a, const.sigma_acc, const.sigma_acc_gm);
	accelTruth = zeros(3, numPoints);
	gyroTruth = zeros(3, numPoints);
	accelMeas = zeros(3, numPoints);
	gyroMeas = zeros(3, numPoints);
	accelBiasEstimate = zeros(3, numPoints);
	gyroBiasEstimate = zeros(3, numPoints);
	accelBiasTruth = zeros(3, numPoints);
	gyroBiasTruth = zeros(3, numPoints);

  ctrl = controller; % Create controller object
  options = odeset('AbsTol',1e-8,'RelTol',1e-8); % Set integration tolerences

  % Initial state vector and memory allocation
  s0 = [initPosition; initq_bn; initVelocity; initRates; initW];
  s = zeros(length(s0), numPoints);
  s(:, 1) = s0;

  % State estimates from the EKF
  ekfState = [initPosition; initVelocity; zeros(9, 1)];
  sEstimate = zeros(13, numPoints);
  sEstimate(:, 1) = s0(1:13);
  measUpdatePrev = 0;

  % This will track where to access and save states in the sim
  sIndex = 1;  

  % Will be useful to have the times that correspond to flight controller updates
  tFc = zeros(1, numUpdates);  

  % For recording all of the setpoints that are generated/passed into the controllers
  setpoints.position = zeros(3, numUpdates);
  setpoints.velocity = zeros(3, numUpdates);
  setpoints.euler = zeros(3, numUpdates);
  setpoints.rotRate = zeros(4, numUpdates);
  setpoints.thrust = zeros(1, numUpdates);

  % Generate wind for full sim
  dist_u = wind(tSim, const.da_u, const.db_u, const.velDistMax);
  dist_v = wind(tSim, const.da_v, const.db_v, const.velDistMax);
  dist_w = wind(tSim, const.da_w, const.db_w, const.velDistMax);
  dist_uvw = [dist_u; dist_v; dist_w];

  % DEBUG: To pass into the C++ version of the EKF
  if opt.debugEKFFiles == true
    outputData = zeros(numPoints, 17);
    updateCounter = 0;
  end

  disp("Simulation starting...");
  tic;
  %%%%%%%%%%%%%%%%
  %  Simulation  %
  %%%%%%%%%%%%%%%%

  % Waitbar for displaying simulation progress. There are occasions where this window will stay open
  % after the program fails to exit cleanly. It can only be closed using this command:
  %   delete(findall(groot, 'type', 'figure'))
  wb = waitbar(0, "Running...", 'CreateCancelBtn','setappdata(gcbf,''canceling'',1)');

  % Run the sim
  for LV1 = 1:numUpdates 

    if getappdata(wb,'canceling')
      break
    end

    % Controller update
    [sp.position, sp.velocity, flightVar] = GetSetpoints(tSim(sIndex), sEstimate(:, sIndex), traj, true, true, flightVar);
    % [rotRate, sp, pid] = ctrl.update(sEstimate(:, sIndex), sp, const, dt_flightControl);
    [rotRate, sp, pid] = ctrl.update(sEstimate(:, sIndex), sp, const, dt_sim);

    % Save the setpoints for plotting later
    setpoints.position(:, LV1) = sp.position;
    setpoints.velocity(:, LV1) = sp.velocity;
    setpoints.euler(:, LV1) = [sp.roll, sp.pitch, sp.yaw]';
    setpoints.rotRate(:, LV1) = rotRate;
    setpoints.thrust(:, LV1) = sp.thrust;
    tFc(LV1) = tSim(sIndex);

    % This loop runs the simulation and saves points between flight controller updates.
    for LV2 = 1:numSnapshots
      
      [t_temp, s_temp] = ode45(@(t, s)ODEs(t, s, rotRate, const, dist_uvw(:, sIndex)), [0, dt_sim], s(:, sIndex), options);
      s(:, sIndex + 1) = s_temp(end, :);
      
      if opt.useEKF == true
        % For generating sensor measurements, calculate the state derivative at the current time
        ds = ODEs(0, s(:, sIndex), rotRate, const, dist_uvw(:, sIndex));
        % ds_ = ODEs(0, s(:, sIndex+1), rotRate, const, windVel(:, sIndex+1));
        % Take the acceleration, put it in the body frame, add gravity, add noise
        accelTruth(:, sIndex) = Quaternion2DCM(s(4:7, sIndex))*(ds(8:10) - [0;0;const.g]);
        % accelMeas(:, sIndex) = accelTruth(:, sIndex);
        accelMeas(:, sIndex) = accelTriad.GetMeasurement(accelTruth(:, sIndex), dt_sim) + const.ba0;
				accelBiasTruth(:, sIndex) = accelTriad.inRunBias + const.ba0;
        % Take the body rotation rates, add noise
        gyroTruth(:, sIndex) = s(11:13, sIndex);
        % gyroMeas(:, sIndex) = gyroTruth(:, sIndex);
        gyroMeas(:, sIndex) = gyroTriad.GetMeasurement(gyroTruth(:, sIndex), dt_sim) + const.bg0;
	      gyroBiasTruth(:, sIndex) = gyroTriad.inRunBias + const.bg0;

        % Determine availability of position measurement
        if tSim(sIndex+1) - measUpdatePrev > 1/measUpdateRate
          Y = s(1:3, sIndex+1) + const.sigma_pos.*randn(3, 1);
          measUpdatePrev = tSim(sIndex+1);

          % DEBUG: To pass into the C++ version of the EKF
          % tow = tow + 1;
          % outputData(sIndex, :) = [tSim(sIndex)*1e6, [accelMeas', gyroMeas'], Y', tow, s(1:3, sIndex+1)', DCM2Euler321(Quaternion2DCM(s(4:7, sIndex+1)))'];
        else
          Y = [];
          % DEBUG: To pass into the C++ version of the EKF
          % outputData(sIndex, :) = [tSim(sIndex)*1e6, [accelMeas', gyroMeas'], [0, 0, 0], tow, s(1:3, sIndex+1)', DCM2Euler321(Quaternion2DCM(s(4:7, sIndex+1)))'];
        end
        
        % Run the EKF to get an estimated state
        [ekfState, P, qEst] = EKF(ekfState, Y, [accelMeas(:, sIndex); gyroMeas(:, sIndex)], P, const, dt_sim, sEstimate(4:7, sIndex));
				accelBiasEstimate(:, sIndex + 1) = ekfState(10:12);
				gyroBiasEstimate(:, sIndex + 1) = ekfState(13:15);

        sEstimate(:, sIndex + 1) = [ekfState(1:3); qEst; ekfState(4:6); gyroMeas(:, sIndex)];
      else
        sEstimate(:, sIndex + 1) = s(1:13, sIndex + 1);
      end
      sIndex = sIndex + 1;
    end

    waitbar(LV1/numUpdates, wb)
  end
  toc
  delete(wb);

  % DEBUG: To pass into the C++ version of the EKF
  % writematrix(outputData, "flightData.csv");
  % mocapPosition_i = s(1:3, :)';
  % q_i = s(4:7, :)';
  % time = tSim;
  % save("mocapData", "mocapPosition_i", "q_i", "time");

  disp("Post processing...")
  postprocessing;

  if opt.makePlots == true
    disp("Plotting...")
    plotscript;
  end
  
  if displayThrustViolation == true
    rotRateMessage = msgbox("Maximum thrust input exceeded! Adjust your gains!");
		set(rotRateMessage, 'WindowStyle', 'modal');
  end
  if displayPositionViolation == true
    positionMessage = msgbox("Maximum horizontal error exceeded (probably due to aggressive gains)! Adjust your gains!");
		set(positionMessage, 'WindowStyle', 'modal');
  end
  if opt.makeAnimation == true
    animateQuad(s(1:3, :), s(4:7, :), tSim, [stepZ*0; stepZ*0; stepZ; tZ], traj, 10);
  end
  save simData;

catch ME
  if exist('wb')
    delete(wb)
  end
  rethrow(ME)
end
