%%% SET PID GAINS HERE %%%
Kp_z = 0;
Ki_z = 0;
Kd_z = 0;



% Configurations:
bat_3s_3300mAh = false;
bat_4s_5200mAh = true;


rad2deg = 180/pi;
deg2rad = 1/rad2deg;

const.g = 9.81;   % Gravity [m/s^2]

% Motor locations relative to center of mass
% Real
const.dxmf = 0.08665;  % x distance to front motors [m]
const.dymf = 0.13938;  % y distance to front motors [m]
const.dxmb = 0.10345;  % x distance to rear motors [m]
const.dymb = 0.11383;  % y distance to rear motors [m]
% Symmetric
% const.dxmf = 0.095;  % x distance to front motors [m]
% const.dymf = 0.127;  % y distance to front motors [m]
% const.dxmb = 0.095;  % x distance to rear motors [m]
% const.dymb = 0.127;  % y distance to rear motors [m]
const.dzm = 0.021;  % z distance to motors [m]

% const.mB = 0.842;  % Quad mass (3s 3300mAh) [kg]
const.mB = 1.095;  % Quad mass (4s 5200mAh) [kg]
const.mB_ctrl = 0.95;  % Quad mass used in controller [kg]
const.mB_ctrl = 1.095;

% Inertia matrix [kg*m^2]

% TEST STAND (3s 3300 mAh)
% const.Ib = [0.00654, 0, 0;
%             0, 0.00476, 0;
%             0, 0, 0.00945];

if bat_3s_3300mAh == true
const.Ib = [0.0059, 0, 0;
					  0, 0.0042, 0;
						0, 0, 0.0094];
elseif bat_4s_5200mAh == true
const.Ib = [6144384.31, 5120.80, 215538.86;
            5120.80, 4616039.78, -645.56;
            215538.86, -645.56, 9758320.53]*1e-9;
end

const.Ib_inv = inv(const.Ib);
% Rotor moment of inertia about z.
const.Irzz = 1.5e-5;  

% Motor constants
% FIXME: These constants need updating
const.kt = 4.9e-6;  % thrust coefficient [N/(rad/s)^2]
% Approximate moment coefficient [Nm/(rad/s)^2]
const.km = 7.9e-8;
const.minThrust = 1.0;  % Minimum total thrust [N]
const.minW = 277;  % minimum motor rotation speed [rad/s]

if bat_3s_3300mAh == true
	const.maxThrust = 32.20;  % Maximum total thrust [N]
	const.maxW = 1094;  % maximum motor rotation speed [rad/s]
	const.kw = 8.3551e-7;  % normalized command to angular rate: u = kw*w^2
elseif bat_4s_5200mAh == true
	const.maxThrust = 32.20;  % Maximum total thrust [N]
	const.maxW = 1500;  % maximum motor rotation speed [rad/s]
	% w = kw1*u^2 + kw2*u
	const.kw1 = 3.9e-7;
	const.kw2 = 1.2e-4;
end



% Motor dynamics (modeled as 1st order system)
const.tauM = 0.015;  % Time constant [s]

% Control mixer coefficient matrix
kt = const.kt;
km = const.km;
dymf = const.dymf;
dymb = const.dymb;
dxmf = const.dxmf;
dxmb = const.dxmb;
const.mixerCoefficients = [kt, kt, kt, kt;
                     dymf*kt, -dymf*kt, -dymb*kt, dymb*kt;
                     dxmf*kt, dxmf*kt, -dxmb*kt, -dxmb*kt;
                     -km, km, -km, km];
const.mixerCoefficientsInv = inv(const.mixerCoefficients);

clear kt km dymf dymb dxmf dxmb;

% Drag force (Not used, because ...)
const.Cd = 0.05;  % Drag coefficient


% Controller gains
% Attitude
% const.Kp_att = diag([1.66, 1.66, 0.11]);
% const.Ki_att = diag([4.81, 4.81, 0.00]);
% const.Kd_att = diag([0.34, 0.34, 0.00]);
const.Kp_att = diag([1.66, 1.66, 0.11]);
const.Ki_att = diag([4.81, 4.81, 0.0]);
const.Kd_att = diag([0.34, 0.34, 0.1]);

% Position
const.Kp_pos = diag([3.48,  3.48, Kp_z]);
const.Ki_pos = diag([3,  3,  Ki_z]);
const.Kd_pos = diag([3.11,  3.11, Kd_z]);

const.k_x = 13;
const.k_v = 5.5*2;
const.k_i = 1.28;
const.k_R = 1.5;
const.k_w = 0.11;
const.k_I = 0.06;
const.c1 = 3.6;
const.c2 = 0.8;
const.sat = 1;


% Limit on pitch and roll angles
const.maxPitchRoll = 30*deg2rad;

% Limit on the integral term
const.iMax_pos = 1;  % For position controller
const.iMax_att = 50;  % For attitude controller



%-------------------------------------------------------------------------
%  EKF
%-------------------------------------------------------------------------
% Number of states
N = 15;

% Accel/gyro model values
% GM time constant
const.tau_a = [400 400 400]';
const.tau_g = [300 300 300]'; 
% GM sigma
const.sigma_acc_gm = [0.018 0.018 0.018]';
const.sigma_gyro_gm = [4E-4 4E-4 4E-4]';
% White noise
const.sigma_acc = [0.04 0.03 0.05]';
const.sigma_gyro = [0.0015 0.0015 0.0014]';

% Inital conditions
const.sigma_initial_pos = 1*ones(3,1);
const.sigma_initial_vel = 1*ones(3,1);
const.sigma_initial_att = ([pi/12 pi/12 pi/12]');
const.sigma_initial_acc_bias = 0.5*ones(3,1); 
const.sigma_initial_gyro_bias = 0.001*ones(3,1);

% State covariance
P = zeros(N,N);
P(1:3,1:3) = diag( const.sigma_initial_pos.^2 );
P(4:6,4:6) = diag( const.sigma_initial_vel.^2 );
P(7:9,7:9) = diag( const.sigma_initial_att.^2 );
P(10:12,10:12) = diag(const.sigma_initial_acc_bias.^2);
P(13:15,13:15) = diag(const.sigma_initial_gyro_bias.^2);

% Process noise covariance
const.Q = zeros(12,12);
const.Q(1:3,1:3) = diag( const.sigma_acc.^2 );
const.Q(4:6,4:6) = diag( const.sigma_gyro.^2 );
const.Q(7:9,7:9) = diag( 2*const.sigma_acc_gm.^2./const.tau_a );
const.Q(10:12,10:12) = diag( 2*const.sigma_gyro_gm.^2./const.tau_g );

% State to measurement
const.H = zeros(3, N);
const.H(1:3,1:3) = eye(3);

% Measurement noise covariance
const.sigma_pos = [0.05 0.05 0.05]';
const.R = diag( const.sigma_pos.^2 );
