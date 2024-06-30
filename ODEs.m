function  [dot_x] = ODEs(t, s, u, const)
  % DESCRIPTION:
  %   Computes the state vector derivative at time t. 
  % USAGE:
  %   [dot_x] = ODEs(t, x, const)
  % INPUT:
  %   t             time
  %   x   (21, 1)   state vector (described below)
  %   u   (4, 1)    rotation rate setpoint for each motor [rad/s]
  %   const   structure containing physical constants
  % OUTPUT:
  %   dot_x   state vector derivative
  % NOTES:
  %   The state vector is:
  %     [x y z q0 q1 q2 q3 xdot ydot zdot p q r wm1 dot_wm1 wm2 dot_wm2 wm3
  %     dot_wm3 wm4 dot_wm4]'
  %   The first 13 states relate to the quadcopter's rigid body dynamics. The
  %   last 8 relate to the motor dynamics
  
  % Written by: James Johnson
  % Written: March 2024
  % Last updated: April 2024
  % Based on the quadcopter sim developed by John Bass :
  %   github.com/bobzwik/Quadcopter_SimCon

  % Extract from const
  Cd = const.Cd;
  IBxx = const.Ib(1, 1);
  IByy = const.Ib(2, 2);
  IBzz = const.Ib(3, 3);
  IRzz = const.Irzz;
  dxmf = const.dxmf;
  dymf = const.dymf;
  dxmb = const.dxmb;
  dymb = const.dymb;
	kt = const.kt;
	km = const.km;
	tauM = const.tauM;
	minW = const.minW;
	maxW = const.maxW;
  mB = const.mB;
  g = const.g;

  % Extract from state vector
  x = s(1);
  y = s(2);
  z = s(3);
  q0 = s(4);
  q1 = s(5);
  q2 = s(6);
  q3 = s(7);
  xdot = s(8);
  ydot = s(9);
  zdot = s(10);
  p = s(11);
  q = s(12);
  r = s(13);

	quat = [q0; q1; q2; q3];
	quat = quat/norm(quat);
	q0 = quat(1);
	q1 = quat(2);
	q2 = quat(3);
	q3 = quat(4);

  wM = s(14:17);

  % Motor dynamics
	wM = min(max(wM, minW), maxW);
	wdotM = 1/tauM*(u - wM);
  % Should keep the motors spinning within their experimentally determined 
	% ranges
	for i = 1:4
		if any(abs(wM(i) - minW) < 2)
			wdotM(i) = max(wdotM(i), 0);
		end
		if any(abs(wM(i) - maxW) < 2)
			wdotM(i) = min(wdotM(i), 0);
		end
	end
	
  thrust = kt*wM.^2;
  torque = km*wM.^2;

  thrM1 = thrust(1);
  thrM2 = thrust(2);
  thrM3 = thrust(3);
  thrM4 = thrust(4);

  TorM1 = torque(1);
  TorM2 = torque(2);
  TorM3 = torque(3);
  TorM4 = torque(4);

  % TODO: add wind
  velW = 0;
  qW1 = 2*pi*t;  % change this name
  qW2 = pi*t;  % change this name
	

  % Rigid body dynamics
  dot_x_quad =  [xdot;
                 ydot;
                 zdot;
                 -0.5*p*q1 - 0.5*q*q2 - 0.5*q3*r;
                 0.5*p*q0 - 0.5*q*q3 + 0.5*q2*r;
                 0.5*p*q3 + 0.5*q*q0 - 0.5*q1*r;
                 -0.5*p*q2 + 0.5*q*q1 + 0.5*q0*r;
                 (Cd*sign(velW*cos(qW1)*cos(qW2) - xdot)*(velW*cos(qW1)*cos(qW2) - xdot)^2 - 2*(q0*q2 + q1*q3)*(thrM1 + thrM2 + thrM3 + thrM4))/mB;
                 (Cd*sign(velW*sin(qW1)*cos(qW2) - ydot)*(velW*sin(qW1)*cos(qW2) - ydot)^2 + 2*(q0*q1 - q2*q3)*(thrM1 + thrM2 + thrM3 + thrM4))/mB;
                 (-Cd*sign(velW*sin(qW2) + zdot)*(velW*sin(qW2) + zdot)^2 + -(thrM1 + thrM2 + thrM3 + thrM4)*(q0^2 - q1^2 - q2^2 + q3^2))/mB + g;
                 (IByy*q*r - IBzz*q*r - IRzz*(wM(1) - wM(2) + wM(3) - wM(4))*q + thrM1*dymf - thrM2*dymf - thrM3*dymb + thrM4*dymb)/IBxx;
                 (-IBxx*p*r + IBzz*p*r + IRzz*(wM(1) - wM(2) + wM(3) - wM(4))*p + thrM1*dxmf + thrM2*dxmf - thrM3*dxmb - thrM4*dxmb - 0.5)/IByy;
                 (IBxx*p*q - IByy*p*q - TorM1 + TorM2 - TorM3 + TorM4)/IBzz];

	dot_x = [dot_x_quad; wdotM];


end