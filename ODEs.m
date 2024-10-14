function  [dot_x] = ODEs(t, s, u, const, dist_uvw)
  % DESCRIPTION:
  %   Computes the state vector derivative at time t. 
  % USAGE:
  %   [dot_x] = ODEs(t, x, const)
  % INPUT:
  %   t                 time
  %   x        (21, 1)  state vector (described below)
  %   u         (4, 1)  rotation rate setpoint for each motor [rad/s]
  %   const             structure containing physical constants
  %   dist_uvw  (3, 1)  disturbances to add to velocity vector [m/s]
  % OUTPUT:
  %   dot_x    (21, 1)  state vector derivative
  % NOTES:
  %   The state vector is:
  %     [x y z q0 q1 q2 q3 xdot ydot zdot p q r wm1 dot_wm1 wm2 dot_wm2 wm3
  %     dot_wm3 wm4 dot_wm4]'
  %   The first 13 states relate to the quadcopter's rigid body dynamics. The
  %   last 8 relate to the motor dynamics
  
  % Written by: James Johnson
  % Written: March 2024
  % Last updated: October 2024
  % Based on the quadcopter sim developed by John Bass :
  %   github.com/bobzwik/Quadcopter_SimCon


  % Extract from const
	tauM = const.tauM;
	minW = const.minW;
	maxW = const.maxW;

  % Normalize quaternion and calculate DCM
	s(4:7) = s(4:7)/norm(s(4:7));
  C_ba = Quaternion2DCM(s(4:7));

  % Motor dynamics
  wM = s(14:17);
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

  % Calculate thrust and torques based on motor rotation rates
  motorAllocations = const.mixerCoefficients*wM.^2;
  
  % Save for later
  % w_ba_x = crossm(s(11:13));
  w_ba_x = [0, -s(13), s(12);
            s(13), 0, -s(11);
            -s(12), s(11), 0];
  wM_gyr = wM(1) - wM(2) + wM(3) - wM(4);
  constRotDist = 1e-2*[0; 0; 1];
  
  % Position derivative
  X_dot = s(8:10) + dist_uvw;
  % Quaternion attitude derivative
  Q_dot = GammaQuaternion(s(4:7))*s(11:13);
  % Velocity derivative
  V_dot = -C_ba'*[0; 0; motorAllocations(1)]/const.mB + [0; 0; const.g];
  % Body rate derivative
  W_dot = const.Ib_inv*(motorAllocations(2:4) + constRotDist + const.Irzz*w_ba_x*[0; 0; wM_gyr] - w_ba_x*const.Ib*s(11:13));
  
	
  % velW = 0;
  % qW1 = 0;
  % qW2 = 0;
  % Rigid body dynamics
  % dot_x_quad =  [xdot;
  %                ydot;
  %                zdot;
  %                -0.5*p*q1 - 0.5*q*q2 - 0.5*q3*r;
  %                0.5*p*q0 - 0.5*q*q3 + 0.5*q2*r;
  %                0.5*p*q3 + 0.5*q*q0 - 0.5*q1*r;
  %                -0.5*p*q2 + 0.5*q*q1 + 0.5*q0*r;
  %                (Cd*sign(velW*cos(qW1)*cos(qW2) - xdot)*(velW*cos(qW1)*cos(qW2) - xdot)^2 - 2*(q0*q2 + q1*q3)*(thrM1 + thrM2 + thrM3 + thrM4))/mB + wind_u;
  %                (Cd*sign(velW*sin(qW1)*cos(qW2) - ydot)*(velW*sin(qW1)*cos(qW2) - ydot)^2 + 2*(q0*q1 - q2*q3)*(thrM1 + thrM2 + thrM3 + thrM4))/mB + wind_v;
  %                (-Cd*sign(velW*sin(qW2) + zdot)*(velW*sin(qW2) + zdot)^2 + -(thrM1 + thrM2 + thrM3 + thrM4)*(q0^2 - q1^2 - q2^2 + q3^2))/mB + g + wind_w;
  %                (IByy*q*r - IBzz*q*r - IRzz*(wM(1) - wM(2) + wM(3) - wM(4))*q + thrM1*dymf - thrM2*dymf - thrM3*dymb + thrM4*dymb)/IBxx;
  %                (-IBxx*p*r + IBzz*p*r + IRzz*(wM(1) - wM(2) + wM(3) - wM(4))*p + thrM1*dxmf + thrM2*dxmf - thrM3*dxmb - thrM4*dxmb)/IByy;
  %                (IBxx*p*q - IByy*p*q - TorM1 + TorM2 - TorM3 + TorM4)/IBzz];

	dot_x = [X_dot; Q_dot; V_dot; W_dot; wdotM];


end
