classdef controller < handle
  properties
    integralPrev_attitude = [0; 0; 0];
    errorPrev_attitude = [0; 0; 0];
    C_ba;
    integralPrev_position = [0; 0; 0];

    alpha = 1 / (1 + 200/(2*pi*10000));
    filterprev = [0; 0; 0];

    time = 0;
  end

  methods
		function [motorCommands, ref, PIDOutput] = update(obj, s, ref, const, Ts)
      % Update the DCM
      obj.C_ba = Quaternion2DCM(s(4:7));

      % Calculate desired roll, pitch, and thrust
      [ref.roll, ref.pitch, ref.thrust] = positionPID(obj, s, [ref.position; ref.velocity], const, Ts);
      % Perform the attitude PID. Passing in desired yaw rate of 0 for now
      ref.yaw = pi;
      [PIDOutput] = attitudePID(obj, s, [ref.roll; ref.pitch; ref.yaw], const, Ts);

			% b1d = [1; 0; 0];
			% [ref.C_ba, ref.thrust] = positionPID2(obj, s, ref.position, ref.velocity, const, Ts, b1d);
			% ref.C_ba = Euler3212DCM([0; 60*pi/180; 0]);
			% PIDOutput = attitudePID2(obj, s, ref.C_ba, const, Ts);
			% rpy = DCM2Euler321(ref.C_ba);
			% ref.roll = rpy(1);
			% ref.pitch = rpy(2);
			% ref.yaw = rpy(3);

			% Map inputs to motors
			motorCommands = ControlAllocator(obj, [ref.thrust; PIDOutput], const);
    end
		function [PIDOutput] = attitudePID(obj, s, attDes, const, Ts)
      % Note that this controls the rate for yaw and angle for pitch/roll
      attEuler = DCM2Euler321(obj.C_ba);
      error = attDes - attEuler;
      w = s(11:13);
      % error(3) = attDes(3) - w(3);

      % Handle wrapping
      if error(3) <= -pi
        error(3) = error(3) + 2*pi;
      elseif error(3) > pi
        error(3) = error(3) - 2*pi;
      end

      derivative(1:2, 1) = -w(1:2);  % Approximation of the derivative term for pitch and roll. Assumes small changes in reference and prevents kick.
      % derivative(3, 1) = (error(3) - obj.errorPrev_attitude(3))/Ts;  % Simple approximation of yaw rate derivative
      derivative(3, 1) = -w(3);
      integral = obj.integralPrev_attitude + error*Ts;
      % Prevent integral buildup
      integral = constrain(integral, -const.iMax_att, const.iMax_att);
      Kp = const.Kp_att; Ki = const.Ki_att; Kd = const.Kd_att;
      PIDOutput = Kp*error + Kd*derivative + Ki*integral;
      % Store the error and integral terms from the current time step
      obj.integralPrev_attitude = integral;
      obj.errorPrev_attitude = error;
    end

		function [PIDOutput] = attitudePID2(obj, s, Cba_d, const, Ts)
      % Attitude error vector
			Cba = obj.C_ba;
			e_R = 1/2*uncross(Cba_d*Cba' - Cba*Cba_d');
			% Angular velocity error vector (just the angular velocity)
			e_w = s(11:13);
			% Integral term (euler integration)
			e_I = obj.integralPrev_attitude + (e_w + const.c2*e_R)*Ts;
			% Control torque
			PIDOutput = -const.k_R*e_R - const.k_w*e_w - const.k_I*e_I;
			obj.integralPrev_attitude = e_I;
    end

    function [desRoll, desPitch, desThrust] = positionPID(obj, s, sDes, const, Ts)
      error = sDes(1:3) - s(1:3);  % The position error [m]
      integral = obj.integralPrev_position + error*Ts;
      % Prevent integral buildup
      integral = constrain(integral, -const.iMax_pos, const.iMax_pos);
      % The derivative term used is the error between the desired velocity and
      % actual velocity
      derivative = sDes(4:6) - s(8:10);
      Kp = const.Kp_pos; Ki = const.Ki_pos; Kd = const.Kd_pos;
      % The output of the PID controller is the desired acceleration in the
      % local NED frame
      % derivTerm = obj.filterprev*(1 - obj.alpha) + Kd*derivative*obj.alpha;
      % obj.filterprev = derivTerm;
			derivTerm = Kd*derivative;
      desAcc_n = Kp*error + derivTerm + Ki*integral;

      % Now we decouple this acceleration and calculate the projection of the n3
      % component onto the b3 axis
      attEuler = DCM2Euler321(obj.C_ba);
      Sy = sin(attEuler(3));
      Cr = cos(attEuler(1));
      Cp = cos(attEuler(2));
      Cy = cos(attEuler(3));
      desAcc_b3 = (desAcc_n(3) - 9.81)/Cr/Cp;
      desThrust = -desAcc_b3 * const.mB_ctrl;
      desThrust = constrain(desThrust, const.minThrust, const.maxThrust);
      desAcc_b3 = -desThrust/const.mB_ctrl;  % Constrained b3 acceleration

      % Using the available thrust and the desired acceleration in the NED
      % frame, we can calculate desired roll and pitch
      if desThrust > const.minThrust
        maxAngle_sinArg = const.maxPitchRoll;
        sinRoll = (Sy * desAcc_n(1) - Cy * desAcc_n(2)) / desAcc_b3;
        sinRoll = constrain(sinRoll, -maxAngle_sinArg, maxAngle_sinArg);
        desRoll = asin(sinRoll);
    
        sinPitch = (Cy * desAcc_n(1) + Sy * desAcc_n(2)) / desAcc_b3 / ...
                   sqrt(1 - sinRoll * sinRoll);
        sinPitch = constrain(sinPitch, -maxAngle_sinArg, maxAngle_sinArg);
        desPitch = asin(sinPitch);
      else
        % If the thrust is small, please don't try to use it.
        desRoll = 0.0;
        desPitch = 0.0;
      end
      % Store the integral term from the current time step
      obj.integralPrev_position = integral;
    end

		function [Cba_d, f] = positionPID2(obj, s, xd, vd, const, Ts, b1d)
			k_x = const.k_x;
			k_v = const.k_v;
			k_i = const.k_i;
			m = const.mB_ctrl;
			g = const.g;
			% Position and velocity errors
			e_x = s(1:3) - xd;
      % This is a quick fix for constraining velocity/attitude. Not a good fix.
      e_x = constrain(e_x, -2, 2);
			e_v = s(8:10) - vd;
			% Integral term (euler integration)
			e_i = obj.integralPrev_position + (e_v + const.c1*e_x)*Ts;
			e_i_sat = min(max(e_i, -const.iMax_pos), const.iMax_pos);
			% Computed b3 axis direction
			b3c = -k_x*e_x - k_v*e_v - k_i*e_i_sat - m*g*[0;0;1];
			b3c = -b3c/norm(b3c);
			% b1c is the projection of b1d on the plane orthogonal to b3c.
      b1c = - 1/norm(crossm(b3c)*b1d)*crossm(b3c)*crossm(b3c)*b1d;
			Cba_d = [b1c crossm(b3c)*b1c b3c]';
			f = (k_x*e_x + k_v*e_v + k_i*e_i_sat+ m*g*[0;0;1])'*obj.C_ba'*[0;0;1];
			obj.integralPrev_position = e_i_sat;
    end
    
		function angularRates = ControlAllocator(obj, u, const)
			angularRates = real(sqrt(const.mixerCoefficientsInv*u));
		end
  end
end
