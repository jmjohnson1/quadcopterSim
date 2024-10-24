%%%%%%%%%%%%
%  Options %
%%%%%%%%%%%%
lw = 2.0;  % linewidth
defaultLS = "-";
setpointLS = "--";
estimateLS = "-";
colors = [0.1974    0.5129    0.7403;
          0.5978    0.8408    0.6445;
          0.9438    0.3910    0.2668];
defaultColor = colors(1, :);
setpointColor = colors(2, :);
estimateColor = colors(3, :);

%%%%%%%%%%%%
% Position
if opt.plotPosition == true
	figure(Name="Position", WindowStyle='docked')
	labels = ["x (m)", "y (m)", "z (m)"];
	for i = 1:3
		axPos(i) = subplot(3, 1, i);
			plot(tSim, position(i, :), ... 
						Color=defaultColor, ...
						DisplayName="Truth", ...
						LineWidth=lw, ...
						LineStyle=defaultLS);
			hold on;
			plot(tFc, setpoints.position(i, :), ...
						Color=setpointColor, ...
						DisplayName="Setpoint", ...
						LineWidth=lw, ...
						LineStyle=setpointLS);
			if opt.useEKF == true
				plot(tSim, sEstimate(i, :), ...
							Color=estimateColor, ...
							DisplayName="Estimate", ...
							LineWidth=lw, ...
							LineStyle=estimateLS);
			end
			hold off;
			ylabel(labels(i))
	end
	xlabel("Time (s)");
	title(axPos(1), "Quadcopter Position in Local NED Frame")
	grid(axPos, 'on');
	legend(axPos(1));
end

% Velocity
if opt.plotVelocity == true
	figure(Name="Velocity", WindowStyle='docked');
	labels = ["V_x (m/s)", "V_y (m/s)", "V_z (m/s)"];
	for i = 1:3
	axVel(i) = subplot(3, 1, i);
		plot(tSim, s(i+7, :), ...
						Color=defaultColor, ...
						DisplayName="Truth", ...
						LineWidth=lw, ...
						LineStyle=defaultLS);
		hold on
		plot(tFc, setpoints.velocity(i, :), ...
						Color=setpointColor, ...
						DisplayName="Setpoint", ...
						LineWidth=lw, ...
						LineStyle=setpointLS);
		if opt.useEKF == true
			plot(tSim, sEstimate(i+7, :), ...
							Color=estimateColor, ...
							DisplayName="Estimate", ...
							LineWidth=lw, ...
							LineStyle=estimateLS);
		end
		hold off;
		ylabel(labels(i));
		grid on
	end
	xlabel("Time (s)");
	legend(axVel(1));
end

% Attitude
if opt.plotAttitude == true
	figure(Name="Attitude", WindowStyle='docked')
	labels = ["Roll (deg)", "Pitch (deg)", "Yaw (deg)"];
	for i = 1:3
		axAtt(i) = subplot(3, 1, i);
		plot(tSim, eulerAngles(i, :), ...
					Color=defaultColor, ...
					DisplayName="Truth", ...
					LineWidth=lw, ...
					LineStyle=defaultLS);
		hold on; 
		if i ~= 3
		plot(tFc, setpoints.euler(i, :), ...
					Color=setpointColor, ...
					DisplayName="Setpoint", ...
					LineWidth=lw, ...
					LineStyle=setpointLS);
		end
		if opt.useEKF == true
			plot(tSim, eulerAnglesEst(i, :), ...
						Color=estimateColor, ...
						DisplayName="Estimate", ...
						LineWidth=lw, ...
						LineStyle=estimateLS);
		end
		hold off;
		ylabel(labels(i))
	end
	xlabel("Time (s)");
	title(axAtt(1), "Attitude")
	grid(axAtt, 'on');
	legend(axAtt(1));
end

% Angular rates
if opt.plotBodyRates == true
	figure(Name="Angular Rates", WindowStyle='docked');
	labels = {'P (deg/s)', 'Q (deg/s)', 'R (deg/s)'};
	for i = 1:3
	axGyro(i) = subplot(3, 1, i);
	plot(tSim, s(10+i, :)*180/pi, ...
					Color=defaultColor, ...
					DisplayName="Truth", ...
					LineWidth=lw, ...
					LineStyle=defaultLS);
	if i == 3
		hold on;
		plot(tFc, setpoints.euler(3, :), ...
					Color=setpointColor, ...
					DisplayName="Setpoint", ...
					LineWidth=lw, ...
					LineStyle=setpointLS);
		hold off;
	end
	ylabel(labels(i));
	end
	xlabel("Time (s)");
	legend(axGyro(3));
end


% Motor angular rates
if opt.plotMotorRates == true
	figure(Name="Motors", WindowStyle='docked');
	for i = 1:4
		axMot(i) = subplot(2, 2, i);
		plot(tFc, setpoints.rotRate(i, :), ...
						Color=setpointColor, ...
						DisplayName="Setpoint", ...
						LineWidth=lw, ...
						LineStyle=setpointLS);
		hold on;
		plot(tSim, motorRotRate(i, :), ...
						Color=defaultColor, ...
						DisplayName="Truth", ...
						LineWidth=lw, ...
						LineStyle=defaultLS);
		hold off;
		ylabel(sprintf("M%d \\omega (rad/s)", i));
	end
	xlabel("Time (s)");
	grid(axMot, 'on');
	legend(axMot(2));
end

% Thrust setpoints
if opt.plotThrust == true
	figure(Name="thrust", WindowStyle='docked');
	plot(tSim, thrust, ...
						Color=defaultColor, ...
						DisplayName="Truth", ...
						LineWidth=lw, ...
						LineStyle=defaultLS);
	hold on
	plot(tFc, setpoints.thrust, ...
						Color=setpointColor, ...
						DisplayName="Setpoint", ...
						LineWidth=lw, ...
						LineStyle=setpointLS);
	yline(maxAllowableThrust, ...
				Color=estimateColor, ...
				DisplayName="Max Thrust Allowed", ...
				LineWidth=lw, ...
				LineStyle=setpointLS);
	hold off
	title("Total Thrust");
	ylabel("Thrust (N)");
	xlabel("Time (s)");
	legend();
end

% 3D Position
if opt.plot3dTraj == true
	% Room
	xLimits = [-6.2197, 2.165];
	yLimits = [-0.8, 2.6975];
	zLimits = [-2.5, 0.2];
	f = figure(Name="3D Trajectory", WindowStyle='docked');

	ax = gca;
	plot3(traj(1, :), traj(2, :), traj(3, :), ...
						Color=setpointColor, ...
						DisplayName="Setpoint", ...
						LineWidth=lw, ...
						LineStyle=setpointLS);
	hold on
	% scatter3(waypoints(1, :), waypoints(2, :), waypoints(3, :), 30, 'r', 'filled', ...
						% DisplayName="RequiredWaypoints");
	plot3(position(1, :), position(2, :), position(3, :), ...
						Color=defaultColor, ...
						DisplayName="Truth", ...
						LineWidth=lw, ...
						LineStyle=defaultLS);
	hold off
	set(ax, "XLim", xLimits, "YLim", yLimits, "ZLim", zLimits);
	ax.Clipping = "off";
	f.CurrentAxes.ZDir = 'Reverse';
	f.CurrentAxes.YDir = 'Reverse';
	axis equal;
	xlabel("x (m)");
	ylabel("y (m)");
	zlabel("z (m)");
	legend();
	clear f;
end


% Additional plots for when EKF is enabled
if (opt.useEKF == true) && (opt.makeEKFPlots == true)
	% Plot accelerometer measurements
	figure(Name="Accelerometer", WindowStyle='docked')
	for i = 1:3
		s(i) = subplot(3, 1, i);
		plot(tSim, accelTruth(i, :), LineWidth=lw, DisplayName="Truth", LineStyle=defaultLS, Color=defaultColor);
		hold on
		plot(tSim, accelMeas(i, :), LineStyle="none", Marker=".", MarkerSize=12, Color=estimateColor, DisplayName="Measured")
		hold off
		ylabel(['a_', num2str(i), '(m/s^2)']);
	end
	xlabel(s(3), 'time (s)')
	legend(s(1))

	% Plot gyro measurements
	figure(Name="Gyro", WindowStyle='docked')
	for i = 1:3
		s(i) = subplot(3, 1, i);
		plot(tSim, gyroTruth(i, :), LineWidth=lw, DisplayName="Truth", LineStyle=defaultLS, Color=defaultColor);
		hold on
		plot(tSim, gyroMeas(i, :), LineStyle="none", Marker=".", MarkerSize=12, Color=estimateColor, DisplayName="Measured")
		hold off
		ylabel(['w_', num2str(i), '(rad/s)']);
	end
	xlabel(s(3), 'time (s)')
	legend(s(1))

	% Plot accelerometer bias	
	figure(Name="Accelerometer bias", WindowStyle='docked')
	for i = 1:3
		s(i) = subplot(3, 1, i);
		plot(tSim, accelBiasTruth(i, :), LineWidth=lw, DisplayName="Truth", LineStyle=defaultLS, Color=defaultColor);
		hold on
		plot(tSim, accelBiasEstimate(i, :), LineStyle=estimateLS, LineWidth=lw, Color=estimateColor, DisplayName="Estimate")
		hold off
		ylabel(['b_{a', num2str(i), '} (m/s^2)']);
	end
	xlabel(s(3), 'time (s)')
	legend(s(1))

	% Plot gyro bias	
	figure(Name="Gyro bias", WindowStyle='docked')
	for i = 1:3
		s(i) = subplot(3, 1, i);
		plot(tSim, gyroBiasTruth(i, :), LineWidth=lw, DisplayName="Truth", LineStyle=defaultLS, Color=defaultColor);
		hold on
		plot(tSim, gyroBiasEstimate(i, :), LineStyle=estimateLS, LineWidth=lw, Color=estimateColor, DisplayName="Estimate")
		hold off
		ylabel(['b_{g', num2str(i), '} (m/s^2)']);
	end
	xlabel(s(3), 'time (s)')
	legend(s(1))
end
