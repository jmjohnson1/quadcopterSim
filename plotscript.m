%%%%%%%%%%%%
%  Options %
%%%%%%%%%%%%
lw = 1.0;  % linewidth
colors = linspecer(12);  % Generates 12 distinguishable colors
%%%%%%%%%%%%
% Position
figure(Name="Position")
axPos(1) = subplot(311);
  plot(tSim, position(1, :));
  hold on;
  plot(tFc, setpoints.position(1, :));
  hold off;
  ylabel("x (m)")
axPos(2) = subplot(312);
  plot(tSim, position(2, :))
  hold on;
  plot(tFc, setpoints.position(2, :));
  hold off;
  ylabel("y (m)")
axPos(3) = subplot(313);
  plot(tSim, position(3, :))
  hold on;
  plot(tFc, setpoints.position(3, :));
  hold off;
  ylabel("z (m)")
xlabel("Time (s)");
title(axPos(1), "Quadcopter Position in Local NED Frame")
grid(axPos, 'on');
legend(axPos(1), ["Simulated", "Setpoint"]);

% Velocity
figure(Name="Velocity");
labels = ["V_x (m/s)", "V_y (m/s)", "V_z (m/s)"];
for i = 1:3
axVel(i) = subplot(3, 1, i);
	plot(tSim, s(i+7, :));
	hold on
	plot(tFc, setpoints.velocity(i, :));
	hold off;
	ylabel(labels(i));
	grid on
end
xlabel("Time (s)");
legend(axVel(1), ["Simulated", "Setpoint"]);

% Attitude
figure(Name="Attitude")
labels = ["Roll (deg)", "Pitch (deg)", "Yaw (deg)"];
for i = 1:3
	axAtt(i) = subplot(3, 1, i);
  	plot(tSim, eulerAngles(i, :));
  	hold on;
		if i ~= 3
			plot(tFc, setpoints.euler(i, :));
		end
  	hold off;
  	ylabel(labels(i))
end
xlabel("Time (s)");
title(axAtt(1), "Attitude")
grid(axAtt, 'on');
legend(axAtt(1), ["Simulated", "Setpoint"]);

% Angular rates
figure(Name="Angular Rates");
labels = {'P (deg/s)', 'Q (deg/s)', 'R (deg/s)'};
for i = 1:3
axGyro(i) = subplot(3, 1, i);
	plot(tSim, s(10+i, :)*180/pi);
	if i == 3
		hold on;
		plot(tFc, setpoints.euler(3, :));
		hold off;
		legend(["Simulated", "Setpoint"]);
	end
	ylabel(labels(i));
end
xlabel("Time (s)");


% Motor angular rates
figure(Name="Motors");
for i = 1:4
axMot(i) = subplot(2, 2, i);
	plot(tFc, setpoints.rotRate(i, :));
	hold on;
	plot(tSim, motorRotRate(i, :));
	hold off;
	ylabel(sprintf("M%d \\omega (rad/s)", i));
end
xlabel("Time (s)");
grid(axMot, 'on');
legend(axMot(2), ["Setpoint", "Simulated"]);

% Thrust setpoints
figure(Name="thrust");
plot(tSim, thrust);
hold on
plot(tFc, setpoints.thrust);
hold off
title("Total Thrust");
ylabel("Thrust (N)");
xlabel("Time (s)");
legend(["Simulated", "Setpoint"]);

% 3D Position
% Room
xLimits = [-6.2197, 2.165];
yLimits = [-0.8, 2.6975];
zLimits = [-2.5, 0.2];
f = figure(Name="3D Trajectory");

ax = gca;
plot3(traj(1, :), traj(2, :), traj(3, :));
hold on
scatter3(waypoints(1, :), waypoints(2, :), waypoints(3, :), 30, 'r', 'filled');
plot3(position(1, :), position(2, :), position(3, :));
hold off
set(ax, "XLim", xLimits, "YLim", yLimits, "ZLim", zLimits);
ax.Clipping = "off";
f.CurrentAxes.ZDir = 'Reverse';
f.CurrentAxes.YDir = 'Reverse';
axis equal;
xlabel("x (m)");
ylabel("y (m)");
zlabel("z (m)");
legend(["Desired Trajectory", "Required Waypoints", "Simulated Trajectory"]);
clear f;


%% Debug
% % PID outputs from controller
% figure(Name="PIDOutput");
% plot(tFc, pidOutput/100);
% 
% figure(Name="Individual P, I, D")
% subplot(3, 1, 1)
% 	plot(tFc, pids.p);
% subplot(3, 1, 2)
% 	plot(tFc, pids.i);
% subplot(3, 1, 3)
% 	plot(tFc, pids.d);
