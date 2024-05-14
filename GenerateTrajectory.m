function traj = GenerateTrajectory(waypoints, type, makePlots)
	if nargin < 3
		makePlots = true;
	end
	
	%% Steps
  if strcmp(type, 'testAttitudeStabilize')
    traj = testAttitudeStabilize();
	elseif strcmp(type, 'testStepX')
		wps = [1.5, 0, 0, 0]';
    traj = linearsteps(wps);
	elseif strcmp(type, 'testStepY')
		wps = [0, 1.5, 0, 0]';
    traj = linearsteps(wps);
	elseif strcmp(type, 'testStepZ')
		wps = [0, 0, -1, 0]';
    traj = linearsteps(wps);

%% Minimum snap
	elseif strcmp(type, 'minsnap')
		traj = minsnap(waypoints);
	end

%% Linear steps function
	function traj = linearsteps(wps)
		pos = wps(1:3, :);
		vel = zeros(size(pos));
		traj = [pos; vel; wps(4, :)];
	end
%% Attitude stabilize function
	function traj = testAttitudeStabilize()
  	traj = [0, 0, 0, 0, 0, 0, 0]';
	end
	
%% Minimum snap function
	function traj = minsnap(wps)
		wp = wps(1:3, :);
		t_wp = wps(4, :);
		numSamples = 100;
	
		[pos,vel,acc,jerk,snap,pp,timepoints,tsamples] = ...
			minsnappolytraj(wp,t_wp,numSamples);
	
		if makePlots == true
			figure(Name="Trajectory:Position")
			labelSnippets = ["x", "y", "z"];
			labelUnit = "m";
			for i = 1:3
				subplot(3, 1, i)
				plot(tsamples, pos(i, :), '.-');
				ylabel(sprintf("%s (%s)", labelSnippets(i), labelUnit));
				grid on
				if i == 1
					title("Trajectory Positions");
				end
			end
			xlabel("Time (s)");

			figure(Name="Trajectory:Velocity")
			labelSnippets = ["v_x", "v_y", "v_z"];
			labelUnit = "m/s";
			for i = 1:3
				subplot(3, 1, i)
				plot(tsamples, vel(i, :), '.-');
				ylabel(sprintf("%s (%s)", labelSnippets(i), labelUnit));
				grid on
				if i == 1
					title("Trajectory Velocity");
				end
			end
			xlabel("Time (s)");

			figure(Name="Trajectory:Acceleration")
			labelSnippets = ["a_x", "a_y", "a_z"];
			labelUnit = "m/s^2";
			for i = 1:3
				subplot(3, 1, i)
				plot(tsamples, acc(i, :), '.-');
				ylabel(sprintf("%s (%s)", labelSnippets(i), labelUnit));
				grid on
				if i == 1
					title("Trajectory Accelerations");
				end
			end
			xlabel("Time (s)");

			figure(Name="Trajectory:Jerk")
			labelSnippets = ["Jerk_x", "Jerk_y", "Jerk_z"];
			labelUnit = "m/s^3";
			for i = 1:3
				subplot(3, 1, i)
				plot(tsamples, jerk(i, :), '.-');
				ylabel(sprintf("%s (%s)", labelSnippets(i), labelUnit));
				grid on
				if i == 1
					title("Trajectory Jerk");
				end
			end
			xlabel("Time (s)");

			figure(Name="Trajectory:Snap")
			labelSnippets = ["Snap_x", "Snap_y", "Snap_z"];
			labelUnit = "m/s^4";
			for i = 1:3
				subplot(3, 1, i)
				plot(tsamples, snap(i, :), '.-');
				ylabel(sprintf("%s (%s)", labelSnippets(i), labelUnit));
				grid on
				if i == 1
					title("Trajectory Snap");
				end
			end
			xlabel("Time (s)");
			
			figure(Name="Trajectory:3D")
			plot3(pos(1, :), pos(2, :), pos(3, :), '.-');
			xlabel("x");
			ylabel("y");
			zlabel("z");
		end

		traj = [pos; vel; tsamples];
		writeToFile(traj);
	end
end

function writeToFile(traj)
	for i = 1:width(traj)
		S.mission.items(i).Altitude = 0;
		S.mission.items(i).AltitudeMode = 1;
		S.mission.items(i).autoContinue = true;
		S.mission.items(i).command = 16;
		S.mission.items(i).doJumpId = 1;
		S.mission.items(i).frame = 1;
		p = [traj(4:6, i); traj(7, i); traj(1:2, i)*1e-3; traj(3, i)];
		S.mission.items(i).params = p;
		S.mission.items(i).type = "SimpleItem";
		S.mission.items(i).AMSLAltAboveTerrain = NaN;
	end
	S.mission.cruiseSpeed = 0;
	S.mission.firmwareType = 12;
	S.mission.globalPlanAltitudeMode = 0;
	S.mission.hoverSpeed = 0;
	S.mission.plannedHomePosition = [0, 0, 0];
	S.mission.vehicleType = 2;
	S.mission.version = 2;
	S.fileType = "Plan";
	S.groundStation = "QGroundControl";
	S.version = 1;
	S.rallyPoints.points = [];
	S.rallyPoints.version = 2;
	S.geoFence.circles = [];
	S.geoFence.polygons = [];
	S.geoFence.version = 2;
	
	fileID = fopen("traj.plan", "w");
	fprintf(fileID, jsonencode(S, PrettyPrint=true, ConvertInfAndNaN=true));
	fclose(fileID);
end

