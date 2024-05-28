function [positionSp, velocitySp, flightVar] = GetSetpoints(t, s, traj,...
																						doTakeoff, doLanding, flightVar)


	takeoffAlt = 0.75;
	k = 0;
	waypointArrivedThresh = 0.05;

	if doTakeoff == false && strcmp(flightVar.mode, 'takeoff')
		flightVar.mode = "mission";
	end

	if strcmp(flightVar.mode, 'takeoff')
		if flightVar.takeoffFlag == false
			flightVar.takeoffFlag = true;
			flightVar.xSetpoint_TO = s(1);
			flightVar.ySetpoint_TO = s(2);

		end
		positionSp = [flightVar.xSetpoint_TO; flightVar.ySetpoint_TO; -takeoffAlt];
		velocitySp = [0; 0; k*(-takeoffAlt - s(3))];

		if abs(-takeoffAlt - s(3)) < waypointArrivedThresh
			flightVar.waypointArrived = true;
		end

		% The time being empty means we just got here
		if (isempty(flightVar.waypointArrivedTime) && ...
									flightVar.waypointArrived == true)
			flightVar.waypointArrivedTime = t;
		end

		if  (flightVar.waypointArrived==true && ...
									(t - flightVar.waypointArrivedTime) > 3)
			flightVar.mode = 'mission';
			% Reset waypointArrived
			flightVar.waypointArrived = false;
			flightVar.waypointArrivedTime = [];
		end

	elseif strcmp(flightVar.mode, 'mission')

		if flightVar.okToStartMission == false
			% Need to navigate to the first mission waypoint
			positionSp = traj(1:3, 1);
			velocitySp = k*(positionSp - s(1:3));
			
			% Check if near first waypoint
			if norm(positionSp - s(1:3)) < waypointArrivedThresh
				flightVar.waypointArrived = true;
			end
				% The time being empty means we just got here
			if (isempty(flightVar.waypointArrivedTime) && ...
										flightVar.waypointArrived == true)
				flightVar.waypointArrivedTime = t;
			end
			if  (flightVar.waypointArrived==true && ...
									(t - flightVar.waypointArrivedTime) > 3)
				flightVar.okToStartMission = true;
				% Reset waypointArrived
				flightVar.waypointArrived = false;
				flightVar.waypointArrivedTime = [];
			end

		else
			% If this is empty, that means this is our first time here and the
			% mission start time should be recorded
			if isempty(flightVar.missionStartTime)
				flightVar.missionStartTime = t;
			end
			% Elapsed mission time
  		tmt = (t - flightVar.missionStartTime) - traj(7, :);
			% Check if the last waypoint has been reached
  		if (t - flightVar.missionStartTime) > traj(7, end)
    		t_idx = width(traj);
				positionSp = traj(1:3, t_idx);
				velocitySp = traj(4:6, t_idx);
				
				if doLanding==true
					flightVar.mode = 'landing';
          flightVar.landingFlag = true;
			    flightVar.xSetpoint_LD = s(1);
			    flightVar.ySetpoint_LD = s(2);
          positionSp = [flightVar.xSetpoint_LD; flightVar.ySetpoint_LD; -0.1];
		      velocitySp = [0; 0; 0];
				end
			else
				% Interpolates between setpoints
				GeneralTrajectorySetpoint(tmt, t - flightVar.missionStartTime, traj);
			end
		end

	elseif doLanding==true && strcmp(flightVar.mode, 'landing')
		if flightVar.landingFlag == false
			flightVar.landingFlag = true;
			flightVar.xSetpoint_LD = s(1);
			flightVar.ySetpoint_LD = s(2);
		end
		positionSp = [flightVar.xSetpoint_LD; flightVar.ySetpoint_LD; -0.1];
		velocitySp = [0; 0; 0];

		% if abs(s(3)) < waypointArrivedThresh
		% 	flightVar.waypointArrived = true;
		% end
		% 
		% % The time being empty means we just got here
		% if (isempty(flightVar.waypointArrivedTime) && ...
			% 						flightVar.waypointArrived == true)
		% 	flightVar.waypointArrivedTime = t;
		% end
		% 
		% % if  (flightVar.waypointArrived==true && ...
		% 	% 						(t - flightVar.waypointArrivedTime) > 3)
		% % 	flightVar.mode = 'done';
		% % 	% Reset waypointArrived
		% % 	flightVar.waypointArrived = false;
		% % 	flightVar.waypointArrivedTime = [];
		% % end

	end

	function GeneralTrajectorySetpoint(tmt, tLocal, traj)
		tRight_idx = find(tmt <= 0, 1, "first");
		if tRight_idx == 1
			positionSp = traj(1:3, 1);
			velocitySp = traj(4:6, 1);
		else
			tLeft_idx = tRight_idx - 1;
			timeLeft = traj(7, tLeft_idx);
			timeRight = traj(7, tRight_idx);
			posLeft = traj(1:3, tLeft_idx);
			posRight = traj(1:3, tRight_idx);
			velLeft = traj(4:6, tLeft_idx);
			velRight = traj(4:6, tRight_idx);
			frac = (tLocal - timeLeft)/(timeRight - timeLeft);

			positionSp = posLeft + (posRight - posLeft)*frac;
			velocitySp = velLeft + (velRight - velLeft)*frac;
		end
	end
  
end