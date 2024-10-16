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