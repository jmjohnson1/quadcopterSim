function animateQuad(pos, quat, time, waypoints, trajectory, fps)

  tic
	% Downsample
	timeStep = 1/fps;
	dsIndex = [];
	for t = 0:timeStep:time(end)
		[~, idx] = min(abs(time - t));
		dsIndex = [dsIndex, idx];
	end
	N = length(dsIndex);

	% Downsample inputs
	time_ds = time(dsIndex);
	pos_ds = pos(:, dsIndex);
	q_ds = quaternion(quat(:, dsIndex)'); % Quaternion object for poseplot

	% Room
	xLimits = [-6.2197, 2.165];
	yLimits = [-0.8, 2.6975];
	zLimits = [-2.5, 0.2];

	disp("Generating video...");
	vid = VideoWriter("movie.avi");
	set(vid, "FrameRate", fps);
	open(vid);

	hFig = figure(Name="Animation");
	hFig.Position = [10 10, 300 500];
	hFig.Visible = 'off';
  % The quadmesh.stl model slows things down too much
	% frame = poseplot(q_ds(1), pos_ds(:, 1), MeshFileName="quadmesh.stl");
  frame = poseplot(q_ds(1), pos_ds(:, 1), ScaleFactor=0.1);

	ax = gca;
	hold on
	plot3(trajectory(1, :), trajectory(2, :), trajectory(3, :));
	scatter3(waypoints(1, :), waypoints(2, :), waypoints(3, :));
	hold off
	set(ax, "XLim", xLimits, "YLim", yLimits, "ZLim", zLimits);
	axis equal;
  xlabel("N [m]");
  ylabel("E [m]");
  zlabel("D [m]");
  M = struct('cdata', cell(1,N), 'colormap', cell(1,N));
	M(1) = getframe(gcf);
	for i = 2:N
    fprintf("Frame %d/%d\n", i, N);
		set(frame, Position=pos_ds(:, i), Orientation=q_ds(i));
		drawnow;
		M(i) = getframe(gcf);
	end
	
	disp("Saving video...");
	writeVideo(vid, M);
	close(vid);
	disp("Video saved")
  toc
end