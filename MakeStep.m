function StepMat = MakeStep(altitudes, times)
    % Make a function here that takes these and puts them here
   traj = zeros(7,length(altitudes));
   traj(3,:) = altitudes;
   traj(7,:) = times;
   placehldr = zeros(7,1);
   for i=2:length(times)
       placehldr(:,i-1) = [0 0 altitudes(i-1) 0 0 0 times(i)-0.01]';
   end
   j = 1;
   k = 1;
   for i=1:length(times)*2-2
       if bitget(i,1)
           StepMat(:,i) = traj(:,j);
           j = j + 1;
       else
           StepMat(:,i) = placehldr(:,k);
           k = k + 1;
       end
   end
   StepMat(:,end+1) = traj(:,end);
end