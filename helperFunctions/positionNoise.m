function rNoisy = positionNoise(r)
  % Standard deviation of position error
  stdP = 0.01; % [m]
  rNoisy = r + randn(3, 1)*stdP;
end