function vNoisy = velocityNoise(v)
  stdV = 0.005;
  vNoisy = v + randn(3, 1)*stdV;
end