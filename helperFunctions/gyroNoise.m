function wNoisy = gyroNoise(w)
  stdW = 0.01;
  wNoisy = w + randn(3, 1)*stdW;
end