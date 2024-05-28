function qNoisy = quaternionNoise(q)
  eulerAngles = DCM2Euler321(Quaternion2DCM(q));
  % Standard deviation of about 1 degree. This is arbitrary since I don't have
  % good ground truth and haven't looked at the covariance estimates from EKF
  eulerNoisy = eulerAngles + randn(3, 1)*0.1*pi/180;
  qNoisy = DCM2Quaternion(Euler3212DCM(eulerNoisy));
end