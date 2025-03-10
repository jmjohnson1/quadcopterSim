function ned = ECEF_to_NED(ECEF, lla_ref)
  % ECEF: ECEF coordinates [X, Y, Z] (meters)
  % lat0, lon0, alt0: Reference latitude, longitude, altitude (rad, rad, meters)

  % Earth's constants
  a = 6378137; % Semi-major axis (meters)
  f = 1 / 298.257223563; % Flattening
  e2 = 2 * f - f^2; % Eccentricity squared

  lat0 = lla_ref(1);
  lon0 = lla_ref(2);
  alt0 = lla_ref(3);
  

  % Compute reference position in ECEF
  N = a / sqrt(1 - e2 * sin(lat0)^2);
  X0 = (N + alt0) * cos(lat0) * cos(lon0);
  Y0 = (N + alt0) * cos(lat0) * sin(lon0);
  Z0 = (N * (1 - e2) + alt0) * sin(lat0);

  % Rotation matrix from ECEF to NED
  R = [-sin(lat0)*cos(lon0), -sin(lat0)*sin(lon0), cos(lat0);
       -sin(lon0),            cos(lon0),            0;
        cos(lat0)*cos(lon0),  cos(lat0)*sin(lon0),  sin(lat0)];

  % Compute the NED coordinates
  ned = R' * (ECEF - [X0; Y0; Z0]);
end

