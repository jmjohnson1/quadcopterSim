function [ECEF, C_en] = NED_to_ECEF(ned, lla_ref)
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

  % Rotation matrix from NED to ECEF
  C_en = [-sin(lat0)*cos(lon0), -sin(lon0), cos(lat0)*cos(lon0);
         -sin(lat0)*sin(lon0),   cos(lon0), cos(lat0)*sin(lon0);
               cos(lat0),            0,          sin(lat0)];

  % Apply the rotation and translation
  ECEF = [X0; Y0; Z0] + C_en * ned(:);
end

