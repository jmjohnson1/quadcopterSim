function [posOut, DCM] = ChangeFrame(posIn, ref_lla, fromFrame, toFrame)
  %%%
  % DESCRIPTION:
  %  Transforms a position vector given in one frame, relative to the origin of that frame, to another reference frame.
  % SYNTAX:
  %  [posOut, DCM] = ChangeFrame(posIn, ref_lla, fromFrame, toFrame)
  % INPUTS:
  %  | var       | type   | size  | description
  %  ------------------------------------------
  %  | posIn     | float  | (3,1) | Position vector to be transformed
  %  | ref_lla   | float  | (3,1) | Reference position for local tangent frame in latitude, longitude, altitude (rad, rad, m).
  %  | fromFrame | string | -     | "NED", "ECEF", or "LLA"
  %  | toFrame   | string | -     | "NED", "ECEF", or "LLA"
  % OUTPUTS:
  %  | posOut    | float  | (3,1) | Transformed position vector
  %  | DCM       | float  | (3,3) | Direction cosine matrix used in transformation
  %%%

  posIn = posIn(:);

  % Convert reference LLA to ECEF for NED calculations
  ref_ecef = lla_to_ecef(ref_lla);
  
  % Determine the transformation
  if strcmp(fromFrame, toFrame)
      posOut = posIn;
      DCM = eye(3);
      return;
  end
  
  if strcmp(fromFrame, "LLA") && strcmp(toFrame, "ECEF")
      posOut = lla_to_ecef(posIn);
      DCM = eye(3);
  elseif strcmp(fromFrame, "ECEF") && strcmp(toFrame, "LLA")
      posOut = ecef_to_lla(posIn);
      DCM = eye(3);
  elseif strcmp(fromFrame, "ECEF") && strcmp(toFrame, "NED")
      [posOut, DCM] = ecef_to_ned(posIn, ref_ecef, ref_lla);
  elseif strcmp(fromFrame, "NED") && strcmp(toFrame, "ECEF")
      [posOut, DCM] = ned_to_ecef(posIn, ref_ecef, ref_lla);
  else
      error('Unsupported transformation.');
  end
end


function ecef = lla_to_ecef(lla)
  % Convert Latitude, Longitude, Altitude to ECEF coordinates (meters)
  a = 6378137; % WGS-84 semi-major axis
  e2 = 0.00669437999014; % WGS-84 first eccentricity squared
  lat = lla(1);
  lon = lla(2);
  alt = lla(3);
  
  N = a / sqrt(1 - e2 * sin(lat)^2);
  
  x = (N + alt) * cos(lat) * cos(lon);
  y = (N + alt) * cos(lat) * sin(lon);
  z = (N * (1 - e2) + alt) * sin(lat);
  
  ecef = [x; y; z];
end

function lla = ecef_to_lla(ecef)
  % Convert ECEF coordinates to Latitude, Longitude, Altitude
  X = ecef(1); Y = ecef(2); Z = ecef(3);
  a = 6378137;
  e2 = 0.00669437999014112;
  p = sqrt(X^2 + Y^2);

  lon = atan2(Y, X);
  % Use an iterative approach to solve for lat and h
  lat_prev = atan2(Z, p*(1 - e2));
  res = 1;
  while res > 1e-9
    N = a / sqrt(1 - e2 * sin(lat_prev)^2);
    h = p/cos(lat_prev) - N;
    lat = atan2(Z, p*(1 - e2*N/(N + h)));
    res = abs(lat - lat_prev);
    lat_prev = lat;
  end
  
  lla = [lat; lon; h];
end

function [ned, C_ne] = ecef_to_ned(ecef, ref_ecef, ref_lla)
  % Convert ECEF to NED
  d = ecef - ref_ecef;
  C_ne = ned_dcm(ref_lla);
  ned = C_ne * d;
end

function [ecef, C_en] = ned_to_ecef(ned, ref_ecef, ref_lla)
  % Convert NED to ECEF
  C_ne = ned_dcm(ref_lla);
  C_en = C_ne';
  ecef = ref_ecef + C_en * ned;
end

function C_ne = ned_dcm(lla)
  % Compute direction cosine matrix from ECEF to NED
  lat = lla(1);
  lon = lla(2);
  
  C_ne = [-sin(lat) * cos(lon), -sin(lat) * sin(lon), cos(lat);
          -sin(lon),            cos(lon),             0;
          -cos(lat) * cos(lon), -cos(lat) * sin(lon), -sin(lat)];
end

