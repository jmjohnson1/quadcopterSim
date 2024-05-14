function C_ba = Euler3212DCM(euler) 
  % DESCRIPTION:
  %   Computes the direction cosine matrix given a 3-2-1 Euler angle sequence.
  % USAGE:
  %   C_ba = Euler3212DCM(roll, pitch, yaw) 
  % INPUT:
  %   euler    (3, 1)    euler angles (roll, pitch, yaw) [rad]
  % OUTPUT:
  %   C_ba    DCM from frame a to frame b

  sy = sin(euler(3));
  cy = cos(euler(3));
  sp = sin(euler(2));
  cp = cos(euler(2));
  sr = sin(euler(1));
  cr = cos(euler(1));

  C3 = [cy, sy, 0;
        -sy, cy, 0;
        0, 0, 1];
  C2 = [cp, 0, -sp;
        0, 1, 0;
        sp, 0, cp];
  C1 = [1, 0, 0;
        0, cr, sr;
        0, -sr, cr];
  
  C_ba = C1*C2*C3;

end