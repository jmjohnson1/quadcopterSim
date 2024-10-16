function [Fc, Gc]=state_matrix1(q,u, Ts, const)

  w = u(4:6);
  Rb2t = Quaternion2DCM(q)';
  O=zeros(3);
  I=eye(3);

  % Position derivatives
  dp_p = O;
  dp_v = I;
  dp_e = O;
  dp_ba = O;
  dp_bg = O;

  % Velocity derivatives
  dv_p = O;
  dv_v = O;
  % dv_e = -Rb2t*skew(u(1:3));
  dv_e = -2*Rb2t*skew(u(1:3));
  dv_ba = -Rb2t;
  dv_bg = O;

  % Attitude error derivatives
  de_p = O;
  de_v = O;
  de_e = -skew(w);
  de_ba = O;
  % de_bg = -I;
  de_bg = -0.5*I;


  % Accel bias derivatives
  dba_p = O;
  dba_v = O;
  dba_e = O;
  dba_ba = diag(-1./const.tau_a);
  dba_bg = O;

  % Gyro bias derivatives
  dbg_p = O;
  dbg_v = O;
  dbg_e = O;
  dbg_ba = O;
  dbg_bg = diag(-1./const.tau_g);

  % Continuous state transition
  Fc = [dp_p   dp_v   dp_e   dp_ba   dp_bg;
        dv_p   dv_v   dv_e   dv_ba   dv_bg;
        de_p   de_v   de_e   de_ba   de_bg;
        dba_p  dba_v  dba_e  dba_ba  dba_bg;
        dbg_p  dbg_v  dbg_e  dbg_ba  dbg_bg];
  
  % Gc = [ O      O    O   O; 
  %        -Rb2t  O    O   O; 
  %        O     -I    O   O; 
  %        O      O    I   O; 
  %        O      O    O   I];

  Gc = [ O      O    O   O; 
         -Rb2t  O    O   O; 
         O     -0.5*I    O   O; 
         O      O    I   O; 
         O      O    O   I];

end
