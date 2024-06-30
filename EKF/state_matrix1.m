function [Fc, Gc]=state_matrix1(q,u, Ts, const)

Lat = 44.975551204890714;
omega_e = 7.292115e-5;
omega_et = [0 -omega_e*cosd(Lat) -omega_e*sind(Lat)]';
Omega_et =skew(omega_et);

w = u(4:6);

Rb2t = Quaternion2DCM(q)';

f_t = Rb2t*u(1:3);

St=[0 -f_t(3) f_t(2); f_t(3) 0 -f_t(1); -f_t(2) f_t(1) 0];

O=zeros(3);

I=eye(3);

Da=diag(u(1:3));
Dg=diag(u(4:6));

% acc_bias_instability_time_constant_filter=100;
% gyro_bias_instability_time_constant_filter=300;
% B1=-1/acc_bias_instability_time_constant_filter*eye(3);
% B2=-1/gyro_bias_instability_time_constant_filter*eye(3);

% Fc=[O I O O O;
%     O O St Rb2t O;
%     O O O O -Rb2t;
%     O O O B1 O;
%     O O O O B2];
% Gc=[O O O O; Rb2t O O O; O -Rb2t O O; O O I O; O O O I];

Fc = [O    I     O     O     O;
      O   O  -Rb2t*skew(u(1:3))   -Rb2t    O;
      O    O     -skew(w)     O    -I;
      O    O     O     diag(-1./const.tau_a)*0     O;
      O    O     O     O     diag(-1./const.tau_g)*0];

Gc = [ O      O    O   O; 
       -Rb2t  O    O   O; 
       O     -I    O   O; 
       O      O    I*0   O; 
       O      O    O   I*0];


% F=eye(size(Fc))+Ts*Fc;
% G=Ts*Gc;


end