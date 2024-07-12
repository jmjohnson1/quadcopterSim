function [x_out,q] = Navigation_equations( x_in,u,q,Ts )
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%  This function takes as input x - state vector, u - imu vector and 
%  q - quaternion, returns updated state vector - y and updated 
%  quaternion - q
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
x_out = zeros(size(x_in));

w_bn = u(4:6);

w1t = w_bn(1)*Ts;
w2t = w_bn(2)*Ts;
w3t = w_bn(3)*Ts;

dq = [1; 0.5*w1t; 0.5*w2t; 0.5*w3t];
q = quatmultiply(q', dq')';
q = q./norm(q);

if (q(1) < 0) 
  q = -q;
end

% DCM from NED to body frame
C_bn = Quaternion2DCM(q);
C_nb = C_bn';

latitude = 44.975551204890714;
altitude = 256;
g = gravity(latitude,altitude);
g = 9.81;

g_n = [0 0 g]';
f_n = C_nb*u(1:3);
acc_n = f_n + g_n;

x_out(4:6) = x_in(4:6) + Ts*acc_n;
x_out(1:3) = x_in(1:3) + Ts*x_in(4:6) + 1/2*acc_n*Ts^2;
x_out(10:15) = x_in(10:15);
end