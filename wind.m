function wind = windVel(x, a, b, amp) 
  e = exp(1);
  wind = amp/sum(a)*(a(1)*sin(b(1)*x) + a(2)*sin(b(2)*e*x) + a(3)*sin(b(3)*pi*x));
end

% clear; close all; clc;
% overallAmplitude = 0.1;
% a1 = 1*randn();
% b1 = 1*randn();
% 
% a2 = 1*randn();
% b2 = 1*randn();
% 
% a3 = 1*randn();
% b3 = 1*randn();
% 
% a = [a1, a2, a3]; b = [b1, b2, b3];
% 
% t = 0:0.01:45;
% u = windVel(t, a, b, overallAmplitude);
% 
% figure()
% plot(t, u, Color='r', Linestyle='-', Linewidth=1);
% xlabel("t")
% ylabel("u")
% grid on
% 
% function vel = windVel(x, a, b, amp)
%   e = exp(1);
%   vel = amp/(a(1) + a(2) + a(3))*(a(1)*sin(b(1)*x) + a(2)*sin(b(2)*e*x) + a(3)*sin(b(3)*pi*x));
% end