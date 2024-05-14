function [att] = DCM2Euler321(C_ba)
%DCM2EULER321
% DESCRIPTION
%   Computes the 3-2-1 euler angle sequence that gives the DCM C_ba
% USAGE
%   att = DCM2Euler321(C_ba)
% INPUT
%   C_ba - Direction cosine matrix (3x3)
% OUTPUT
%   att - (3, 1) - attitude (roll, pitch, yaw)

r = atan2(C_ba(2,3),C_ba(3,3));
p = -asin(C_ba(1,3));
y = atan2(C_ba(1,2),C_ba(1,1));

att = [r; p; y];

end

