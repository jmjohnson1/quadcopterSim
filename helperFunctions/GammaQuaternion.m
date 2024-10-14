function Gamma_ba = GammaQuaternion(q_ba)
%GammaQuaternion
% DESCRIPTION
%   Computes the Gamma matrix that maps the angular velocity to quaternion
%   rates
% USAGE
%   Gamma_ba = GammaQuaternion(q_ba)
% INPUT
%   q_ba    Quaternion with scalar on top (4x1)
% OUTPUT
%   Gamma_ba    Mapping matrix (3x3)

if (isrow(q_ba)) 
    q_ba = q_ba';
end
eps_ba = q_ba(2:4);
eta_ba = q_ba(1);

% Gamma_ba = 1/2*[-eps_ba'
%                 (eta_ba*eye(3) + crossm(eps_ba))];
Gamma_ba = 1/2*[-eps_ba'
                (eta_ba*eye(3) + [0 -eps_ba(3) eps_ba(2);eps_ba(3) 0 -eps_ba(1);-eps_ba(2) eps_ba(1) 0])];



end

function [M] = crossm(v)
%CROSSM  Cross product matrix calculation.
% [M] = CROSSM(v) solves for cross product matrix of v.
%
% INPUT PARAMETERS:
% v = 3x1 column matrix
%
% OUTPUT PARAMETERS:
% M = 3x3 cross product matrix of v
%
% Ryan Caverly
% Updated February 2019
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

M = [0 -v(3) v(2);v(3) 0 -v(1);-v(2) v(1) 0];

end