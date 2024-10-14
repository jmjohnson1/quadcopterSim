function [Cba] = Quaternion2DCM(q_ba)
%QUATERNION2DCM  Quaternion to DCM conversion.
% [Cba] = QUATERNION2DCM(q_ba) solves for the DCM based on the quaternion 
% using expression on p. 25 of de Ruiter (2013).
%
% INPUT PARAMETERS:
% q_ba = 4x1 column matrix containing quaternion parameters
%
% OUTPUT PARAMETERS:
% Cba = 3x3 DCM calculated from q
%
% Ryan Caverly
% Updated January 2019
%
% Modified by James Johnson, March 2024
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Extract column matrix and scalar part of quaternion
epsilon_ba = [q_ba(2);q_ba(3);q_ba(4)];
eta_ba = q_ba(1);

% Cba = (2*eta_ba^2-1)*eye(3) + 2*(epsilon_ba*epsilon_ba') - 2*eta_ba*crossm(epsilon_ba);
Cba = (2*eta_ba^2-1)*eye(3) + 2*(epsilon_ba*epsilon_ba') - ...
       2*eta_ba*[0, -epsilon_ba(3), epsilon_ba(2); 
                 epsilon_ba(3), 0, -epsilon_ba(1); 
                 -epsilon_ba(2), epsilon_ba(1), 0];