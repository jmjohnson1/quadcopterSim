function q_ba = DCM2Quaternion(C_ba)
%DCM2QUATERNION
% DESCRIPTION
%   Takes in the direction cosine matrix C and returns a quaternion that
%   parameterizes it as described in de Ruiter
% USAGE
%   q_ba = DCM2Quaternion(C_ba)
% INPUT
%   C - DCM (3x3)
% OUTPUT
%   q - Quaternion. SCALAR FIRST. (4x1)

% The positive solution for eta is chosen
eta_ba = sqrt(1 + trace(C_ba))/2;

if (eta_ba ~= 0)
    eps_ba(1) = (C_ba(2,3) - C_ba(3,2))/4/eta_ba;
    eps_ba(2) = (C_ba(3,1) - C_ba(1,3))/4/eta_ba;
    eps_ba(3) = (C_ba(1,2) - C_ba(2,1))/4/eta_ba;
else
    % This represents a rotation of pi, so the direction can be arbitrary.
    % In this case, let eps(1) > 0.
    eps_ba(1) = sqrt((C_ba(1,1) + 1)/2);
    eps_ba(2) = sign(C_ba(1,2)) * sqrt((C_ba(2,2) + 1)/2);
    eps_ba(3) = sign(C_ba(1,3)) * sqrt((C_ba(3,3) + 1)/2);
end

q_ba = [eta_ba, eps_ba]';

end

