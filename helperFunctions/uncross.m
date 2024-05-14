function v = uncross(U)
%UNCROSS Computes the uncross of matrix U
% INPUT:
%		U		(3x3)		Matrix to uncross
% OUTPUT:
%		v		(3x1)		vector

v = [U(3, 2);
    -U(3, 1);
     U(2, 1)];

end

