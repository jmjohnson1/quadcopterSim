function [constrainedValue] = constrain(value, lowerLim, upperLim)
%CONSTRAIN Constrains a value between min and max
%
%INPUTS:
%   value = (1xn) The value to constrain
%   min = (1x1) The minimum value
%   max = (1x1) The maximum value
%OUTPUTS:
%   constrainedValue

constrainedValue = min(max(value,lowerLim),upperLim);

end

