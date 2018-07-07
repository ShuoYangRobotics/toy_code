function [ dnorm ] = gradientDeltaNorm( vec, param )
%GRADIENTDELTANORM gradient of delta norm
%   Reza paper Equation 9

if param.epsilon <= 0
    error('invalid epsilon value')
else
    dnorm = vec./sqrt(1+param.epsilon*norm(vec)*norm(vec));
end

end

