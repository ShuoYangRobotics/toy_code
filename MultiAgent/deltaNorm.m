function [ dnorm ] = deltaNorm( vec, param )
%DELTANORM A nonnegative map that smooth normal norm
%   Reza paper Equation 8

if param.epsilon <= 0
    error('invalid epsilon value')
else
    dnorm = 1/param.epsilon*(sqrt(1+param.epsilon*norm(vec)*norm(vec))-1);
end

end

