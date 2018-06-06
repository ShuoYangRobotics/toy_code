function [ out ] = actionFuc( z, param )
%ACTIONFUNC action function, I don't know why...
%   Reza paper Equation 15 first part

    ra = deltaNorm(param.r,param);
    da = deltaNorm(param.d,param);
    
    out = bump(z/ra, param)*phi(z-da, param);

end

