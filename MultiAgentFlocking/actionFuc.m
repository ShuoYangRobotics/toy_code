function [ out ] = actionFuc( z, param )
%ACTIONFUNC action function, I don't know why...
%   Reza paper Equation 15 first part

    ra = deltaNorm(param.r,param);
    da = deltaNorm(param.d,param);
    % originally in Reza's paper it is just like this, but I think we need
    % to add a norm of z-da? to prevent it from having negative values
    %out = bump(z/ra, param)*phi(abs(z-da), param);
    out = bump(z/ra, param)*phi(z-da, param);

end

