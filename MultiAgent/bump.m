function [ p ] = bump( z,param )
% BUMPER smooth input to [0,1] range
%   Reza paper Equation 10
    h = param.bumph;
    if (z >= 0) && (z < h)
        p = 1;
    elseif (z>=h) && (z<=1)
        p = 0.5*(1+cos(pi*(z-h)/(1-h)));
    else
        p = 0;
    end
end


