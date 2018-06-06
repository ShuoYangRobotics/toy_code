function [ a ] = spatialAdjacency( qi,qj,param )
%spatialAdjacency calculate spatial adjacency of two node value
%   Reza paper Equation 11

if (qi == qj)
    a = 0;
else
    a = bump( deltaNorm(qj-qi,param)/deltaNorm(param.r,param), param);
end
    
end

