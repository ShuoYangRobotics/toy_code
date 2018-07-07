function [ out ] = attractivePotential( z, param )
%attractivePotential part of the collective potential function
%   Reza paper Equation 16
    da = deltaNorm(param.d,param);
    if z <= da
       out = 0;
       for s = z:0.001:da
           out = out + actionFuc(s, param)*0.001;
       end
    else
       out = 0;
       for s = da:0.001:z
           out = out + actionFuc(s, param)*0.001;
       end
       
    end

end

