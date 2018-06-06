function [ out ] = phi( z, param )
%PHI part of the pairwise attractive/repulsive potential function
%   Reza paper Equation 15
    a = param.a;
    b = param.b;
    c = abs(a-b)/sqrt(4*a*b);
    
    zc = z + c;
    delta1 = zc/(sqrt(1+zc*zc));
    
    out = 0.5*((a+b)*delta1+(a-b));
end

