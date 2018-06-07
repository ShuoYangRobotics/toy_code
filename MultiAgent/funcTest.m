% flocking relate variables (From Reza-Olifati paper Section 8)
param = {};
param.d = 7;
param.r = 1.2*param.d;
param.dp = 0.6*param.d;
param.rp = 1.2*param.dp;
param.epsilon = 0.1;
param.a = 5; param.b = 5;
param.bumph = 0.2;


z = -10:0.1:10;
out = z;
for i = 1:size(z,2)
    out(i) = deltaNorm(z(i), param);
end
plot(z, out)

figure
z = -1:0.05:3;
out = z;
for i = 1:size(z,2)
    out(i) = bump(z(i), param);
end
plot(z, out)

figure
d = 0:0.05:25;  % spatial adjacency is around 5
out = d;
for i = 1:size(d,2)
    out(i) = spatialAdjacency([0;0],[1;1].*d(i), param);
end
plot(d, out)

%%
figure
z = 0:0.05:25;
out = z;
for i = 1:size(z,2)
    out(i) = attractivePotential(z(i), param);
end
plot(z, out)
%%
figure
z = 0:0.05:25;
out = z;
for i = 1:size(z,2)
    ra = deltaNorm(param.r,param);
    da = deltaNorm(param.d,param);

    out(i) = bump(z(i)/ra, param)*phi(z(i)-da, param);
end
plot(z, out)
%%
figure
z = 0:0.05:25;
out = z;
for i = 1:size(z,2)
    ra = deltaNorm(param.r,param);
    da = deltaNorm(param.d,param);

    out(i) = bump(z(i)/ra, param)*phi(z(i)-da, param)*gradientDeltaNorm(z(i), param);
end
plot(z, out)
