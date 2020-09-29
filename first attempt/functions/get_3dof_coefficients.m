function [a] = get_3dof_coefficients(m1, rc1x, rc1y, rc1z, I1yy, m2, rc2x, rc2y, rc2z, I2xx, I2yy, I2zz, m3, rc3x, rc3y, rc3z, I3xx, I3yy, I3zz)
%DINAMIC_COEFFICIENTS Summary of this function goes here
%   Detailed explanation goes here

g0 = 9.81;
b = -rc2x;
c = -rc2z;
d = -rc3x;
d2 = -0.09;
L2 = 0.3;
L3 = 0.2;

a1= I1yy + m2*(c+ d2)^2 + m3 * d2^2;
a2 = m2*(L2 -b)^2 + m3 * L2^2 + I2yy;
a3 = I2xx;
a4 = I3yy + m3*(L3 - d)^2;
a5 = I3xx;
a6 = m3*(L3 -d)*L2;
a7 = I2zz + m2*(L2 - b)^2 + I3zz + m3 * (L2^2 + (L3 - d)^2 );
a8 = m3*(L3 -d)^2 + I3zz;
a9 = m2*(L2 - b)*(c+ d2) + m3*L2*d2;
a10 = m3*(L3 - d)*d2;
a11 = g0 * m2 * (L2 - b);
a12 = g0 * m3 * (L3 - d);
a13 = g0 * m3 * L2;

a = [a1; a2; a3; a4; a5; a6; a7; a8; a9; a10; a11; a12; a13];
end

