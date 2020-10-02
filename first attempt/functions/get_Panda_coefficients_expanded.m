% author: Claudio Gaz, Marco Cognetti
% date: August 2, 2019
% 
% -------------------------------------------------
% Parameters Retrieval Algorithm
% -------------------------------------------------
% C. Gaz, M. Cognetti, A. Oliva, P. Robuffo Giordano, A. De Luca, 'Dynamic
% Identification of the Franka Emika Panda Robot With Retrieval of Feasible
% Parameters Using Penalty-Based Optimization'. IEEE RA-L, 2019.
%
% the following code has been tested on Matlab 2018b

% get_Panda_coefficients_expanded returns the numerical value of the
% dynamic coefficients vector \pi(p) , given the values of the dynamic
% parameters p
function P_li_full_subs = get_Panda_coefficients_expanded(I2xx,I2xy,I3xx,I2xz,I3xy,I4xx,I3xz,I4xy,I5xx,I4xz,I5xy,I6xx,I5xz,I6xy,I7xx,I6xz,I7xy,I7xz,I2yy,I2yz,I3yy,I3yz,I4yy,I4yz,I5yy,I5yz,I6yy,I6yz,I7yy,I7yz,I1zz,I2zz,I3zz,I4zz,I5zz,I6zz,I7zz,c1x,c2x,c3x,c4x,c5x,c6x,c7x,c1y,c2y,c3y,c4y,c5y,c6y,c7y,c2z,c3z,c4z,c5z,c6z,c7z,fc1,fc2,fc3,fc4,fc5,fc6,fc7,fo1,fo2,fo3,fo4,fo5,fo6,fo7,fv1,fv2,fv3,fv4,fv5,fv6,fv7,m1,m2,m3,m4,m5,m6,m7)
%GET_PANDA_COEFFICIENTS_EXPANDED
%    P_LI_FULL_SUBS = GET_PANDA_COEFFICIENTS_EXPANDED(I2XX,I2XY,I3XX,I2XZ,I3XY,I4XX,I3XZ,I4XY,I5XX,I4XZ,I5XY,I6XX,I5XZ,I6XY,I7XX,I6XZ,I7XY,I7XZ,I2YY,I2YZ,I3YY,I3YZ,I4YY,I4YZ,I5YY,I5YZ,I6YY,I6YZ,I7YY,I7YZ,I1ZZ,I2ZZ,I3ZZ,I4ZZ,I5ZZ,I6ZZ,I7ZZ,C1X,C2X,C3X,C4X,C5X,C6X,C7X,C1Y,C2Y,C3Y,C4Y,C5Y,C6Y,C7Y,C2Z,C3Z,C4Z,C5Z,C6Z,C7Z,FC1,FC2,FC3,FC4,FC5,FC6,FC7,FO1,FO2,FO3,FO4,FO5,FO6,FO7,FV1,FV2,FV3,FV4,FV5,FV6,FV7,M1,M2,M3,M4,M5,M6,M7)

%    This function was generated by the Symbolic Math Toolbox version 8.2.
%    10-Jun-2019 17:50:26

t2 = c2x.^2;
t3 = m2.*t2;
t4 = m3.*9.9856e-2;
t5 = c3z.*m3.*(7.9e1./1.25e2);
t6 = c3x.^2;
t7 = m3.*t6;
t8 = c2y.^2;
t9 = m2.*t8;
t10 = c3z.^2;
t11 = m3.*t10;
t12 = c4x.^2;
t13 = m4.*t12;
t14 = c3y.^2;
t15 = m3.*t14;
t16 = c4z.^2;
t17 = m4.*t16;
t18 = c5z.*m5.*(9.6e1./1.25e2);
t19 = c5x.^2;
t20 = m5.*t19;
t21 = c4y.^2;
t22 = m4.*t21;
t23 = c5z.^2;
t24 = m5.*t23;
t25 = c6x.^2;
t26 = m6.*t25;
t27 = c5y.^2;
t28 = m5.*t27;
t29 = c6z.^2;
t30 = m6.*t29;
t31 = c7x.^2;
t32 = m7.*t31;
t33 = c6y.^2;
t34 = m6.*t33;
t35 = c7z.^2;
t36 = m7.*t35;
t37 = c7y.^2;
t38 = m7.*t37;
t39 = m5.*(3.3e1./4.0e2);
t40 = m6.*(3.3e1./4.0e2);
t41 = m7.*(3.3e1./4.0e2);
P_li_full_subs = [I2yy+I1zz+t3+c1x.^2.*m1+c1y.^2.*m1+c2z.^2.*m2;I2xx-I2yy+I3yy+m4.*1.066622499999973e-1+m5.*1.066622500000027e-1+m6.*1.066622499999878e-1+m7.*1.066622499999634e-1-t3+t4+t5+t7+t9+t11;I2xy-c2x.*c2y.*m2;I2xz-c2x.*c2z.*m2;I2yz-c2y.*c2z.*m2;I3yy+I2zz+m4.*1.066622499999996e-1+m5.*1.066622500000015e-1+m6.*1.066622500000046e-1+m7.*1.066622499999844e-1+t3+t4+t5+t7+t9+t11;I3xx-I3yy+I4yy-m4.*6.806250000001852e-3-t7+t13+t15+t17;I3xy+c4z.*m4.*8.249999999999809e-2-c3x.*c3y.*m3;I3xz-c3x.*c3z.*m3;I3yz-c3y.*c3z.*m3;I4yy+I3zz+m4.*6.806249999999603e-3+m5.*1.36125e-2+m6.*1.361250000000224e-2+m7.*1.361250000000133e-2+t7+t13+t15+t17;I4xx-I4yy+I5yy+m5.*1.406497499999985e-1+m6.*1.406497499999834e-1+m7.*1.4064974999999e-1-t13+t18+t20+t22+t24;I4xy+m5.*3.167999999999923e-2+m6.*3.168000000000109e-2+m7.*3.168e-2+c5z.*m5.*(3.3e1./4.0e2)-c4x.*c4y.*m4;I4xz-c4x.*c4z.*m4;I4yz-c4y.*c4z.*m4;I5yy+I4zz+m5.*1.542622499999982e-1+m6.*1.542622499999996e-1+m7.*1.542622499999975e-1+t13+t18+t20+t22+t24;I5xx-I5yy+I6yy+m7.*7.744000000003362e-3-t20+t26+t28+t30;I5xy-c5x.*c5y.*m5;I5xz-c5x.*c5z.*m5;I5yz-c5y.*c5z.*m5;I6yy+I5zz+m7.*7.744000000001959e-3+t20+t26+t28+t30;I6xx-I6yy+I7yy-m7.*7.744000000006857e-3-t26+t32+t34+t36;I6xy+c7z.*m7.*(1.1e1./1.25e2)-c6x.*c6y.*m6;I6xz-c6x.*c6z.*m6;I6yz-c6y.*c6z.*m6;I7yy+I6zz+m7.*7.74400000000188e-3+t26+t32+t34+t36;I7xx-I7yy-t32+t38;I7xy-c7x.*c7y.*m7;I7xz-c7x.*c7z.*m7;I7yz-c7y.*c7z.*m7;I7zz+t32+t38;c2x.*m2;m3.*(-7.9e1./2.5e2)-m4.*(7.9e1./2.5e2)-m5.*(7.9e1./2.5e2)-m6.*(7.9e1./2.5e2)-m7.*(7.9e1./2.5e2)+c2y.*m2-c3z.*m3;m4.*(3.3e1./4.0e2)+t39+t40+t41+c3x.*m3;c3y.*m3-c4z.*m4;-t39-t40-t41+c4x.*m4;m5.*(4.8e1./1.25e2)+m6.*(4.8e1./1.25e2)+m7.*(4.8e1./1.25e2)+c4y.*m4+c5z.*m5;c5x.*m5;c5y.*m5-c6z.*m6;m7.*(1.1e1./1.25e2)+c6x.*m6;c6y.*m6-c7z.*m7;c7x.*m7;c7y.*m7;fv1;fv2;fv3;fv4;fv5;fv6;fv7;fc1;fc2;fc3;fc4;fc5;fc6;fc7;fo1;fo2;fo3;fo4;fo5;fo6;fo7];
