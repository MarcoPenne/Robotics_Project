function [pos,vel, acc] = generate_exciting_traj()
%GENERATE_EXCITING_TRAJ Summary of this function goes here
%   Detailed explanation goes here

limit = 1;
a = rand(5,1) -0.5;
b = rand(5,1) -0.5;
q0 = rand(1,1) - 0.5;
w = 0.3*pi;

syms pos(t);

pos = @(t) (a(1)/w)*sin(w*t) - (b(1)/w)*cos(w*t) ...
         + (a(2)/(2*w))*sin(2*w*t) - (b(2)/(2*w))*cos(2*w*t) + q0(1);
         %+ (a(3)/(3*w))*sin(3*w*t) - (b(3)/(3*w))*cos(3*w*t) ...
         %+ (a(4)/(4*w))*sin(4*w*t) - (b(4)/(4*w))*cos(4*w*t) ...
         %+ (a(5)/(5*w))*sin(5*w*t) - (b(5)/(5*w))*cos(5*w*t) ...
g = diff(pos, t);
i=1;
sol = [];
for n = 1:20
    S = vpasolve(g==0,t,[-2 5],'Random',true);
    sol(i)=S;
    i = i+1;
end
extrema = vpa(sol, 6);
maximum = max(abs(pos(extrema)));

pos = @(t) pos(t)*limit/maximum;
vel_tmp(t) = diff(pos, t);
vel = @(t) double(vel_tmp(t));
acc_tmp(t) = diff(vel_tmp, t);
acc = @(t) double(acc_tmp(t));
end