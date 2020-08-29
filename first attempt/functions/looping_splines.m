function [pos,vel, acc, period] = looping_splines(positions,times)
%SPLINES Summary of this function goes here
%   Detailed explanation goes here

len= size(positions);
len = len(2);
if size(positions)~=size(times)
    disp("Error! You should  provide vectors of the same size");
end

second = times+times(len);
third = times+second(len);
times = [times, second(2:len), third(2:len)];

q = positions;
q = [q, q(2:len), q(2:len)];

pp = spline(times, q);

pp.coefs = pp.coefs(len:len+len-2, :);
pp.breaks = pp.breaks(len:len+len-1);
pp.pieces = len-1;

pp_vel = fnder(pp, 1);
pp_acc = fnder(pp_vel, 1);

tmp_pos = @(x) ppval(pp, (x+times(len)));
pos = @(x) tmp_pos(mod(x, times(len)));
tmp_vel = @(x) ppval(pp_vel, (x+times(len)));
vel = @(x) tmp_vel(mod(x, times(len)));
tmp_acc = @(x) ppval(pp_acc, (x+times(len)));
acc = @(x) tmp_acc(mod(x, times(len)));
period = times(len);
end

