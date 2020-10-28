clear all
close all
clc
addpath("functions/");

%% ----------------------
% GENERATE EXCITING TRAJECTORY
%------------------------
 
[position1, velocity1, acceleration1, time1] = load_trajectory('data/3-dof/new_trajectory1');
[position2, velocity2, acceleration2, time2] = load_trajectory('data/3-dof/new_trajectory2');
[position3, velocity3, acceleration3, time3] = load_trajectory('data/3-dof/new_trajectory3');

time_tmp = [time1; time2; time3];
position_tmp = [position1; position2; position3];
velocity_tmp = [velocity1; velocity2; velocity3];
acceleration_tmp = [acceleration1; acceleration2; acceleration3];

[x1, y1, z1] = looping_splines(position_tmp(1,:), time_tmp(1,:));
[x2, y2, z2] = looping_splines(position_tmp(2,:), time_tmp(2,:));
[x3, y3, z3] = looping_splines(position_tmp(3,:), time_tmp(3,:));

period = double(lcm(sym([time_tmp(1, size(time_tmp, 2)), time_tmp(2, size(time_tmp, 2)), time_tmp(3, size(time_tmp, 2))])));

position = {x1, x2, x3};
velocity = {y1, y2, y3};
acceleration = {z1, z2, z3};

figure('Name', 'Trajectories');
for i=1:3
    subplot(3,3,i);
    fplot(position(i), [0 period]);
    xlabel("time (s)")
    ylabel("position (rad)")
    legend ("position j"+i);
    subplot(3,3,i+3);
    fplot(velocity(i), [0 period]);
    xlabel("time (s)")
    ylabel("velocity (rad/s)")
    legend ("velocity j"+i);
    subplot(3,3,i+6);
    fplot(acceleration(i), [0 period]);
    xlabel("time (s)")
    ylabel("acceleration (rad/s^2)");
    legend("acceleration j"+i);
end

dt = 0.02;
timestamps = 0:dt:period;

freq = 1/dt;

pos1 = position{1}(timestamps);
pos2 = position{2}(timestamps);
pos3 = position{3}(timestamps);

vel1 = velocity{1}(timestamps);
vel2 = velocity{2}(timestamps);
vel3 = velocity{3}(timestamps);

acc1 = acceleration{1}(timestamps);
acc2 = acceleration{2}(timestamps);
acc3 = acceleration{3}(timestamps);

Y_stack = zeros((length(timestamps))*3,10);
u_stack = zeros((length(timestamps))*3,1);

for j=1:length(timestamps)
    Y = Y_matrix(pos1(j), pos2(j), pos3(j), vel1(j), vel2(j), vel3(j), acc1(j), acc2(j), acc3(j));
    a = get_3dof_coefficients(10, 0, -0.15, 0, 4.167e-04*10, 1.125, -0.15, 0, -0.06, 4.167e-04*1.125, 7.708e-03*1.125, 7.708e-03*1.125, 0.75, -0.1, 0, 0, 4.167e-04*0.75, 3.542e-03*0.75, 3.542e-03*0.75);
    torque(:, j) = Y*a;
    
    Y_stack(j*3-2:j*3,:) = Y;
    u_stack(j*3-2:j*3,:) = torque(:, j);
end

figure('Name', 'Torques');
subplot(3,1,1);
plot(timestamps, torque(1, :));
legend ("torque1");
xlabel("time (s)");
ylabel("torque (Nm)");

subplot(3,1,2);
plot(timestamps, torque(2, :));
legend ("torque2");
xlabel("time (s)");
ylabel("torque (Nm)");

subplot(3,1,3);
plot(timestamps, torque(3, :));
legend ("torque3");
xlabel("time (s)");
ylabel("torque (Nm)");

a
coefficients = pinv(Y_stack)*u_stack
error = norm(coefficients-a)

experiment_path = 'data/3-dof/new_experiment123';
save(fullfile(experiment_path,'Y_stack.mat'), 'Y_stack');
save(fullfile(experiment_path,'u_stack.mat'), 'u_stack');