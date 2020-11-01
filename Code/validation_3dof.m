clear all
close all
clc
addpath("functions/");

%% ----------------------
% GENERATE EXCITING TRAJECTORY
%------------------------
 
[position1, velocity1, acceleration1, time1] = load_trajectory('data/3-dof/new_trajectory7');
[position2, velocity2, acceleration2, time2] = load_trajectory('data/3-dof/new_trajectory8');
[position3, velocity3, acceleration3, time3] = load_trajectory('data/3-dof/new_trajectory9');

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
duration = period;
timestamps = 0:dt:duration;

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
    nominal_a = get_3dof_coefficients(10, 0, -0.15, 0, 4.167e-04*10, 1.125, -0.15, 0, -0.06, 4.167e-04*1.125, 7.708e-03*1.125, 7.708e-03*1.125, 0.75, -0.1, 0, 0, 4.167e-04*0.75, 3.542e-03*0.75, 3.542e-03*0.75);
    nominal_torque(:, j) = Y*nominal_a;
    
    Y_stack(j*3-2:j*3,:) = Y;
    u_stack(j*3-2:j*3,:) = nominal_torque(:, j);
end

load('results/3-dof/new_experiment123/results.mat', 'optimal_solution')

Y_stack_validation = zeros((length(timestamps))*3,10);
u_stack_validation = zeros((length(timestamps))*3,1);

for j=1:length(timestamps)
    Y = Y_matrix(pos1(j), pos2(j), pos3(j), vel1(j), vel2(j), vel3(j), acc1(j), acc2(j), acc3(j));
    validation_a = get_3dof_coefficients(optimal_solution(1),optimal_solution(4),optimal_solution(5),optimal_solution(6),optimal_solution(13),optimal_solution(2),optimal_solution(7),optimal_solution(8),optimal_solution(9),optimal_solution(14),optimal_solution(15),optimal_solution(16),optimal_solution(3),optimal_solution(10),optimal_solution(11),optimal_solution(12),optimal_solution(17),optimal_solution(18),optimal_solution(19));
    validation_torque(:, j) = Y*validation_a;
    
    Y_stack_validation(j*3-2:j*3,:) = Y;
    u_stack_validation(j*3-2:j*3,:) = validation_torque(:, j);
end

coefficients = nominal_a
validation_coefficients = validation_a
error = norm(coefficients-validation_coefficients)

figure('Name', 'Torques');
subplot(3,1,1);
plot(timestamps, nominal_torque(1, :), timestamps, validation_torque(1, :),timestamps, nominal_torque(1, :)-validation_torque(1, :));
legend ("nominal torque1", "validation torque1","error torque1");
xlabel("time (s)");
ylabel("torque (Nm)");

subplot(3,1,2);
plot(timestamps, nominal_torque(2, :), timestamps, validation_torque(2, :),timestamps, nominal_torque(2, :)-validation_torque(2, :));
legend ("nominal torque2", "validation torque2", "error torque2");
xlabel("time (s)");
ylabel("torque (Nm)");

subplot(3,1,3);
plot(timestamps, nominal_torque(3, :), timestamps, validation_torque(3, :),timestamps, nominal_torque(3, :)-validation_torque(3, :));
legend ("nominal torque3", "validation torque3", "error torque 3");
xlabel("time (s)");
ylabel("torque (Nm)");