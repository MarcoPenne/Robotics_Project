clear all
close all
clc
addpath("functions/");

%% ----------------------
% GENERATE EXCITING TRAJECTORY
%------------------------

% position_traj = [0 pi/4 -pi/4 0];
% time_traj = [0 5 15 20];
% [position, velocity, acceleration] = looping_splines(position_traj, time_traj);
% 
% period = 20;

[position, velocity, acceleration, time] = load_trajectory('data/1-dof/new_trajectory2');

[position, velocity, acceleration] = looping_splines(position, time);

period = time(length(time));

figure('Name', 'Trajectories');
subplot(3,1,1);
fplot(position, [0 period]);
xlabel("time (s)")
ylabel("position (rad)")
subplot(3,1,2);
fplot(velocity, [0 period]);
xlabel("time (s)")
ylabel("velocity (rad/s)")
subplot(3,1,3);
fplot(acceleration, [0 period]);
xlabel("time (s)")
ylabel("acceleration (rad/s^2)");

dt = 0.02;
duration = period;
timestamps = 0:dt:duration;

freq = 1/dt;

pos = position(timestamps);

vel = velocity(timestamps);

acc = acceleration(timestamps);

Y_stack = zeros((length(timestamps)),2);
u_stack = zeros((length(timestamps)),1);

for j=1:length(timestamps)
    Y = Y_matrix_1dof(pos(j), acc(j));
    nominal_a = get_1dof_coefficients(5,0.5,5*(8.417e-02));
    nominal_torque(j) = Y*nominal_a;
    
    Y_stack(j,:) = Y;
    u_stack(j,:) = nominal_torque(j);
end

load('results/1-dof/new_experiment_easy1/results.mat', 'optimal_solution')

Y_stack_validation = zeros((length(timestamps)),2);
u_stack_validation = zeros((length(timestamps)),1);

for j=1:length(timestamps)
    Y = Y_matrix_1dof(pos(j), acc(j));
    validation_a = get_1dof_coefficients(optimal_solution(1),optimal_solution(2),optimal_solution(3));
    validation_torque(j) = Y*validation_a;
    
    Y_stack_validation(j,:) = Y;
    u_stack_validation(j,:) = validation_torque(j);
end

coefficients = nominal_a
validation_coefficients = validation_a
error = norm(coefficients-validation_coefficients)

figure('Name', 'Torques');
subplot(1,1,1);
plot(timestamps, nominal_torque, timestamps, validation_torque, timestamps, nominal_torque-validation_torque);
legend ("nominal torque", "validation torque","error torque");
xlabel("time (s)");
ylabel("torque (Nm)");
