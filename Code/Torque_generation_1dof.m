clear all
close all
clc
addpath("functions/");

%% ----------------------
% GENERATE EXCITING TRAJECTORY
%------------------------

position_traj = [0 pi/4 -pi/4 0];
time_traj = [0 5 15 20];
[position, velocity, acceleration] = looping_splines(position_traj, time_traj);

period = 20;

% [position, velocity, acceleration, time] = load_trajectory('data/1-dof/new_trajectory1');
% 
% [position, velocity, acceleration] = looping_splines(position, time);
% 
% period = time(length(time));

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
    a = get_1dof_coefficients(5,0.5,5*(8.417e-02));
    torque(j) = Y*a;
    
    Y_stack(j,:) = Y;
    u_stack(j,:) = torque(j);
end

figure('Name', 'Torques');
plot(timestamps, torque);
xlabel("time (s)");
ylabel("torque (Nm)");

a
coefficients = pinv(Y_stack)*u_stack
error = norm(coefficients-a)

experiment_path = 'data/1-dof/new_experiment_easy2';
save(fullfile(experiment_path,'Y_stack.mat'), 'Y_stack');
save(fullfile(experiment_path,'u_stack.mat'), 'u_stack');
save(fullfile(experiment_path,'duration.mat'), 'duration');