clc
close all
clear all
addpath("functions/");

if ismac
  addpath("coppelia_files/mac")
elseif isunix
  addpath("coppelia_files/linux")
end

disp('Program started');
% sim=remApi('remoteApi','extApi.h'); % using the header (requires a compiler)
sim=remApi('remoteApi'); % using the prototype file (remoteApiProto.m)
sim.simxFinish(-1); % just in case, close all opened connections
clientID=sim.simxStart('127.0.0.1',19997,true,true,5000,5);

if (clientID>-1)
    disp('Connected to remote API server');
    [returnCode]=sim.simxLoadScene(clientID,'scenes/1-dof_position_control.ttt',1,sim.simx_opmode_blocking);
    
    % Now send some data to CoppeliaSim in a non-blocking fashion:
    sim.simxAddStatusbarMessage(clientID,'Hello CoppeliaSim!',sim.simx_opmode_oneshot);
    
    [returnCode,joint] = sim.simxGetObjectHandle(clientID,'Joint', sim.simx_opmode_blocking);
    [returnCode,link] = sim.simxGetObjectHandle(clientID,'Link1', sim.simx_opmode_blocking);
else
    disp('Failed connecting to remote API server');
end

% position_traj = [0 pi/4 0];
% time_traj = [0 10 20];
% [position, velocity, acceleration] = looping_splines(position_traj, time_traj);
% 
% period = 20;

trajectory_path = 'data/1-dof/trajectory3';
[position, velocity, acceleration, time] = load_trajectory(trajectory_path);
[position, velocity, acceleration] = looping_splines(position, time);

period = time(length(time));

figure('Name', 'Trajectories');
subplot(3,1,1);
fplot(position, [0 period]);
subplot(3,1,2);
fplot(velocity, [0 period]);
subplot(3,1,3);
fplot(acceleration, [0 period]);

sim.simxStartSimulation(clientID, sim.simx_opmode_oneshot);

measured_pos = zeros(1, 500000);
measured_vel = zeros(1, 500000);
reference_pos = zeros(1, 500000);
measured_torque = zeros(1, 500000);
time_axis = zeros(1, 500000);

started = false;
finished = false;
starting_i = 1;
finishing_i = 1;
pause(5);

start = datetime('now');

i = 1;
while seconds(datetime('now')-start) < period * 5
    
    t = seconds(datetime('now')-start);
    if t>period && ~started
       starting_i = i;
       started = true;
    end
    if t>period*4 && ~finished
        finishing_i = i-1;
        finished = true;
    end
    time_axis(i) = t;
    
    [returnCode,joint_position]=sim.simxGetJointPosition(clientID,joint,sim.simx_opmode_streaming);
    if returnCode==0
        measured_pos(i) = joint_position;
    else
        measured_pos(i) = 0;
    end
    [returnCode, linear, angular] = sim.simxGetObjectVelocity(clientID, link, sim.simx_opmode_streaming);
    measured_vel(i) = angular(1);
    
    [returnCode,u]=sim.simxGetJointForce(clientID,joint,sim.simx_opmode_streaming);
    measured_torque(i) = [-u];
    
    % Sending commands
    sim.simxPauseCommunication(clientID,1);
    sim.simxSetJointTargetPosition(clientID, joint, position(t), sim.simx_opmode_streaming);
    sim.simxPauseCommunication(clientID,0);
    reference_pos(i) = [position(t)];
    reference_vel(i) = [velocity(t)];
    reference_acc(i) = [acceleration(t)];
    
    frame_period = seconds(datetime('now')-start)/i;
    i=i+1;
    pause(0.02);
end

sim.simxStopSimulation(clientID, sim.simx_opmode_oneshot);

figure('Name', 'Experiments');
subplot(3,1,1);
plot(time_axis(1, starting_i:finishing_i), reference_pos(1, starting_i:finishing_i), time_axis(1, starting_i:finishing_i), measured_pos(1, starting_i:finishing_i));
legend ("reference pos", "measured pos")

estimated_velocity = [gradient(measured_pos(1, 1:i-1),frame_period)];

ORDER = 16;
FC_HIGH = 0.35*pi;%0.1*pi*5;  % Hz, used in low-pass and band-pass filters

filt = designfilt('lowpassiir', 'FilterOrder', ORDER, 'HalfPowerFrequency', FC_HIGH, 'SampleRate', 1/frame_period);

filtered_vel = filtfilt(filt, measured_vel);

subplot(3,1,2);
plot(time_axis(1, starting_i:finishing_i), reference_vel(1, starting_i:finishing_i), time_axis(1, starting_i:finishing_i), filtered_vel(1, starting_i:finishing_i));
legend ("reference vel", "filtered velocity")

estimated_acceleration = [gradient(filtered_vel,frame_period)];

filtered_acc = filtfilt(filt, estimated_acceleration);

subplot(3,1,3);
plot(time_axis(1, starting_i:finishing_i), reference_acc(1, starting_i:finishing_i), time_axis(1, starting_i:finishing_i), filtered_acc(1, starting_i:finishing_i));
legend ("reference acc", "filtered acceleration")

for j=1:i-1
    Y = Y_matrix_1dof(reference_pos(j), reference_acc(j));
    a = get_1dof_coefficients(5,0.5,5*(8.417e-02));
    reference_torque(j) = Y*a;
end

filtered_torque = filtfilt(filt, measured_torque);

figure('Name', 'Torques');
subplot(1,1,1);
plot(time_axis(1, starting_i:finishing_i), reference_torque(1, starting_i:finishing_i), time_axis(1, starting_i:finishing_i), filtered_torque(1, starting_i:finishing_i));
legend ("reference torque", "filtered torque")

Y_1dof = zeros(finishing_i-starting_i+1,2);
u_1dof = zeros(finishing_i-starting_i+1,1);

j = 1;
for k=starting_i:finishing_i
    Y_1dof(j, :) = Y_matrix_1dof(measured_pos(k), filtered_acc(k));
    u_1dof(j,:) = filtered_torque(k);
    j = j+1;
end

coefficients = pinv(Y_1dof)*u_1dof;
error = norm(coefficients-a)

experiment_path = 'data/1-dof/new_experiment5';
save(fullfile(experiment_path,'Y_1dof.mat'), 'Y_1dof');
save(fullfile(experiment_path,'u_1dof.mat'), 'u_1dof');

disp('Finish!');