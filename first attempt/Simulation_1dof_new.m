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
    [returnCode]=sim.simxLoadScene(clientID,'scenes/1-dof_position_control.ttt',1,sim.simx_opmode_blocking)
    display(returnCode);
    
    % Now send some data to CoppeliaSim in a non-blocking fashion:
    sim.simxAddStatusbarMessage(clientID,'Hello CoppeliaSim!',sim.simx_opmode_oneshot);
    
    [returnCode,joint] = sim.simxGetObjectHandle(clientID,'Joint', sim.simx_opmode_blocking);
else
    disp('Failed connecting to remote API server');
end

% position_traj = [0 pi/4 0];
% time_traj = [0 10 20];
% [position, velocity, acceleration] = looping_splines(position_traj, time_traj);

% period = 20;

trajectory_path = 'data/1-dof/trajectory1';
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
reference_pos = zeros(1, 500000);
measured_torque = zeros(1, 500000);
time_axis = zeros(1, 500000);
pause(5);

start = datetime('now');

i = 1;
while seconds(datetime('now')-start) < period
    
    t = seconds(datetime('now')-start);
    time_axis(i) = t;
    
    [returnCode,joint_position]=sim.simxGetJointPosition(clientID,joint,sim.simx_opmode_blocking);
    measured_pos(i) = [joint_position];
    
    % Sending commands
    sim.simxPauseCommunication(clientID,1);
    sim.simxSetJointTargetPosition(clientID, joint, position(t), sim.simx_opmode_streaming);
    sim.simxPauseCommunication(clientID,0);
    reference_pos(i) = [position(t)];
    reference_vel(i) = [velocity(t)];
    reference_acc(i) = [acceleration(t)];
    
    [returnCode,u]=sim.simxGetJointForce(clientID,joint,sim.simx_opmode_blocking);
    measured_torque(i) = [-u];
    
    frame_period = seconds(datetime('now')-start)/i;
    i=i+1;
    
end

sim.simxStopSimulation(clientID, sim.simx_opmode_oneshot);

figure('Name', 'Experiments');
subplot(3,1,1);
plot(time_axis(1, 1:i-1), reference_pos(1, 1:i-1), time_axis(1, 1:i-1), measured_pos(1, 1:i-1));
legend ("reference pos", "measured pos")

estimated_velocity = [gradient(measured_pos(1, 1:i-1),frame_period)];

subplot(3,1,2);
plot(time_axis(1, 1:i-1), reference_vel(1, 1:i-1), time_axis(1, 1:i-1), estimated_velocity(1, 1:i-1));
legend ("reference vel", "velocity")

estimated_acceleration = [gradient(gradient(measured_pos(1, 1:i-1),frame_period),frame_period)];

subplot(3,1,3);
plot(time_axis(1, 1:i-1), reference_acc(1, 1:i-1), time_axis(1, 1:i-1), estimated_acceleration(1, 1:i-1));
legend ("reference acc", "acceleration")

for j=1:i-1
    Y = Y_matrix_1dof(reference_pos(j), reference_acc(j));
    a = get_1dof_coefficients(5,0.5,5*(8.417e-02));
    reference_torque(j) = Y*a;
end

figure('Name', 'Torques');
subplot(1,1,1);
plot(time_axis(1, 1:i-1), measured_torque(1, 1:i-1), time_axis(1, 1:i-1), reference_torque(1, 1:i-1));
legend ("torque", "reference torque")
