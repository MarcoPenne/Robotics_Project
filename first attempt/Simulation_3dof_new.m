clear all
addpath("functions/");

if ismac
  addpath("coppelia_files/mac")
elseif isunix
  addpath("coppelia_files/linux")
end

sim=remApi('remoteApi');

sim.simxFinish(-1); % just in case, close all opened connections
clientID=sim.simxStart('127.0.0.1',19997,true,true,5000,5);

if (clientID>-1)
    disp('Connected to remote API server');
    [returnCode]=sim.simxLoadScene(clientID,'scenes/3-dof_position_control.ttt',1,sim.simx_opmode_blocking)
    display(returnCode);
    
    % Now send some data to CoppeliaSim in a non-blocking fashion:
    sim.simxAddStatusbarMessage(clientID,'Hello CoppeliaSim!',sim.simx_opmode_oneshot);
    
    [returnCode,joint1] = sim.simxGetObjectHandle(clientID,'Joint1', sim.simx_opmode_blocking)
    [returnCode,joint2] = sim.simxGetObjectHandle(clientID,'Joint2', sim.simx_opmode_blocking)
    [returnCode,joint3] = sim.simxGetObjectHandle(clientID,'Joint3', sim.simx_opmode_blocking)
    
    joints = [joint1,joint2,joint3]
    
else
    disp('Failed connecting to remote API server');
end

[x1, y1, z1] = looping_splines([0 pi/2 0 -pi/2 0],[0 5 10 15 20]);
[x2, y2, z2] = looping_splines([-pi/2 -pi/4 -pi/2],[0 10 20]);
[x3, y3, z3] = looping_splines([0 pi/4 -pi/3 0],[0 5 15 20]);

% [x1, y1, z1] = looping_splines([-pi/2 -pi/4 -pi/2],[0 10 20]);
% [x2, y2, z2] = looping_splines([-pi/2 -pi/4 -pi/2],[0 10 20]);
% [x3, y3, z3] = looping_splines([-pi/2 -pi/4 -pi/2],[0 10 20]);
% 

%period = double(lcm(sym([time_tmp(1, size(time_tmp, 2)), time_tmp(2, size(time_tmp, 2)), time_tmp(3, size(time_tmp, 2))])));
period = 20;

position = {x1, x2, x3};
velocity = {y1, y2, y3};
acceleration = {z1, z2, z3};

figure('Name', 'Trajectories');
for i=1:3
    subplot(3,3,i);
    fplot(position(i), [0 period]);
    subplot(3,3,i+3);
    fplot(velocity(i), [0 period]);
    subplot(3,3,i+6);
    fplot(acceleration(i), [0 period]);
end
    
sim.simxStartSimulation(clientID, sim.simx_opmode_oneshot);

measured_pos = zeros(3, 500000);
reference_pos = zeros(3, 500000);
measured_torque = zeros(3, 500000);
time_axis = zeros(1, 500000);
pause(5);

start = datetime('now');

i = 1;
while seconds(datetime('now')-start) < period * 3
    
    t = seconds(datetime('now')-start);
    time_axis(i) = t;
    
    [returnCode,joint_position1]=sim.simxGetJointPosition(clientID,joints(1),sim.simx_opmode_blocking);
    [returnCode,joint_position2]=sim.simxGetJointPosition(clientID,joints(2),sim.simx_opmode_blocking);
    [returnCode,joint_position3]=sim.simxGetJointPosition(clientID,joints(3),sim.simx_opmode_blocking);
    measured_pos(:, i) = [joint_position1; joint_position2; joint_position3];
    
    % Sending commands
    sim.simxPauseCommunication(clientID,1);
    sim.simxSetJointTargetPosition(clientID, joints(1), position{1}(t), sim.simx_opmode_streaming);
    sim.simxSetJointTargetPosition(clientID, joints(2), position{2}(t), sim.simx_opmode_streaming);
    sim.simxSetJointTargetPosition(clientID, joints(3), position{3}(t), sim.simx_opmode_streaming);
    sim.simxPauseCommunication(clientID,0);
    reference_pos(:,i) = [position{1}(t); position{2}(t); position{3}(t)];
    reference_vel(:,i) = [velocity{1}(t); velocity{2}(t); velocity{3}(t)];
    reference_acc(:,i) = [acceleration{1}(t); acceleration{2}(t); acceleration{3}(t)];
    
    [returnCode,u1]=sim.simxGetJointForce(clientID,joints(1),sim.simx_opmode_blocking);
    [returnCode,u2]=sim.simxGetJointForce(clientID,joints(2),sim.simx_opmode_blocking);
    [returnCode,u3]=sim.simxGetJointForce(clientID,joints(3),sim.simx_opmode_blocking);
    measured_torque(:, i) = [-u1; -u2; -u3];
    
    frame_period = seconds(datetime('now')-start)/i;
    i=i+1;
    
end

sim.simxStopSimulation(clientID, sim.simx_opmode_oneshot);

figure('Name', 'Experiments');
subplot(3,3,1);
plot(time_axis(1, 1:i-1), reference_pos(1, 1:i-1), time_axis(1, 1:i-1), measured_pos(1, 1:i-1));
legend ("reference pos j1", "measured pos j1")

subplot(3,3,2);
plot(time_axis(1, 1:i-1), reference_pos(2, 1:i-1), time_axis(1, 1:i-1), measured_pos(2, 1:i-1));
legend ("reference pos j2", "measured pos j2")

subplot(3,3,3);
plot(time_axis(1, 1:i-1), reference_pos(3, 1:i-1), time_axis(1, 1:i-1), measured_pos(3, 1:i-1));
legend ("reference pos j3", "measured pos j3")

estimated_velocity = [gradient(measured_pos(1, 1:i-1),frame_period); gradient(measured_pos(2, 1:i-1),frame_period); gradient(measured_pos(3, 1:i-1),frame_period);];

subplot(3,3,4);
plot(time_axis(1, 1:i-1), reference_vel(1, 1:i-1), time_axis(1, 1:i-1), estimated_velocity(1, 1:i-1));
legend ("reference vel j1", "velocity j1")

subplot(3,3,5);
plot(time_axis(1, 1:i-1), reference_vel(2, 1:i-1), time_axis(1, 1:i-1), estimated_velocity(2, 1:i-1));
legend ("reference vel j2", "velocity j2")

subplot(3,3,6);
plot(time_axis(1, 1:i-1), reference_vel(3, 1:i-1), time_axis(1, 1:i-1), estimated_velocity(3, 1:i-1));
legend ("reference vel j3", "velocity j3");

estimated_acceleration = [gradient(gradient(measured_pos(1, 1:i-1),frame_period),frame_period); gradient(gradient(measured_pos(2, 1:i-1),frame_period),frame_period); gradient(gradient(measured_pos(3, 1:i-1),frame_period),frame_period);];

subplot(3,3,7);
plot(time_axis(1, 1:i-1), reference_acc(1, 1:i-1), time_axis(1, 1:i-1), estimated_acceleration(1, 1:i-1));
legend ("reference acc j1", "acceleration j1")

subplot(3,3,8);
plot(time_axis(1, 1:i-1), reference_acc(2, 1:i-1), time_axis(1, 1:i-1), estimated_acceleration(2, 1:i-1));
legend ("reference acc j2", "acceleration j2")

subplot(3,3,9);
plot(time_axis(1, 1:i-1), reference_acc(3, 1:i-1), time_axis(1, 1:i-1), estimated_acceleration(3, 1:i-1));
legend ("reference vel j3", "acceleration j3");

for j=1:i-1
    Y = Y_matrix(reference_pos(1, j), reference_pos(2, j), reference_pos(3, j), reference_vel(1, j), reference_vel(2, j), reference_vel(3, j), reference_acc(1, j), reference_acc(2, j), reference_acc(3, j));
    a = get_3dof_coefficients(1.5, 0, -0.15, 0, 4.167e-04*1.5, 1.125, -0.15, 0, -0.06, 4.167e-04*1.125, 7.708e-03*1.125, 7.708e-03*1.125, 0.75, -0.1, 0, 0, 4.167e-04*0.75, 3.542e-03*0.75, 3.542e-03*0.75);
    reference_torque(:, j) = Y*a;
end

figure('Name', 'Torques');
subplot(3,1,1);
plot(time_axis(1, 1:i-1), measured_torque(1, 1:i-1), time_axis(1, 1:i-1), reference_torque(1, 1:i-1));
legend ("torque j1", "reference torque1")

subplot(3,1,2);
plot(time_axis(1, 1:i-1), measured_torque(2, 1:i-1), time_axis(1, 1:i-1), reference_torque(2, 1:i-1));
legend ("torque j2", "reference torque2")

subplot(3,1,3);
plot(time_axis(1, 1:i-1), measured_torque(3, 1:i-1), time_axis(1, 1:i-1), reference_torque(3, 1:i-1));
legend ("torque j3", "reference torque3");

