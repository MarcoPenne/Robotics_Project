clc
clear all
close all
addpath("functions/");

if ismac
  addpath("coppelia_files/mac")
elseif isunix
  addpath("coppelia_files/linux")
end

%%

sim=remApi('remoteApi');

sim.simxFinish(-1); % just in case, close all opened connections
clientID=sim.simxStart('127.0.0.1',19997,true,true,5000,5);

if (clientID>-1)
    disp('Connected to remote API server');
    [returnCode]=sim.simxLoadScene(clientID,'scenes/3-dof_torque_control.ttt',1,sim.simx_opmode_blocking)
    display(returnCode);
    
    % Now send some data to CoppeliaSim in a non-blocking fashion:
    sim.simxAddStatusbarMessage(clientID,'Hello CoppeliaSim!',sim.simx_opmode_oneshot);
    
    [returnCode,joint1] = sim.simxGetObjectHandle(clientID,'Joint1', sim.simx_opmode_blocking)
    [returnCode,joint2] = sim.simxGetObjectHandle(clientID,'Joint2', sim.simx_opmode_blocking)
    [returnCode,joint3] = sim.simxGetObjectHandle(clientID,'Joint3', sim.simx_opmode_blocking)
    [returnCode,link1] = sim.simxGetObjectHandle(clientID,'Link1', sim.simx_opmode_blocking)
    [returnCode,link2] = sim.simxGetObjectHandle(clientID,'Link2', sim.simx_opmode_blocking)
    [returnCode,link3] = sim.simxGetObjectHandle(clientID,'Link3', sim.simx_opmode_blocking)
    
    joints = [joint1,joint2,joint3]
    links = [link1, link2, link3]
else
    disp('Failed connecting to remote API server');
end
 
%%
% [x1, y1, z1] = looping_splines([0 pi/2 -pi/2 pi/2 0],[0 5 10 15 20]);
% [x2, y2, z2] = looping_splines([-pi/2 -pi/4 -pi/2],[0 10 20]);
% [x3, y3, z3] = looping_splines([0 pi/4 -pi/3 0],[0 5 15 20]);

% [x1, y1, z1] = looping_splines([0 pi/4 -pi/3 0],[0 5 15 20]);
% [x2, y2, z2] = looping_splines([-pi/2 -pi/4 -pi/2],[0 10 20]);
% [x3, y3, z3] = looping_splines([0 pi/4 -pi/3 0],[0 5 15 20]);

% [x1, y1, z1] = looping_splines([-pi/2 -pi/4 -pi/2],[0 10 20]);
% [x2, y2, z2] = looping_splines([-pi/2 -pi/4 -pi/2],[0 10 20]);
% [x3, y3, z3] = looping_splines([-pi/2 -pi/4 -pi/2],[0 10 20]);
 
[position1, velocity1, acceleration1, time1] = load_trajectory('data/3-dof/trajectory1');
[position2, velocity2, acceleration2, time2] = load_trajectory('data/3-dof/trajectory2');
[position3, velocity3, acceleration3, time3] = load_trajectory('data/3-dof/trajectory3');

time_tmp = [time1; time2; time3];
position_tmp = [position1; position2; position3];
velocity_tmp = [velocity1; velocity2; velocity3];
acceleration_tmp = [acceleration1; acceleration2; acceleration3];

[x1, y1, z1] = looping_splines(position_tmp(1,:), time_tmp(1,:));
[x2, y2, z2] = looping_splines(position_tmp(2,:), time_tmp(2,:));
[x3, y3, z3] = looping_splines(position_tmp(3,:), time_tmp(3,:));

period = double(lcm(sym([time_tmp(1, size(time_tmp, 2)), time_tmp(2, size(time_tmp, 2)), time_tmp(3, size(time_tmp, 2))])));
%period = 20;

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
   
%%

controller_gain = [0.01; 0.0001; 0.00001]; %[0.01; 0.01; 0.01];

Kp = [1; 0.5; 0.1].*controller_gain;
Kd = [1; 0.5; 0.2].*controller_gain;
Ki = [3; 3; 3].*controller_gain;

sim.simxStartSimulation(clientID, sim.simx_opmode_oneshot);

measured_pos = zeros(3, 500000);
measured_vel = zeros(3, 500000);
measured_1bdf = zeros(3, 500000);
reference_pos = zeros(3, 500000);
measured_torque = zeros(3, 500000);
time_axis = zeros(1, 500000);
queue = [0 0 0 0 0; 0 0 0 0 0; 0 0 0 0 0];
coeffs_BDF = [25/12 -4 +3 -4/3 1/4];

started = false;
finished = false;
starting_i = 1;
started = true;
finishing_i = 1;
integral = [0; 0; 0];
pause(5);

start = datetime('now');
t0 = start;
t1 = t0;
i = 1;
while seconds(datetime('now')-start) < period * 3
    
    t = seconds(datetime('now')-start);
    t1 = datetime('now');
    if t>period && ~started
       starting_i = i;
       started = true;
    end
    if t>period*2 && ~finished
        finishing_i = i-1;
        finished = true;
    end
    time_axis(i) = t;
    
    [returnCode,joint_position1]=sim.simxGetJointPosition(clientID,joints(1),sim.simx_opmode_streaming);
    if returnCode~=0
        joint_position1 = 0;
    end
    [returnCode,joint_position2]=sim.simxGetJointPosition(clientID,joints(2),sim.simx_opmode_streaming);
    if returnCode~=0
        joint_position2 = -pi/2;
    end
    [returnCode,joint_position3]=sim.simxGetJointPosition(clientID,joints(3),sim.simx_opmode_streaming);
    if returnCode~=0
        joint_position3 = 0;
    end
    
    measured_pos(:, i) = [joint_position1; joint_position2; joint_position3];
    q = [joint_position1; joint_position2; joint_position3];

    qd1 = position{1}(t);
    qd2 = position{2}(t);
    qd3 = position{3}(t);
    qd = [qd1; qd2; qd3];

    queue(:,2:5) = queue(:,1:4);
    queue(:,1) = q;

    v = (queue(:, 1)-queue(:, 2))/seconds(t1-t0); %
    %measured_vel(:, i) = (coeffs_BDF * queue')'/seconds(t1-t0);
    measured_vel(:, i) = v;
    
    dqd1 = velocity{1}(t);
    dqd2 = velocity{2}(t);
    dqd3 = velocity{3}(t);
    dqd = [dqd1; dqd2; dqd3];
    
    integral = integral + (qd - q)*seconds(t1-t0);
    
    feedforward = Y_matrix(position{1}(t), position{2}(t), position{3}(t), velocity{1}(t), velocity{2}(t), velocity{3}(t), acceleration{1}(t), acceleration{2}(t), acceleration{3}(t))* get_3dof_coefficients(10, 0, -0.15, 0, 4.167e-04*10, 1.125, -0.15, 0, -0.06, 4.167e-04*1.125, 7.708e-03*1.125, 7.708e-03*1.125, 0.75, -0.1, 0, 0, 4.167e-04*0.75, 3.542e-03*0.75, 3.542e-03*0.75); % * get_3dof_coefficients(8.2248, 0, -0.0743, 0, 0, 1.5018, -0.1174, 0, 0, 0, 0, 0, 0.3902, -0.0053, 0, 0, 0.0000, 0.0000, 0.0000);
    c = Kp.*(qd - q) + Kd.*(dqd - v); % + Ki.*integral;
    u = c + feedforward;

    % Sending commands
    sim.simxPauseCommunication(clientID,1);
    for j=1:3
        sim.simxSetJointTargetVelocity(clientID, joints(j), sign(u(j))*10e10, sim.simx_opmode_oneshot);
        sim.simxSetJointMaxForce(clientID, joints(j), abs(u(j)), sim.simx_opmode_oneshot);
    end
    sim.simxPauseCommunication(clientID,0);
    measured_torque(:, i) = double(u);
    
    reference_pos(:,i) = [position{1}(t); position{2}(t); position{3}(t)];
    reference_vel(:,i) = [velocity{1}(t); velocity{2}(t); velocity{3}(t)];
    reference_acc(:,i) = [acceleration{1}(t); acceleration{2}(t); acceleration{3}(t)];
    
    frame_period = seconds(datetime('now')-start)/i;
    i=i+1;
    t0 = t1;
    pause(0.02);
    
end

sim.simxStopSimulation(clientID, sim.simx_opmode_oneshot);
%%

figure('Name', 'Experiments');
subplot(3,3,1);
plot(time_axis(1, starting_i:finishing_i), reference_pos(1, starting_i:finishing_i), time_axis(1, starting_i:finishing_i), measured_pos(1, starting_i:finishing_i));
legend ("reference pos j1", "measured pos j1")

subplot(3,3,2);
plot(time_axis(1, starting_i:finishing_i), reference_pos(2, starting_i:finishing_i), time_axis(1, starting_i:finishing_i), measured_pos(2, starting_i:finishing_i));
legend ("reference pos j2", "measured pos j2")

subplot(3,3,3);
plot(time_axis(1, starting_i:finishing_i), reference_pos(3, starting_i:finishing_i), time_axis(1, starting_i:finishing_i), measured_pos(3, starting_i:finishing_i));
legend ("reference pos j3", "measured pos j3")

ORDER = 16;
FC_HIGH = 0.1*pi*5;  % Hz, used in low-pass and band-pass filters

filt = designfilt('lowpassiir', 'FilterOrder', ORDER, 'HalfPowerFrequency', FC_HIGH, 'SampleRate', 1/frame_period);

estimated_velocity = [gradient(measured_pos(1, 1:i-1),frame_period); gradient(measured_pos(2, 1:i-1),frame_period); gradient(measured_pos(3, 1:i-1),frame_period);];

filtered_vel = zeros(3, i-1);
filtered_vel(1, :) = filtfilt(filt, estimated_velocity(1,:));
filtered_vel(2, :) = filtfilt(filt, estimated_velocity(2,:));
filtered_vel(3, :) = filtfilt(filt, estimated_velocity(3,:));

subplot(3,3,4);
plot(time_axis(1, starting_i:finishing_i), reference_vel(1, starting_i:finishing_i), time_axis(1, starting_i:finishing_i), filtered_vel(1, starting_i:finishing_i));
legend ("reference vel j1", "filtered velocity j1")

subplot(3,3,5);
plot(time_axis(1, starting_i:finishing_i), reference_vel(2, starting_i:finishing_i), time_axis(1, starting_i:finishing_i), filtered_vel(2, starting_i:finishing_i));
legend ("reference vel j2", "filtered velocity j2")

subplot(3,3,6);
plot(time_axis(1, starting_i:finishing_i), reference_vel(3, starting_i:finishing_i), time_axis(1, starting_i:finishing_i), filtered_vel(3, starting_i:finishing_i));
legend ("reference vel j3", "filtered velocity j3");

FC_HIGH = 0.3*pi;  % Hz, used in low-pass and band-pass filters

filt_acc = designfilt('lowpassiir', 'FilterOrder', ORDER, 'HalfPowerFrequency', FC_HIGH, 'SampleRate', 1/frame_period);

estimated_acceleration = [gradient(gradient(measured_pos(1, 1:i-1),frame_period),frame_period); gradient(gradient(measured_pos(2, 1:i-1),frame_period),frame_period); gradient(gradient(measured_pos(3, 1:i-1),frame_period),frame_period)];

filtered_acc = zeros(3, i-1);
filtered_acc(1, :) = filtfilt(filt_acc, estimated_acceleration(1,:));
filtered_acc(2, :) = filtfilt(filt_acc, estimated_acceleration(2,:));
filtered_acc(3, :) = filtfilt(filt_acc, estimated_acceleration(3,:));

subplot(3,3,7);
plot(time_axis(1, starting_i:finishing_i), reference_acc(1, starting_i:finishing_i), time_axis(1, starting_i:finishing_i), filtered_acc(1, starting_i:finishing_i));
legend ("reference acc j1", "filtered acceleration j1")

subplot(3,3,8);
plot(time_axis(1, starting_i:finishing_i), reference_acc(2, starting_i:finishing_i), time_axis(1, starting_i:finishing_i), filtered_acc(2, starting_i:finishing_i));
legend ("reference acc j2", "filtered acceleration j2")

subplot(3,3,9);
plot(time_axis(1, starting_i:finishing_i), reference_acc(3, starting_i:finishing_i), time_axis(1, starting_i:finishing_i), filtered_acc(3, starting_i:finishing_i));
legend ("reference vel j3", "filtered acceleration j3");

figure()
subplot(3,1,1);
plot(time_axis(1, starting_i:finishing_i), reference_vel(1, starting_i:finishing_i), time_axis(1, starting_i:finishing_i), filtered_vel(1, starting_i:finishing_i), time_axis(1, starting_i:finishing_i), measured_vel(1, starting_i:finishing_i), time_axis(1, starting_i:finishing_i), measured_1bdf(1, starting_i:finishing_i));
legend ("reference vel j1", "filtered velocity j1", "4step", "1step")

subplot(3,1,2);
plot(time_axis(1, starting_i:finishing_i), reference_vel(2, starting_i:finishing_i), time_axis(1, starting_i:finishing_i), filtered_vel(2, starting_i:finishing_i), time_axis(1, starting_i:finishing_i), measured_vel(2, starting_i:finishing_i), time_axis(1, starting_i:finishing_i), measured_1bdf(2, starting_i:finishing_i));
legend ("reference vel j2", "filtered velocity j2", "4step", "1step")

subplot(3,1,3);
plot(time_axis(1, starting_i:finishing_i), reference_vel(3, starting_i:finishing_i), time_axis(1, starting_i:finishing_i), filtered_vel(3, starting_i:finishing_i), time_axis(1, starting_i:finishing_i), measured_vel(3, starting_i:finishing_i), time_axis(1, starting_i:finishing_i), measured_1bdf(3, starting_i:finishing_i));
legend ("reference vel j3", "filtered velocity j3", "4step", "1step");


for j=1:i-1
    Y = Y_matrix(reference_pos(1, j), reference_pos(2, j), reference_pos(3, j), reference_vel(1, j), reference_vel(2, j), reference_vel(3, j), reference_acc(1, j), reference_acc(2, j), reference_acc(3, j));
    a = get_3dof_coefficients(10, 0, 0.15, 0, 4.167e-04*10, 1.125, -0.15, 0, -0.06, 4.167e-04*1.125, 7.708e-03*1.125, 7.708e-03*1.125, 0.75, -0.1, 0, 0, 4.167e-04*0.75, 3.542e-03*0.75, 3.542e-03*0.75);
    reference_torque(:, j) = Y*a;
end

FC_HIGH2 = 0.25*pi;  % Hz, used in low-pass and band-pass filters

filt2 = designfilt('lowpassiir', 'FilterOrder', ORDER, 'HalfPowerFrequency', FC_HIGH2, 'SampleRate', 1/frame_period);

filtered_torque = zeros(3, length(measured_torque));
filtered_torque(1, :) = filtfilt(filt2, measured_torque(1,:));
filtered_torque(2, :) = filtfilt(filt, measured_torque(2,:));
filtered_torque(3, :) = filtfilt(filt, measured_torque(3,:));

figure('Name', 'Torques');
subplot(3,1,1);
plot(time_axis(1, starting_i:finishing_i), reference_torque(1, starting_i:finishing_i), time_axis(1, starting_i:finishing_i), filtered_torque(1, starting_i:finishing_i));
legend ("reference torque1", "filtered torque1")

subplot(3,1,2);
plot(time_axis(1, starting_i:finishing_i), reference_torque(2, starting_i:finishing_i), time_axis(1, starting_i:finishing_i), filtered_torque(2, starting_i:finishing_i));
legend ("reference torque2", "filtered torque2")

subplot(3,1,3);
plot(time_axis(1, starting_i:finishing_i), reference_torque(3, starting_i:finishing_i), time_axis(1, starting_i:finishing_i), filtered_torque(3, starting_i:finishing_i));
legend ("reference torque3", "filtered torque3");

Y_stack = zeros((finishing_i-starting_i+1)*3,13);
u_stack = zeros((finishing_i-starting_i+1)*3,1);

j = 1;
for k=starting_i:finishing_i
    Y_stack(j*3-2:j*3,:) = Y_matrix(measured_pos(1,k),measured_pos(2,k),measured_pos(3,k),filtered_vel(1,k),filtered_vel(2,k),filtered_vel(3,k),filtered_acc(1,k),filtered_acc(2,k),filtered_acc(3,k));
    u_stack(j*3-2:j*3,:) = filtered_torque(:,k);
    j = j+1;
end

coefficients = pinv(Y_stack)*u_stack;
error = norm(coefficients-a)

experiment_path = 'data/3-dof/experiment4';
save(fullfile(experiment_path,'Y_stack.mat'), 'Y_stack');
save(fullfile(experiment_path,'u_stack.mat'), 'u_stack');