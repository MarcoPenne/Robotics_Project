close all
clc
clear all

addpath('functions');

num_of_joints = 3; % DoFs of the robot

% load regressor matrix Y and vector of stacked torques
load('data/3-dof/new_experiment123/Y_stack.mat', 'Y_stack')
load('data/3-dof/new_experiment123/u_stack.mat', 'u_stack')
load('data/3-dof/new_experiment123/duration.mat', 'duration')

% take only the absolute value of the torques
u_3dof_abs = abs(u_stack);

num_of_samples = size(Y_stack,1)/num_of_joints;

% ---------------------------
% read lower and upper bounds
% ---------------------------
[LB,UB] = read_bounds('data/3-dof/bounds/bound_3dof.csv');


u1_3dof_abs = u_3dof_abs(1:3:3*num_of_samples);
u2_3dof_abs = u_3dof_abs(2:3:3*num_of_samples);
u3_3dof_abs = u_3dof_abs(3:3:3*num_of_samples);

Y1_stack = Y_stack(1:3:3*num_of_samples, :);
Y2_stack = Y_stack(2:3:3*num_of_samples, :);
Y3_stack = Y_stack(3:3:3*num_of_samples, :);

% computing the 10% of min abs values
sorted_u1 = sort(u1_3dof_abs);
threshold1 = sorted_u1(int32(num_of_samples*0.1)+1);

sorted_u2 = sort(u2_3dof_abs);
threshold2 = sorted_u2(int32(num_of_samples*0.1)+1);

sorted_u3 = sort(u3_3dof_abs);
threshold3 = sorted_u3(int32(num_of_samples*0.1)+1);

threshold = [threshold1, threshold2, threshold3];

% finding the segments with constant sign torque 
% indices contains the start and the finish of each segment
indices1 = change_of_sign(u1_3dof_abs, threshold(1));
indices2 = change_of_sign(u2_3dof_abs, threshold(2));
indices3 = change_of_sign(u3_3dof_abs, threshold(3));

indices_long1 = [];
indices_long2 = [];
indices_long3 = [];

for i=1:size(indices1, 1)
    if indices1(i, 2)-indices1(i, 1)>=20
        indices_long1 = [indices_long1; indices1(i, :)];
    end
end
for i=1:size(indices2, 1)
    if indices2(i, 2)-indices2(i, 1)>=20
        indices_long2 = [indices_long2; indices2(i, :)];
    end
end
for i=1:size(indices3, 1)
    if indices3(i, 2)-indices3(i, 1)>=20
        indices_long3 = [indices_long3; indices3(i, :)];
    end
end

indices1 = indices_long1;
indices2 = indices_long2;
indices3 = indices_long3;

%-----------------------------------
% BUILD THE TREE
%-----------------------------------
% Let the tree PBRP algorithm estimate the signs for each segment annording
% to the indices. It will return:
% - a regressor matrix: Y_final, with rows in correct order
% - a vector: u_final, with estimated sign and elements in correct order 
% - a vector: signs, with the estimated sign for each segment.

[Y_final, u_final, signs] = tree_3dof(Y1_stack, Y2_stack, Y3_stack, u1_3dof_abs, u2_3dof_abs, u3_3dof_abs, indices1, indices2, indices3, LB, UB);

%----------------------------------
% RUN THE TRADITIONAL PBRP ALGORITHM
%----------------------------------
num_of_runs = 3; % independent (parallelizable) runs of the entire algorithm (29)
num_of_SA_steps = 3; % successive runs of the algorithm (29). It is the variable \kappa in (29)

num_x = length(LB); % number of parameters

% LOSSES vector contains the values of the loss functions for each
% independent run in num_of_runs. They are initialized to +infinity
LOSSES = Inf*ones(num_of_runs,1);

% SOL vector contains the solution (that is, the estimated values of the
% dynamic parameters) for each independent run in num_of_runs
SOL = zeros(num_x,num_of_runs);

% OUTPUTS and EXITFLAGS are variables related to 'simulannealbnd' Matlab
% function
OUTPUTS = cell(num_of_runs,1);
EXITFLAGS = zeros(num_of_runs,1);

%----------------------------------
% optimizator: Simulated Annealing
%----------------------------------

for i=1:num_of_runs
    for SA_step=1:num_of_SA_steps
        stringtodisp = sprintf('RUN %d of %d',i,num_of_runs);
        disp(stringtodisp);
        if SA_step==1
            X0 = rand(num_x,1).*(UB-LB) + LB; % random initial point inside bounds
        end
        
        options = saoptimset('HybridFcn',{@fmincon}); % use Nelder-Mead optimization as hybrid function

        [X,FVAL,EXITFLAG,OUTPUT] = simulannealbnd(@(x) error_fcn_gM_LMI_regressor_3dof(x, Y_final, u_final, SA_step),X0,LB,UB,options);

        X0 = X;
    end
    disp('---------------------------');
    disp(sprintf('.........LOSS = %f',FVAL));
    disp('---------------------------');
    LOSSES(i) = FVAL;
    SOL(:,i) = X;
    OUTPUTS{i} = OUTPUT;
    EXITFLAGS(i) = EXITFLAG;
  
end

% retrieve optimal solution
min_idx = find(LOSSES==min(LOSSES));
optimal_solution = SOL(:,min_idx);


%----------------------------------
% Self-validation
%----------------------------------

m1 = optimal_solution(1);
m2 = optimal_solution(2);
m3 = optimal_solution(3);
rc1x = optimal_solution(4);
rc1y = optimal_solution(5);
rc1z = optimal_solution(6);
rc2x = optimal_solution(7);
rc2y = optimal_solution(8);
rc2z = optimal_solution(9);
rc3x = optimal_solution(10);
rc3y = optimal_solution(11);
rc3z = optimal_solution(12);
I1yy = optimal_solution(13);
I2xx = optimal_solution(14);
I2yy = optimal_solution(15);
I2zz = optimal_solution(16);
I3xx = optimal_solution(17);
I3yy = optimal_solution(18);
I3zz = optimal_solution(19);

% compute coefficients vector with the estimated dynamic parameters
P_li_expanded_eval = get_3dof_coefficients(m1,rc1x,rc1y,rc1z,I1yy,m2,rc2x,rc2y,rc2z,I2xx,I2yy,I2zz,m3,rc3x,rc3y,rc3z,I3xx,I3yy,I3zz);

% joint torques estimation
tau_stack_estimation1 = Y1_stack*P_li_expanded_eval;
tau_stack_estimation2 = Y2_stack*P_li_expanded_eval;
tau_stack_estimation3 = Y3_stack*P_li_expanded_eval;

u1_stack = u_stack(1:3:3*num_of_samples);
u2_stack = u_stack(2:3:3*num_of_samples);
u3_stack = u_stack(3:3:3*num_of_samples);

tau_stack_estimation = [tau_stack_estimation1 tau_stack_estimation2 tau_stack_estimation3];
u_stack_final = [u1_stack u2_stack u3_stack];

% reshape tau_stack vectors (measured and estimated)
TAU_TRUE = zeros(num_of_joints,num_of_samples);
TAU_ESTD = zeros(num_of_joints,num_of_samples);
for i = 1 : num_of_samples
    for j = 1:num_of_joints
        TAU_TRUE(j,i) = u_stack_final(i, j);
        TAU_ESTD(j,i) = tau_stack_estimation(i, j);
    end
end

indices = {indices1, indices2, indices3};

% plot validation results
figure
dt = 0.02;
samples = 0:dt:duration;
for i=1:num_of_joints
    subplot(3,1,i);
    hold on
    plot(samples,TAU_TRUE(i,:),samples,TAU_ESTD(i,:));
    for seg=1:size(indices{i},1)
        xline(indices{i}(seg, 1)*dt, '--g');
        xline(indices{i}(seg, 2)*dt, '--r');
        if signs{i}(seg)>0
            lab = '+';
        else
            lab = '-';
        end
        %lab = 'o';
        text(0.5*indices{i}(seg, 2)*dt + 0.5*indices{i}(seg, 1)*dt, 0, lab, 'HorizontalAlignment', 'center');
    end
    yline(threshold(i), '--o');
    yline(-threshold(i), '--o');
    hold off
    grid;
    xlabel('time [s]');
    ylabel('torque [Nm]');
    if i==num_of_joints
        legend('true','estimated');
    end
end

estimated_coefficients = get_3dof_coefficients(m1,rc1x,rc1y,rc1z,I1yy,m2,rc2x,rc2y,rc2z,I2xx,I2yy,I2zz,m3,rc3x,rc3y,rc3z,I3xx,I3yy,I3zz);
ground_coefficients = get_3dof_coefficients(10, 0, -0.15, 0, 4.167e-04*10, 1.125, -0.15, 0, -0.06, 4.167e-04*1.125, 7.708e-03*1.125, 7.708e-03*1.125, 0.75, -0.1, 0, 0, 4.167e-04*0.75, 3.542e-03*0.75, 3.542e-03*0.75);

error_measure = norm(estimated_coefficients - ground_coefficients);

disp('The estimated dynamic coefficients w/o torque signs are:')
disp(estimated_coefficients)
disp('The ground values of the dynamic coefficients are:')
disp(ground_coefficients)
disp('The norm of the difference between estimated dynamic coefficients w/o torque signs and ground values is:')
disp(error_measure)

u1_segments = {};
u2_segments = {};
u3_segments = {};

total_segments1 = 0;
correct_segments1 = 0;
correct_signs1 = 0;

n_segments1 = size(indices1,1);
n_segments2 = size(indices2,1);
n_segments3 = size(indices3,1);

for i=1:n_segments1
    u1_segments{i} = u1_stack(indices1(i, 1):indices1(i, 2));
    total_segments1 = total_segments1 + 1;
    if ~any(diff(sign(u1_segments{i}(u1_segments{i}~=0))))
        correct_segments1 = correct_segments1 + 1;
        if signs{1}(i)*u1_segments{i}(1) > 0
            correct_signs1 = correct_signs1 + 1;
        end
    end   
end

total_segments2 = 0;
correct_segments2 = 0;
correct_signs2 = 0;

for i=1:n_segments2
    u2_segments{i} = u2_stack(indices2(i, 1):indices2(i, 2));
    total_segments2 = total_segments2 + 1;
    if ~any(diff(sign(u2_segments{i}(u2_segments{i}~=0))))
        correct_segments2 = correct_segments2 + 1;
        if signs{2}(i)*u2_segments{i}(1) > 0
            correct_signs2 = correct_signs2 + 1;
        end
    end   
end

total_segments3 = 0;
correct_segments3 = 0;
correct_signs3 = 0;

for i=1:n_segments3
    u3_segments{i} = u3_stack(indices3(i, 1):indices3(i, 2));
    total_segments3 = total_segments3 + 1;
    if ~any(diff(sign(u3_segments{i}(u3_segments{i}~=0))))
        correct_segments3 = correct_segments3 + 1;
        if signs{3}(i)*u3_segments{i}(1) > 0
            correct_signs3 = correct_signs3 + 1;
        end
    end   
end

acc_seg1 = correct_segments1/total_segments1;
acc_seg2 = correct_segments2/total_segments2;
acc_seg3 = correct_segments3/total_segments3;

acc_seg_total = (correct_segments1+correct_segments2+correct_segments3)/(total_segments1+total_segments2+total_segments3);

acc_seg_table = table(acc_seg1,acc_seg2,acc_seg3,acc_seg_total, 'VariableNames', {'Torque 1','Torque 2','Torque 3','Total'},'RowNames',{'Accuracy on segments'})

acc_sign1 = correct_signs1/correct_segments1;
acc_sign2 = correct_signs2/correct_segments2;
acc_sign3 = correct_signs3/correct_segments3;

acc_sign_total = (correct_signs1+correct_signs2+correct_signs3)/(correct_segments1+correct_segments2+correct_segments3);

acc_sign_table = table(acc_sign1,acc_sign2,acc_sign3,acc_sign_total, 'VariableNames', {'Torque 1','Torque 2','Torque 3','Total'},'RowNames',{'Accuracy on signs'})