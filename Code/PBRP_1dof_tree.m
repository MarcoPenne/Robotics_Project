close all
clc
clear all

addpath('functions');

num_of_joints = 1; % DoFs of the robot

% load regressor matrix Y and vector of stacked torques
load('data/1-dof/new_experiment1/Y_stack.mat', 'Y_stack')
load('data/1-dof/new_experiment1/u_stack.mat', 'u_stack')

Y_1dof = Y_stack;
u_1dof = u_stack;

% take only the absolute value of the torques
u_1dof_abs = abs(u_1dof);

num_of_samples = size(Y_1dof,1)/num_of_joints;

% computing the 10% of min abs values
sorted_u = sort(u_1dof_abs);
threshold = sorted_u(int32(num_of_samples*0.1)+1);

% ---------------------------
% read lower and upper bounds
% ---------------------------
[LB,UB] = read_bounds('data/1-dof/bounds/bound_1dof.csv');

% finding the segments with constant sign torque 
% indices contains the start and the finish of each segment
indices = change_of_sign(u_1dof_abs, threshold);
indices_long = [];
for i=1:size(indices, 1)
    if indices(i, 2)-indices(i, 1)>=20
        indices_long = [indices_long; indices(i, :)];
    end
end
indices = indices_long;

%-----------------------------------
% BUILD THE TREE
%-----------------------------------
% Let the tree PBRP algorithm estimate the signs for each segment annording
% to the indices. It will return:
% - a regressor matrix: Y_estimated, with rows in correct order
% - a vector: u_estimated_sign, with estimated sign and elements in correct order 
% - a vector: signs, with the estimated sign for each segment.

[Y_estimated, u_estimated_sign, signs] = tree_1dof(Y_1dof, u_1dof_abs, indices, LB, UB);


%----------------------------------
% RUN THE TRADITIONAL PBRP ALGORITHM
%----------------------------------
num_of_runs = 3; % independent (parallelizable) runs of the entire algorithm (29)

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
    stringtodisp = sprintf('RUN %d of %d',i,num_of_runs);
    disp(stringtodisp);

    X0 = rand(num_x,1).*(UB-LB) + LB; % random initial point inside bounds

    options = saoptimset('HybridFcn',{@fmincon}); % use Nelder-Mead optimization as hybrid function

    [X,FVAL,EXITFLAG,OUTPUT] = simulannealbnd(@(x) error_fcn_gM_LMI_regressor_1dof(x, Y_estimated, u_estimated_sign),X0,LB,UB,options);

    X0 = X;

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

m = optimal_solution(1);
d = optimal_solution(2);
I = optimal_solution(3);

   
% compute coefficients vector with the estimated dynamic parameters
P_li_expanded_eval = get_1dof_coefficients(m, d, I);

% joint torques estimation
tau_stack_estimation = Y_1dof*P_li_expanded_eval;

% reshape tau_stack vectors (measured and estimated)
TAU_TRUE = zeros(num_of_joints,num_of_samples);
TAU_ESTD = zeros(num_of_joints,num_of_samples);
TAU_SIGN_ESTD = zeros(num_of_joints,num_of_samples);
for i = 1 : num_of_samples
    for j = 1:num_of_joints
        TAU_TRUE(j,i) = u_1dof((i-1) + j);
        TAU_ESTD(j,i) = tau_stack_estimation((i-1) + j);
    end
end

% plot validation results
figure
dt = 0.02;
period = 10;
samples = 0:dt:period;
for i=1:num_of_joints
    subplot(1,1,i);
    hold on
    plot(samples,TAU_TRUE(i,:),samples,TAU_ESTD(i,:));
    
    for seg=1:size(indices,1)
        xline(indices(seg, 1)*dt, '--g');
        xline(indices(seg, 2)*dt, '--r');
        if signs(seg)>0
            lab = '+';
        else
            lab = '-';
        end
        text(0.5*indices(seg, 2)*dt + 0.5*indices(seg, 1)*dt, 0, lab, 'HorizontalAlignment', 'center');
    end
    yline(threshold, '--o');
    yline(-threshold, '--o');
    hold off
    grid;
    xlabel('time [s]');
    ylabel('torque [Nm]');
    if i==num_of_joints
        legend('true','estimated');
    end
end

estimated_coefficients = get_1dof_coefficients(optimal_solution(1),optimal_solution(2),optimal_solution(3));
ground_coefficients = get_1dof_coefficients(5,0.5,5*(8.417e-02));

error_measure = norm(estimated_coefficients - ground_coefficients);

disp('The estimated dynamic coefficients w/o torque signs are:')
disp(estimated_coefficients)
disp('The ground values of the dynamic coefficients are:')
disp(ground_coefficients)
disp('The norm of the difference between estimated dynamic coefficients w/o torque signs and ground values is:')
disp(error_measure)

u_segments = {};

total_segments = 0;
correct_segments = 0;
correct_signs = 0;

n_segments = size(indices,1);

for i=1:n_segments
    u_segments{i} = u_1dof(indices(i, 1):indices(i, 2));
    total_segments = total_segments + 1;
    if ~any(diff(sign(u_segments{i}(u_segments{i}~=0))))
        correct_segments = correct_segments + 1;
        if signs(i)*u_segments{i}(1) > 0
            correct_signs = correct_signs + 1;
        end
    end   
end

acc_seg = correct_segments/total_segments;

acc_seg_table = table(acc_seg, 'RowNames', {'Accuracy on segments'})

acc_sign = correct_signs/correct_segments;

acc_sign_table = table(acc_sign, 'RowNames', {'Accuracy on signs'})