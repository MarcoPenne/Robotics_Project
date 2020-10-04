% author: Claudio Gaz, Marco Cognetti
% date: August 2, 2019
% 
% -------------------------------------------------
% Parameters Retrieval Algorithm
% -------------------------------------------------
% C. Gaz, M. Cognetti, A. Oliva, P. Robuffo Giordano, A. De Luca, 'Dynamic
% Identification of the Franka Emika Panda Robot With Retrieval of Feasible
% Parameters Using Penalty-Based Optimization'. IEEE RA-L, 2019.
%
% the following code has been tested on Matlab 2018b

close all
clc
clear all

addpath('functions');

num_of_joints = 3; % DoFs of the Franka Emika Panda robot

% load numerical evaluated regressor and symbolic dynamic coefficients,
% useful for final cross-validation and in case use_complete_regressor =
% true.
%
% In particular:
%  - Y_stack_LI contains a stacked evaluated regressor (exciting
%  trajectories have been used)
%  - tau_stack contains the vector of stacked measurements (joint
%  torques)
%  - P_li_full contains the symbolic dynamic coefficients vector, with
%  the inertia tensors expressed w.r.t. link frames
%  - P_li_full_subs contains the symbolic dynamic coefficients vector,
%  with the inertia tensors expressed w.r.t. link CoMs

% total samples retrieved during exciting trajectories
load('data/3-dof/experiment4/Y_stack.mat', 'Y_stack')
load('data/3-dof/experiment4/u_stack.mat', 'u_stack')
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

sorted_u1 = sort(u1_3dof_abs);
threshold1 = sorted_u1(int32(num_of_samples*0.1)+1);

sorted_u2 = sort(u2_3dof_abs);
threshold2 = sorted_u2(int32(num_of_samples*0.1)+1);

sorted_u3 = sort(u3_3dof_abs);
threshold3 = sorted_u3(int32(num_of_samples*0.1)+1);

%threshold = [0.005, 0.3, 0.2];
threshold = [threshold1, threshold2, threshold3];

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

[Y_final, u_final, signs] = tree_3dof(Y1_stack, Y2_stack, Y3_stack, u1_3dof_abs, u2_3dof_abs, u3_3dof_abs, indices1, indices2, indices3, LB, UB)

%----------------------------------
% initializations
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
samples = 1:num_of_samples;
for i=1:num_of_joints
    subplot(3,1,i);
    hold on
    plot(samples,TAU_TRUE(i,:),samples,TAU_ESTD(i,:));
    for seg=1:size(indices{i},1)
        xline(indices{i}(seg, 1), '--g');
        xline(indices{i}(seg, 2), '--r');
        if signs{i}(seg)>0
            lab = '+';
        else
            lab = '-';
        end
        text(0.5*indices{i}(seg, 2) + 0.5*indices{i}(seg, 1), 0, lab, 'HorizontalAlignment', 'center');
    end
    yline(threshold(i), '--o');
    yline(-threshold(i), '--o');
    hold off
    grid;
    xlabel('samples [#]');
    ylabel('torque [Nm]');
    if i==num_of_joints
        legend('true','estimated');
    end
end

estimated_coefficients = get_3dof_coefficients(m1,rc1x,rc1y,rc1z,I1yy,m2,rc2x,rc2y,rc2z,I2xx,I2yy,I2zz,m3,rc3x,rc3y,rc3z,I3xx,I3yy,I3zz);
ground_coefficients = get_3dof_coefficients(10, 0, 0.15, 0, 4.167e-04*10, 1.125, 0.15, 0, 0.06, 4.167e-04*1.125, 7.708e-03*1.125, 7.708e-03*1.125, 0.75, 0.1, 0, 0, 4.167e-04*0.75, 3.542e-03*0.75, 3.542e-03*0.75);

a_3dof = pinv(Y_stack)*u_stack;

error_measure = norm(estimated_coefficients - ground_coefficients);

disp('The estimated dynamic coefficients w/o torque signs are:')
disp(estimated_coefficients)
disp('The ground values of the dynamic coefficients are:')
disp(ground_coefficients)
disp('The estimated dynamic coefficients with torque signs are:')
disp(a_3dof)
disp('The norm of the difference between estimated dynamic coefficients w/o torque signs and ground values is:')
disp(error_measure)