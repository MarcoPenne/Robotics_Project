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
load('data/3-dof/experiment1/Y_stack.mat', 'Y_stack')
load('data/3-dof/experiment1/u_stack.mat', 'u_stack')
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

indices1 = change_of_sign(u1_3dof_abs, 0.3);
indices2 = change_of_sign(u2_3dof_abs, 3);
indices3 = change_of_sign(u3_3dof_abs, 0.4);

indices_long1 = []
indices_long2 = []
indices_long3 = []

for i=1:size(indices1, 1)
    if indices1(i, 2)-indices1(i, 1)>=20
        indices_long1 = [indices_long1; indices1(i, :)]
    end
end
for i=1:size(indices2, 1)
    if indices2(i, 2)-indices2(i, 1)>=20
        indices_long2 = [indices_long2; indices2(i, :)]
    end
end
for i=1:size(indices3, 1)
    if indices3(i, 2)-indices3(i, 1)>=20
        indices_long3 = [indices_long3; indices3(i, :)]
    end
end

indices1 = indices_long1;
indices2 = indices_long2;
indices3 = indices_long3;

signs1 = []

for i=1:size(indices1, 1)
    [sign1, positive_optimal_solution1, negative_optimal_solution1] = choose_sign_3dof(Y1_stack(indices1(i, 1):indices1(i, 2), :), u1_3dof_abs(indices1(i, 1):indices1(i, 2)), LB, UB);
    signs1 = [signs1, sign1];
    disp(sign1)
    disp(positive_optimal_solution1)
    disp(negative_optimal_solution1)
end

signs2 = []

for i=1:size(indices2, 1)
    [sign2, positive_optimal_solution2, negative_optimal_solution2] = choose_sign_3dof(Y2_stack(indices2(i, 1):indices2(i, 2), :), u2_3dof_abs(indices2(i, 1):indices2(i, 2)), LB, UB);
    signs2 = [signs2, sign2];
    disp(sign2)
    disp(positive_optimal_solution2)
    disp(negative_optimal_solution2)
end

signs3 = []

for i=1:size(indices3, 1)
    [sign3, positive_optimal_solution3, negative_optimal_solution3] = choose_sign_3dof(Y3_stack(indices3(i, 1):indices3(i, 2), :), u3_3dof_abs(indices3(i, 1):indices3(i, 2)), LB, UB);
    signs3 = [signs3, sign3];
    disp(sign3)
    disp(positive_optimal_solution3)
    disp(negative_optimal_solution3)
end

u1_estimated_sign = [];
u2_estimated_sign = [];
u3_estimated_sign = [];

Y1_estimated = [];
Y2_estimated = [];
Y3_estimated = [];

for i=1:length(signs1)
    u1_estimated_sign = [u1_estimated_sign; signs1(i) * u1_3dof_abs(indices1(i, 1):indices1(i, 2))];
    Y1_estimated = [Y1_estimated; Y1_stack(indices1(i, 1):indices1(i, 2), :)];
end

for i=1:length(signs2)
    u2_estimated_sign = [u2_estimated_sign; signs2(i) * u2_3dof_abs(indices2(i, 1):indices2(i, 2))];
    Y2_estimated = [Y2_estimated; Y2_stack(indices2(i, 1):indices2(i, 2), :)];
end

for i=1:length(signs3)
    u3_estimated_sign = [u3_estimated_sign; signs3(i) * u3_3dof_abs(indices3(i, 1):indices3(i, 2))];
    Y3_estimated = [Y3_estimated; Y3_stack(indices3(i, 1):indices3(i, 2), :)];
end

u_final = [u1_estimated_sign;u2_estimated_sign;u3_estimated_sign];
Y_final = [Y1_estimated;Y2_estimated;Y3_estimated];

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
P_li_expanded_eval = dinamic_coefficients(m1,rc1x,rc1y,rc1z,I1yy,m2,rc2x,rc2y,rc2z,I2xx,I2yy,I2zz,m3,rc3x,rc3y,rc3z,I3xx,I3yy,I3zz);

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

% plot validation results
figure
samples = 1:num_of_samples;
for i=1:num_of_joints
    subplot(3,1,i);
    plot(samples,TAU_TRUE(i,:),samples,TAU_ESTD(i,:));
    grid;
    xlabel('samples [#]');
    ylabel('torque [Nm]');
    if i==num_of_joints
        legend('true','estimated');
    end
end