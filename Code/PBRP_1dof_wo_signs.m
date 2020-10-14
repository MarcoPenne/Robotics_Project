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

num_of_joints = 1; % DoFs of the Franka Emika Panda robot

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
load('data/1-dof/experiment11/Y_1dof.mat', 'Y_1dof')
load('data/1-dof/experiment11/u_1dof.mat', 'u_1dof')
u_1dof_abs = abs(u_1dof);

num_of_samples = size(Y_1dof,1)/num_of_joints;

sorted_u = sort(u_1dof_abs);
threshold = sorted_u(int32(num_of_samples*0.1)+1);

% ---------------------------
% read lower and upper bounds
% ---------------------------

[LB,UB] = read_bounds('data/1-dof/bounds/bound_1dof.csv');
%threshold = 1;
indices = change_of_sign(u_1dof_abs, threshold);
indices_long = [];
for i=1:size(indices, 1)
    if indices(i, 2)-indices(i, 1)>=20
        indices_long = [indices_long; indices(i, :)];
    end
end
indices = indices_long;

signs = [];

for i=1:size(indices, 1)
    stringtodisp = sprintf('Finding best sign segment %d/%d',i,size(indices, 1));
    disp(stringtodisp);
    
    [sign, positive_optimal_solution, negative_optimal_solution] = choose_sign_1dof(Y_1dof(indices(i, 1):indices(i, 2), :), u_1dof_abs(indices(i, 1):indices(i, 2)), LB, UB);
    signs = [signs, sign];
    disp("Choosen: "+sign)
    %disp(positive_optimal_solution)
    %disp(negative_optimal_solution)
end

u_estimated_sign = [];
Y_estimated = [];

for i=1:length(signs)
    u_estimated_sign = [u_estimated_sign; signs(i) * u_1dof_abs(indices(i, 1):indices(i, 2))];
    Y_estimated = [Y_estimated; Y_1dof(indices(i, 1):indices(i, 2), :)];
end

%----------------------------------
% initializations
%----------------------------------
num_of_runs = 3; % independent (parallelizable) runs of the entire algorithm (29)
% num_of_SA_steps = 5; % successive runs of the algorithm (29). It is the variable \kappa in (29)

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
    %for SA_step=1:num_of_SA_steps
        stringtodisp = sprintf('RUN %d of %d',i,num_of_runs);
        disp(stringtodisp);
        
        X0 = rand(num_x,1).*(UB-LB) + LB; % random initial point inside bounds
        
        options = saoptimset('HybridFcn',{@fmincon}); % use Nelder-Mead optimization as hybrid function

        [X,FVAL,EXITFLAG,OUTPUT] = simulannealbnd(@(x) error_fcn_gM_LMI_regressor_1dof(x, Y_estimated, u_estimated_sign),X0,LB,UB,options);

        X0 = X;
    %end
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
samples = 1:num_of_samples;
for i=1:num_of_joints
    subplot(1,1,i);
    hold on
    plot(samples,TAU_TRUE(i,:),samples,TAU_ESTD(i,:));
    
    for seg=1:size(indices,1)
        xline(indices(seg, 1), '--g');
        xline(indices(seg, 2), '--r');
        if signs(seg)>0
            lab = '+';
        else
            lab = '-';
        end
        text(0.5*indices(seg, 2) + 0.5*indices(seg, 1), 0, lab, 'HorizontalAlignment', 'center');
    end
    yline(threshold, '--o');
    yline(-threshold, '--o');
    hold off
    grid;
    xlabel('samples [#]');
    ylabel('torque [Nm]');
    if i==num_of_joints
        legend('true','estimated');
    end
end

estimated_coefficients = get_1dof_coefficients(optimal_solution(1),optimal_solution(2),optimal_solution(3));
ground_coefficients = get_1dof_coefficients(5,0.5,5*(8.417e-02));

a_1dof = pinv(Y_1dof)*u_1dof;
error_measure = norm(estimated_coefficients - ground_coefficients);

disp('The estimated dynamic coefficients w/o torque signs are:')
disp(estimated_coefficients)
disp('The ground values of the dynamic coefficients are:')
disp(ground_coefficients)
disp('The estimated dynamic coefficients with torque signs are:')
disp(a_1dof)
disp('The norm of the difference between estimated dynamic coefficients w/o torque signs and ground values is:')
disp(error_measure)