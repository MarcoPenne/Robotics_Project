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

num_of_joints = 7; % DoFs of the Franka Emika Panda robot

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
load('data/franka_emika_panda/regressor_and_pars_data.mat', 'Y_stack_LI')
load('data/franka_emika_panda/regressor_and_pars_data.mat', 'tau_stack')
tau_abs = abs(tau_stack);

num_of_samples = size(Y_stack_LI,1)/num_of_joints;


% ---------------------------
% read lower and upper bounds
% ---------------------------

[LB,UB] = read_bounds('data/franka_emika_panda/bounds_gM_friction.csv');

tau1_abs = tau_abs(1:7:7*num_of_samples);
tau2_abs = tau_abs(2:7:7*num_of_samples);
tau3_abs = tau_abs(3:7:7*num_of_samples);
tau4_abs = tau_abs(4:7:7*num_of_samples);
tau5_abs = tau_abs(5:7:7*num_of_samples);
tau6_abs = tau_abs(6:7:7*num_of_samples);
tau7_abs = tau_abs(7:7:7*num_of_samples);

Y1_stack = Y_stack_LI(1:7:7*num_of_samples, :);
Y2_stack = Y_stack_LI(2:7:7*num_of_samples, :);
Y3_stack = Y_stack_LI(3:7:7*num_of_samples, :);
Y4_stack = Y_stack_LI(4:7:7*num_of_samples, :);
Y5_stack = Y_stack_LI(5:7:7*num_of_samples, :);
Y6_stack = Y_stack_LI(6:7:7*num_of_samples, :);
Y7_stack = Y_stack_LI(7:7:7*num_of_samples, :);

sorted_tau1 = sort(tau1_abs);
threshold1 = sorted_tau1(int32(num_of_samples*0.1)+1);

sorted_tau2 = sort(tau2_abs);
threshold2 = sorted_tau2(int32(num_of_samples*0.1)+1);

sorted_tau3 = sort(tau3_abs);
threshold3 = sorted_tau3(int32(num_of_samples*0.1)+1);

sorted_tau4 = sort(tau4_abs);
threshold4 = sorted_tau4(int32(num_of_samples*0.1)+1);

sorted_tau5 = sort(tau5_abs);
threshold5 = sorted_tau5(int32(num_of_samples*0.1)+1);

sorted_tau6 = sort(tau6_abs);
threshold6 = sorted_tau6(int32(num_of_samples*0.1)+1);

sorted_tau7 = sort(tau7_abs);
threshold7 = sorted_tau7(int32(num_of_samples*0.1)+1);

threshold = [threshold1, threshold2, threshold3, threshold4, threshold5, threshold6, threshold7];

indices1 = change_of_sign(tau1_abs, threshold(1));
indices2 = change_of_sign(tau2_abs, threshold(2));
indices3 = change_of_sign(tau3_abs, threshold(3));
indices4 = change_of_sign(tau4_abs, threshold(4));
indices5 = change_of_sign(tau5_abs, threshold(5));
indices6 = change_of_sign(tau6_abs, threshold(6));
indices7 = change_of_sign(tau7_abs, threshold(7));

indices_long1 = [];
indices_long2 = [];
indices_long3 = [];
indices_long4 = [];
indices_long5 = [];
indices_long6 = [];
indices_long7 = [];

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
for i=1:size(indices4, 1)
    if indices4(i, 2)-indices4(i, 1)>=20
        indices_long4 = [indices_long4; indices4(i, :)];
    end
end
for i=1:size(indices5, 1)
    if indices5(i, 2)-indices5(i, 1)>=20
        indices_long5 = [indices_long5; indices5(i, :)];
    end
end
for i=1:size(indices6, 1)
    if indices6(i, 2)-indices6(i, 1)>=20
        indices_long6 = [indices_long6; indices6(i, :)];
    end
end
for i=1:size(indices7, 1)
    if indices7(i, 2)-indices7(i, 1)>=20
        indices_long7 = [indices_long7; indices7(i, :)];
    end
end
indices1 = indices_long1;
indices2 = indices_long2;
indices3 = indices_long3;
indices4 = indices_long4;
indices5 = indices_long5;
indices6 = indices_long6;
indices7 = indices_long7;

load('data/franka_emika_panda/signs.mat', 'signs1')
load('data/franka_emika_panda/signs.mat', 'signs2')
load('data/franka_emika_panda/signs.mat', 'signs3')
load('data/franka_emika_panda/signs.mat', 'signs4')
load('data/franka_emika_panda/signs.mat', 'signs5')
load('data/franka_emika_panda/signs.mat', 'signs6')
load('data/franka_emika_panda/signs.mat', 'signs7')

% signs1 = [];
% 
% for i=1:size(indices1, 1)
%     stringtodisp = sprintf('Finding best sign segment %d/%d JOINT 1',i,size(indices1, 1));
%     disp(stringtodisp);
%     
%     [sign1, positive_optimal_solution1, negative_optimal_solution1] = choose_sign_franka(Y1_stack(indices1(i, 1):indices1(i, 2), :), tau1_abs(indices1(i, 1):indices1(i, 2)), LB, UB);
%     signs1(i) = sign1;
%     stringtodisp = sprintf("Choosen segment %d/%d: %d",i,size(indices1, 1), sign1);
%     disp(stringtodisp);
%     %disp(positive_optimal_solution1)
%     %disp(negative_optimal_solution1)
% end
% 
% save('data/franka_emika_panda/signs.mat', 'signs1');
% 
% signs2 = [];
% 
% for i=1:size(indices2, 1)
%     stringtodisp = sprintf('Finding best sign segment %d/%d JOINT 2',i,size(indices2, 1));
%     disp(stringtodisp);
%     
%     [sign2, positive_optimal_solution2, negative_optimal_solution2] = choose_sign_franka(Y2_stack(indices2(i, 1):indices2(i, 2), :), tau2_abs(indices2(i, 1):indices2(i, 2)), LB, UB);
%     signs2 = [signs2, sign2];
%     disp("Choosen: "+sign2)
%     %disp(positive_optimal_solution1)
%     %disp(negative_optimal_solution1)
% end
% 
% save('data/franka_emika_panda/signs.mat', 'signs1','signs2');

% signs3 = [];
% 
% for i=1:size(indices3, 1)
%     stringtodisp = sprintf('Finding best sign segment %d/%d JOINT 3',i,size(indices3, 1));
%     disp(stringtodisp);
%     
%     [sign3, positive_optimal_solution3, negative_optimal_solution3] = choose_sign_franka(Y3_stack(indices3(i, 1):indices3(i, 2), :), tau3_abs(indices3(i, 1):indices3(i, 2)), LB, UB);
%     signs3 = [signs3, sign3];
%     disp("Choosen: "+sign3)
%     %disp(positive_optimal_solution1)
%     %disp(negative_optimal_solution1)
% end
% 
% save('data/franka_emika_panda/signs.mat', 'signs1','signs2','signs3');
% signs4 = [];
% 
% for i=1:size(indices4, 1)
%     stringtodisp = sprintf('Finding best sign segment %d/%d JOINT 4',i,size(indices4, 1));
%     disp(stringtodisp);
%     
%     [sign4, positive_optimal_solution4, negative_optimal_solution4] = choose_sign_franka(Y4_stack(indices4(i, 1):indices4(i, 2), :), tau4_abs(indices4(i, 1):indices4(i, 2)), LB, UB);
%     signs4 = [signs4, sign4];
%     disp("Choosen: "+sign4)
%     %disp(positive_optimal_solution1)
%     %disp(negative_optimal_solution1)
% end
% save('data/franka_emika_panda/signs.mat', 'signs1','signs2','signs3','signs4');

% signs5 = [];
% 
% for i=1:size(indices5, 1)
%     stringtodisp = sprintf('Finding best sign segment %d/%d JOINT 5',i,size(indices5, 1));
%     disp(stringtodisp);
%     
%     [sign5, positive_optimal_solution5, negative_optimal_solution5] = choose_sign_franka(Y5_stack(indices5(i, 1):indices5(i, 2), :), tau5_abs(indices5(i, 1):indices5(i, 2)), LB, UB);
%     signs5 = [signs5, sign5];
%     disp("Choosen: "+sign5)
%     %disp(positive_optimal_solution1)
%     %disp(negative_optimal_solution1)
% end
% save('data/franka_emika_panda/signs.mat',  'signs1','signs2','signs3','signs4','signs5');
% signs6 = [];
% 
% for i=1:size(indices6, 1)
%     stringtodisp = sprintf('Finding best sign segment %d/%d JOINT 6',i,size(indices6, 1));
%     disp(stringtodisp);
%     
%     [sign6, positive_optimal_solution6, negative_optimal_solution6] = choose_sign_franka(Y6_stack(indices6(i, 1):indices6(i, 2), :), tau6_abs(indices6(i, 1):indices6(i, 2)), LB, UB);
%     signs6 = [signs6, sign6];
%     disp("Choosen: "+sign6)
%     %disp(positive_optimal_solution1)
%     %disp(negative_optimal_solution1)
% end
% save('data/franka_emika_panda/signs.mat', 'signs1','signs2','signs3','signs4','signs5','signs6');
% signs7 = [];
% 
% for i=1:size(indices7, 1)
%     stringtodisp = sprintf('Finding best sign segment %d/%d JOINT 7',i,size(indices7, 1));
%     disp(stringtodisp);
%     
%     [sign7, positive_optimal_solution7, negative_optimal_solution7] = choose_sign_franka(Y7_stack(indices7(i, 1):indices7(i, 2), :), tau7_abs(indices7(i, 1):indices7(i, 2)), LB, UB);
%     signs7 = [signs7, sign7];
%     disp("Choosen: "+sign7)
%     %disp(positive_optimal_solution1)
%     %disp(negative_optimal_solution1)
% end
% save('data/franka_emika_panda/signs.mat', 'signs1','signs2','signs3','signs4','signs5','signs6','signs7');

tau1_estimated_sign = [];
tau2_estimated_sign = [];
tau3_estimated_sign = [];
tau4_estimated_sign = [];
tau5_estimated_sign = [];
tau6_estimated_sign = [];
tau7_estimated_sign = [];

Y1_estimated = [];
Y2_estimated = [];
Y3_estimated = [];
Y4_estimated = [];
Y5_estimated = [];
Y6_estimated = [];
Y7_estimated = [];

for i=1:length(signs1)
    tau1_estimated_sign = [tau1_estimated_sign; signs1(i) * tau1_abs(indices1(i, 1):indices1(i, 2))];
    Y1_estimated = [Y1_estimated; Y1_stack(indices1(i, 1):indices1(i, 2), :)];
end

for i=1:length(signs2)
    tau2_estimated_sign = [tau2_estimated_sign; signs2(i) * tau2_abs(indices2(i, 1):indices2(i, 2))];
    Y2_estimated = [Y2_estimated; Y2_stack(indices2(i, 1):indices2(i, 2), :)];
end

for i=1:length(signs3)
    tau3_estimated_sign = [tau3_estimated_sign; signs3(i) * tau3_abs(indices3(i, 1):indices3(i, 2))];
    Y3_estimated = [Y3_estimated; Y3_stack(indices3(i, 1):indices3(i, 2), :)];
end

for i=1:length(signs4)
    tau4_estimated_sign = [tau4_estimated_sign; signs4(i) * tau4_abs(indices4(i, 1):indices4(i, 2))];
    Y4_estimated = [Y4_estimated; Y4_stack(indices4(i, 1):indices4(i, 2), :)];
end

for i=1:length(signs5)
    tau5_estimated_sign = [tau5_estimated_sign; signs5(i) * tau5_abs(indices5(i, 1):indices5(i, 2))];
    Y5_estimated = [Y5_estimated; Y5_stack(indices5(i, 1):indices5(i, 2), :)];
end

for i=1:length(signs6)
    tau6_estimated_sign = [tau6_estimated_sign; signs6(i) * tau6_abs(indices6(i, 1):indices6(i, 2))];
    Y6_estimated = [Y6_estimated; Y6_stack(indices6(i, 1):indices6(i, 2), :)];
end

for i=1:length(signs7)
    tau7_estimated_sign = [tau7_estimated_sign; signs7(i) * tau7_abs(indices7(i, 1):indices7(i, 2))];
    Y7_estimated = [Y7_estimated; Y7_stack(indices7(i, 1):indices7(i, 2), :)];
end

tau_final = [tau1_estimated_sign;tau2_estimated_sign;tau3_estimated_sign;tau4_estimated_sign;tau5_estimated_sign;tau6_estimated_sign;tau7_estimated_sign ];
Y_final = [Y1_estimated;Y2_estimated;Y3_estimated;Y4_estimated;Y5_estimated;Y6_estimated;Y7_estimated];

%----------------------------------
% initializations
%----------------------------------
num_of_runs = 5; % independent (parallelizable) runs of the entire algorithm (29)
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
        stringtodisp = sprintf('RUN %d of %d, SA_step %d of %d',i,num_of_runs, SA_step, num_of_SA_steps);
        disp(stringtodisp);
        if SA_step==1
            X0 = rand(num_x,1).*(UB-LB) + LB; % random initial point inside bounds
        end
        
        options = optimoptions('patternsearch','Display', 'off', 'UseParallel',true); % use Nelder-Mead optimization as hybrid function

        [X,FVAL,EXITFLAG,OUTPUT] = patternsearch(@(x) error_fcn_gM_LMI_regressor_franka(x, Y_final, tau_final, SA_step), X0, [], [], [], [],LB,UB,[],options);
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
m4 = optimal_solution(4);
m5 = optimal_solution(5);
m6 = optimal_solution(6);
m7 = optimal_solution(7);
c1x = optimal_solution(8);
c1y = optimal_solution(9);
c1z = optimal_solution(10);
c2x = optimal_solution(11);
c2y = optimal_solution(12);
c2z = optimal_solution(13);
c3x = optimal_solution(14);
c3y = optimal_solution(15);
c3z = optimal_solution(16);
c4x = optimal_solution(17);
c4y = optimal_solution(18);
c4z = optimal_solution(19);
c5x = optimal_solution(20);
c5y = optimal_solution(21);
c5z = optimal_solution(22);
c6x = optimal_solution(23);
c6y = optimal_solution(24);
c6z = optimal_solution(25);
c7x = optimal_solution(26);
c7y = optimal_solution(27);
c7z = optimal_solution(28);

I1xx = optimal_solution(29);
I1xy = optimal_solution(30);
I1xz = optimal_solution(31);
I1yy = optimal_solution(32);
I1yz = optimal_solution(33);
I1zz = optimal_solution(34);
I2xx = optimal_solution(35);
I2xy = optimal_solution(36);
I2xz = optimal_solution(37);
I2yy = optimal_solution(38);
I2yz = optimal_solution(39);
I2zz = optimal_solution(40);
I3xx = optimal_solution(41);
I3xy = optimal_solution(42);
I3xz = optimal_solution(43);
I3yy = optimal_solution(44);
I3yz = optimal_solution(45);
I3zz = optimal_solution(46);
I4xx = optimal_solution(47);
I4xy = optimal_solution(48);
I4xz = optimal_solution(49);
I4yy = optimal_solution(50);
I4yz = optimal_solution(51);
I4zz = optimal_solution(52);
I5xx = optimal_solution(53);
I5xy = optimal_solution(54);
I5xz = optimal_solution(55);
I5yy = optimal_solution(56);
I5yz = optimal_solution(57);
I5zz = optimal_solution(58);
I6xx = optimal_solution(59);
I6xy = optimal_solution(60);
I6xz = optimal_solution(61);
I6yy = optimal_solution(62);
I6yz = optimal_solution(63);
I6zz = optimal_solution(64);
I7xx = optimal_solution(65);
I7xy = optimal_solution(66);
I7xz = optimal_solution(67);
I7yy = optimal_solution(68);
I7yz = optimal_solution(69);
I7zz = optimal_solution(70);

fc1 = optimal_solution(71);
fc2 = optimal_solution(72);
fc3 = optimal_solution(73);
fc4 = optimal_solution(74);
fc5 = optimal_solution(75);
fc6 = optimal_solution(76);
fc7 = optimal_solution(77);

fv1 = optimal_solution(78);
fv2 = optimal_solution(79);
fv3 = optimal_solution(80);
fv4 = optimal_solution(81);
fv5 = optimal_solution(82);
fv6 = optimal_solution(83);
fv7 = optimal_solution(84);

fo1 = optimal_solution(85);
fo2 = optimal_solution(86);
fo3 = optimal_solution(87);
fo4 = optimal_solution(88);
fo5 = optimal_solution(89);
fo6 = optimal_solution(90);
fo7 = optimal_solution(91);

P_li_expanded_eval = get_Panda_coefficients_expanded(I2xx,I2xy,I3xx,I2xz,I3xy,I4xx,I3xz,I4xy,I5xx,I4xz,I5xy,I6xx,I5xz,I6xy,I7xx,I6xz,I7xy,I7xz,I2yy,I2yz,I3yy,I3yz,I4yy,I4yz,I5yy,I5yz,I6yy,I6yz,I7yy,I7yz,I1zz,I2zz,I3zz,I4zz,I5zz,I6zz,I7zz,c1x,c2x,c3x,c4x,c5x,c6x,c7x,c1y,c2y,c3y,c4y,c5y,c6y,c7y,c2z,c3z,c4z,c5z,c6z,c7z,fc1,fc2,fc3,fc4,fc5,fc6,fc7,fo1,fo2,fo3,fo4,fo5,fo6,fo7,fv1,fv2,fv3,fv4,fv5,fv6,fv7,m1,m2,m3,m4,m5,m6,m7);

% joint torques estimation
tau_stack_estimation1 = Y1_stack*P_li_expanded_eval;
tau_stack_estimation2 = Y2_stack*P_li_expanded_eval;
tau_stack_estimation3 = Y3_stack*P_li_expanded_eval;
tau_stack_estimation4 = Y4_stack*P_li_expanded_eval;
tau_stack_estimation5 = Y5_stack*P_li_expanded_eval;
tau_stack_estimation6 = Y6_stack*P_li_expanded_eval;
tau_stack_estimation7 = Y7_stack*P_li_expanded_eval;

tau1_stack = tau_stack(1:7:7*num_of_samples);
tau2_stack = tau_stack(2:7:7*num_of_samples);
tau3_stack = tau_stack(3:7:7*num_of_samples);
tau4_stack = tau_stack(4:7:7*num_of_samples);
tau5_stack = tau_stack(5:7:7*num_of_samples);
tau6_stack = tau_stack(6:7:7*num_of_samples);
tau7_stack = tau_stack(7:7:7*num_of_samples);

tau_stack_estimation = [tau_stack_estimation1 tau_stack_estimation2 tau_stack_estimation3 tau_stack_estimation4 tau_stack_estimation5 tau_stack_estimation6 tau_stack_estimation7];
tau_stack_final = [tau1_stack tau2_stack tau3_stack tau4_stack tau5_stack tau6_stack tau7_stack];

% reshape tau_stack vectors (measured and estimated)
TAU_TRUE = zeros(num_of_joints,num_of_samples);
TAU_ESTD = zeros(num_of_joints,num_of_samples);
for i = 1 : num_of_samples
    for j = 1:num_of_joints
        TAU_TRUE(j,i) = tau_stack_final(i, j);
        TAU_ESTD(j,i) = tau_stack_estimation(i, j);
    end
end

indices = {indices1, indices2, indices3, indices4, indices5, indices6, indices7};
signs = {signs1, signs2, signs3, signs4, signs5, signs6, signs7};
% plot validation results
figure
samples = 1:num_of_samples;
for i=1:num_of_joints
    subplot(4,2,i);
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

estimated_coefficients = get_Panda_coefficients_expanded(I2xx,I2xy,I3xx,I2xz,I3xy,I4xx,I3xz,I4xy,I5xx,I4xz,I5xy,I6xx,I5xz,I6xy,I7xx,I6xz,I7xy,I7xz,I2yy,I2yz,I3yy,I3yz,I4yy,I4yz,I5yy,I5yz,I6yy,I6yz,I7yy,I7yz,I1zz,I2zz,I3zz,I4zz,I5zz,I6zz,I7zz,c1x,c2x,c3x,c4x,c5x,c6x,c7x,c1y,c2y,c3y,c4y,c5y,c6y,c7y,c2z,c3z,c4z,c5z,c6z,c7z,fc1,fc2,fc3,fc4,fc5,fc6,fc7,fo1,fo2,fo3,fo4,fo5,fo6,fo7,fv1,fv2,fv3,fv4,fv5,fv6,fv7,m1,m2,m3,m4,m5,m6,m7);
estimated_coefficients = estimated_coefficients(1:41)
ground_coefficients = [2.921667e-02;
9.818184e-01;
-5.191244e-03;
2.841995e-02;
-3.493413e-03;
1.042777e+00;
1.059003e-02;
-1.043855e-02;
-4.845789e-03;
1.168757e-01;
5.324312e-01;
1.500656e-01;
4.772233e-03;
-2.731790e-03;
6.369356e-01;
3.215369e-02;
-3.730935e-03;
-6.133119e-03;
7.715324e-03;
1.806937e-02;
-6.531748e-03;
5.365261e-03;
8.204412e-04;
2.504593e-02;
-1.767591e-03;
1.185322e-03;
1.757815e-03;
-5.969360e-04;
1.272440e-03;
-5.368710e-03;
-3.102576e+00;
6.873555e-01;
2.337654e-02;
-4.888364e-01;
1.718486e+00;
-1.000420e-02;
7.705313e-02;
1.656804e-01;
-6.771177e-02;
6.231087e-03;
-4.880859e-04];

a_franka = pinv(Y_stack_LI)*tau_stack;

error_measure = norm(estimated_coefficients - ground_coefficients);

disp('The estimated dynamic coefficients w/o torque signs are:')
disp(estimated_coefficients)
disp('The ground values of the dynamic coefficients are:')
disp(ground_coefficients)
disp('The estimated dynamic coefficients with torque signs are using classical approach (CLS):')
disp(a_franka)
disp('The norm of the difference between estimated dynamic coefficients w/o torque signs and ground values is:')
disp(error_measure)