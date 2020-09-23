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

function [loss] = error_fcn_gM_LMI_regressor_1dof(x)

% global SA_step
% 
% if SA_step == 1
%     penalty = 0;
% else
%     penalty = 10*(SA_step-1);
% end

m = x(1);
d = x(2);
I = x(3);

P_li_expanded_eval = get_1dof_coefficients(m, d, I);

global experiment_path;
load(fullfile(experiment_path,'Y_1dof.mat'), 'Y_1dof')
load(fullfile(experiment_path,'u_1dof.mat'), 'u_1dof')

% compute error vector, as the difference of current dynamic coeff values
% and previously estimated dyn coeff values, as follows:
% Y_stack*pi(p_k) - tau_stack
error = Y_1dof * P_li_expanded_eval - u_1dof;

loss = (error'*error);

end