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

function [loss] = error_fcn_gM_LMI_regressor_3dof(x, Y_3dof, u_3dof, SA_step)

if SA_step == 1
    penalty = 0;
else
    penalty = 10*(SA_step-1);
end

m1 = x(1);
m2 = x(2);
m3 = x(3);
rc1x = x(4);
rc1y = x(5);
rc1z = x(6);
rc2x = x(7);
rc2y = x(8);
rc2z = x(9);
rc3x = x(10);
rc3y = x(11);
rc3z = x(12);
I1yy = x(13);
I2xx = x(14);
I2yy = x(15);
I2zz = x(16);
I3xx = x(17);
I3yy = x(18);
I3zz = x(19);

P_li_expanded_eval = get_3dof_coefficients(m1,rc1x,rc1y,rc1z,I1yy,m2,rc2x,rc2y,rc2z,I2xx,I2yy,I2zz,m3,rc3x,rc3y,rc3z,I3xx,I3yy,I3zz);

% compute error vector, as the difference of current dynamic coeff values
% and previously estimated dyn coeff values, as follows:
% Y_stack*pi(p_k) - tau_stack
error = Y_3dof * P_li_expanded_eval - u_3dof;

loss = (error'*error);

%------------------------------------------------
% External Penalties
%------------------------------------------------

% conditions on total mass

min_mass = 100;
max_mass = 120;

if m1+m2+m3 < min_mass
    loss = loss + penalty*(min_mass-(m1+m2+m3));
end
if m1+m2+m3 > max_mass
    loss = loss + penalty*(m1+m2+m3-max_mass);
end

% conditions on inertia tensors: triangle inequalities

% % link 1
% I1 = [I1xx,I1xy,I1xz ; I1xy,I1yy,I1yz ; I1xz,I1yz,I1zz];
% loss = check_inertia_condition(I1,loss,penalty);
% link 2
I2 = [I2xx,0,0 ; 0,I2yy,0 ; 0,0,I2zz];
loss = check_inertia_condition(I2,loss,penalty);
% link 3
I3 = [I3xx,0,0 ; 0,I3yy,0 ; 0,0,I3zz];
loss = check_inertia_condition(I3,loss,penalty);

end