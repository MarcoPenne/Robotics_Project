function [loss, solution] = solve_optimization_pinv(Y, u, LB, UB, X0)
%SOLVE_OPTIMIZATION_PINV Summary of this function goes here
%   Detailed explanation goes here

a = pinv(Y)*u;
u_tilde = Y*a;
error = u-u_tilde;
loss = error'*error;
solution = a;
end