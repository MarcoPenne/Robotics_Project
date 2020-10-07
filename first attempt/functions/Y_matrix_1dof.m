function [Y] = Y_matrix_1dof(q, ddq)
Y = [sin(q) ddq];
end