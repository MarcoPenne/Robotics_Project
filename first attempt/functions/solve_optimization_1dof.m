function [loss, solution] = solve_optimization_1dof(Y, u, LB, UB, X0)

    num_of_samples = size(Y,1); 

    num_of_runs = 3; % independent (parallelizable) runs of the entire algorithm (29)

    num_x = length(LB); % number of parameters

    LOSSES = Inf*ones(num_of_runs,1);

    SOL = zeros(num_x,num_of_runs);
    OUTPUTS = cell(num_of_runs,1);
    EXITFLAGS = zeros(num_of_runs,1);

    for i=1:num_of_runs

        if i~=1
            X0 = rand(num_x,1).*(UB-LB) + LB; % random initial point inside bounds
        end
        
        options = optimoptions('patternsearch','Display', 'off', 'UseParallel',true); % use Nelder-Mead optimization as hybrid function

        [X,FVAL,EXITFLAG,OUTPUT] = patternsearch(@(x) error_fcn_gM_LMI_regressor_1dof(x, Y, u), X0, [], [], [], [],LB,UB,[],options);

        LOSSES(i) = FVAL;
        SOL(:,i) = X;
        OUTPUTS{i} = OUTPUT;
        EXITFLAGS(i) = EXITFLAG;
    end

    % retrieve optimal solution
    min_idx = find(LOSSES==min(LOSSES));
    solution = SOL(:,min_idx);
    loss = LOSSES(min_idx);

end