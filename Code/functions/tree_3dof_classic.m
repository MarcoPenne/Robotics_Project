function [Y_final, u_final, signs_finals] = tree_3dof(Y1, Y2, Y3, u1, u2, u3, indices1, indices2, indices3, LB, UB)

    n_segments1 = size(indices1,1);
    n_segments2 = size(indices2,1);
    n_segments3 = size(indices3,1);
    n_segments = n_segments1 + n_segments2 + n_segments3;
    n_top_pool = 5;
    num_x = length(LB);
    TOP_POOL = zeros(n_top_pool, num_x);
    TOP_SIGNS = {zeros(n_segments,1)};
    TOP_RESULTS = {rand(num_x,1).*(UB-LB) + LB};
    TOP_LOSSES = [Inf];
    TOP_Y = {[]};
    TOP_U = {[]};
    signs = ones(n_segments,1);

    permuted_order = [];%randperm(n_segments);
    
    Y_segments = {};
    u_segments = {};
    for i=1:n_segments1
        Y_segments{i} = Y1(indices1(i, 1):indices1(i, 2), :);
        u_segments{i} = u1(indices1(i, 1):indices1(i, 2));
    end
    
    for i=1:n_segments2
        Y_segments{i+n_segments1} = Y2(indices2(i, 1):indices2(i, 2), :);
        u_segments{i+n_segments1} = u2(indices2(i, 1):indices2(i, 2));
    end
    
    for i=1:n_segments3
        Y_segments{i+n_segments1+n_segments2} = Y3(indices3(i, 1):indices3(i, 2), :);
        u_segments{i+n_segments1+n_segments2} = u3(indices3(i, 1):indices3(i, 2));
    end
    %save ws Y_segments
    
    % finding minimum amount of segments that make Y well conditioned
    best_condition = Inf;
    best_condition_indices = [1];
    i=1;
    while best_condition > 1000
        best_condition = Inf;
        ij_order = nchoosek(1:n_segments, i);
    
        for n=1:length(ij_order)
            ij = ij_order(n, :);
            Y_tmp = [];
            for l=1:length(ij)
                Y_tmp = [Y_tmp; Y_segments{ij(l)}];
            end
            c = cond(Y_tmp);
            if c<best_condition
                best_condition = c;
                best_condition_indices = ij;
            end
        end
        i = i+1;
    end
    disp("Found well-defined problem using segments")
    disp(best_condition_indices)
    for l=1:length(best_condition_indices)
        permuted_order(end+1) = best_condition_indices(l);
    end
    
    remaining_segments = 1:n_segments;
    for l=1:length(best_condition_indices)
        remaining_segments(best_condition_indices(l)-l+1) = [];
    end
    
    Y_now = [];
    for l=1:length(best_condition_indices)
        Y_now = [Y_now; Y_segments{best_condition_indices(l)}];
    end
    disp("Generating order")
    for k=1:n_segments-length(best_condition_indices)
        best_condition = Inf;
        best_condition_index = 0;
        best_condition_index_reman = 0;
        for i=1:length(remaining_segments)
            j = remaining_segments(i);
            c = cond([Y_now; Y_segments{j}]);
            if c<best_condition
                best_condition = c;
                best_condition_index = j;
                best_condition_index_reman = i;
            end
        end
        permuted_order(end+1) = best_condition_index;
        Y_now = [Y_now; Y_segments{best_condition_index}];
        remaining_segments(best_condition_index_reman) = [];
    end
    disp(permuted_order)
    
    %initializing problem
    disp("INITIALIZING DATA STRUCTURES WITH WELL-DEFINED PROBLEM ("+length(best_condition_indices)+" segments)...")
    LOSSES = [];
    SIGNS = {};
    RESULTS = {};
    Y = {};
    U = {};
    Y_now = [];
    for j=1:length(best_condition_indices)
        Y_now = [Y_now; Y_segments{permuted_order(j)}];
    end
    signs_now = zeros(1, n_segments);
    for i=0:2^length(best_condition_indices)-1
        bits = flip(dec2bin(i, length(best_condition_indices)));
        % creating U
        U_now = [];
        for j=1:length(best_condition_indices)
            if bits(j)=='0'
                U_now = [U_now; -u_segments{permuted_order(j)}];
                signs_now(permuted_order(j)) = -1;
            else
                U_now = [U_now;  u_segments{permuted_order(j)}];
                signs_now(permuted_order(j)) = 1;
            end 
        end
        
        [loss_pos, solution_pos] = solve_optimization_pinv(Y_now, U_now, LB, UB, TOP_RESULTS{1});
        LOSSES = [LOSSES, loss_pos];
        RESULTS = [RESULTS, solution_pos];
        SIGNS = [SIGNS, signs_now];
        Y{end+1} = Y_now;
        U{end+1} = U_now;
        
    end
    [~,LOSS_sort] = sort(LOSSES);
    Y = Y(LOSS_sort);
    U = U(LOSS_sort);
    RESULTS = RESULTS(LOSS_sort);
    SIGNS = SIGNS(LOSS_sort);
    LOSSES = LOSSES(LOSS_sort);

    if n_top_pool<length(LOSSES)
        TOP_Y = Y(1:n_top_pool);
        TOP_U = U(1:n_top_pool);
        TOP_LOSSES = LOSSES(1:n_top_pool);
        TOP_RESULTS = RESULTS(1:n_top_pool);
        TOP_SIGNS = SIGNS(1:n_top_pool);
    else
        TOP_Y = Y;
        TOP_U = U;
        TOP_LOSSES = LOSSES;
        TOP_RESULTS = RESULTS;
        TOP_SIGNS = SIGNS;
    end
    
    Y_final = {};
    u_final = {};
    
    LOSSES = [];
    SIGNS = {};
    RESULTS = {};
    Y = {};
    U = {};
    for k = length(best_condition_indices)+1:length(permuted_order)
        i = permuted_order(k);
        
        string2disp = sprintf("PROCESSING SEGMENT %d/%d  (%d)", k, n_segments, i);
        disp(string2disp);
        
        for pool=1:size(TOP_Y, 2)
            Y_now = [TOP_Y{pool}; Y_segments{i}];
            U_now = [TOP_U{pool}; u_segments{i}];
            Y_now = double(Y_now);
            U_now = double(U_now);

            Y{length(Y)+1} = Y_now;
            U{length(U)+1} = U_now;
            
            [loss_pos, solution_pos] = solve_optimization_pinv(Y_now, U_now, LB, UB, TOP_RESULTS{pool});
            LOSSES = [LOSSES, loss_pos];
            RESULTS = [RESULTS, solution_pos];
            signs_now = TOP_SIGNS{pool};
            signs_now(i) = 1;
            SIGNS = [SIGNS, signs_now];
            
            Y_now = [TOP_Y{pool}; Y_segments{i}];
            U_now = [TOP_U{pool}; -u_segments{i}];
            
            Y{length(Y)+1} = Y_now;
            U{length(U)+1} = U_now;
            
            [loss_neg, solution_neg] = solve_optimization_pinv(Y_now, U_now, LB, UB, TOP_RESULTS{pool});
            LOSSES = [LOSSES, loss_neg];
            RESULTS = [RESULTS, solution_neg];
            signs_now = TOP_SIGNS{pool};
            signs_now(i) = -1;
            SIGNS = [SIGNS, signs_now];
        end
        
        [~,LOSS_sort] = sort(LOSSES);
        Y = Y(LOSS_sort);
        U = U(LOSS_sort);
        RESULTS = RESULTS(LOSS_sort);
        SIGNS = SIGNS(LOSS_sort);
        LOSSES = LOSSES(LOSS_sort);
        
        if n_top_pool<length(LOSSES)
            TOP_Y = Y(1:n_top_pool);
            TOP_U = U(1:n_top_pool);
            TOP_LOSSES = LOSSES(1:n_top_pool);
            TOP_RESULTS = RESULTS(1:n_top_pool);
            TOP_SIGNS = SIGNS(1:n_top_pool);
        else
            TOP_Y = Y;
            TOP_U = U;
            TOP_LOSSES = LOSSES;
            TOP_RESULTS = RESULTS;
            TOP_SIGNS = SIGNS;
        end
        
        LOSSES = [];
        Y = {};
        U = {};
        RESULTS = {};
        SIGNS = {};
            
    end
     
    top_solution_returned = 2;
    Y_final = TOP_Y{1:top_solution_returned};
    u_final = TOP_U{1:top_solution_returned};
    signs_finals = TOP_SIGNS{1:top_solution_returned};

    Y_final = {};
    u_final = {};
    signs_finals = {};
    for i=1:top_solution_returned
        signs_tmp = TOP_SIGNS{i};
        signs1 = signs_tmp(1:n_segments1);
        signs2 = signs_tmp(1+n_segments1:n_segments1+n_segments2);
        signs3 = signs_tmp(1+n_segments1+n_segments2:n_segments1+n_segments2+n_segments3);
        
        signs_finals{end+1} = {signs1, signs2, signs3};
        Y_final{end+1} = TOP_Y{i};
        u_final{end+1} = TOP_U{i};
    end
end