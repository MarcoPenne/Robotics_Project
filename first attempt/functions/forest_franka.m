function [Y_final, u_final, signs_finals] = forest_franka(Y_segments, u_segments, LB, UB, n_segments, n_trees, maximum_depth)
    
    estimated_signs = zeros(n_segments, 1);
    %load data/franka_emika_panda/sub_trees_signs estimated_signs
    
    for i=1:n_trees
        str2disp = sprintf("COMPUTING TREE %d of %d", i, n_trees);
        disp(str2disp);
        
        [~,~,signs] = subset_tree_franka(Y_segments, u_segments, LB, UB, n_segments, maximum_depth)
        estimated_signs = estimated_signs+signs;
        save data/franka_emika_panda/sub_trees_signs estimated_signs
    end
    
    signs_finals = sign(estimated_signs);
    u_final = [];
    Y_final = [];

    for i=1:length(signs)
        if signs_finals(i) ~= 0
            u_final = [u_final; signs_finals(i) * u_segments{i}];
            Y_final = [Y_final; Y_segments{i}];
        end
    end
    
end