function [nearest_targets] = findNearestTarget(input_positions)
% the goal of this function is to find the closest target for each entry of
% the input_positions
% input_positions has to be a column vector
% @Antoine de Comite 
tar_pos = [-0.05 0 0.05];
tar_pos = repmat(tar_pos,length(input_positions),1);
dist_mat = (tar_pos - input_positions).^2;
[~,nearest_targets] = min(dist_mat,[],2);


end