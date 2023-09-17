function Q_targets = computeQ(w1,w2,n_red)
%computeQ computes the error cost at the end of the time horizon for
%redundant targets. 
%
% INPUTS
% ======
% - w1 and w2 are the x- and y-error terms for each individual matrix
% - n_red is the number of targets composing the redundant target
%
% OUTPUTS
% =======
% - Q_targets is the total cost matrix capturing redundant target
%
% @ Antoine de Comite
Q_targets = cell(n_red,1);
for ii = 1 : n_red
   Q_tmp = zeros((n_red+1)*8);
   Q_tmp(1,1) = w1; Q_tmp(1,ii*8+1) = -w1; Q_tmp(ii*8+1,1) = -w1; Q_tmp(ii*8+1,ii*8+1) = w1;
   Q_tmp(2,2) = w1; Q_tmp(2,ii*8+2) = -w1; Q_tmp(ii*8+2,2) = -w1; Q_tmp(ii*8+2,ii*8+2) = w1;
   Q_tmp(3,3) = w2; Q_tmp(3,ii*8+3) = -w2; Q_tmp(ii*8+3,3) = -w2; Q_tmp(ii*8+3,ii*8+3) = w2;
   Q_tmp(4,4) = w2; Q_tmp(4,ii*8+4) = -w2; Q_tmp(ii*8+4,4) = -w2; Q_tmp(ii*8+4,ii*8+4) = w2;
   Q_targets{ii} = Q_tmp;
    
end
end