function[L_matrix, S_matrix] = computeLQMPC(A,B,nsteps,QN,R)
%computeLQMPC performs the backward recursion for the set of QN matrices
%provided as an input. This should be able to encapsulate any change in
%target structure during movement.
%
% INPUTS
% ======
% - A and B are the matrices capturing the dynamics of the system
% - nsteps is the movement horizon
% - QN contains the time-varying error cost, capturing the change in target
% structure
% - R is the motor cost
%
% OUTPUTS
% =======
% - L_matrix contains the time-varying feedback gains
% - S_matrix contains the time-varying cost-to-go terms
%
% @Antoine de Comite
L_matrix = zeros(size(B,2), size(B,1), nsteps);
S_matrix = zeros(size(QN{1},1), size(QN{1},1), nsteps+1);
S_matrix(:,:,end) = QN{length(QN)};

for ii = 1 : size(L_matrix,3)
    L_tmp = zeros(size(B,2), size(B,1),ii);
    S_tmp = zeros(size(QN{1},1), size(QN{1},1),ii+1);
    S_tmp(:,:,end) = QN{nsteps-ii+1};
    for jj = 1 : size(L_tmp,3)
        L_tmp(:,:,size(L_tmp,3)+1-jj) = (R+B'*S_tmp(:,:,size(S_tmp,3)+1-jj)*B)\(B'*S_tmp(:,:,size(S_tmp,3)+1-jj)*A);
        S_tmp(:,:,size(S_tmp,3)-jj) = A'*S_tmp(:,:,size(S_tmp,3)+1-jj)*(A-B*L_tmp(:,:,size(L_tmp,3)-jj+1));
    end
    L_matrix(:,:,nsteps-ii+1) = L_tmp(:,:,1);
    S_matrix(:,:,nsteps-ii+1) = S_tmp(:,:,1);
end
end