function [L_input,S_input,s_input] = ComputeInputs(Q,R,A,B,omegaXi,nsteps,s0)
%ComputeInputs solve the backward recursion of the OFC problem for a set of
%targets.
%
% INPUTS
% ======
% - Q and R are the error and motor penalty terms in the cost function
% - A and B capture the state space dynamics
% - omegaXi is the covariance matrix of the motor noise
% - nsteps is the movement horizon
% - s0 is the vector of initial bias terms
%
% OUTPUTS
% =======
% - L_input contains the time-varying feedback gains 
% - S_input contains the time-varying cost-to-go parameters
% - s_input contains the time-varying cost-to-go bias
%
% @Antoine de Comite

n_red = length(Q);
L_input = cell(n_red,1);
S_input = cell(n_red,1);
s_input = cell(n_red,1);
for ii = 1 : n_red
    L_matrix = zeros(size(B,2),size(B,1),nsteps);
    S_matrix = zeros(size(Q{ii},1), size(Q{ii},1),nsteps+1);
    s_vector = zeros(1,nsteps+1);
    s_vector(end) = s0(ii);
    S_matrix(:,:,end) = Q{ii};
    for jj = 1 : nsteps
       L_matrix(:,:,nsteps+1-jj) = (R+B'*S_matrix(:,:,nsteps-jj+2)*B)\(B'*S_matrix(:,:,nsteps-jj+2)*A);
       S_matrix(:,:,nsteps+1-jj) = A'*S_matrix(:,:,nsteps-jj+2)*(A-B*L_matrix(:,:,nsteps-jj+1));
       s_vector(nsteps+1-jj) = s_vector(nsteps+2-jj) + trace(S_matrix(:,:,nsteps+2-jj)*omegaXi);
    end
    L_input{ii} = L_matrix;
    S_input{ii} = S_matrix;
    s_input{ii} = s_vector;
end
end