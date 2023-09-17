function [L_matrix,S_matrix,s_vector] = computeLQC2Gr(Q, QN, R, A, B, omegaXi, nsteps,s0)
% computeLQC2Gr implements the backward recursion to compute the feedback
% gains of the finite horizon LQG controller.
%
% INPUTS
% ======
% - Q, QN, and R are the cost term associated with the running error cost,
% the final cost error and the motor cost
% - A and B are the matrices capturing the dynamics of the linera plant
% - omegaXi is the covariance matrix of the motor noise
% - nsteps is the duration of the movement horizon
% - s0 is the bias term for the cost-to-go, to be changed to study
% different reward value
%
% OUTPUTS
% =======
% - L_matrix contains the time-varying feedback gains matrices
% - S_matrix contains the time-varying terms used to computed the
% cost-to-go
% - s_vector contains the time-varying evolution of the bias term of the
% cost-to-go
%
% @ Antoine de Comite


L_matrix = zeros(size(B,2), size(B,1), nsteps);
S_matrix = zeros(size(QN,1), size(QN,1), nsteps+1);
s_vector = zeros(1,nsteps+1);
s_vector(end) = s0;
S_matrix(:,:,end) = QN;

for ii = 1 : nsteps
    L_matrix(:,:,nsteps+1-ii) = (R+B'*S_matrix(:,:,nsteps-ii+2)*B)\(B'*S_matrix(:,:,nsteps-ii+2)*A);
    S_matrix(:,:,nsteps+1-ii) = Q(:,:,nsteps+1-ii) + A'*S_matrix(:,:,nsteps-ii+2)*(A-B*L_matrix(:,:,nsteps-ii+1));
    s_vector(nsteps+1-ii) = s_vector(nsteps+2-ii) + trace(S_matrix(:,:,nsteps+2-ii)*omegaXi);
end
end