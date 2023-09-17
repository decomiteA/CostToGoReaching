function [X,xest,c2g] = apply_OFC_DM_MPS(x0,x0est,Sigma0,omega_mot,omega_sen,A,B,H,nsteps,L_family,S_family,s_family,force)
%apply_OFC_DM_MPS simulates reaching movements controlled by a LQG
%controller and implements the forward recursion to computes the feedback
%gains. This version of the code is written to work with three potential
%goal targets.
%
% INPUTS
% ======
% - x0 and x0est are the initial values of the state and state estimate
% - Sigma0 is the initial value of the covariance of the state estimate
% - omega_mot and omega_sen are the covariance matrices of the motor and
% sensory noise
% - A, B, and H are the matrices capturing the state space equations
% - nsteps is the movement horizon
% - L_family, S_family, and s_family are the results of the backward
% recursion computing the feedback gains and the parameters required for
% the evaluation of the cost-to-go
% - force is the value of the perturbation (occuring after 20 timestep and
% oriented along the x-axis).
% 
% OUTPUTS
% =======
% - X contains the time-varying state vectors
% - xest contains the time-varying estimations of the state vector
% - c2g contains the cost-to-go values associated with each target
%
% @ Antoine de Comite


X =  zeros(length(x0),nsteps+1);
X(:,1) = x0;
xest = x0est;
Sigma = Sigma0;
c2g1 = zeros(nsteps,1); c2g2 = zeros(nsteps,1); c2g3 = zeros(nsteps,1);

for ii = 2 :nsteps+1
    c2g1(ii) = xest(:,ii-1)' * S_family{1}(:,:,ii-1) * xest(:,ii-1) + s_family{1}(ii-1);
    c2g2(ii) = xest(:,ii-1)' * S_family{2}(:,:,ii-1) * xest(:,ii-1) + s_family{2}(ii-1);
    c2g3(ii) = xest(:,ii-1)' * S_family{3}(:,:,ii-1) * xest(:,ii-1) + s_family{3}(ii-1);
    [~,idx_min] = min([c2g1(ii), c2g2(ii), c2g3(ii)]); % Selection of the best target based on the current values of the cost-to-go
    L = L_family{idx_min};
    X(:,ii) = A * X(:,ii-1) - B * L(:,:,ii-1) * xest(:,ii-1) + mvnrnd(zeros(length(x0),1),omega_mot)';
    y = H * X(:,ii-1) + mvnrnd(zeros(size(H,1),1),omega_sen)';
    
    K = (A * Sigma*  H') / (H * Sigma * H' + omega_sen);

    xest(:,ii) = A * xest(:,ii-1) - B * L(:,:,ii-1) * xest(:,ii-1) + K*(y - H * xest(:,ii-1));
    
    Sigma = omega_mot + (A - K * H) * Sigma * A';
    
    if ii == 20 
        X(7,ii) = force;
    end
end
c2g = {c2g1,c2g2,c2g3};
end