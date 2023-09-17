function [X,xest,idx_targets] = apply_OFC_DM_red_force(x0,x0est,Sigma0,omega_mot,omega_sen,A,B,H,nsteps,L_family,S_family,s_family,force)
%apply_OFC_DM_red_force simulates reaching movements controlled by a LQG
%controller and implements the forward recursion to computes the feedback
%gains. This version of the code is written to work with a redundant target
%in presence of perturbation
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
% - force is the intensity of the mechanical perturbation
% 
% OUTPUTS
% =======
% - X contains the time-varying state vectors
% - xest contains the time-varying estimations of the state vector
% - idx_targets contains the time-varying current target (decided based on
% the cost-to-go)
%
% @ Antoine de Comite


X = zeros(length(x0),nsteps+1);
X(:,1) = x0;
xest = x0est;
Sigma = Sigma0;
c2g1 = zeros(length(L_family),1);
idx_targets = zeros(nsteps,1);

for ii = 2:nsteps+1
    for jj = 1 : length(c2g1)
        c2g1(jj) = xest(:,ii-1)' * S_family{jj}(:,:,ii-1) * xest(:,ii-1)+ s_family{jj}(ii-1);
    end
    [~,idx_min] = min(c2g1);
    if (idx_min==1 && c2g1(1)==c2g1(end))
        idx_tmp = randi(2);
        switch idx_tmp
            case 1
                idx_min = 1;
            case 2
                idx_min = length(c2g1);
        end
    end
    idx_targets(ii) = idx_min;
    L = L_family{idx_min};
    
    X(:,ii) = A * X(:,ii-1) - B * L(:,:,ii-1) * xest(:,ii-1) + mvnrnd(zeros(length(x0),1),omega_mot)';
    y = H * X(:,ii-1) + mvnrnd(zeros(size(H,1),1),omega_sen)';
    
    K = (A * Sigma*  H') / (H * Sigma * H' + omega_sen);
    
    xest(:,ii) = A * xest(:,ii-1) - B * L(:,:,ii-1) * xest(:,ii-1) + K*(y - H * xest(:,ii-1));
    
    Sigma = omega_mot + (A - K * H) * Sigma * A';
    
    if ii == 25
        X(7,ii) = force;
    end
    
end
end

