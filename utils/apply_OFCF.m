function [X, xest] = apply_OFCF(x0,x0est,Sigma0,omega_mot,omega_sen,A,B,H,nsteps,L)
%apply_OFCF simulates reaching movements and perform the forward recursion
%for the computation of the estimator gains in presence of mechanical
%perturbations.
%
% INPUTS
% ======
% - x0 and x0est are the initial value for the state and state estimate
% vectors 
% - Sigma0 is the initial covariance matrix for the state estimation vector
% - omega_mot and omega_sen are the covariance matrices of the motor and
% sensory noises
% - A and B are the matrices capturing the dynamics of the system
% - H is the observability matrix
% - nsteps is the movement horizon
% - L is the time-varying matrices of feedback gains
%
% OUTPUTS
% =======
% - X contains the time-varying state vectors
% -xest contains the time-varying state estimation vectors
%
% @ Antoine de Comite
X = zeros(length(x0),nsteps+1);
xest = X;
X(:,1) = x0;
xest(:,1) = x0est;
Sigma = Sigma0;
for ii = 2:nsteps+1
    
    X(:,ii) = A * X(:,ii-1) - B * L(:,:,ii-1) * xest(:,ii-1) + mvnrnd(zeros(length(x0),1), omega_mot)';
    
    y = H * X(:,ii-1) + mvnrnd(zeros(size(H,1),1), omega_sen)';

    K = (A * Sigma * H') / (H * Sigma * H' + omega_sen);
    
    xest(:,ii) = A * xest(:,ii-1) - B * L(:,:,ii-1) * xest(:,ii-1) + K * (y - H * xest(:,ii-1));

    Sigma = omega_mot + (A - K * H) * Sigma * A';
    
    if ii == 25
        X(7,ii) = 10;
    end

end

end