clear all; clc; close all;


% Parameters definition
tau = 0.04; m = 1; G = 0.1; dt = 0.01; nsteps = 55;
A1 = [1-G*dt/m 0 0 0 dt/m 0 dt/m 0;
    0 1-G*dt 0 0 0 dt/m 0 dt/m;
    dt 0 1 0 0 0 0 0;
    0 dt 0 1 0 0 0 0;
    0 0 0 0 1-dt/tau 0 0 0;
    0 0 0 0 0 1-dt/tau 0 0;
    0 0 0 0 0 0 1 0;
    0 0 0 0 0 0 0 1];

B1 = [0 0; 0 0; 0 0; 0 0; dt/tau 0; 0 dt/tau; 0 0; 0 0];

Ae = [A1 zeros(size(A1));
    zeros(size(A1)), eye(size(A1))];
Be = [B1; zeros(size(B1,1),size(B1,2))];

QN_big = zeros(16);
QN_small = zeros(16);

% Defining the small target
w1 = 1; w2 = 100;
QN_small(1,1) = w1; QN_small(1,9) = -w1; QN_small(9,1) = -w1; QN_small(9,9) = w1;
QN_small(2,2) = w1; QN_small(2,10)= -w1; QN_small(10,2)= -w1; QN_small(10,10)=w1;
QN_small(3,3) = 100*w2; QN_small(3,11) = -100*w2; QN_small(11,3) = -100*w2; QN_small(11,11) = 100*w2;
QN_small(4,4) = w2; QN_small(4,12) = -w2; QN_small(12,4) = -w2; QN_small(12,12) = w2;
% Defining the large target
w1 = 1; w2 = 10000;
QN_big(1,1) = w1; QN_big(1,9) = -w1; QN_big(9,1) = -w1; QN_big(9,9) = w1;
QN_big(2,2) = w1; QN_big(2,10)= -w1; QN_big(10,2)= -w1; QN_big(10,10)=w1;
QN_big(3,3) = 100*w2; QN_big(3,11) = -100*w2; QN_big(11,3) = -100*w2; QN_big(11,11) = 100*w2;
QN_big(4,4) = w2; QN_big(4,12) = -w2; QN_big(12,4) = -w2; QN_big(12,12) = w2;

Q = zeros(16,16,nsteps);
R = 1e-6*eye(2);
oXmotor = zeros(16);
oXmotor(5:6,5:6) = 0.02*eye(2);
oXmotor(7:8,7:8) = oXmotor(5:6,5:6);
oMeasure = 0.0001*eye(16);
H = eye(16); Sig = 0.0001*eye(16);
% Defining the initial targets
x0c1 = [zeros(8,1); 0; 0; 0; 0.25; 0; 0; 0; 0]; % Central target
x0c2 = [zeros(8,1); 0; 0; -0.05; 0.25; 0; 0; 0; 0]; % Leftward target
x0c3 = [zeros(8,1); 0; 0; 0.05; 0.25; 0; 0; 0; 0]; % Rightward target

% Adding the delay

[Adel,Bdel,Hdel,x0delc1,Qdel,QNdel_big,OmegaXidel,OmegaOmdel,Sigdel] = addDelay2(Ae,Be,H,x0c1,5,10,Q,QN_big,oXmotor,oMeasure,Sig);
[~,~,~,x0delc2,~,QNdel_small,~,~,~] = addDelay2(Ae,Be,H,x0c2,5,10,Q,QN_small,oXmotor,oMeasure,Sig);
[~,~,~,x0delc3,~,QNdel3,~,~,~] = addDelay2(Ae,Be,H,x0c3,5,10,Q,QN_small,oXmotor,oMeasure,Sig);

% Preparing the inputs for the LQG with decision making
x0input = {x0delc1,x0delc2,x0delc3};
x0est = x0input;


[Ls,Ss,ss] = computeLQC2G(Qdel,QNdel_small,R,Adel,Bdel,OmegaXidel,nsteps);
[Lb,Sb,sb] = computeLQC2G(Qdel,QNdel_big,R,Adel,Bdel,OmegaXidel,nsteps);
% Task 1 : all the targets have the same size
L_task1 = {Ls,Ls,Ls}; S_task1 = {Ss,Ss,Ss}; s_task1 = {ss,ss,ss};
% Task 2 : the central target is smaller than the other 2
L_task2 = {Ls,Lb,Lb}; S_task2 = {Ss,Sb,Sb}; s_task2 = {ss,sb,sb};
%% simulations
n_simulations = 50;
force_vector = linspace(-10,10,5);
x_pert_task1 = zeros(length(x0delc1),nsteps+1,n_simulations,length(force_vector));
x_pert_task2 = zeros(length(x0delc1),nsteps+1,n_simulations,length(force_vector));

nearestTargets_task1 = zeros(n_simulations,length(force_vector));
nearestTargets_task2 = zeros(n_simulations,length(force_vector));

for jj = 1 : length(force_vector)
    for ii = 1 : n_simulations
        [x_pert_task1(:,:,ii,jj),~] = apply_OFC_DM_MP(x0input,x0est,Sigdel,OmegaXidel,OmegaOmdel,Adel,Bdel,Hdel,nsteps,L_task1,S_task1,s_task1,force_vector(jj));
        [x_pert_task2(:,:,ii,jj),~] = apply_OFC_DM_MP(x0input,x0est,Sigdel,OmegaXidel,OmegaOmdel,Adel,Bdel,Hdel,nsteps,L_task2,S_task2,s_task2,force_vector(jj));
    end
    nearestTargets_task1(:,jj) = findNearestTarget(reshape(x_pert_task1(3,end,:,jj),size(x_pert_task1,3),1));
    nearestTargets_task2(:,jj) = findNearestTarget(reshape(x_pert_task2(3,end,:,jj),size(x_pert_task2,3),1));
end

FigureDMSize;
