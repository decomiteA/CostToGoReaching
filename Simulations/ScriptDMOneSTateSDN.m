% The goal of this script is to test another version of the decision making
% during movement thing... 


clear all; close all; clc;
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
 
% We will extend the state once for each target...
 
Ae = [A1 zeros(size(A1,1),size(A1,2)*3);
       zeros(size(A1)), eye(size(A1)), zeros(size(A1,1),size(A1,2)*2);
       zeros(size(A1,1),size(A1,2)*2), eye(size(A1)), zeros(size(A1));
       zeros(size(A1,1),size(A1,2)*3), eye(size(A1))];
   
Be = [B1; zeros(size(B1,1)*3, size(B1,2))];

w1 = 1; w2 = 100;
QN_left = zeros(32);
QN_cent = zeros(32);
QN_righ = zeros(32);
QN_left(1,1) = w1; QN_left(1,9) = -w1; QN_left(9,1) = -w1; QN_left(9,9) = w1;
QN_left(2,2) = w1; QN_left(2,10) = -w1; QN_left(10,2) = -w1; QN_left(10,10) = w1;
QN_left(3,3) = 100*w2; QN_left(3,11) = -100*w2; QN_left(11,3) = -100*w2; QN_left(11,11) = 100*w2;
QN_left(4,4) = 100*w2; QN_left(4,12) = -100*w2; QN_left(12,4) = -100*w2; QN_left(12,12) = 100*w2;

QN_cent(1,1) = w1; QN_cent(1,17) = -w1; QN_cent(17,1) = -w1; QN_cent(17,17) = w1;
QN_cent(2,2) = w1; QN_cent(2,18) = -w1; QN_cent(18,2) = -w1; QN_cent(18,18) = w1;
QN_cent(3,3) = 100*w2; QN_cent(3,19) = -100*w2; QN_cent(19,3) = -100*w2; QN_cent(19,19) = 100*w2;
QN_cent(4,4) = w2; QN_cent(4,20) = -w2; QN_cent(20,4) = -w2; QN_cent(20,20) = w2;

QN_righ(1,1) = w1; QN_righ(1,25) = -w1; QN_righ(25,1) = -w1; QN_righ(25,25) = w1;
QN_righ(2,2) = w1; QN_righ(2,26) = -w1; QN_righ(26,2) = -w1; QN_righ(26,26) = w1;
QN_righ(3,3) = 100*w2; QN_righ(3,27) = -100*w2; QN_righ(27,3) = -100*w2; QN_righ(27,27) = 100*w2;
QN_righ(4,4) = w2; QN_righ(4,28) = -w2; QN_righ(28,4) = -w2; QN_righ(28,28) = w2;
Q = zeros(32,32,nsteps);
R = 1e-4 * eye(2);
oXmotor = zeros(32);
oXmotor(5:6,5:6) = 0.15*eye(2);
oXmotor(7:8,7:8) = oXmotor(5:6,5:6);
oMeasure = 0.0001*eye(32);
H = eye(32); Sig = 0.0001*eye(32);
x0 = zeros(32,1); x0(11:12) = [-0.05 0.25]; x0(19:20) = [0 0.25]; x0(27:28) = [0.05 0.25];
x0est = x0;
% Adding the delay
[Adel,Bdel,Hdel,x0del,Qdel,QNdel_left,OmegaXidel,OmegaOmdel,Sigdel] = addDelay2(Ae,Be,H,x0,5,10,Q,QN_left,oXmotor,oMeasure,Sig);
[~,~,~,~,~,QNdel_cent,~,~,~] = addDelay2(Ae,Be,H,x0,5,10,Q,QN_cent,oXmotor,oMeasure,Sig);
[~,~,~,~,~,QNdel_righ,~,~,~] = addDelay2(Ae,Be,H,x0,5,10,Q,QN_righ,oXmotor,oMeasure,Sig);

% Signal depedent noise 
C1 = Bdel * [0.004 0; 0 0];
C2 = Bdel * [0 0; 0 0.004];
Cdel = zeros(size(Bdel,1),2,2);
Cdel(:,:,1) = C1; Cdel(:,:,2) = C2;
Ddel = 0.00005*Hdel;
%%
% Computing the controller gains 
[L_left,S_left,Se_left,s_left,K_left] = solve_iteratively_LQDM(Adel,Bdel,Cdel,Ddel,Hdel,R,QNdel_left,OmegaOmdel,OmegaXidel,nsteps,300,0);
[L_righ,S_righ,Se_righ,s_righ,K_righ] = solve_iteratively_LQDM(Adel,Bdel,Cdel,Ddel,Hdel,R,QNdel_righ,OmegaOmdel,OmegaXidel,nsteps,300,0);
[L_cent,S_cent,Se_cent,s_cent,K_cent] = solve_iteratively_LQDM(Adel,Bdel,Cdel,Ddel,Hdel,R,QNdel_cent,OmegaOmdel,OmegaXidel,nsteps,300,0);

[L_leftr,S_leftr,Se_leftr,s_leftr,K_leftr] = solve_iteratively_LQDM(Adel,Bdel,Cdel,Ddel,Hdel,R,QNdel_left,OmegaOmdel,OmegaXidel,nsteps,300,5e-2);
[L_righr,S_righr,Se_righr,s_righr,K_righr] = solve_iteratively_LQDM(Adel,Bdel,Cdel,Ddel,Hdel,R,QNdel_righ,OmegaOmdel,OmegaXidel,nsteps,300,5e-2);
[L_centr,S_centr,Se_centr,s_centr,K_centr] = solve_iteratively_LQDM(Adel,Bdel,Cdel,Ddel,Hdel,R,QNdel_cent,OmegaOmdel,OmegaXidel,nsteps,300,5e-2);

[L_leftr2,S_leftr2,Se_leftr2,s_leftr2,K_leftr2] = solve_iteratively_LQDM(Adel,Bdel,Cdel,Ddel,Hdel,R,QNdel_left,OmegaOmdel,OmegaXidel,nsteps,300,2.5e-2);
[L_righr2,S_righr2,Se_righr2,s_righr2,K_righr2] = solve_iteratively_LQDM(Adel,Bdel,Cdel,Ddel,Hdel,R,QNdel_righ,OmegaOmdel,OmegaXidel,nsteps,300,2.5e-2);
[L_centr2,S_centr2,Se_centr2,s_centr2,K_centr2] = solve_iteratively_LQDM(Adel,Bdel,Cdel,Ddel,Hdel,R,QNdel_cent,OmegaOmdel,OmegaXidel,nsteps,300,2.5e-2);

L_family = {L_left,L_cent,L_righ}; S_family = {S_left,S_cent,S_righ}; s_family = {s_left,s_cent,s_righ}; Se_family = {Se_left,Se_cent,Se_righ};
L_familyr = {L_leftr,L_centr,L_righr}; S_familyr = {S_leftr,S_centr,S_righr}; s_familyr = {s_leftr,s_centr,s_righr}; Se_familyr = {Se_leftr,Se_centr,Se_righr};
L_familyr2 = {L_leftr2,L_centr2,L_righr2}; S_familyr2 = {S_leftr2,S_centr2,S_righr2}; s_familyr2 = {s_leftr2,s_centr2,s_righr2}; Se_familyr2 = {Se_leftr2,Se_centr2,Se_righr2};
%%
% Modeling with changes in the force value 
n_simulations = 30;
force_vector = linspace(-5,5,5);
x_pert = zeros(length(x0del),nsteps+1,n_simulations,length(force_vector));
x_pertr = zeros(length(x0del),nsteps+1,n_simulations,length(force_vector));
x_pertr2 = zeros(length(x0del),nsteps+1,n_simulations,length(force_vector));
nearestTarget_xpert = zeros(n_simulations,length(force_vector));
nearestTarget_xpertr = zeros(n_simulations,length(force_vector));
nearestTarget_xpertr2 = zeros(n_simulations,length(force_vector));
for jj = 1 : length(force_vector)
   for ii = 1 : n_simulations
      [x_pert(:,:,ii,jj),~,c2g1] = apply_OFC_DM_MPS(x0del,x0del,Sigdel,OmegaXidel,OmegaOmdel,Adel,Bdel,Hdel,nsteps,L_family,S_family,s_family,force_vector(jj));
      [x_pertr(:,:,ii,jj),~,c2g2] = apply_OFC_DM_MPS(x0del,x0del,Sigdel,OmegaXidel,OmegaOmdel,Adel,Bdel,Hdel,nsteps,L_familyr,S_familyr,s_familyr,force_vector(jj));
      [x_pertr2(:,:,ii,jj),~,c2g3] = apply_OFC_DM_MPS(x0del,x0del,Sigdel,OmegaXidel,OmegaOmdel,Adel,Bdel,Hdel,nsteps,L_familyr2,S_familyr2,s_familyr2,force_vector(jj));
   end
   nearestTarget_xpert(:,jj) = findNearestTarget(reshape(x_pert(3,end,:,jj),size(x_pert,3),1));
   nearestTarget_xpertr(:,jj) = findNearestTarget(reshape(x_pertr(3,end,:,jj),size(x_pertr,3),1));
   nearestTarget_xpertr2(:,jj) = findNearestTarget(reshape(x_pertr2(3,end,:,jj),size(x_pertr2,3),1));
end

FigureFullState;
% ScriptStatsMnrFit;
% Add some investigation of the triggered event based on the difference in
% reward values between the two targets 
% TriggeredEvents; 

%%
% Modeling with signal dependent noise ... 
% For each target solve the control problem and save L, S, s, and K
C1 = Bdel * [0.004 0; 0 0];
C2 = Bdel * [0 0; 0 0.004];
Cdel = zeros(size(Bdel,1),2,2);
Cdel(:,:,1) = C1; Cdel(:,:,2) = C2;
Ddel = 0.00005*Hdel;
n_iters=50;
% Computing the controller gains and cost to go elements for each target
% All the targets have the same reward 
[L_left_sdn,S_left_sdn,Se_left_sdn,s_left_sdn,K_left_sdn] = solve_iteratively_LQDM(Adel,Bdel,Cdel,Ddel,Hdel,R,QNdel_left,OmegaOmdel,OmegaXidel,nsteps,n_iters,0);
[L_cent_sdn,S_cent_sdn,Se_cent_sdn,s_cent_sdn,K_cent_sdn] = solve_iteratively_LQDM(Adel,Bdel,Cdel,Ddel,Hdel,R,QNdel_cent,OmegaOmdel,OmegaXidel,nsteps,n_iters,0);
[L_righ_sdn,S_righ_sdn,Se_righ_sdn,s_righ_sdn,K_righ_sdn] = solve_iteratively_LQDM(Adel,Bdel,Cdel,Ddel,Hdel,R,QNdel_righ,OmegaOmdel,OmegaXidel,nsteps,n_iters,0);

% Central target more rewarding
[L_left_sdn_r,S_left_sdn_r,Se_left_sdn_r,s_left_sdn_r,K_left_sdn_r] = solve_iteratively_LQDM(Adel,Bdel,Cdel,Ddel,Hdel,R,QNdel_left,OmegaOmdel,OmegaXidel,nsteps,n_iters,5e-2);
[L_cent_sdn_r,S_cent_sdn_r,Se_cent_sdn_R,s_cent_sdn_r,K_cent_sdn_r] = solve_iteratively_LQDM(Adel,Bdel,Cdel,Ddel,Hdel,R,QNdel_cent,OmegaOmdel,OmegaXidel,nsteps,n_iters,0);
[L_righ_sdn_r,S_righ_sdn_r,Se_righ_sdn_r,s_righ_sdn_r,K_righ_sdn_r] = solve_iteratively_LQDM(Adel,Bdel,Cdel,Ddel,Hdel,R,QNdel_righ,OmegaOmdel,OmegaXidel,nsteps,n_iters,5e-2);
%% 
% Let's implement michalski experiment
% Participants are moving toward a target and, as they where moving a new
% target appears at another location. The targets appear on a circle or
% radius r=12.5cm and centered around the mid reaching point 
% 
% targets_matrix = 0.01 * [12.5, (sqrt(3)+1)/(2*sqrt(2))*12.5, sqrt(3)*12.5/2,(sqrt(3)-1)/(2*sqrt(2))*12.5, sqrt(2)*12.5/2, 12.5/2, 0, -12.5/2, -sqrt(2)*12.5/2, -sqrt(3)*12.5/2, -12.5,-(sqrt(3)-1)/(2*sqrt(2))*12.5,-(sqrt(3)+1)/(2*sqrt(2))*12.5;
%                   12.5, 12.5+(sqrt(3)-1)/(2*sqrt(2))*12.5, 3*12.5/2,12.5+(sqrt(3)+1)/(2*sqrt(2))*12.5, 12.5+sqrt(2)*12.5/2, 12.5+sqrt(3)*12.5/2, 25, 12.5+sqrt(3)*12.5/2, 12.5+sqrt(2)*12.5/2, 3*12.5/2, 12.5,12.5+(sqrt(3)+1)/(2*sqrt(2))*12.5,12.5+(sqrt(3)-1)/(2*sqrt(2))*12.5];
%               
% % FigureTargets;
% n_targets = size(targets_matrix,2);
% 
% % Parameters of the system
% tau = 0.04; m = 1; G = 0.1; dt = 0.01; nsteps = 55;
% A1 = [1-G*dt/m 0 0 0 dt/m 0 dt/m 0;
%       0 1-G*dt 0 0 0 dt/m 0 dt/m;
%       dt 0 1 0 0 0 0 0;
%       0 dt 0 1 0 0 0 0;
%       0 0 0 0 1-dt/tau 0 0 0;
%       0 0 0 0 0 1-dt/tau 0 0;
%       0 0 0 0 0 0 1 0;
%       0 0 0 0 0 0 0 1];
%   
% B1 = [0 0; 0 0; 0 0; 0 0; dt/tau 0; 0 dt/tau; 0 0; 0 0];
%  
% % We will extend the state once for each target...
%  
% Ae = [A1 zeros(size(A1,1),size(A1,2)*2);
%       zeros(size(A1,1)*2,size(A1,2)),eye(size(A1)*2)];
%    
% Be = [B1; zeros(size(B1,1)*2, size(B1,2))];
% 
% w1 = 1; w2 = 100;
% QN_left = zeros(24);
% QN_cent = zeros(24);
% QN_left(1,1) = w1; QN_left(1,9) = -w1; QN_left(9,1) = -w1; QN_left(9,9) = w1;
% QN_left(2,2) = w1; QN_left(2,10) = -w1; QN_left(10,2) = -w1; QN_left(10,10) = w1;
% QN_left(3,3) = 100*w2; QN_left(3,11) = -100*w2; QN_left(11,3) = -100*w2; QN_left(11,11) = 100*w2;
% QN_left(4,4) = w2; QN_left(4,12) = -w2; QN_left(12,4) = -w2; QN_left(12,12) = w2;
% 
% QN_cent(1,1) = w1; QN_cent(1,17) = -w1; QN_cent(17,1) = -w1; QN_cent(17,17) = w1;
% QN_cent(2,2) = w1; QN_cent(2,18) = -w1; QN_cent(18,2) = -w1; QN_cent(18,18) = w1;
% QN_cent(3,3) = 100*w2; QN_cent(3,19) = -100*w2; QN_cent(19,3) = -100*w2; QN_cent(19,19) = 100*w2;
% QN_cent(4,4) = w2; QN_cent(4,20) = -w2; QN_cent(20,4) = -w2; QN_cent(20,20) = w2;
% Q = zeros(24,24,nsteps);
% R = 1e-4 * eye(2);
% oXmotor = zeros(24);
% oXmotor(5:6,5:6) = 0.15*eye(2);
% oXmotor(7:8,7:8) = oXmotor(5:6,5:6);
% oMeasure = 0.0001*eye(24);
% H = eye(24); Sig = 0.0001*eye(24);
% x0 = zeros(24,n_targets);
% for ii = 1 : n_targets
%     x0(11:12,ii) = [0 0.25]; x0(19:20,ii) = targets_matrix(:,ii);
% end
% x0est = x0;
% % Adding the delay
% [Adel,Bdel,Hdel,x0del,Qdel,QNdel_left,OmegaXidel,OmegaOmdel,Sigdel] = addDelay2(Ae,Be,H,x0,5,10,Q,QN_left,oXmotor,oMeasure,Sig);
% [~,~,~,~,~,QNdel_cent,~,~,~] = addDelay2(Ae,Be,H,x0,5,10,Q,QN_cent,oXmotor,oMeasure,Sig);
% 
% % Doing the backward recursion for all the targets
% [L_left,S_left,s_left] = computeLQC2Gr(Qdel,QNdel_left,R,Adel,Bdel,OmegaXidel,nsteps,0); % initial target
% [L_cent,S_cent,s_cent] = computeLQC2Gr(Qdel,QNdel_cent,R,Adel,Bdel,OmegaXidel,nsteps,0); % alternative target 
% 
% % Creating the cells 
% L_family = {L_left,L_cent}; S_family = {S_left,S_cent}; s_family = {s_left,0.95*s_cent};
% 
% % Modeling the behavior for the different targets 
% n_simulations = 1;
% x_pert = zeros(length(x0del),nsteps+1,n_simulations,n_targets);
% nearestTarget_xpert = zeros(n_simulations,n_targets);
% for jj = 1 : n_targets
%    for ii = 1 : n_simulations
%       [x_pert(:,:,ii,jj),~,c2g1] = apply_OFC_DM_MPSb(x0del(:,jj),x0del(:,jj),Sigdel,OmegaXidel,OmegaOmdel,Adel,Bdel,Hdel,nsteps,L_family,S_family,s_family);
%    end
%    nearestTarget_xpert(:,jj) = findNearestTargetb(reshape(x_pert(3,end,:,jj),size(x_pert,3),1));
% end
%  
% FigureTrajTargets;

%% 
% Let's try to push it to model target redundancy

targets = [-0.05 0.25;
           -0.04 0.25;
           -0.03 0.25;
           -0.02 0.25;
           -0.01 0.25;
            0    0.25;
            0.01 0.25;
            0.02 0.25;
            0.03 0.25;
            0.04 0.25;
            0.05 0.25];
n_red = size(targets,1);

A1 = [1-G*dt/m 0 0 0 dt/m 0 dt/m 0;
      0 1-G*dt 0 0 0 dt/m 0 dt/m;
      dt 0 1 0 0 0 0 0;
      0 dt 0 1 0 0 0 0;
      0 0 0 0 1-dt/tau 0 0 0;
      0 0 0 0 0 1-dt/tau 0 0;
      0 0 0 0 0 0 1 0;
      0 0 0 0 0 0 0 1];
  
B1 = [0 0; 0 0; 0 0; 0 0; dt/tau 0; 0 dt/tau; 0 0; 0 0];

Ae = extendA(A1,n_red);

   
Be = [B1; zeros(size(B1,1)*n_red, size(B1,2))];
w2=1000;
% definition des matrices Q  - il en faut une par cible 
Q_targets = computeQ(w1,w2,n_red);
Q_targets_del = cell(length(Q_targets),1);
Qint = zeros(size(Ae));
x0 = zeros(size(Ae,1),1); 
for ii = 1 : n_red
   x0(ii*8+3:ii*8+4) = targets(ii,:); 
end
oXmotor = zeros(size(Ae));
oXmotor(5:6,5:6) = 0.015*eye(2);
oXmotor(7:8,7:8) = oXmotor(5:6,5:6);
oMeasure = 0.0001*eye(size(Ae));
H = eye(size(Ae)); Sig = 0.0001*eye(size(Ae));
for ii=1:length(Q_targets)
    [Adel,Bdel,Hdel,x0del,Qdel_int,Q_targets_del{ii},OmegaXidel,OmegaOmdel,Sigdel] = addDelay2(Ae,Be,H,x0,5,10,Qint,Q_targets{ii},oXmotor,oMeasure,Sig);
end 
x_s = [-0.05 -0.04 -0.03 -0.02 -0.01 0 0.01 0.02 0.03 0.04 0.05];
s0 = -10*x_s.^2 - min(-10*x_s.^2);
s0_left = [-15*x_s(1:6).^2 -5*x_s(7:end).^2] - min([-15*x_s(1:6).^2 -5*x_s(7:end).^2]);
s0_right = fliplr(s0_left);
% calculer les trucs pour les différentes conditions 
[L_input,S_input,s_sym] = ComputeInputs(Q_targets_del,R,Adel,Bdel,OmegaXidel,nsteps,2*s0);  
[~,~,s_leftb] = ComputeInputs(Q_targets_del,R,Adel,Bdel,OmegaXidel,nsteps,2*s0_left);
[~,~,s_rightb] =ComputeInputs(Q_targets_del,R,Adel,Bdel,OmegaXidel,nsteps,2*s0_right);
%%
% simulate and create output... 
n_simulations = 50; 
x_redundancyf = zeros(length(x0del),nsteps+1,n_simulations);
x_redundancyc = zeros(length(x0del),nsteps+1,n_simulations);
x_redundancyl = zeros(length(x0del),nsteps+1,n_simulations);
x_redundancyr = zeros(length(x0del),nsteps+1,n_simulations);
x_redundancycf= zeros(length(x0del),nsteps+1,n_simulations);
nearestTargetcf = zeros(n_simulations,1);
x_redundancylf= zeros(length(x0del),nsteps+1,n_simulations);
x_redundancyrf= zeros(length(x0del),nsteps+1,n_simulations);
xestc = zeros(length(x0del),nsteps+1,n_simulations);
targetc = zeros(nsteps+1,n_simulations);
for ii = 1 : n_simulations
    [x_redundancyc(:,:,ii),~] = apply_OFC_DM_red(x0del,x0del,Sigdel,OmegaXidel,OmegaOmdel,Adel,Bdel,Hdel,nsteps,L_input,S_input,s_sym);
    [x_redundancyl(:,:,ii),~] = apply_OFC_DM_red(x0del,x0del,Sigdel,OmegaXidel,OmegaOmdel,Adel,Bdel,Hdel,nsteps,L_input,S_input,s_leftb);
    [x_redundancyr(:,:,ii),~] = apply_OFC_DM_red(x0del,x0del,Sigdel,OmegaXidel,OmegaOmdel,Adel,Bdel,Hdel,nsteps,L_input,S_input,s_rightb);
    [x_redundancycf(:,:,ii),xestc(:,:,ii),targetc(:,jj)]=apply_OFC_DM_red_force(x0del,x0del,Sigdel,OmegaXidel,OmegaOmdel,Adel,Bdel,Hdel,nsteps,L_input,S_input,s_sym,4);
    [x_redundancylf(:,:,ii),~]=apply_OFC_DM_red_force(x0del,x0del,Sigdel,OmegaXidel,OmegaOmdel,Adel,Bdel,Hdel,nsteps,L_input,S_input,s_leftb,4);
    [x_redundancyrf(:,:,ii),~]=apply_OFC_DM_red_force(x0del,x0del,Sigdel,OmegaXidel,OmegaOmdel,Adel,Bdel,Hdel,nsteps,L_input,S_input,s_rightb,4);
end

%%
nearestTargetcf = findNearestTargetc(reshape(x_redundancycf(3,end,:),n_simulations,1));
FigureTrajRed;



%%
% Implementing the change in reward distribution during movement. We will
% only consider the case where the asymetric distribution flips during
% movement. 
n_simulations = 10;
s_leftb_switch = s_leftb;
s_rightb_switch = s_rightb;
for ii = 1:length(s_leftb_switch)
    s_leftb_switch{ii} = 10*[s_leftb{ii}(1:35), s_leftb{11-ii+1}(36:end)];
    s_rightb_switch{ii} = 10*[s_rightb{ii}(1:35), s_rightb{11-ii+1}(36:end)];
end

x_switch = zeros(length(x0del),nsteps+1,n_simulations);
x_switchf= zeros(length(x0del),nsteps+1,n_simulations);
x_switchr = zeros(length(x0del),nsteps+1,n_simulations);
x_switchrf= zeros(length(x0del),nsteps+1,n_simulations);
xest_switch = zeros(length(x0del),nsteps+1,n_simulations);
xest_switchf= zeros(length(x0del),nsteps+1,n_simulations);
idx_switch = zeros(nsteps,n_simulations);
for ii = 1 : n_simulations
    [x_switch(:,:,ii),xest_switch(:,:,ii),idx_switch(:,ii)]= apply_OFC_DM_red(x0del,x0del,Sigdel,OmegaXidel,OmegaOmdel,Adel,Bdel,Hdel,nsteps,L_input,S_input,s_leftb_switch);
    [x_switchf(:,:,ii),xest_switchf(:,:,ii),~]=apply_OFC_DM_red(x0del,x0del,Sigdel,OmegaXidel,OmegaOmdel,Adel,Bdel,Hdel,nsteps,L_input,S_input,s_leftb);
    [x_switchr(:,:,ii),~,~]=apply_OFC_DM_red(x0del,x0del,Sigdel,OmegaXidel,OmegaOmdel,Adel,Bdel,Hdel,nsteps,L_input,S_input,s_rightb);
    [x_switchrf(:,:,ii),~,~]=apply_OFC_DM_red(x0del,x0del,Sigdel,OmegaXidel,OmegaOmdel,Adel,Bdel,Hdel,nsteps,L_input,S_input,s_rightb_switch);
end

FigureSwitch; 