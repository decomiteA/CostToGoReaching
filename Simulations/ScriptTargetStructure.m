% The aim of this script is to investigate the impact of target structure
% on goal-directed behavior in human reaching movements.

clc; close all; clear all; format long;
tau = 0.06; m = 1; G = 0.1; dt = 0.01; nsteps = 55;

A1 = [1-G*dt/m 0 0 0 dt/m 0 dt/m 0;
    0 1-G*dt/m 0 0 0 dt/m 0 dt/m;
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

w1 = 1; w2 = 100;
QN = zeros(16);
QN(1,1) = w1; QN(1,9) = -w1;  QN(9,1) = -w1;  QN(9,9) = w1;
QN(2,2) = w1; QN(2,10) = -w1; QN(10,2) = -w1; QN(10,10) = w1;
QN(3,3) = w2; QN(3,11) = -w2; QN(11,3) = -w2; QN(11,11) = w2;
QN(4,4) = 0.01*w2; QN(4,12) = -0.01*w2; QN(12,4) = -0.01*w2; QN(12,12) = 0.01*w2;
QR = QN;
QR(3,3) = 0; QR(3,11) = 0; QR(11,3) = 0; QR(3,11) = 0;

Qi = QN;
Qi(3,3) = w2/1000; Qi(3,11) = -w2/1000; Qi(11,11) = w2/1000; Qi(11,3) = -w2/1000;

Q = zeros(16,16,nsteps+1);
R = 1e-6 * eye(2);
oXmotor = zeros(16);
oXmotor(5:6,5:6) = 0.02*eye(2);
oXmotor(7:8,7:8) = oXmotor(5:6,5:6);
oMeasure = 0.0001*eye(16);
H = eye(16); Sig = oMeasure;

x0 = [zeros(8,1); 0; 0; 0; 0.25; 0; 0; 0; 0];
% Adding the delay to the system
[Adel,Bdel,Hdel,x0del,Qdel,QNdel,OmegaXidel,OmegaOmdel,Sigdel] = addDelay2(Ae,Be,H,x0,5,10,Q,QN,oXmotor,oMeasure,Sig);
[~,~,~,~,~,QRdel,oMeasureDel,~,~] = addDelay2(Ae,Be,H,x0,5,10,Q,QR,oXmotor,oMeasure,Sig);
[~,~,~,~,~,Qidel,oMeasureDel,~,~] = addDelay2(Ae,Be,H,x0,5,10,Q,Qi,oXmotor,oMeasure,Sig);

% Computing the controller gains

[L,S,s] =  computeLQ(Qdel,QNdel,R,Adel,Bdel,nsteps, oMeasureDel);
[LR,SR,sr] = computeLQ(Qdel,QRdel,R,Adel,Bdel,nsteps, oMeasureDel);
[Li,Si,si] = computeLQ(Qdel,Qidel,R,Adel,Bdel,nsteps, oMeasureDel);

% Running the simulations

n_simulations = 25;
x_mat  =  zeros(length(x0del),nsteps+1,n_simulations); xest_mat = zeros(length(x0del),nsteps+1,n_simulations);
x_matF  = zeros(length(x0del),nsteps+1,n_simulations); xest_matF = zeros(length(x0del),nsteps+1,n_simulations);
xi_mat  = zeros(length(x0del),nsteps+1,n_simulations); xiest_mat = zeros(length(x0del),nsteps+1,n_simulations);
xi_matF = zeros(length(x0del),nsteps+1,n_simulations); xiest_matF = zeros(length(x0del),nsteps+1,n_simulations);
xR_mat  = zeros(length(x0del),nsteps+1,n_simulations); xRest_mat = zeros(length(x0del),nsteps+1,n_simulations);
xR_matF = zeros(length(x0del),nsteps+1,n_simulations); xRest_matF = zeros(length(x0del),nsteps+1,n_simulations);

for ii = 1 : n_simulations
    [x_mat(:,:,ii),xest_mat(:,:,ii)]   = apply_OFC(x0del,x0del,Sigdel,OmegaXidel,OmegaOmdel,Adel,Bdel,Hdel,nsteps,L );
    [x_matF(:,:,ii),xest_matF(:,:,ii)]  = apply_OFCF(x0del,x0del,Sigdel,OmegaXidel,OmegaOmdel,Adel,Bdel,Hdel,nsteps,L );
    [xi_mat(:,:,ii),xiest_mat(:,:,ii)]  = apply_OFC(x0del,x0del,Sigdel,OmegaXidel,OmegaOmdel,Adel,Bdel,Hdel,nsteps,Li);
    [xi_matF(:,:,ii),xiest_matF(:,:,ii)] = apply_OFCF(x0del,x0del,Sigdel,OmegaXidel,OmegaOmdel,Adel,Bdel,Hdel,nsteps,Li);
    [xR_mat(:,:,ii),xRest_mat(:,:,ii)]  = apply_OFC(x0del,x0del,Sigdel,OmegaXidel,OmegaOmdel,Adel,Bdel,Hdel,nsteps,LR);
    [xR_matF(:,:,ii),xRest_matF(:,:,ii)] = apply_OFCF(x0del,x0del,Sigdel,OmegaXidel,OmegaOmdel,Adel,Bdel,Hdel,nsteps,LR);
end

FigureMeanTargetStructure;

FigureTestC2G;


%%
% In this section we implement on one side a switch between different
% control policies and a continuous adaptation of target size (MPC
% applied to optimal feedback control).
% TODO
% Change the state dimension and the dimensions of the subsequent matrices
% to handle change in target structure
clc;
% Let's start by comparing the mpc vs the classical implementation
Q_MPC = cell(nsteps,1);
for ii = 1 : length(Q_MPC)
    Q_MPC{ii} = QNdel;
end
[L_MPC,~] = computeLQMPC(Adel,Bdel,nsteps,Q_MPC,R);
n_simulations = 25;
x_classical = zeros(length(x0del),nsteps+1,n_simulations);
x_mpc =       zeros(length(x0del),nsteps+1,n_simulations);

x_classicalF = zeros(length(x0del),nsteps+1,n_simulations);
x_mpcF       = zeros(length(x0del),nsteps+1,n_simulations);

for ii = 1 : n_simulations
    [x_classical(:,:,ii),~] = apply_OFC(x0del,x0del,Sigdel,OmegaXidel,OmegaOmdel,Adel,Bdel,Hdel,nsteps,L);
    [x_mpc(:,:,ii),~] = apply_OFC(x0del,x0del,Sigdel,OmegaXidel,OmegaOmdel,Adel,Bdel,Hdel,nsteps,L_MPC);
    [x_classicalF(:,:,ii),~] = apply_OFCF(x0del,x0del,Sigdel,OmegaXidel,OmegaOmdel,Adel,Bdel,Hdel,nsteps,L);
    [x_mpcF(:,:,ii),~] = apply_OFCF(x0del,x0del,Sigdel,OmegaXidel,OmegaOmdel,Adel,Bdel,Hdel,nsteps,L_MPC);
end

FigureMPCvsClassical;

% We don't observe any difference between the Mpc and the LQG formulation
% in terms of behavior.

%%
% Let's now compare the behavior observed when the target has changed
% during movement (from rectangle to square for instance)
%
Q_MPC_RC = cell(nsteps,1);
Q_MPC_CR = cell(nsteps,1);
for ii = 1 : length(Q_MPC_RC)
    if (ii <= 35)
        Q_MPC_CR{ii} = QNdel;
        Q_MPC_RC{ii} = QRdel;
    else
        Q_MPC_CR{ii} = QRdel;
        Q_MPC_RC{ii} = QNdel;
    end
end

L_MPC_CR = cat(3,L(:,:,1:35), LR(:,:,36:end));
S_MPC_CR = cat(3,S(:,:,1:35), SR(:,:,36:end));
s_MPC_CR = [s(1:35),sr(36:end)];
L_MPC_RC = cat(3,LR(:,:,1:35), L(:,:,36:end));
S_MPC_RC = cat(3,SR(:,:,1:35), S(:,:,36:end));
s_MPC_RC = [sr(1:35),s(36:end)];

x_rectangle = zeros(length(x0del),nsteps+1,n_simulations); xestrectangle = zeros(length(x0del),nsteps+1,n_simulations);
x_square = zeros(length(x0del),nsteps+1,n_simulations); xestsquare = zeros(length(x0del),nsteps+1,n_simulations);
x_switchRC = zeros(length(x0del),nsteps+1,n_simulations); xestswRC = zeros(length(x0del),nsteps+1,n_simulations);
x_switchCR = zeros(length(x0del),nsteps+1,n_simulations); xestswCR = zeros(length(x0del),nsteps+1,n_simulations);

x_rectangleF = zeros(length(x0del),nsteps+1,n_simulations); xestrectangleF = zeros(length(x0del),nsteps+1,n_simulations);
x_squareF = zeros(length(x0del),nsteps+1,n_simulations); xestsquareF = zeros(length(x0del),nsteps+1,n_simulations);
x_switchRCF = zeros(length(x0del),nsteps+1,n_simulations); xestswRCF = zeros(length(x0del),nsteps+1,n_simulations);
x_switchCRF = zeros(length(x0del),nsteps+1,n_simulations); xestswCRF = zeros(length(x0del),nsteps+1,n_simulations);

for ii = 1:n_simulations
    [x_rectangle(:,:,ii),xestrectangle(:,:,ii)] = apply_OFC(x0del,x0del,Sigdel,OmegaXidel,OmegaOmdel,Adel,Bdel,Hdel,nsteps,LR);
    [x_square(:,:,ii),xestsquare(:,:,ii)] = apply_OFC(x0del,x0del,Sigdel,OmegaXidel,OmegaOmdel,Adel,Bdel,Hdel,nsteps,L);
    [x_switchRC(:,:,ii),xestswRC(:,:,ii)] = apply_OFC(x0del,x0del,Sigdel,OmegaXidel,OmegaOmdel,Adel,Bdel,Hdel,nsteps,L_MPC_RC);
    [x_switchCR(:,:,ii),xestswCR(:,:,ii)] = apply_OFC(x0del,x0del,Sigdel,OmegaXidel,OmegaOmdel,Adel,Bdel,Hdel,nsteps,L_MPC_CR);
    [x_rectangleF(:,:,ii),xestrectangleF(:,:,ii)] = apply_OFCF(x0del,x0del,Sigdel,OmegaXidel,OmegaOmdel,Adel,Bdel,Hdel,nsteps,LR);
    [x_squareF(:,:,ii),xestsquareF(:,:,ii)] = apply_OFCF(x0del,x0del,Sigdel,OmegaXidel,OmegaOmdel,Adel,Bdel,Hdel,nsteps,L);
    [x_switchRCF(:,:,ii),xestswRCF(:,:,ii)] = apply_OFCF(x0del,x0del,Sigdel,OmegaXidel,OmegaOmdel,Adel,Bdel,Hdel,nsteps,L_MPC_RC);
    [x_switchCRF(:,:,ii),xestswCRF(:,:,ii)] = apply_OFCF(x0del,x0del,Sigdel,OmegaXidel,OmegaOmdel,Adel,Bdel,Hdel,nsteps,L_MPC_CR);
end

FigureMPCSquareRectangle;

%%
% Let's model the behavior of the second experiment (mainly the slow and
% fast conditions that are super interesting to investigate)
Q_MPC_slow = cell(nsteps,1);
Q_MPC_fast = cell(nsteps,1);
Q_MPC_switch = cell(nsteps,1);

slow_vec = linspace(0,w2/1000,nsteps);
fast_vec = linspace(0,w2/100,nsteps);
x_vec = 15:nsteps;
slow_sig = [zeros(1,14), (w2/1000)./(1+exp(-(x_vec-10)/3))];
fast_sig = [zeros(1,14), (w2/100)./(1+exp(-(x_vec-10)/3))];
for ii = 1 : length(Q_MPC_slow)
    Q_tmps = QN; Q_tmps(11,3) = -slow_sig(ii); Q_tmps(3,11) = -slow_sig(ii); Q_tmps(3,3) = slow_sig(ii); Q_tmps(11,11) = slow_sig(ii);
    Q_tmpf = QN; Q_tmpf(11,3) = -fast_sig(ii); Q_tmpf(3,11) = -fast_sig(ii); Q_tmpf(3,3) = fast_sig(ii); Q_tmpf(11,11) = fast_sig(ii);
    [~,~,~,~,~,Q_tmpsdel,~,~,~] = addDelay2(Ae,Be,H,x0,5,10,Q,Q_tmps,oXmotor,oMeasure,Sig);
    [~,~,~,~,~,Q_tmpfdel,~,~,~] = addDelay2(Ae,Be,H,x0,5,10,Q,Q_tmpf,oXmotor,oMeasure,Sig);
    Q_MPC_slow{ii} = Q_tmpsdel;
    Q_MPC_fast{ii} = Q_tmpfdel;    
    if ii <=10
        Q_MPC_switch{ii} = QRdel;
    else
        Q_MPC_switch{ii} = QNdel;
    end
end

[L_MPC_slow,~] = computeLQMPC(Adel,Bdel,nsteps,Q_MPC_slow,R);
[L_MPC_fast,~] = computeLQMPC(Adel,Bdel,nsteps,Q_MPC_fast,R);
[L_MPC_switch,~] = computeLQMPC(Adel,Bdel,nsteps,Q_MPC_switch,R);

x_rectangle = zeros(length(x0del),nsteps+1,n_simulations); xest_rectangle = zeros(length(x0del),nsteps+1,n_simulations);
x_switch = zeros(length(x0del),nsteps+1,n_simulations); xest_switch = zeros(length(x0del),nsteps+1,n_simulations);
x_slow = zeros(length(x0del),nsteps+1,n_simulations); xest_slow = zeros(length(x0del),nsteps+1,n_simulations);
x_fast = zeros(length(x0del),nsteps+1,n_simulations); xest_fast = zeros(length(x0del),nsteps+1,n_simulations);
x_rectangleF = zeros(length(x0del),nsteps+1,n_simulations); xest_rectangleF = zeros(length(x0del),nsteps+1,n_simulations);
x_switchF = zeros(length(x0del),nsteps+1,n_simulations); xest_switchF = zeros(length(x0del),nsteps+1,n_simulations);
x_slowF = zeros(length(x0del),nsteps+1,n_simulations); xest_slowF = zeros(length(x0del),nsteps+1,n_simulations);
x_fastF = zeros(length(x0del),nsteps+1,n_simulations); xest_fastF = zeros(length(x0del),nsteps+1,n_simulations);

for ii = 1:n_simulations
    [x_rectangleF(:,:,ii),xest_rectangleF(:,:,ii)] = apply_OFCF(x0del,x0del,Sigdel,OmegaXidel,OmegaOmdel,Adel,Bdel,Hdel,nsteps,LR);
    [x_switchF(:,:,ii),xest_switchF(:,:,ii)] = apply_OFCF(x0del,x0del,Sigdel,OmegaXidel,OmegaOmdel,Adel,Bdel,Hdel,nsteps,L_MPC_switch);
    [x_slowF(:,:,ii),xest_slowF(:,:,ii)] = apply_OFCF(x0del,x0del,Sigdel,OmegaXidel,OmegaOmdel,Adel,Bdel,Hdel,nsteps,L_MPC_slow);
    [x_fastF(:,:,ii),xest_fastF(:,:,ii)] = apply_OFCF(x0del,x0del,Sigdel,OmegaXidel,OmegaOmdel,Adel,Bdel,Hdel,nsteps,L_MPC_fast);
    [x_rectangle(:,:,ii),xest_rectangle(:,:,ii)] = apply_OFC(x0del,x0del,Sigdel,OmegaXidel,OmegaOmdel,Adel,Bdel,Hdel,nsteps,LR);
    [x_switch(:,:,ii),xest_switch(:,:,ii)] = apply_OFC(x0del,x0del,Sigdel,OmegaXidel,OmegaOmdel,Adel,Bdel,Hdel,nsteps,L_MPC_switch);
    [x_slow(:,:,ii),xest_slow(:,:,ii)] = apply_OFC(x0del,x0del,Sigdel,OmegaXidel,OmegaOmdel,Adel,Bdel,Hdel,nsteps,L_MPC_slow);
    [x_fast(:,:,ii),xest_fast(:,:,ii)] = apply_OFC(x0del,x0del,Sigdel,OmegaXidel,OmegaOmdel,Adel,Bdel,Hdel,nsteps,L_MPC_fast);
end
FigureMPCSlowFast;
FigureControllerGains; 
