% Generation of Figure 3
% @ Antoine de Comite 

close all;

figure('Name','Modeling of experiment 3','units','normalized','OuterPosition',[0 0 1 1]);
subplot(2,2,1); hold on; set(gca,'Color','none'); set(gca,'LineWidth',2); set(gca,'FontSize',14);
plot(mean(x_rectangleF(3,:,:),3),mean(x_rectangleF(4,:,:),3),'k','LineWidth',2);
plot(mean(x_switchF(3,:,:),3),mean(x_switchF(4,:,:),3),'m','LineWidth',2);
plot(mean(x_slowF(3,:,:),3),mean(x_slowF(4,:,:),3),'g','LineWidth',2);
plot(mean(x_fastF(3,:,:),3),mean(x_fastF(4,:,:),3),'b','LineWidth',2);
rectangle('Position',[-0.01 0.24 0.02 0.02],'Curvature',[1 1],'EdgeColor','m','LineWidth',1,'LineStyle','--');
rectangle('Position',[-0.07 0.24 0.14 0.02],'Curvature',[1 1],'EdgeColor','g','LineWidth',1,'LineStyle','--');
rectangle('Position',[-0.03 0.24 0.06 0.02],'Curvature',[1 1],'EdgeColor','b','LineWidth',1,'LineStyle','--');
rectangle('Position',[-0.12 0.24 0.24 0.02],'Curvature',[1 1],'EdgeColor','k','LineWidth',1,'LineStyle','--');
title('Mean hand path'); xlabel('x-position [m]'); ylabel('y-position [m]');

subplot(2,2,2); hold on; set(gca,'Color','none'); set(gca,'LineWidth',2); set(gca,'FontSize',14); title('y-position');
xlabel('Time [s]'); ylabel('y-position [m]');
plot(1:size(x_rectangleF,2), mean(x_rectangleF(4,:,:),3),'k','LineWidth',2);
plot(1:size(x_rectangleF,2), mean(x_switchF(4,:,:),3),'m','LineWidth',2);
plot(1:size(x_rectangleF,2), mean(x_slowF(4,:,:),3),'g','LineWidth',2);
plot(1:size(x_rectangleF,2), mean(x_fastF(4,:,:),3),'b','LineWidth',2);
xticks([0 10 20 30 40 50 60]); xticklabels({'0','0.1','0.2','0.3','0.4','0.5','0.6'});

subplot(2,2,3); hold on; set(gca,'Color','none'); set(gca,'LineWidth',2); set(gca,'FontSize',14); title('x-position');
xlabel('Time [s]'); ylabel('x-position [m]');
p1 = plot(1:size(x_rectangleF,2), mean(x_rectangleF(3,:,:),3),'k','LineWidth',2);
p2 = plot(1:size(x_rectangleF,2), mean(x_switchF(3,:,:),3),'m','LineWidth',2);
p3 = plot(1:size(x_rectangleF,2), mean(x_slowF(3,:,:),3),'g','LineWidth',2);
p4 = plot(1:size(x_rectangleF,2), mean(x_fastF(3,:,:),3),'b','LineWidth',2);
xticks([0 10 20 30 40 50 60]); xticklabels({'0','0.1','0.2','0.3','0.4','0.5','0.6'});

plot(1:size(x_rectangleF,2), mean(x_rectangleF(3,:,:),3) + std(x_rectangleF(3,:,:),0,3),'k:','LineWidth',2);
plot(1:size(x_rectangleF,2), mean(x_rectangleF(3,:,:),3) - std(x_rectangleF(3,:,:),0,3),'k:','LineWidth',2);
plot(1:size(x_rectangleF,2), mean(x_switchF(3,:,:),3) + std(x_switchF(3,:,:),0,3),'m:','LineWidth',2);
plot(1:size(x_rectangleF,2), mean(x_switchF(3,:,:),3) - std(x_switchF(3,:,:),0,3),'m:','LineWidth',2);
plot(1:size(x_rectangleF,2), mean(x_slowF(3,:,:),3) + std(x_slowF(3,:,:),0,3),'g:','LineWidth',2);
plot(1:size(x_rectangleF,2), mean(x_slowF(3,:,:),3) - std(x_slowF(3,:,:),0,3),'g:','LineWidth',2);
plot(1:size(x_rectangleF,2), mean(x_fastF(3,:,:),3) + std(x_fastF(3,:,:),0,3),'b:','LineWidth',2);
plot(1:size(x_rectangleF,2), mean(x_fastF(3,:,:),3) - std(x_fastF(3,:,:),0,3),'b:','LineWidth',2);
legend([p1,p2,p3,p4],{'No change','Switch','Slow','Fast'}); xline(25,'k');

subplot(4,2,6); hold on; set(gca,'Color','none'); set(gca,'LineWidth',2); set(gca,'FontSize',14); title('x-velocity');
plot(1:size(x_rectangleF,2), mean(x_rectangleF(1,:,:),3),'k','LineWidth',2);
plot(1:size(x_rectangleF,2), mean(x_switchF(1,:,:),3),'m','LineWidth',2);
plot(1:size(x_rectangleF,2), mean(x_slowF(1,:,:),3),'g','LineWidth',2);
plot(1:size(x_rectangleF,2), mean(x_fastF(1,:,:),3),'b','LineWidth',2);
xticks([0 10 20 30 40 50 60]); xticklabels({'0','0.1','0.2','0.3','0.4','0.5','0.6'});

mc_rectangleF = zeros(2,nsteps,n_simulations);
mc_switchF = zeros(2,nsteps,n_simulations);
mc_slowF = zeros(2,nsteps,n_simulations);
mc_fastF = zeros(2,nsteps,n_simulations);
for ii = 1 : size(mc_rectangleF,2)
    for jj = 1 : n_simulations
        mc_rectangleF(:,ii,jj) = -LR(:,:,ii)*xest_rectangleF(:,ii,jj);
        mc_switchF(:,ii,jj) = -L_MPC_switch(:,:,ii)*xest_switchF(:,ii,jj);
        mc_slowF(:,ii,jj) = -L_MPC_slow(:,:,ii)*xest_slowF(:,ii,jj);
        mc_fastF(:,ii,jj) = -L_MPC_fast(:,:,ii)*xest_fastF(:,ii,jj);
    end
end
xline(25,'k');

subplot(4,2,8); hold on; set(gca,'Color','none'); set(gca,'LineWidth',2); set(gca,'FontSize',14); title('x-motor command');
xline(25,'k');
plot(1:size(x_rectangleF,2)-1, -mean(mc_rectangleF(1,:,:),3),'k','LineWidth',2);
plot(1:size(x_rectangleF,2)-1, -mean(mc_switchF(1,:,:),3),'m','LineWidth',2);
plot(1:size(x_rectangleF,2)-1, -mean(mc_slowF(1,:,:),3),'g','LineWidth',2);
plot(1:size(x_rectangleF,2)-1, -mean(mc_fastF(1,:,:),3),'b','LineWidth',2);
xticks([0 10 20 30 40 50 60]); xticklabels({'0','0.1','0.2','0.3','0.4','0.5','0.6'});
xlabel('Time [s]');