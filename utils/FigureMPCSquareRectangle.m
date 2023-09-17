% Generating Figure 2
%
% @Antoine de Comite
close all;

figure('Name','Application of MPC to switch from wide to narrow','units','normalized','OuterPosition',[0 0 1 1]);
subplot(2,2,1);hold on; set(gca,'Color','none'); set(gca,'FontSize',14); set(gca,'LineWidth',2); title('Mean hand path');
xlabel('x-position [m]'); ylabel('y-position [m]');
plot(mean(x_square(3,:,:),3),mean(x_square(4,:,:),3),'r','LineWidth',2);
plot(mean(x_rectangle(3,:,:),3),mean(x_rectangle(4,:,:),3),'b','LineWidth',2);
plot(mean(x_switchCR(3,:,:),3),mean(x_switchCR(4,:,:),3),'r:','LineWidth',2);
plot(mean(x_switchRC(3,:,:),3),mean(x_switchRC(4,:,:),3),'b:','LineWidth',2);

plot(mean(x_squareF(3,:,:),3),mean(x_squareF(4,:,:),3),'r','LineWidth',2);
plot(mean(x_rectangleF(3,:,:),3),mean(x_rectangleF(4,:,:),3),'b','LineWidth',2);
plot(mean(x_switchCRF(3,:,:),3),mean(x_switchCRF(4,:,:),3),'r:','LineWidth',2);
plot(mean(x_switchRCF(3,:,:),3),mean(x_switchRCF(4,:,:),3),'b:','LineWidth',2);
rectangle('Position',[-0.01 0.24 0.02 0.02],'Curvature',[1 1],'EdgeColor','r','LineWidth',1,'LineStyle','--');
rectangle('Position',[-0.12 0.24 0.24 0.02],'Curvature',[1 1],'EdgeColor','b','LineWidth',1,'LineStyle','--');

subplot(2,2,2); hold on; set(gca,'Color','none'); set(gca,'LineWidth',2); set(gca,'FontSize',14); title('y-position');
xlabel('Time [ms]'); ylabel('y-position [m]');
plot(1:size(x_square,2),mean(x_squareF(4,:,:),3),'r','LineWidth',2);
plot(1:size(x_square,2),mean(x_rectangleF(4,:,:),3),'b','LineWidth',2);
plot(1:size(x_square,2),mean(x_switchCRF(4,:,:),3),'r:','LineWidth',2);
plot(1:size(x_square,2),mean(x_switchRCF(4,:,:),3),'b:','LineWidth',2);
xticks([0 10 20 30 40 50 60]); xticklabels({'0','0.1','0.2','0.3','0.4','0.5','0.6'});
xline(20);
xline(25);

subplot(2,2,3); hold on; set(gca,'Color','none'); set(gca,'LineWidth',2); set(gca,'FontSize',14); title('x-position');
xlabel('Time [ms]'); ylabel('x-position [m]');
p1 = plot(1:size(x_square,2),mean(x_squareF(3,:,:),3),'r','LineWidth',2);
p2 = plot(1:size(x_square,2),mean(x_rectangleF(3,:,:),3),'b','LineWidth',2);
p3 = plot(1:size(x_square,2),mean(x_switchCRF(3,:,:),3),'r:','LineWidth',2);
p4 = plot(1:size(x_square,2),mean(x_switchRCF(3,:,:),3),'b:','LineWidth',2);
xline(20);
xline(25);

plot(1:size(x_square,2),mean(x_squareF(3,:,:),3) + std(x_squareF(3,:,:),0,3),'r--','LineWidth',2);
plot(1:size(x_square,2),mean(x_squareF(3,:,:),3) - std(x_squareF(3,:,:),0,3),'r--','LineWidth',2);
plot(1:size(x_square,2),mean(x_switchCRF(3,:,:),3) + std(x_switchCRF(3,:,:),0,3),'r--','LineWidth',2);
plot(1:size(x_square,2),mean(x_switchCRF(3,:,:),3) - std(x_switchCRF(3,:,:),0,3),'r--','LineWidth',2);
plot(1:size(x_square,2),mean(x_rectangleF(3,:,:),3) + std(x_rectangleF(3,:,:),0,3),'b--','LineWidth',2);
plot(1:size(x_square,2),mean(x_rectangleF(3,:,:),3) - std(x_rectangleF(3,:,:),0,3),'b--','LineWidth',2);
plot(1:size(x_square,2),mean(x_switchRCF(3,:,:),3) + std(x_switchRCF(3,:,:),0,3),'b--','LineWidth',2);
plot(1:size(x_square,2),mean(x_switchRCF(3,:,:),3) - std(x_switchRCF(3,:,:),0,3),'b--','LineWidth',2);
xticks([0 10 20 30 40 50 60]); xticklabels({'0','0.1','0.2','0.3','0.4','0.5','0.6'});
legend([p1,p2,p3,p4],{'Narrow','Wide','Narrow to wide','Wide to narrow'});


subplot(4,2,6); hold on; set(gca,'Color','none'); set(gca,'LineWidth',2); set(gca,'FontSize',14); title('x-velocity');
xline(0.20);
xline(0.25);

plot((1:size(x_square,2))/100,mean(x_squareF(1,:,:),3),'r','LineWidth',2);
plot((1:size(x_square,2))/100,mean(x_rectangleF(1,:,:),3),'b','LineWidth',2);
plot((1:size(x_square,2))/100,mean(x_switchCRF(1,:,:),3),'r:','LineWidth',2);
plot((1:size(x_square,2))/100,mean(x_switchRCF(1,:,:),3),'b:','LineWidth',2);

mc_squareF = zeros(2,nsteps-1,n_simulations);
mc_rectangleF = zeros(2,nsteps-1,n_simulations);
mc_switchCRF = zeros(2,nsteps-1,n_simulations);
mc_switchRCF = zeros(2,nsteps-1,n_simulations);
for ii = 1 : size(mc_rectangleF,2)
    for jj = 1:n_simulations
        mc_squareF(:,ii,jj) = -L (:,:,ii)*xestsquareF(:,ii,jj);
        mc_rectangleF(:,ii,jj) = -LR(:,:,ii)*xestrectangleF(:,ii,jj);
        mc_switchCRF(:,ii,jj) = -L_MPC_CR(:,:,ii)*xestswCRF(:,ii,jj);
        mc_switchRCF(:,ii,jj) = -L_MPC_RC(:,:,ii)*xestswRCF(:,ii,jj);
    end
end

subplot(4,2,8); hold on; set(gca,'Color','none'); set(gca,'LineWidth',2); set(gca,'FontSize',14); title('x-motor command');
xlabel('Time [ms]');
xline(0.20);
xline(0.25);
plot((2:size(x_square,2)-1)/100,-mean(mc_squareF(1,:,:),3),'r','LineWidth',2);
plot((2:size(x_square,2)-1)/100,-mean(mc_rectangleF(1,:,:),3),'b','LineWidth',2);
plot((2:size(x_square,2)-1)/100,-mean(mc_switchCRF(1,:,:),3),'r:','LineWidth',2);
plot((2:size(x_square,2)-1)/100,-mean(mc_switchRCF(1,:,:),3),'b:','LineWidth',2);


% Let's quickly test F-tests here 

[hredx,predx] = vartest2(squeeze(x_mat(3,end,:)),squeeze(x_switchCR(3,end,:)));
[hredy,predy] = vartest2(squeeze(x_mat(4,end,:)),squeeze(x_switchCR(4,end,:)));
[hblux,pblux] = vartest2(squeeze(xR_mat(3,end,:)),squeeze(x_switchRC(3,end,:)));
[hbluy,pbluy] = vartest2(squeeze(xR_mat(4,end,:)),squeeze(x_switchRC(4,end,:)));