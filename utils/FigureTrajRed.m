% Representation of the left panel of Figure 5
% @ Antoine de Comite 

% Comparison of the the different reward distributions
n_simulations=50; close all;
%%
close all;
vec_red = linspace(0, 0.8, n_simulations);
figure('Name','Representation','units','normalized'); hold on;
subplot(2,3,1); hold on; set(gca,'Color','none'); set(gca,'FontSize',14); set(gca,'LineWidth',2);
for ii = 1 : n_simulations
    plot(x_redundancyc(3,:,ii),x_redundancyc(4,:,ii),'Color',[1,vec_red(ii),vec_red(ii)],'LineWidth',1);
end
plot(x_s*1.1,-2*s0+0.26,'k:','LineWidth',2);
xlim([-0.07 0.07]); ylim([-0.05 0.35]);
rectangle('Position',[-0.055 0.24 0.11 0.02],'LineWidth',2);

subplot(2,3,2); hold on; set(gca,'Color','none'); set(gca,'FontSize',14); set(gca,'LineWidth',2);
for ii = 1 : n_simulations
    plot(x_redundancyl(3,:,ii),x_redundancyl(4,:,ii),'Color',[vec_red(ii),vec_red(ii),1],'LineWidth',1);
end
plot(x_s*1.1,-2*s0_left+0.26,'k:','LineWidth',2);
xlim([-0.07 0.07]); ylim([-0.05 0.35]);
rectangle('Position',[-0.055 0.24 0.11 0.02],'LineWidth',2);

subplot(2,3,3); hold on; set(gca,'Color','none'); set(gca,'FontSize',14); set(gca,'LineWidth',2);
for ii = 1 : n_simulations
    plot(x_redundancyr(3,:,ii),x_redundancyr(4,:,ii),'Color',[vec_red(ii),vec_red(ii),vec_red(ii)],'LineWidth',1);
end
plot(x_s*1.1,-2*s0_right+0.26,'k:','LineWidth',2);
xlim([-0.07 0.07]); ylim([-0.05 0.35]);
rectangle('Position',[-0.055 0.24 0.11 0.02],'LineWidth',2);

subplot(2,3,4); hold on; set(gca,'Color','none'); set(gca,'FontSize',14); set(gca,'LineWidth',2);
for ii = 1 : n_simulations
    plot(x_redundancycf(3,:,ii),x_redundancycf(4,:,ii),'Color',[1,vec_red(ii),vec_red(ii)],'LineWidth',1);
end
plot(x_s*1.1,-2*s0+0.26,'k:','LineWidth',2);
xlim([-0.07 0.07]); ylim([-0.05 0.35]);
rectangle('Position',[-0.055 0.24 0.11 0.02],'LineWidth',2);

subplot(2,3,5); hold on; set(gca,'Color','none'); set(gca,'FontSize',14); set(gca,'LineWidth',2);
for ii = 1 : n_simulations
    plot(x_redundancylf(3,:,ii),x_redundancylf(4,:,ii),'Color',[vec_red(ii),vec_red(ii),1],'LineWidth',1);
end
plot(x_s*1.1,-2*s0_left+0.26,'k:','LineWidth',2);
xlim([-0.07 0.07]); ylim([-0.05 0.35]);
rectangle('Position',[-0.055 0.24 0.11 0.02],'LineWidth',2);

subplot(2,3,6); hold on; set(gca,'Color','none'); set(gca,'FontSize',14); set(gca,'LineWidth',2);
for ii = 1 : n_simulations
    plot(x_redundancyrf(3,:,ii),x_redundancyrf(4,:,ii),'Color',[vec_red(ii),vec_red(ii),vec_red(ii)],'LineWidth',1);
end
plot(x_s*1.1,-2*s0_right+0.26,'k:','LineWidth',2);
xlim([-0.07 0.07]); ylim([-0.05 0.35]);
rectangle('Position',[-0.055 0.24 0.11 0.02],'LineWidth',2);
