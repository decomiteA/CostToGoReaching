% Generation of the right panel of Figure 5
% @ Antoine de Comite

close all; 
vec_rgba = linspace(0,0.8,n_simulations);
figure('Name','Investigate the switch','units','normalized'); hold on;
subplot(2,3,1); hold on; set(gca,'LineWidth',2); set(gca,'FontSize',14); set(gca,'Color','none'); set(gca,'XColor','none'); set(gca,'YColor','none');
for ii = 1 : n_simulations
    plot(x_switchf(3,:,ii),x_switchf(4,:,ii),'Color',[1 vec_rgba(ii) vec_rgba(ii)],'LineWidth',1);
end
xlim([-0.06 0.06]); 
rectangle('Position',[-0.055 0.24 0.11 0.02],'LineWidth',2);
plot([-0.05 -0.04 -0.03 -0.02 -0.01 0 0.01 0.02 0.03 0.04 0.05],-2*s0_left+0.26,'k','LineWidth',2);
xlim([-0.07 0.07]); ylim([-0.05 0.35]);

subplot(2,3,4); hold on; set(gca,'LineWidth',2); set(gca,'FontSize',14); set(gca,'Color','none'); set(gca,'XColor','none'); set(gca,'YColor','none');
for ii = 1 : n_simulations
    plot(x_switch(3,:,ii),x_switch(4,:,ii),'Color',[1 vec_rgba(ii) vec_rgba(ii)],'LineWidth',1);
end
xlim([-0.06 0.06]); 
rectangle('Position',[-0.055 0.24 0.11 0.02],'LineWidth',2);
plot([-0.05 -0.04 -0.03 -0.02 -0.01 0 0.01 0.02 0.03 0.04 0.05],-2*s0_left+0.26,'k:','LineWidth',2);
plot([-0.05 -0.04 -0.03 -0.02 -0.01 0 0.01 0.02 0.03 0.04 0.05],-2*s0_right+0.26,'k','LineWidth',2);
xlim([-0.07 0.07]); ylim([-0.05 0.35]);

subplot(2,3,2); hold on; set(gca,'LineWidth',2); set(gca,'FontSize',14); set(gca,'Color','none'); set(gca,'XColor','none'); set(gca,'YColor','none');
for ii = 1 : n_simulations
    plot(x_switchr(3,:,ii),x_switchr(4,:,ii),'Color',[1 vec_rgba(ii) vec_rgba(ii)],'LineWidth',1);
end
xlim([-0.06 0.06]); 
rectangle('Position',[-0.055 0.24 0.11 0.02],'LineWidth',2);
plot([-0.05 -0.04 -0.03 -0.02 -0.01 0 0.01 0.02 0.03 0.04 0.05],-2*s0_right+0.26,'k','LineWidth',2); 
xlim([-0.07 0.07]); ylim([-0.05 0.35]);

subplot(2,3,5); hold on; set(gca,'LineWidth',2); set(gca,'FontSize',14); set(gca,'Color','none'); set(gca,'XColor','none'); set(gca,'YColor','none');
for ii = 1 : n_simulations
    plot(x_switchrf(3,:,ii),x_switchrf(4,:,ii),'Color',[1 vec_rgba(ii) vec_rgba(ii)],'LineWidth',1);
end
xlim([-0.06 0.06]); 
rectangle('Position',[-0.055 0.24 0.11 0.02],'LineWidth',2);
plot([-0.05 -0.04 -0.03 -0.02 -0.01 0 0.01 0.02 0.03 0.04 0.05],-2*s0_right+0.26,'k','LineWidth',2);
plot([-0.05 -0.04 -0.03 -0.02 -0.01 0 0.01 0.02 0.03 0.04 0.05],-2*s0_left+0.26,'k:','LineWidth',2);
xlim([-0.07 0.07]); ylim([-0.05 0.35]);

