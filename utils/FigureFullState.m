% Generating subplots of Figure 4
% @ Antoine de Comite  

close all; clc;
vec_gradient = linspace(0,0.5, size(nearestTarget_xpert,1));
figure('Name','Representation of the individual behavior','units','normalized','outerposition',[0 0 1 1]);
subplot(3,5,1); hold on; set(gca,'Color','none'); set(gca,'FontSize',14); set(gca,'LineWidth',2); xlim([-0.1 0.1]); set(gca,'visible','off');
for ii = 1 : size(nearestTarget_xpert,1)
    switch nearestTarget_xpert(ii,1)
        case 1
            plot(x_pert(3,:,ii,1), x_pert(4,:,ii,1),'Color',[vec_gradient(ii) 0.7 vec_gradient(ii)],'LineWidth',1);
        case 2
            plot(x_pert(3,:,ii,1), x_pert(4,:,ii,1),'Color',[1 vec_gradient(ii) 1], 'LineWidth',1);
        case 3
            plot(x_pert(3,:,ii,1), x_pert(4,:,ii,1),'Color', [vec_gradient(ii) vec_gradient(ii) 1],'LineWidth',1);
    end
end
rectangle('Position',[-0.06 0.24 0.02 0.02],'Curvature',[1 1],'FaceColor','k');
rectangle('Position',[-0.01 0.24 0.02 0.02],'Curvature',[1 1],'FaceColor','k');
rectangle('Position',[ 0.04 0.24 0.02 0.02],'Curvature',[1 1],'FaceColor','k');
axis equal; ylabel('Same reward','fontweight','bold','fontsize',16);

subplot(3,5,2); hold on; set(gca,'Color','none'); set(gca,'FontSize',14); set(gca,'LineWidth',2); xlim([-0.1 0.1]); set(gca,'visible','off')
for ii = 1 : size(nearestTarget_xpert,1)
    switch nearestTarget_xpert(ii,2)
        case 1
            plot(x_pert(3,:,ii,2), x_pert(4,:,ii,2),'Color', [vec_gradient(ii), 0.7, vec_gradient(ii)], 'LineWidth',1);
        case 2
            plot(x_pert(3,:,ii,2), x_pert(4,:,ii,2),'Color', [1, vec_gradient(ii), 1],'LineWidth',1);
        case 3
            plot(x_pert(3,:,ii,2), x_pert(4,:,ii,2),'Color', [vec_gradient(ii), vec_gradient(ii), 1], 'LineWidth',1);
    end
end
rectangle('Position',[-0.06 0.24 0.02 0.02],'Curvature',[1 1],'FaceColor','k');
rectangle('Position',[-0.01 0.24 0.02 0.02],'Curvature',[1 1],'FaceColor','k');
rectangle('Position',[ 0.04 0.24 0.02 0.02],'Curvature',[1 1],'FaceColor','k');
axis equal;
disp(size(nearestTarget_xpert))
subplot(3,5,3); hold on; set(gca,'Color','none'); set(gca,'FontSize',14); set(gca,'LineWidth',2); xlim([-0.1 0.1]); set(gca,'visible','off')
for ii = 1 : size(nearestTarget_xpert,1)
    switch nearestTarget_xpert(ii,3)
        case 1
            plot(x_pert(3,:,ii,3), x_pert(4,:,ii,3),'Color',[vec_gradient(ii) 0.7 vec_gradient(ii)], 'LineWidth', 1);
        case 2
            plot(x_pert(3,:,ii,3), x_pert(4,:,ii,3),'Color',[1 vec_gradient(ii) 1], 'LineWidth', 1);
        case 3
            plot(x_pert(3,:,ii,3), x_pert(4,:,ii,3),'Color', [vec_gradient(ii) vec_gradient(ii) 1], 'LineWidth',1);
    end
end
rectangle('Position',[-0.06 0.24 0.02 0.02],'Curvature',[1 1],'FaceColor','k');
rectangle('Position',[-0.01 0.24 0.02 0.02],'Curvature',[1 1],'FaceColor','k');
rectangle('Position',[ 0.04 0.24 0.02 0.02],'Curvature',[1 1],'FaceColor','k');
axis equal;

subplot(3,5,4); hold on; set(gca,'Color','none'); set(gca,'FontSize',14); set(gca,'LineWidth',2); xlim([-0.1 0.1]); set(gca,'visible','off')
for ii = 1 : size(nearestTarget_xpert,1)
    switch nearestTarget_xpert(ii,4)
        case 1
            plot(x_pert(3,:,ii,4), x_pert(4,:,ii,4),'Color', [vec_gradient(ii), 0.7, vec_gradient(ii)], 'LineWidth', 1);
        case 2
            plot(x_pert(3,:,ii,4), x_pert(4,:,ii,4),'Color', [1, vec_gradient(ii), 1], 'LineWidth', 1);
        case 3
            plot(x_pert(3,:,ii,4), x_pert(4,:,ii,4),'Color', [vec_gradient(ii), vec_gradient(ii), 1], 'LineWidth', 1);
    end
end
rectangle('Position',[-0.06 0.24 0.02 0.02],'Curvature',[1 1],'FaceColor','k');
rectangle('Position',[-0.01 0.24 0.02 0.02],'Curvature',[1 1],'FaceColor','k');
rectangle('Position',[ 0.04 0.24 0.02 0.02],'Curvature',[1 1],'FaceColor','k');
axis equal;

subplot(3,5,5); hold on; set(gca,'Color','none'); set(gca,'FontSize',14); set(gca,'LineWidth',2); xlim([-0.1 0.1]); set(gca,'visible','off')
for ii = 1 : size(nearestTarget_xpert,1)
    switch nearestTarget_xpert(ii,5)
        case 1
            plot(x_pert(3,:,ii,5), x_pert(4,:,ii,5),'Color', [vec_gradient(ii), 0.7, vec_gradient(ii)], 'LineWidth',1);
        case 2
            plot(x_pert(3,:,ii,5), x_pert(4,:,ii,5),'Color', [1, vec_gradient(ii), 1], 'LineWidth',1);
        case 3
            plot(x_pert(3,:,ii,5), x_pert(4,:,ii,5),'Color', [vec_gradient(ii), vec_gradient(ii), 1], 'LineWidth', 1);
    end
end
rectangle('Position',[-0.06 0.24 0.02 0.02],'Curvature',[1 1],'FaceColor','k');
rectangle('Position',[-0.01 0.24 0.02 0.02],'Curvature',[1 1],'FaceColor','k');
rectangle('Position',[ 0.04 0.24 0.02 0.02],'Curvature',[1 1],'FaceColor','k');
axis equal;

%%%%%%%%%%%%%%%%
%%%Bottom row%%%
%%%%%%%%%%%%%%%%
subplot(3,5,6); hold on; set(gca,'Color','none'); set(gca,'FontSize',14); set(gca,'LineWidth',2); xlim([-0.1 0.1]); set(gca,'visible','off')
for ii = 1 : size(nearestTarget_xpertr2,1)
    switch nearestTarget_xpertr2(ii,1)
        case 1
            plot(x_pertr2(3,:,ii,1), x_pertr2(4,:,ii,1),'Color', [vec_gradient(ii), 0.7, vec_gradient(ii)], 'LineWidth',1);
        case 2
            plot(x_pertr2(3,:,ii,1), x_pertr2(4,:,ii,1),'Color', [1, vec_gradient(ii), 1], 'LineWidth',1);
        case 3
            plot(x_pertr2(3,:,ii,1), x_pertr2(4,:,ii,1),'Color', [vec_gradient(ii), vec_gradient(ii), 1], 'LineWidth',1);
    end
end
rectangle('Position',[-0.06 0.24 0.02 0.02],'Curvature',[1 1],'FaceColor','k');
rectangle('Position',[-0.01 0.24 0.02 0.02],'Curvature',[1 1],'FaceColor',[1 1 1],'EdgeColor','k');
rectangle('Position',[ 0.04 0.24 0.02 0.02],'Curvature',[1 1],'FaceColor','k');
axis equal;

subplot(3,5,7); hold on; set(gca,'Color','none'); set(gca,'FontSize',14); set(gca,'LineWidth',2); xlim([-0.1 0.1]); set(gca,'visible','off')
for ii = 1 : size(nearestTarget_xpertr2,1)
    switch nearestTarget_xpertr2(ii,2)
        case 1
            plot(x_pertr2(3,:,ii,2), x_pertr2(4,:,ii,2),'Color', [vec_gradient(ii), 0.7, vec_gradient(ii)], 'LineWidth', 1);
        case 2
            plot(x_pertr2(3,:,ii,2), x_pertr2(4,:,ii,2),'Color', [1, vec_gradient(ii), 1], 'LineWidth', 1);
        case 3
            plot(x_pertr2(3,:,ii,2), x_pertr2(4,:,ii,2),'Color', [vec_gradient(ii), vec_gradient(ii), 1], 'LineWidth', 1);
    end
end
rectangle('Position',[-0.06 0.24 0.02 0.02],'Curvature',[1 1],'FaceColor','k');
rectangle('Position',[-0.01 0.24 0.02 0.02],'Curvature',[1 1],'FaceColor',[1 1 1],'EdgeColor','k');
rectangle('Position',[ 0.04 0.24 0.02 0.02],'Curvature',[1 1],'FaceColor','k');
axis equal;

subplot(3,5,8); hold on; set(gca,'Color','none'); set(gca,'FontSize',14); set(gca,'LineWidth',2); xlim([-0.1 0.1]); set(gca,'visible','off')
for ii = 1 : size(nearestTarget_xpertr2,1)
    switch nearestTarget_xpertr2(ii,3)
        case 1
            plot(x_pertr2(3,:,ii,3), x_pertr2(4,:,ii,3),'Color', [vec_gradient(ii), 0.7, vec_gradient(ii)], 'LineWidth', 1);
        case 2
            plot(x_pertr2(3,:,ii,3), x_pertr2(4,:,ii,3),'Color', [1, vec_gradient(ii), 1], 'LineWidth', 1);
        case 3
            plot(x_pertr2(3,:,ii,3), x_pertr2(4,:,ii,3),'Color', [vec_gradient(ii), vec_gradient(ii), 1], 'LineWidth', 1);
    end
end
rectangle('Position',[-0.06 0.24 0.02 0.02],'Curvature',[1 1],'FaceColor','k');
rectangle('Position',[-0.01 0.24 0.02 0.02],'Curvature',[1 1],'FaceColor',[1 1 1],'EdgeColor','k');
rectangle('Position',[ 0.04 0.24 0.02 0.02],'Curvature',[1 1],'FaceColor','k');
axis equal;

subplot(3,5,9); hold on; set(gca,'Color','none'); set(gca,'FontSize',14); set(gca,'LineWidth',2); xlim([-0.1 0.1]); set(gca,'visible','off')
for ii = 1 : size(nearestTarget_xpertr2,1)
    switch nearestTarget_xpertr2(ii,4)
        case 1
            plot(x_pertr2(3,:,ii,4), x_pertr2(4,:,ii,4),'Color', [vec_gradient(ii), 0.7, vec_gradient(ii)], 'LineWidth', 1);
        case 2
            plot(x_pertr2(3,:,ii,4), x_pertr2(4,:,ii,4),'Color', [1, vec_gradient(ii), 1], 'LineWidth', 1);
        case 3
            plot(x_pertr2(3,:,ii,4), x_pertr2(4,:,ii,4),'Color', [vec_gradient(ii), vec_gradient(ii), 1], 'LineWidth', 1);
    end
end
rectangle('Position',[-0.06 0.24 0.02 0.02],'Curvature',[1 1],'FaceColor','k');
rectangle('Position',[-0.01 0.24 0.02 0.02],'Curvature',[1 1],'FaceColor',[1 1 1],'EdgeColor','k');
rectangle('Position',[ 0.04 0.24 0.02 0.02],'Curvature',[1 1],'FaceColor','k');
axis equal;

subplot(3,5,10); hold on; set(gca,'Color','none'); set(gca,'FontSize',14); set(gca,'LineWidth',2); xlim([-0.1 0.1]); set(gca,'visible','off')
for ii = 1 : size(nearestTarget_xpertr2,1)
    switch nearestTarget_xpertr2(ii,5)
        case 1
            plot(x_pertr2(3,:,ii,5), x_pertr2(4,:,ii,5),'Color', [vec_gradient(ii), 0.7, vec_gradient(ii)], 'LineWidth', 1);
        case 2
            plot(x_pertr2(3,:,ii,5), x_pertr2(4,:,ii,5),'Color', [1, vec_gradient(ii), 1], 'LineWidth', 1);
        case 3
            plot(x_pertr2(3,:,ii,5), x_pertr2(4,:,ii,5),'Color', [vec_gradient(ii), vec_gradient(ii), 1], 'LineWidth', 1);
    end
end
rectangle('Position',[-0.06 0.24 0.02 0.02],'Curvature',[1 1],'FaceColor','k');
rectangle('Position',[-0.01 0.24 0.02 0.02],'Curvature',[1 1],'FaceColor',[1 1 1],'EdgeColor','k');
rectangle('Position',[ 0.04 0.24 0.02 0.02],'Curvature',[1 1],'FaceColor','k');
axis equal;

%%%%%%%%%%%%%%%%%%%%%%%%%
%%% Switch proportion %%%
%%%%%%%%%%%%%%%%%%%%%%%%%

mat_for_plot_line1 = zeros(size(nearestTarget_xpert,2),3);
mat_for_plot_line2 = zeros(size(nearestTarget_xpertr,2),3);
mat_for_plot_line3 = zeros(size(nearestTarget_xpertr2,2),3);

for ii = 1 : size(mat_for_plot_line1,1)
   mat_for_plot_line1(ii,:) = [length(find(nearestTarget_xpert(:,ii)==1)), length(find(nearestTarget_xpert(:,ii)==2)), length(find(nearestTarget_xpert(:,ii)==3))]/size(nearestTarget_xpert,1);
   mat_for_plot_line2(ii,:) = [length(find(nearestTarget_xpertr(:,ii)==1)), length(find(nearestTarget_xpertr(:,ii)==2)), length(find(nearestTarget_xpertr(:,ii)==3))]/size(nearestTarget_xpertr,1);
   mat_for_plot_line3(ii,:) = [length(find(nearestTarget_xpertr2(:,ii)==1)), length(find(nearestTarget_xpertr2(:,ii)==2)), length(find(nearestTarget_xpertr2(:,ii)==3))]/size(nearestTarget_xpertr2,1);
end


figure('Name','Representation of the reach proportion as a function of force','units','normalized');
hold on; set(gca,'LineWidth',2); set(gca,'Color','none'); set(gca,'FontSize',14);
p1 = plot(force_vector,mat_for_plot_line1(:,1),'.-','Color',[0,0.5,0],'LineWidth',2,'MarkerSize',35);
plot(force_vector,mat_for_plot_line1(:,3),'b.-','LineWidth',2,'MarkerSize',35);
p2 = plot(force_vector,mat_for_plot_line2(:,1),'.:','Color',[0,0.5,0],'LineWidth',2,'MarkerSize',35);
plot(force_vector,mat_for_plot_line2(:,3),'b.:','LineWidth',2,'MarkerSize',35);
legend([p1,p2],{'Same','Different'});
xlabel('Force [N]'); ylabel('Proportion of trials'); ylim([-0.05 1.05]); yticks([0 0.25 0.5 0.75 1]); yticklabels({'0','25%','50%','75%','100%'}); xlim([-12 12]);


%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% C2G representation %%%
%%%%%%%%%%%%%%%%%%%%%%%%%%

idx_center = find(nearestTarget_xpertr2(:,4)==2,1);
idx_right = find(nearestTarget_xpertr2(:,4)==3,1);


figure;
subplot(1,2,1); hold on; set(gca,'Color','none'); set(gca,'FontSize',14); set(gca,'LineWidth',2); xlim([-0.1 0.1]); set(gca,'visible','off')
plot(x_pertr2(3,:,idx_center,4),x_pertr2(4,:,idx_center,4),'r');
plot(x_pertr2(3,:,idx_right,4),x_pertr2(4,:,idx_right,4),'b');
plot(x_pertr2(3,20,idx_center,4),x_pertr2(4,20,idx_center,4),'r.','MarkerSize',25);
plot(x_pertr2(3,20,idx_right,4),x_pertr2(4,20,idx_right,4),'b.','MarkerSize',25);
rectangle('Position',[-0.06 0.24 0.02 0.02],'Curvature',[1 1],'FaceColor','k');
rectangle('Position',[-0.01 0.24 0.02 0.02],'Curvature',[1 1],'FaceColor',[0.6 0.6 0.6],'EdgeColor','k');
rectangle('Position',[ 0.04 0.24 0.02 0.02],'Curvature',[1 1],'FaceColor','k');
axis equal;

C2G_matrix = zeros(2,55,3);
for ii = 2 : size(C2G_matrix,2)+1
   C2G_matrix(1,ii,1) = x_pertr2(:,ii-1,idx_center,4)' * S_familyr2{1}(:,:,ii-1) * x_pertr2(:,ii-1,idx_center,4) + s_familyr2{1}(ii-1);
   C2G_matrix(1,ii,2) = x_pertr2(:,ii-1,idx_center,4)' * S_familyr2{2}(:,:,ii-1) * x_pertr2(:,ii-1,idx_center,4) + s_familyr2{2}(ii-1);
   C2G_matrix(1,ii,3) = x_pertr2(:,ii-1,idx_center,4)' * S_familyr2{3}(:,:,ii-1) * x_pertr2(:,ii-1,idx_center,4) + s_familyr2{3}(ii-1);
   C2G_matrix(2,ii,1) = x_pertr2(:,ii-1,idx_right,4)' * S_familyr2{1}(:,:,ii-1) * x_pertr2(:,ii-1,idx_right,4) + s_familyr2{1}(ii-1);
   C2G_matrix(2,ii,2) = x_pertr2(:,ii-1,idx_right,4)' * S_familyr2{2}(:,:,ii-1) * x_pertr2(:,ii-1,idx_right,4) + s_familyr2{2}(ii-1);
   C2G_matrix(2,ii,3) = x_pertr2(:,ii-1,idx_right,4)' * S_familyr2{3}(:,:,ii-1) * x_pertr2(:,ii-1,idx_right,4) + s_familyr2{3}(ii-1);
end

subplot(2,2,2); hold on; set(gca,'Color','none'); set(gca,'FontSize',14); set(gca,'LineWidth',2); set(gca,'XColor','none');
plot(log(squeeze(C2G_matrix(1,:,1))),'g','LineWidth',2);
plot(log(squeeze(C2G_matrix(1,:,2))),'r','LineWidth',2);
plot(log(squeeze(C2G_matrix(1,:,3))),'b','LineWidth',2);
xline(20,'k:','LineWidth',2);

subplot(2,2,4); hold on; set(gca,'Color','none'); set(gca,'FontSize',14); set(gca,'LineWidth',2); 
plot(log(squeeze(C2G_matrix(2,:,1))),'g','LineWidth',2);
plot(log(squeeze(C2G_matrix(2,:,2))),'r','LineWidth',2);
plot(log(squeeze(C2G_matrix(2,:,3))),'b','LineWidth',2);
xline(20,'k:','LineWidth',2); xlabel('Time [s]'); ylabel('Log(v)');