%% 0. Loading and preprocessing the data
clear all; close all; clc;
PilotesM2 = load('PilotesBis.mat');
nSubjects = length(fieldnames(PilotesM2));
Subjects = cell(nSubjects,1);
TotSubjects = zeros(480*nSubjects,1);
for ii = 1 : nSubjects
    Subjects{ii} = strcat('S',num2str(ii));
    TotSubjects((ii-1)*480+1:ii*480) = ii*ones(480,1);
end

TimeBoundaries = zeros(nSubjects,2);
for ii = 1 : size(TimeBoundaries,1)
    TimeBoundaries(ii,:) = [PilotesM2.(Subjects{ii}).time_matrix(1), PilotesM2.(Subjects{ii}).time_matrix(end)];
end

TotVectorTP = zeros(480*nSubjects,1);
TotReached = zeros(480*nSubjects,1);
TotSuccess = zeros(480*nSubjects,1);
for ii = 1 : nSubjects
    TotReached((ii-1)*480+1:480*ii) = BooleanReachedM2Bis(PilotesM2.(Subjects{ii}));
    TotSuccess((ii-1)*480+1:480*ii) = BooleanSuccessM2Bis(PilotesM2.(Subjects{ii}));
    TotVectorTP((ii-1)*480+1:480*ii) = PilotesM2.(Subjects{ii}).vector_TP;
end

PropReached = length(find(TotReached==1)) / length(TotReached);
PropSuccess = length(find(TotSuccess==1)) / length(TotSuccess);

mappingTP = [1 2 4 5 7 8 10 11 13 14];

extremeMin = PilotesM2.(Subjects{1}).time_matrix(1);
extremeMax = PilotesM2.(Subjects{1}).time_matrix(end);
for ii = 2 : nSubjects
    extremeMin = max(extremeMin, PilotesM2.(Subjects{ii}).time_matrix(1));
    extremeMax = min(extremeMax, PilotesM2.(Subjects{ii}).time_matrix(end));
end
TimeVector = extremeMin:extremeMax;
TotKine = zeros(480*nSubjects,length(TimeVector),2);
TotTimeMat = zeros(480*nSubjects,size(PilotesM2.S1.matrix_timing,2));
for ii = 1 : nSubjects
    TotTimeMat((ii-1)*480+1:ii*480,:) = PilotesM2.(Subjects{ii}).matrix_timing;
    TotKine((ii-1)*480+1:ii*480,:,:) = PilotesM2.(Subjects{ii}).matrix_kinematics(:,find(PilotesM2.(Subjects{ii}).time_matrix==TimeVector(1)):find(PilotesM2.(Subjects{ii}).time_matrix==TimeVector(end)),:);
    for jj = 1 : length(mappingTP)
        PropReached(ii,jj) = length(find(TotReached(TotVectorTP==mappingTP(jj) & TotSubjects==ii)==1))/length(TotReached(TotVectorTP==mappingTP(jj) & TotSubjects==ii));
        PropSuccess(ii,jj) = length(find(TotSuccess(TotVectorTP==mappingTP(jj) & TotSubjects==ii)==1))/length(TotSuccess(TotVectorTP==mappingTP(jj) & TotSubjects==ii));
    end
end

TotForce = zeros(480*nSubjects,1);
TotReward = mod(TotVectorTP,3)-1;
for ii = 1 : length(TotForce)
    switch TotVectorTP(ii)
        case {1,2}
            TotForce(ii) = 0;
        case {4,5}
            TotForce(ii) = 2;
        case {7,8}
            TotForce(ii) = -2;
        case {10,11}
            TotForce(ii) = 1;
        case {3,14}
            TotForce(ii) = -1;
    end
end
% analyses_velocity;
%successrate;
%% 1. Investigation of the switch frequencies (individually & globally)
% Speed matrix
TotKineSpeed = zeros(size(TotKine,1),size(TotKine,2),2);
TotKineSpeed(:,3:end-2,:) = (-TotKine(:,5:end,:)+8*TotKine(:,4:end-1,:)-8*TotKine(:,2:end-3,:)+TotKine(:,1:end-4,:))/(12*0.001);

% Nearest target at the end of movement

NearTarget = zeros(nSubjects*480,1);
pos_targets= [-0.01 0.33; 0.09 0.33; 0.19 0.33];
for ii = 1 : nSubjects
    vectrick = PilotesM2.(Subjects{ii}).time_matrix;
    for jj = 1 : 480
        if (TotReached((ii-1)*480+jj)==1)
            idxend = find(PilotesM2.(Subjects{ii}).time_matrix==(min(abs(floor((PilotesM2.(Subjects{ii}).matrix_timing(jj,8))*1000-(PilotesM2.(Subjects{ii}).matrix_timing(jj,6))*1000)),floor(vectrick(end)))));
            NearTarget((ii-1)*480+jj) = NearestTarget([PilotesM2.(Subjects{ii}).matrix_kinematics(jj,idxend,1), PilotesM2.(Subjects{ii}).matrix_kinematics(jj,idxend,2)],pos_targets);
        end
    end
end

TotSwitch = ~(NearTarget==2 | NearTarget==0);

ProportionMatrix = zeros(nSubjects,length(mappingTP),2);
for ii = 1 : nSubjects
    for jj = 1 : length(mappingTP)
        ProportionMatrix(ii,jj,:) = [length(find(NearTarget(TotVectorTP==mappingTP(jj) & TotSubjects==ii)==1))/length(find(NearTarget(TotVectorTP==mappingTP(jj) & TotSubjects==ii)~=0)) length(find(NearTarget(TotVectorTP==mappingTP(jj) & TotSubjects==ii)==3))/length(find(NearTarget(TotVectorTP==mappingTP(jj) & TotSubjects==ii)~=0))];
    end
end
% Graphical representation of the switching phenomenon
close all;
figure('Name','Graphical representation of the switching phenomenon','units','normalized','outerposition',[0 0 1 1]); 
subplot(3,2,5); hold on; set(gca,'LineWidth',2); set(gca,'FontSize',14); set(gca,'Color','none');title('Different rewards','FontSize',16);
for ii = 1 : nSubjects
    plot([1 2 3 4 5],[mean(ProportionMatrix(ii,5,:),3), mean(ProportionMatrix(ii,9,:),3), mean(ProportionMatrix(ii,1,:),3), mean(ProportionMatrix(ii,7,:),3), mean(ProportionMatrix(ii,3,:),3)],'.-','Color',[0.7 0.7 0.7],'LineWidth',2.5,'MarkerSize',15);
end
plot([1 2 3 4 5],[mean(mean(ProportionMatrix(:,5,[1 2]),3),1), mean(mean(ProportionMatrix(:,9,[1 2]),3),1), mean(mean(ProportionMatrix(:,1,[1 2]),3),1), mean(mean(ProportionMatrix(:,7,[1 2]),3),1), mean(mean(ProportionMatrix(:,3,[1 2]),3),1)],'k.-','LineWidth',4,'MarkerSize',25);
xticks([1 2 3 4 5]); xticklabels({'-10 N','-5 N','0 N','5 N','10 N'}); xlim([0.7 5.3]); xlabel('Mechanical perturbation');
ylim([-0.03 1.03]); ylabel('Switch proportion [%]'); yticks([0 0.25 0.5 0.75 1]); yticklabels({'0','25','50','75','100'});



subplot(3,2,6); hold on; set(gca,'LineWidth',2); set(gca,'FontSize',14); set(gca,'Color','none'); title('Same reward','FontSize',16);
for ii = 1 : nSubjects
    plot([1 2 3 4 5],[mean(ProportionMatrix(ii,6,:),3), mean(ProportionMatrix(ii,10,:),3), mean(ProportionMatrix(ii,2,:),3), mean(ProportionMatrix(ii,8,:),3), mean(ProportionMatrix(ii,4,:),3)],'.:','Color',[0.7 0.7 0.7],'LineWidth',2.5,'MarkerSize',15);
end
plot([1 2 3 4 5],[mean(mean(ProportionMatrix(:,6,[1 2]),3),1), mean(mean(ProportionMatrix(:,10,[1 2]),3),1), mean(mean(ProportionMatrix(:,2,[1 2]),3),1), mean(mean(ProportionMatrix(:,8,[1 2]),3),1),mean(mean(ProportionMatrix(:,4,[1 2]),3),1)],'k.:','LineWidth',4,'MarkerSize',25);
xticks([1 2 3 4 5]); xticklabels({'-10 N','-5 N','0 N','5 N','10 N'}); xlim([0.7 5.3]); xlabel('Mechanical perturbation');
ylim([-0.03 1.03]); ylabel('Switch proportion [%]'); yticks([0 0.25 0.5 0.75 1]); yticklabels({'0','25','50','75','100'});

% Essayons la régression logistique avec plusieurs classes
PredictorsMatrix = [TotReward TotForce];
NearTarget(NearTarget==2)=4;
[B,dev,stats] = mnrfit(PredictorsMatrix,categorical(NearTarget));
NearTarget(NearTarget==2)=2;
%% This part investigate the Generalized linear mixed effects models for this logistic regression
tableglme = table(NearTarget,categorical(TotReward),TotForce,TotSubjects,'VariableName',{'target','reward','force','subject'});
mnrglme = fitglme(tableglme,'target~reward+force+(1|subject)','Link','logit','Distribution','Binomial','BinomialSize',length(NearTarget));


disp(mean(TotKineSpeed(TotReward==1,find(TimeVector==0),2),1,'omitnan'));
disp(mean(TotKineSpeed(TotReward==0,find(TimeVector==0),2),1,'omitnan'));
tablespeed = table(TotKineSpeed(:,find(TimeVector==0),2),TotSubjects,TotReward,'VariableNames',{'speed','subjects','reward'});
fitlmespeed = fitlme(tablespeed,'speed~reward+(1|subjects)');

figure('Name','Forward speed at force onset','units','normalized');
hold on; set(gca,'Color','none'); set(gca,'LineWidth',2); set(gca,'FontSize',14); xlim([0.7 2.3]);
for ii = 1 : nSubjects
    plot([1,2],[mean(TotKineSpeed(TotReward==0 & TotSubjects==ii,TimeVector==0,2),1),mean(TotKineSpeed(TotReward==1 & TotSubjects==ii,TimeVector==0,2),1)]-mean(TotKineSpeed(TotSubjects==ii,TimeVector==0,2),1),'.-','Color',[0.7 0.7 0.7],'MarkerSize',25,'LineWidth',2); 
end
plot([1,2],[mean(TotKineSpeed(TotReward==0,TimeVector==0,2),1),mean(TotKineSpeed(TotReward==1,TimeVector==0,2),1)]-mean(TotKineSpeed(:,TimeVector==0,2),1),'k.-','MarkerSize',35,'LineWidth',3); 



NearTarget(NearTarget==4)=2;
close all; 
figure('Name','Cinématique - Expérience 2 reward','units','normalized','outerposition',[0 0 1 1]); hold on;

subplot(3,13,[1 2]); hold on; set(gca,'Color','none');
axis off;
d = 19;
idx81 = find(TotSubjects==d & NearTarget==1 & TotVectorTP==8);
idx82 = find(TotSubjects==d & NearTarget==2 & TotVectorTP==8);
for ii = 1 : length(idx81)
    plot(TotKine(idx81(ii),find(TimeVector==-200):find(TimeVector==700),1), TotKine(idx81(ii),find(TimeVector==-200):find(TimeVector==700),2),'r','LineWidth',1);
end
for jj = 1 : length(idx82)
    plot(TotKine(idx82(jj),find(TimeVector==-200):find(TimeVector==700),1), TotKine(idx82(jj),find(TimeVector==-200):find(TimeVector==700),2),'b','LineWidth',1);
end
xlim([-0.06 0.23]);

subplot(3,13,[3 4]); hold on; set(gca,'Color','none');
axis off;
idx141 = find(TotSubjects==d & NearTarget==1 & TotVectorTP==14);
idx142 = find(TotSubjects==d & NearTarget==2 & TotVectorTP==14);
for ii = 1 : length(idx141)
    plot(TotKine(idx141(ii),find(TimeVector==-200):find(TimeVector==700),1), TotKine(idx141(ii),find(TimeVector==-200):find(TimeVector==700),2),'r','LineWidth',1);
end
for jj = 1 : length(idx142)
    plot(TotKine(idx142(jj),find(TimeVector==-200):find(TimeVector==700),1), TotKine(idx142(jj),find(TimeVector==-200):find(TimeVector==700),2),'b','LineWidth',1);
end
xlim([-0.06 0.23]);

subplot(3,13,[5 6]); hold on; set(gca,'Color','none'); 
axis off;
idx21 = find(TotSubjects==d & NearTarget==1 & TotVectorTP==2);
idx22 = find(TotSubjects==d & NearTarget==2 & TotVectorTP==2);
idx23 = find(TotSubjects==d & NearTarget==3 & TotVectorTP==2);
for ii = 1 : length(idx21)
    plot(TotKine(idx21(ii),find(TimeVector==-200):find(TimeVector==700),1), TotKine(idx21(ii),find(TimeVector==-200):find(TimeVector==700),2),'r','LineWidth',1);
end
for jj = 1 : length(idx22)
    plot(TotKine(idx22(jj),find(TimeVector==-200):find(TimeVector==700),1), TotKine(idx22(jj),find(TimeVector==-200):find(TimeVector==700),2),'b','LineWidth',1);
end
for kk = 1 : length(idx23)
    plot(TotKine(idx23(kk),find(TimeVector==-200):find(TimeVector==700),1), TotKine(idx23(kk),find(TimeVector==-200):find(TimeVector==700),2),'Color',[0 0.4 0],'LineWidth',1);
end
xlim([-0.06 0.23]);

subplot(3,13,[7 8]); hold on; set(gca,'Color','none');
axis off;
idx111 = find(TotSubjects==d & NearTarget==1 & TotVectorTP==11);
idx112 = find(TotSubjects==d & NearTarget==2 & TotVectorTP==11);
idx113 = find(TotSubjects==d & NearTarget==3 & TotVectorTP==11);
for ii = 1 : length(idx111)
    plot(TotKine(idx111(ii),find(TimeVector==-200):find(TimeVector==700),1),TotKine(idx111(ii),find(TimeVector==-200):find(TimeVector==700),2),'r','LineWidth',1);
end
for jj = 1 : length(idx112)
    plot(TotKine(idx112(jj),find(TimeVector==-200):find(TimeVector==700),1),TotKine(idx112(jj),find(TimeVector==-200):find(TimeVector==700),2),'b','LineWidth',1);
end
for kk = 1 : length(idx113)
    plot(TotKine(idx113(kk),find(TimeVector==-200):find(TimeVector==700),1),TotKine(idx113(kk),find(TimeVector==-200):find(TimeVector==700),2),'Color',[0 0.4 0],'LineWidth',1);
end
xlim([-0.06 0.23]);

subplot(3,13,[9 10]); hold on; set(gca,'Color','none');
axis off;
idx52 = find(TotSubjects==d & NearTarget==2 & TotVectorTP==5);
idx53 = find(TotSubjects==d & NearTarget==3 & TotVectorTP==5);

for jj = 1 : length(idx52)
    plot(TotKine(idx52(jj),find(TimeVector==-200):find(TimeVector==700),1),TotKine(idx52(jj),find(TimeVector==-200):find(TimeVector==700),2),'b','LineWidth',1);
end
for kk = 1 : length(idx53)
    plot(TotKine(idx53(kk),find(TimeVector==-200):find(TimeVector==700),1),TotKine(idx53(kk),find(TimeVector==-200):find(TimeVector==700),2),'Color',[0 0.4 0],'LineWidth',1);
end
xlim([-0.06 0.23]);

subplot(3,13,[14 15]); hold on; set(gca,'Color','none');
axis off;
idx81 = find(TotSubjects==d & NearTarget==1 & TotVectorTP==7);
idx82 = find(TotSubjects==d & NearTarget==2 & TotVectorTP==7);
for ii = 1 : length(idx81)
    plot(TotKine(idx81(ii),find(TimeVector==-200):find(TimeVector==700),1), TotKine(idx81(ii),find(TimeVector==-200):find(TimeVector==700),2),'r','LineWidth',1);
end
for jj = 1 : length(idx82)
    plot(TotKine(idx82(jj),find(TimeVector==-200):find(TimeVector==700),1), TotKine(idx82(jj),find(TimeVector==-200):find(TimeVector==700),2),'b','LineWidth',1);
end
xlim([-0.06 0.23]);

subplot(3,13,[16 17]); hold on; set(gca,'Color','none');
axis off;
idx141 = find(TotSubjects==d & NearTarget==1 & TotVectorTP==13);
idx142 = find(TotSubjects==d & NearTarget==2 & TotVectorTP==13);
for ii = 1 : length(idx141)
    plot(TotKine(idx141(ii),find(TimeVector==-200):find(TimeVector==700),1), TotKine(idx141(ii),find(TimeVector==-200):find(TimeVector==700),2),'r','LineWidth',1);
end
for jj = 1 : length(idx142)
    plot(TotKine(idx142(jj),find(TimeVector==-200):find(TimeVector==700),1), TotKine(idx142(jj),find(TimeVector==-200):find(TimeVector==700),2),'b','LineWidth',1);
end
xlim([-0.06 0.23]);

subplot(3,13,[18 19]); hold on; set(gca,'Color','none');
axis off;
idx21 = find(TotSubjects==d & NearTarget==1 & TotVectorTP==1);
idx22 = find(TotSubjects==d & NearTarget==2 & TotVectorTP==1);
idx23 = find(TotSubjects==d & NearTarget==3 & TotVectorTP==1);
for ii = 1 : length(idx21)
    plot(TotKine(idx21(ii),find(TimeVector==-200):find(TimeVector==700),1), TotKine(idx21(ii),find(TimeVector==-200):find(TimeVector==700),2),'r','LineWidth',1);
end
for jj = 1 : length(idx22)
    plot(TotKine(idx22(jj),find(TimeVector==-200):find(TimeVector==700),1), TotKine(idx22(jj),find(TimeVector==-200):find(TimeVector==700),2),'b','LineWidth',1);
end
for kk = 1 : length(idx23)
    plot(TotKine(idx23(kk),find(TimeVector==-200):find(TimeVector==700),1), TotKine(idx23(kk),find(TimeVector==-200):find(TimeVector==700),2),'Color',[0 0.4 0],'LineWidth',1);
end
xlim([-0.06 0.23]);


subplot(3,13,[20 21]); hold on; set(gca,'Color','none');
axis off;
idx111 = find(TotSubjects==d & NearTarget==1 & TotVectorTP==10);
idx112 = find(TotSubjects==d & NearTarget==2 & TotVectorTP==10);
idx113 = find(TotSubjects==d & NearTarget==3 & TotVectorTP==10);
for ii = 1 : length(idx111)
    plot(TotKine(idx111(ii),find(TimeVector==-200):find(TimeVector==700),1),TotKine(idx111(ii),find(TimeVector==-200):find(TimeVector==700),2),'r','LineWidth',1);
end
for jj = 1 : length(idx112)
    plot(TotKine(idx112(jj),find(TimeVector==-200):find(TimeVector==700),1),TotKine(idx112(jj),find(TimeVector==-200):find(TimeVector==700),2),'b','LineWidth',1);
end
for kk = 1 : length(idx113)
    plot(TotKine(idx113(kk),find(TimeVector==-200):find(TimeVector==700),1),TotKine(idx113(kk),find(TimeVector==-200):find(TimeVector==700),2),'Color',[0 0.4 0],'LineWidth',1);
end
xlim([-0.06 0.23]);
subplot(3,13,[22 23]); hold on; set(gca,'Color','none');
axis off;
idx42 = find(TotSubjects==d & NearTarget==2 & TotVectorTP==4);
idx43 = find(TotSubjects==d & NearTarget==3 & TotVectorTP==4);

for jj = 1 : length(idx42)
    plot(TotKine(idx42(jj),find(TimeVector==-200):find(TimeVector==700),1),TotKine(idx42(jj),find(TimeVector==-200):find(TimeVector==700),2),'b','LineWidth',1);
end
for kk = 1 : length(idx43)
    plot(TotKine(idx43(kk),find(TimeVector==-200):find(TimeVector==700),1),TotKine(idx43(kk),find(TimeVector==-200):find(TimeVector==700),2),'Color',[0 0.4 0],'LineWidth',1);
end
xlim([-0.06 0.23]);



figure('units','normalized','outerposition',[0 0 1 1 ]);
% Same reward 
subplot(3,13,[27 28 29 30 31]); hold on; set(gca,'LineWidth',4); set(gca,'FontSize',18); set(gca,'Color','none');
for ii = 1 : nSubjects
    plot([1 2 3 4 5],[ProportionMatrix(ii,6,1), ProportionMatrix(ii,10,1), mean(ProportionMatrix(ii,2,[1,2])), ProportionMatrix(ii,8,2), ProportionMatrix(ii,4,2)],'.-','Color',[0.7 0.7 0.7],'LineWidth',2,'MarkerSize',25);
end
plot([1 2 3 4 5],[mean(ProportionMatrix(:,6,1)), mean(ProportionMatrix(:,10,1)), mean(mean(ProportionMatrix(:,2,[1 2]),3),1), mean(ProportionMatrix(:,8,2)), mean(ProportionMatrix(:,4,2))],'k.-','LineWidth',4,'MarkerSize',30);
xlim([0.7 5.3]); xticks([1 2 3 4 5]); xticklabels({'-10','-6','0','6','10'}); 
ylim([-0.1 1.1]); yticks([0 1]); 
set(gca,'XTickLabel',{'-10','-6','0','6','10'});
set(gca,'YTickLabel',{'0','1'});
set(get(gca,'XLabel'),'String','Total load [N]');
set(get(gca,'YLabel'),'String','Switch proportion');

% Different rewards 
subplot(3,13,[32 33 34 35 36]); hold on; set(gca,'LineWidth',4); set(gca,'FontSize',18); set(gca,'Color','none'); set(gca,'YColor','none');
for ii = 1 : nSubjects
    plot([1 2 3 4 5],[ProportionMatrix(ii,5,1), ProportionMatrix(ii,9,1), mean(ProportionMatrix(ii,1,[1,2])), ProportionMatrix(ii,7,2), ProportionMatrix(ii,3,2)],'.-','Color',[0.7 0.7 0.7],'LineWidth',2,'MarkerSize',25);
end
plot([1 2 3 4 5],[mean(ProportionMatrix(:,5,1)), mean(ProportionMatrix(:,9,1)), mean(mean(ProportionMatrix(:,1,[1 2]),3),1), mean(ProportionMatrix(:,7,2)), mean(ProportionMatrix(:,3,2))],'k.-','LineWidth',4,'MarkerSize',30);
xlim([0.7 5.3]); xticks([1 2 3 4 5]); ylim([-0.1 1.1]);
set(gca,'XTickLabel',{'-10','-6','0','6','10'});
set(get(gca,'XLabel'),'String','Total load [N]');
%%
LeftSame = zeros(nSubjects*5,1);
LeftDiff = zeros(nSubjects*5,1);
RightSame = zeros(nSubjects*5,1);
RightDiff = zeros(nSubjects*5,1);
mapsame = [5 9 1 7 3];
mapdiff = [6 10 2 8 4];
for ii = 1 : 5
    LeftSame((ii-1)*nSubjects+1:ii*nSubjects) = ProportionMatrix(:,mapsame(ii),1);
    LeftDiff((ii-1)*nSubjects+1:ii*nSubjects) = ProportionMatrix(:,mapdiff(ii),1);
    RightSame((ii-1)*nSubjects+1:ii*nSubjects) = ProportionMatrix(:,mapsame(ii),2);
    RightDiff((ii-1)*nSubjects+1:ii*nSubjects) = ProportionMatrix(:,mapdiff(ii),2);
end
[pl,hl,statsl] = signrank(LeftSame,LeftDiff,'tail','left');
[pr,hr,statsr] = signrank(RightSame,RightDiff,'tail','left');

%% Figure 4 : Kinematics Experiment 2 - Barplot 


subplot(6,13,[24 25 26]); hold on; set(gca,'LineWidth',4); set(gca,'FontSize',18); set(gca,'Color','none'); set(gca,'XColor','none'); set(gca,'YAxisLocation','right');
bar([2 1],[mean(ProportionMatrix(:,5,1)) mean(ProportionMatrix(:,6,1))],'LineStyle','none','ShowBaseLine','off','FaceColor',[1 0 0]);
er = errorbar([2 1],[mean(ProportionMatrix(:,5,1)) mean(ProportionMatrix(:,6,1))],[std(ProportionMatrix(:,5,1)) std(ProportionMatrix(:,6,1))]/sqrt(nSubjects),[std(ProportionMatrix(:,5,1)) std(ProportionMatrix(:,6,1))]/sqrt(nSubjects));
er.Color = [0 0 0]; er.LineWidth = 3; xlim([0.3 2.7]); ylim([-0.05 0.60]); yticks([0 0.5]); set(gca,'TickLength',[0.035 0.025]);
set(gca,'YTickLabel',{'0','0.5'});

subplot(6,13,[37 38 39]); hold on; set(gca,'LineWidth',4); set(gca,'FontSize',18); set(gca,'Color','none'); set(gca,'XColor','none'); set(gca,'YAxisLocation','right');
bar([2 1],[mean(ProportionMatrix(:,9,1)) mean(ProportionMatrix(:,10,1))],'LineStyle','none','ShowBaseLine','off','FaceColor',[1 0 0]);
er = errorbar([2 1],[mean(ProportionMatrix(:,9,1)) mean(ProportionMatrix(:,10,1))],[std(ProportionMatrix(:,9,1)) std(ProportionMatrix(:,10,1))]/sqrt(nSubjects),[std(ProportionMatrix(:,9,1)) std(ProportionMatrix(:,10,1))]/sqrt(nSubjects));
er.Color = [0 0 0]; er.LineWidth = 3; xlim([0.3 2.7]); ylim([-0.05 0.60]); yticks([0 0.5]);set(gca,'TickLength',[0.035 0.025]); ylabel('Mean switch proportion');set(gca,'YTickLabel',{'0','0.5'});

subplot(6,13,[50 51 52]); hold on; set(gca,'LineWidth',4); set(gca,'FontSize',18); set(gca,'Color','none'); set(gca,'XColor','none'); set(gca,'YAxisLocation','right');
bar([2 1],[mean(ProportionMatrix(:,3,2)) mean(ProportionMatrix(:,4,2))],'LineStyle','none','ShowBaseLine','off','FaceColor',[1 0 0]);
er = errorbar([2 1],[mean(ProportionMatrix(:,3,2)) mean(ProportionMatrix(:,4,2))],[std(ProportionMatrix(:,3,2)) std(ProportionMatrix(:,4,2))]/sqrt(nSubjects),[std(ProportionMatrix(:,3,2)) std(ProportionMatrix(:,4,2))]/sqrt(nSubjects));
er.Color = [0 0 0]; er.LineWidth = 3; xlim([0.3 2.7]); ylim([-0.05 0.60]); yticks([0 0.5]);set(gca,'TickLength',[0.035 0.025]);set(gca,'YTickLabel',{'0','0.5'});

subplot(6,13,[63 64 65]); hold on; set(gca,'LineWidth',4); set(gca,'FontSize',18); set(gca,'Color','none'); set(gca,'YAxisLocation','right');
bar([2,1],[mean(ProportionMatrix(:,7,2)) mean(ProportionMatrix(:,8,2))],'LineStyle','none','ShowBaseLine','off','FaceColor',[1 0 0]);
er = errorbar([2 1],[mean(ProportionMatrix(:,7,2)) mean(ProportionMatrix(:,8,2))],[std(ProportionMatrix(:,7,2)) std(ProportionMatrix(:,8,2))]/sqrt(nSubjects),[std(ProportionMatrix(:,7,2)) std(ProportionMatrix(:,8,2))]/sqrt(nSubjects));
er.Color = [0 0 0]; er.LineWidth = 3; xlim([0.3 2.7]); ylim([-0.05 0.60]); yticks([0 0.5]);set(gca,'TickLength',[0.035 0.025]);set(gca,'YTickLabel',{'0','0.5'});
xticks([1 2]); xticklabels({'Same','Different'}); xtickangle(90);




clc;
TotNormSpeed = sqrt(TotKineSpeed(:,:,1).^2 + TotKineSpeed(:,:,2).^2);
TotMaxSpeed = max(TotNormSpeed(:,find(TimeVector==-200):find(TimeVector==300)),[],2);
idx_out = (TotMaxSpeed==0 | TotMaxSpeed>2);
TotMaxSpeed(idx_out) = NaN;

% Computation of the reaction time

idx_shown = zeros(length(TotMaxSpeed),1);
for ii = 1 : nSubjects
    for jj = 1 : size(PilotesM2.(Subjects{ii}).array_timing,1)
        tmp = PilotesM2.(Subjects{ii}).array_timing{jj,3};
       if tmp(1)=='g'
           idx_shown((ii-1)*480+jj) = PilotesM2.(Subjects{ii}).matrix_timing(jj,3) -  PilotesM2.(Subjects{ii}).matrix_timing(jj,5);
       end
    end
end

idx_shown = floor(idx_shown*1000);
idx_thre = zeros(length(idx_shown),1);
for ii = 1 : length(idx_shown)
    if(~isnan(TotMaxSpeed(ii)))
    idx_thre(ii) = find(TotNormSpeed(ii,find(TimeVector==idx_shown(ii)):end)>0.05*TotMaxSpeed(ii),1);
    end
end

%%
figure('Name','Investigation of the reaction time computation','units','normalized');hold on;
set(gca,'Color','none'); set(gca,'FontSize',14); set(gca,'LineWidth',2); xlim([-500 500]);
plot(TimeVector,TotNormSpeed(2,:),'r','LineWidth',2);
xline(idx_shown(2),'k:','LineWidth',2);
xline(idx_shown(2)+idx_thre(2),'k:','LineWidth',2);
yline(0.05*TotMaxSpeed(2),'g:','LineWidth',2);

RTime = idx_thre;
tableRT12 = table(RTime((TotVectorTP==mappingTP(8) | TotVectorTP==mappingTP(6) | TotVectorTP==mappingTP(4) | TotVectorTP==mappingTP(10)) & (NearTarget==1 | NearTarget==2)),NearTarget((TotVectorTP==mappingTP(8) | TotVectorTP==mappingTP(6) | TotVectorTP==mappingTP(4) | TotVectorTP==mappingTP(10)) & (NearTarget==1 | NearTarget==2)), TotSubjects((TotVectorTP==mappingTP(8) | TotVectorTP==mappingTP(6) | TotVectorTP==mappingTP(4) | TotVectorTP==mappingTP(10)) & (NearTarget==1 | NearTarget==2)),'VariableNames',{'RT','Target','Subject'});
tableRT23 = table(RTime((TotVectorTP==mappingTP(8) | TotVectorTP==mappingTP(6) | TotVectorTP==mappingTP(4) | TotVectorTP==mappingTP(10)) & (NearTarget==3 | NearTarget==2)),NearTarget((TotVectorTP==mappingTP(8) | TotVectorTP==mappingTP(6) | TotVectorTP==mappingTP(4) | TotVectorTP==mappingTP(10)) & (NearTarget==3 | NearTarget==2)), TotSubjects((TotVectorTP==mappingTP(8) | TotVectorTP==mappingTP(6) | TotVectorTP==mappingTP(4) | TotVectorTP==mappingTP(10)) & (NearTarget==3 | NearTarget==2)),'VariableNames',{'RT','Target','Subject'});
lmeRT12 = fitlme(tableRT12,'RT~Target+(1|Subject)');
lmeRT23 = fitlme(tableRT23,'RT~Target+(1|Subject)');
%% 
% Investigation of the effect of fatigue or habituation

TotBlocks = repmat([1*ones(80,1);2*ones(80,1);3*ones(80,1);4*ones(80,1);5*ones(80,1);6*ones(80,1)],nSubjects,1);
table_fatigue_block = table(TotMaxSpeed(1:size(TotMaxSpeed,1)/2,:,:),TotBlocks(1:size(TotMaxSpeed,1)/2,:,:),TotSubjects(1:size(TotMaxSpeed,1)/2,:,:),'VariableNames',{'pv','block','subjects'});
lme_fatigue_block = fitlme(table_fatigue_block,'pv~block+(1|subjects)');    



close all;
figure('Name','Change of mind in kinematics','units','normalized');
subplot(2,2,1); hold on; set(gca,'LineWidth',2); set(gca,'Color','none'); set(gca,'FontSize',14);
plot(-200:500, mean(TotKine(NearTarget==1 & TotVectorTP==8, find(TimeVector==-200):find(TimeVector==500),1),1),'r','LineWidth',2);
plot(-200:500, mean(TotKine(NearTarget==1 & TotVectorTP==8, find(TimeVector==-200):find(TimeVector==500),1),1)+std(TotKine(NearTarget==1 & TotVectorTP==8, find(TimeVector==-200):find(TimeVector==500),1),0,1)/sqrt(nSubjects),'r:','LineWidth',2);
plot(-200:500, mean(TotKine(NearTarget==1 & TotVectorTP==8, find(TimeVector==-200):find(TimeVector==500),1),1)-std(TotKine(NearTarget==1 & TotVectorTP==8, find(TimeVector==-200):find(TimeVector==500),1),0,1)/sqrt(nSubjects),'r:','LineWidth',2);
plot(-200:500, mean(TotKine(NearTarget==2 & TotVectorTP==8, find(TimeVector==-200):find(TimeVector==500),1),1),'b','LineWidth',2);
plot(-200:500, mean(TotKine(NearTarget==2 & TotVectorTP==8, find(TimeVector==-200):find(TimeVector==500),1),1)+std(TotKine(NearTarget==2 & TotVectorTP==8, find(TimeVector==-200):find(TimeVector==500),1),0,1)/sqrt(nSubjects),'b:','LineWidth',2);
plot(-200:500, mean(TotKine(NearTarget==2 & TotVectorTP==8, find(TimeVector==-200):find(TimeVector==500),1),1)-std(TotKine(NearTarget==2 & TotVectorTP==8, find(TimeVector==-200):find(TimeVector==500),1),0,1)/sqrt(nSubjects),'b:','LineWidth',2);
xlabel('Time [ms]'); ylabel('x-position [cm]');
xline(onsetvalue8,'r:','linewidth',2); xline(0,'k--','linewidth',2);
xticks([-150 0 400]); xticklabels({'-0.15','0','0.4'});
yticks([-0.01 0.04 0.09]); yticklabels({'-10','-5','0'});
ylim([-0.015 0.1]);

subplot(2,2,2); hold on; set(gca,'LineWidth',2); set(gca,'Color','none'); set(gca,'FontSize',14);
plot(-200:500, mean(TotKine(NearTarget==3 & TotVectorTP==5, find(TimeVector==-200):find(TimeVector==500),1),1),'g','LineWidth',2);
plot(-200:500, mean(TotKine(NearTarget==3 & TotVectorTP==5, find(TimeVector==-200):find(TimeVector==500),1),1)+std(TotKine(NearTarget==3 & TotVectorTP==5, find(TimeVector==-200):find(TimeVector==500),1),0,1)/sqrt(nSubjects),'g:','LineWidth',2);
plot(-200:500, mean(TotKine(NearTarget==3 & TotVectorTP==5, find(TimeVector==-200):find(TimeVector==500),1),1)-std(TotKine(NearTarget==3 & TotVectorTP==5, find(TimeVector==-200):find(TimeVector==500),1),0,1)/sqrt(nSubjects),'g:','LineWidth',2);
plot(-200:500, mean(TotKine(NearTarget==2 & TotVectorTP==5, find(TimeVector==-200):find(TimeVector==500),1),1),'b','LineWidth',2);
plot(-200:500, mean(TotKine(NearTarget==2 & TotVectorTP==5, find(TimeVector==-200):find(TimeVector==500),1),1)+std(TotKine(NearTarget==2 & TotVectorTP==5, find(TimeVector==-200):find(TimeVector==500),1),0,1)/sqrt(nSubjects),'b:','LineWidth',2);
plot(-200:500, mean(TotKine(NearTarget==2 & TotVectorTP==5, find(TimeVector==-200):find(TimeVector==500),1),1)-std(TotKine(NearTarget==2 & TotVectorTP==5, find(TimeVector==-200):find(TimeVector==500),1),0,1)/sqrt(nSubjects),'b:','LineWidth',2);
xlabel('Time [ms]'); ylabel('x-position [cm]');
xline(onsetvalue5,'r:','linewidth',2); xline(0,'k--','linewidth',2);
xticks([-150 0 400]); xticklabels({'-0.15','0','0.4'});
yticks([0.09 0.14 0.19]); yticklabels({'0','5','10'});
ylim([0.08 0.2]);

subplot(2,2,3); hold on; set(gca,'LineWidth',2); set(gca,'Color','none'); set(gca,'FontSize',14);
plot(-200:500, mean(TotKine(NearTarget==1 & TotVectorTP==7, find(TimeVector==-200):find(TimeVector==500),1),1),'r','LineWidth',2);
plot(-200:500, mean(TotKine(NearTarget==1 & TotVectorTP==7, find(TimeVector==-200):find(TimeVector==500),1),1)+std(TotKine(NearTarget==1 & TotVectorTP==7, find(TimeVector==-200):find(TimeVector==500),1),0,1)/sqrt(nSubjects),'r:','LineWidth',2);
plot(-200:500, mean(TotKine(NearTarget==1 & TotVectorTP==7, find(TimeVector==-200):find(TimeVector==500),1),1)-std(TotKine(NearTarget==1 & TotVectorTP==7, find(TimeVector==-200):find(TimeVector==500),1),0,1)/sqrt(nSubjects),'r:','LineWidth',2);
plot(-200:500, mean(TotKine(NearTarget==2 & TotVectorTP==7, find(TimeVector==-200):find(TimeVector==500),1),1),'b','LineWidth',2);
plot(-200:500, mean(TotKine(NearTarget==2 & TotVectorTP==7, find(TimeVector==-200):find(TimeVector==500),1),1)+std(TotKine(NearTarget==2 & TotVectorTP==7, find(TimeVector==-200):find(TimeVector==500),1),0,1)/sqrt(nSubjects),'b:','LineWidth',2);
plot(-200:500, mean(TotKine(NearTarget==2 & TotVectorTP==7, find(TimeVector==-200):find(TimeVector==500),1),1)-std(TotKine(NearTarget==2 & TotVectorTP==7, find(TimeVector==-200):find(TimeVector==500),1),0,1)/sqrt(nSubjects),'b:','LineWidth',2);
xlabel('Time [ms]'); ylabel('x-position [cm]');
xline(onsetvalue7,'r:','linewidth',2); xline(0,'k--','linewidth',2);
xticks([-150 0 400]); xticklabels({'-0.15','0','0.4'});
yticks([-0.01 0.04 0.09]); yticklabels({'-10','-5','0'});
ylim([-0.015 0.1]);

subplot(2,2,4); hold on; set(gca,'LineWidth',2); set(gca,'Color','none'); set(gca,'FontSize',14);
plot(-200:500, mean(TotKine(NearTarget==3 & TotVectorTP==4, find(TimeVector==-200):find(TimeVector==500),1),1),'g','LineWidth',2);
plot(-200:500, mean(TotKine(NearTarget==3 & TotVectorTP==4, find(TimeVector==-200):find(TimeVector==500),1),1)+std(TotKine(NearTarget==3 & TotVectorTP==4, find(TimeVector==-200):find(TimeVector==500),1),0,1)/sqrt(nSubjects),'g:','LineWidth',2);
plot(-200:500, mean(TotKine(NearTarget==3 & TotVectorTP==4, find(TimeVector==-200):find(TimeVector==500),1),1)-std(TotKine(NearTarget==3 & TotVectorTP==4, find(TimeVector==-200):find(TimeVector==500),1),0,1)/sqrt(nSubjects),'g:','LineWidth',2);
plot(-200:500, mean(TotKine(NearTarget==2 & TotVectorTP==4, find(TimeVector==-200):find(TimeVector==500),1),1),'b','LineWidth',2);
plot(-200:500, mean(TotKine(NearTarget==2 & TotVectorTP==4, find(TimeVector==-200):find(TimeVector==500),1),1)+std(TotKine(NearTarget==2 & TotVectorTP==4, find(TimeVector==-200):find(TimeVector==500),1),0,1)/sqrt(nSubjects),'b:','LineWidth',2);
plot(-200:500, mean(TotKine(NearTarget==2 & TotVectorTP==4, find(TimeVector==-200):find(TimeVector==500),1),1)-std(TotKine(NearTarget==2 & TotVectorTP==4, find(TimeVector==-200):find(TimeVector==500),1),0,1)/sqrt(nSubjects),'b:','LineWidth',2);
xlabel('Time [s]'); ylabel('x-position [cm]');
xline(onsetvalue4,'r:','linewidth',2); xline(0,'k--','linewidth',2);
xticks([-150 0 400]); xticklabels({'-0.15','0','0.4'});
yticks([0.09 0.14 0.19]); yticklabels({'0','5','10'});
ylim([0.08 0.2]);

figure('Name','ROC pour les forces différentes et le même reward','units','normalized');
subplot(2,2,1); hold on; set(gca,'LineWidth',2); set(gca,'FontSize',14); set(gca,'Color','none');
plot(-200:500, mean(TotKine(NearTarget==1 & TotVectorTP==8, find(TimeVector==-200):find(TimeVector==500),1),1),'r','LineWidth',2);
plot(-200:500, mean(TotKine(NearTarget==1 & TotVectorTP==8, find(TimeVector==-200):find(TimeVector==500),1),1)+std(TotKine(NearTarget==1 & TotVectorTP==8, find(TimeVector==-200):find(TimeVector==500),1),0,1)/sqrt(nSubjects),'r:','LineWidth',2);
plot(-200:500, mean(TotKine(NearTarget==1 & TotVectorTP==8, find(TimeVector==-200):find(TimeVector==500),1),1)-std(TotKine(NearTarget==1 & TotVectorTP==8, find(TimeVector==-200):find(TimeVector==500),1),0,1)/sqrt(nSubjects),'r:','LineWidth',2);
plot(-200:500, mean(TotKine(NearTarget==2 & TotVectorTP==8, find(TimeVector==-200):find(TimeVector==500),1),1),'b','LineWidth',2);
plot(-200:500, mean(TotKine(NearTarget==2 & TotVectorTP==8, find(TimeVector==-200):find(TimeVector==500),1),1)+std(TotKine(NearTarget==2 & TotVectorTP==8, find(TimeVector==-200):find(TimeVector==500),1),0,1)/sqrt(nSubjects),'b:','LineWidth',2);
plot(-200:500, mean(TotKine(NearTarget==2 & TotVectorTP==8, find(TimeVector==-200):find(TimeVector==500),1),1)-std(TotKine(NearTarget==2 & TotVectorTP==8, find(TimeVector==-200):find(TimeVector==500),1),0,1)/sqrt(nSubjects),'b:','LineWidth',2);

subplot(2,2,2); hold on; set(gca,'LineWidth',2); set(gca,'FontSize',14); set(gca,'Color','none');
plot(-200:500, mean(TotKine(NearTarget==3 & TotVectorTP==5, find(TimeVector==-200):find(TimeVector==500),1),1),'r','LineWidth',2);
plot(-200:500, mean(TotKine(NearTarget==3 & TotVectorTP==5, find(TimeVector==-200):find(TimeVector==500),1),1)+std(TotKine(NearTarget==3 & TotVectorTP==5, find(TimeVector==-200):find(TimeVector==500),1),0,1)/sqrt(nSubjects),'r:','LineWidth',2);
plot(-200:500, mean(TotKine(NearTarget==3 & TotVectorTP==5, find(TimeVector==-200):find(TimeVector==500),1),1)-std(TotKine(NearTarget==3 & TotVectorTP==5, find(TimeVector==-200):find(TimeVector==500),1),0,1)/sqrt(nSubjects),'r:','LineWidth',2);
plot(-200:500, mean(TotKine(NearTarget==2 & TotVectorTP==5, find(TimeVector==-200):find(TimeVector==500),1),1),'g','LineWidth',2);
plot(-200:500, mean(TotKine(NearTarget==2 & TotVectorTP==5, find(TimeVector==-200):find(TimeVector==500),1),1)+std(TotKine(NearTarget==2 & TotVectorTP==5, find(TimeVector==-200):find(TimeVector==500),1),0,1)/sqrt(nSubjects),'g:','LineWidth',2);
plot(-200:500, mean(TotKine(NearTarget==2 & TotVectorTP==5, find(TimeVector==-200):find(TimeVector==500),1),1)-std(TotKine(NearTarget==2 & TotVectorTP==5, find(TimeVector==-200):find(TimeVector==500),1),0,1)/sqrt(nSubjects),'g:','LineWidth',2);

subplot(2,2,3); hold on; set(gca,'LineWidth',2); set(gca,'FontSize',14); set(gca,'Color','none')
plot(-200:500, mean(TotKine(NearTarget==1 & TotVectorTP==14, find(TimeVector==-200):find(TimeVector==500),1),1),'r','LineWidth',2);
plot(-200:500, mean(TotKine(NearTarget==1 & TotVectorTP==14, find(TimeVector==-200):find(TimeVector==500),1),1)+std(TotKine(NearTarget==1 & TotVectorTP==14, find(TimeVector==-200):find(TimeVector==500),1),0,1)/sqrt(nSubjects),'r:','LineWidth',2);
plot(-200:500, mean(TotKine(NearTarget==1 & TotVectorTP==14, find(TimeVector==-200):find(TimeVector==500),1),1)-std(TotKine(NearTarget==1 & TotVectorTP==14, find(TimeVector==-200):find(TimeVector==500),1),0,1)/sqrt(nSubjects),'r:','LineWidth',2);

plot(-200:500, mean(TotKine(NearTarget==2 & TotVectorTP==14, find(TimeVector==-200):find(TimeVector==500),1),1),'b','LineWidth',2);
plot(-200:500, mean(TotKine(NearTarget==2 & TotVectorTP==14, find(TimeVector==-200):find(TimeVector==500),1),1)+std(TotKine(NearTarget==2 & TotVectorTP==14, find(TimeVector==-200):find(TimeVector==500),1),0,1)/sqrt(nSubjects),'b:','LineWidth',2);
plot(-200:500, mean(TotKine(NearTarget==2 & TotVectorTP==14, find(TimeVector==-200):find(TimeVector==500),1),1)-std(TotKine(NearTarget==2 & TotVectorTP==14, find(TimeVector==-200):find(TimeVector==500),1),0,1)/sqrt(nSubjects),'b:','LineWidth',2);

subplot(2,2,4); hold on; set(gca,'LineWidth',2); set(gca,'FontSize',14); set(gca,'Color','none');
plot(-200:500, mean(TotKine(NearTarget==3 & TotVectorTP==11, find(TimeVector==-200):find(TimeVector==500),1),1),'r','LineWidth',2);
plot(-200:500, mean(TotKine(NearTarget==3 & TotVectorTP==11, find(TimeVector==-200):find(TimeVector==500),1),1)+std(TotKine(NearTarget==3 & TotVectorTP==11, find(TimeVector==-200):find(TimeVector==500),1),0,1)/sqrt(nSubjects),'r:','LineWidth',2);
plot(-200:500, mean(TotKine(NearTarget==3 & TotVectorTP==11, find(TimeVector==-200):find(TimeVector==500),1),1)-std(TotKine(NearTarget==3 & TotVectorTP==11, find(TimeVector==-200):find(TimeVector==500),1),0,1)/sqrt(nSubjects),'r:','LineWidth',2);

plot(-200:500, mean(TotKine(NearTarget==2 & TotVectorTP==11, find(TimeVector==-200):find(TimeVector==500),1),1),'g','LineWidth',2);
plot(-200:500, mean(TotKine(NearTarget==2 & TotVectorTP==11, find(TimeVector==-200):find(TimeVector==500),1),1)+std(TotKine(NearTarget==2 & TotVectorTP==11, find(TimeVector==-200):find(TimeVector==500),1),0,1)/sqrt(nSubjects),'g:','LineWidth',2);
plot(-200:500, mean(TotKine(NearTarget==2 & TotVectorTP==11, find(TimeVector==-200):find(TimeVector==500),1),1)-std(TotKine(NearTarget==2 & TotVectorTP==11, find(TimeVector==-200):find(TimeVector==500),1),0,1)/sqrt(nSubjects),'g:','LineWidth',2);
%Final figure papier 

figure('Name','Change of mind in kinematics','units','normalized');
subplot(2,2,1); hold on; set(gca,'LineWidth',2); set(gca,'Color','none'); set(gca,'FontSize',14);
plot(-200:500, mean(TotKine(NearTarget==1 & TotVectorTP==8, find(TimeVector==-200):find(TimeVector==500),1),1),'r','LineWidth',2);
plot(-200:500, mean(TotKine(NearTarget==1 & TotVectorTP==8, find(TimeVector==-200):find(TimeVector==500),1),1)+std(TotKine(NearTarget==1 & TotVectorTP==8, find(TimeVector==-200):find(TimeVector==500),1),0,1)/sqrt(nSubjects),'r:','LineWidth',2);
plot(-200:500, mean(TotKine(NearTarget==1 & TotVectorTP==8, find(TimeVector==-200):find(TimeVector==500),1),1)-std(TotKine(NearTarget==1 & TotVectorTP==8, find(TimeVector==-200):find(TimeVector==500),1),0,1)/sqrt(nSubjects),'r:','LineWidth',2);
plot(-200:500, mean(TotKine(NearTarget==2 & TotVectorTP==8, find(TimeVector==-200):find(TimeVector==500),1),1),'b','LineWidth',2);
plot(-200:500, mean(TotKine(NearTarget==2 & TotVectorTP==8, find(TimeVector==-200):find(TimeVector==500),1),1)+std(TotKine(NearTarget==2 & TotVectorTP==8, find(TimeVector==-200):find(TimeVector==500),1),0,1)/sqrt(nSubjects),'b:','LineWidth',2);
plot(-200:500, mean(TotKine(NearTarget==2 & TotVectorTP==8, find(TimeVector==-200):find(TimeVector==500),1),1)-std(TotKine(NearTarget==2 & TotVectorTP==8, find(TimeVector==-200):find(TimeVector==500),1),0,1)/sqrt(nSubjects),'b:','LineWidth',2);
xlabel('Time [ms]'); ylabel('x-position [cm]');
xline(onsetvalue8,'k:','linewidth',2); xline(0,'k--','linewidth',2);
xticks([-150 0 400]); xticklabels({'-0.15','0','0.4'});
yticks([-0.01 0.04 0.09]); yticklabels({'-10','-5','0'});
ylim([-0.015 0.1]);

subplot(2,2,3); hold on; set(gca,'LineWidth',2); set(gca,'Color','none'); set(gca,'FontSize',14);
plot(-200:500, mean(TotKine(NearTarget==1 & TotVectorTP==7, find(TimeVector==-200):find(TimeVector==500),1),1),'r','LineWidth',2);
plot(-200:500, mean(TotKine(NearTarget==1 & TotVectorTP==7, find(TimeVector==-200):find(TimeVector==500),1),1)+std(TotKine(NearTarget==1 & TotVectorTP==7, find(TimeVector==-200):find(TimeVector==500),1),0,1)/sqrt(nSubjects),'r:','LineWidth',2);
plot(-200:500, mean(TotKine(NearTarget==1 & TotVectorTP==7, find(TimeVector==-200):find(TimeVector==500),1),1)-std(TotKine(NearTarget==1 & TotVectorTP==7, find(TimeVector==-200):find(TimeVector==500),1),0,1)/sqrt(nSubjects),'r:','LineWidth',2);
plot(-200:500, mean(TotKine(NearTarget==2 & TotVectorTP==7, find(TimeVector==-200):find(TimeVector==500),1),1),'b','LineWidth',2);
plot(-200:500, mean(TotKine(NearTarget==2 & TotVectorTP==7, find(TimeVector==-200):find(TimeVector==500),1),1)+std(TotKine(NearTarget==2 & TotVectorTP==7, find(TimeVector==-200):find(TimeVector==500),1),0,1)/sqrt(nSubjects),'b:','LineWidth',2);
plot(-200:500, mean(TotKine(NearTarget==2 & TotVectorTP==7, find(TimeVector==-200):find(TimeVector==500),1),1)-std(TotKine(NearTarget==2 & TotVectorTP==7, find(TimeVector==-200):find(TimeVector==500),1),0,1)/sqrt(nSubjects),'b:','LineWidth',2);
xlabel('Time [ms]'); ylabel('x-position [cm]');
xline(onsetvalue7,'k:','linewidth',2); xline(0,'k--','linewidth',2);
xticks([-150 0 400]); xticklabels({'-0.15','0','0.4'});
yticks([-0.01 0.04 0.09]); yticklabels({'-10','-5','0'});
ylim([-0.015 0.1]);

subplot(2,2,2); hold on; set(gca,'LineWidth',2); set(gca,'Color','none'); set(gca,'FontSize',14);
plot(-200:500, mean(TotKine(NearTarget==1 & TotVectorTP==8, find(TimeVector==-200):find(TimeVector==500),1),1),'r','LineWidth',2);
plot(-200:500, mean(TotKine(NearTarget==1 & TotVectorTP==8, find(TimeVector==-200):find(TimeVector==500),1),1)+std(TotKine(NearTarget==1 & TotVectorTP==8, find(TimeVector==-200):find(TimeVector==500),1),0,1)/sqrt(nSubjects),'r:','LineWidth',2);
plot(-200:500, mean(TotKine(NearTarget==1 & TotVectorTP==8, find(TimeVector==-200):find(TimeVector==500),1),1)-std(TotKine(NearTarget==1 & TotVectorTP==8, find(TimeVector==-200):find(TimeVector==500),1),0,1)/sqrt(nSubjects),'r:','LineWidth',2);
plot(-200:500, mean(TotKine(NearTarget==2 & TotVectorTP==8, find(TimeVector==-200):find(TimeVector==500),1),1),'b','LineWidth',2);
plot(-200:500, mean(TotKine(NearTarget==2 & TotVectorTP==8, find(TimeVector==-200):find(TimeVector==500),1),1)+std(TotKine(NearTarget==2 & TotVectorTP==8, find(TimeVector==-200):find(TimeVector==500),1),0,1)/sqrt(nSubjects),'b:','LineWidth',2);
plot(-200:500, mean(TotKine(NearTarget==2 & TotVectorTP==8, find(TimeVector==-200):find(TimeVector==500),1),1)-std(TotKine(NearTarget==2 & TotVectorTP==8, find(TimeVector==-200):find(TimeVector==500),1),0,1)/sqrt(nSubjects),'b:','LineWidth',2);
xlabel('Time [ms]'); ylabel('x-position [cm]');
xline(onsetvalue8,'k:','linewidth',2); xline(0,'k--','linewidth',2);
xticks([-150 0 400]); xticklabels({'-0.15','0','0.4'});
yticks([-0.01 0.04 0.09]); yticklabels({'-10','-5','0'});
ylim([-0.015 0.1]);

subplot(2,2,4); hold on; set(gca,'LineWidth',2); set(gca,'Color','none'), set(gca,'FontSize',14);
plot(-200:500, mean(TotKine(NearTarget==1 & TotVectorTP==14, find(TimeVector==-200):find(TimeVector==500),1),1),'r','LineWidth',2);
plot(-200:500, mean(TotKine(NearTarget==1 & TotVectorTP==14, find(TimeVector==-200):find(TimeVector==500),1),1)+std(TotKine(NearTarget==1 & TotVectorTP==14, find(TimeVector==-200):find(TimeVector==500),1),0,1)/sqrt(nSubjects),'r:','LineWidth',2);
plot(-200:500, mean(TotKine(NearTarget==1 & TotVectorTP==14, find(TimeVector==-200):find(TimeVector==500),1),1)-std(TotKine(NearTarget==1 & TotVectorTP==14, find(TimeVector==-200):find(TimeVector==500),1),0,1)/sqrt(nSubjects),'r:','LineWidth',2);
plot(-200:500, mean(TotKine(NearTarget==2 & TotVectorTP==14, find(TimeVector==-200):find(TimeVector==500),1),1),'b','LineWidth',2);
plot(-200:500, mean(TotKine(NearTarget==2 & TotVectorTP==14, find(TimeVector==-200):find(TimeVector==500),1),1)+std(TotKine(NearTarget==2 & TotVectorTP==14, find(TimeVector==-200):find(TimeVector==500),1),0,1)/sqrt(nSubjects),'b:','LineWidth',2);
plot(-200:500, mean(TotKine(NearTarget==2 & TotVectorTP==14, find(TimeVector==-200):find(TimeVector==500),1),1)-std(TotKine(NearTarget==2 & TotVectorTP==14, find(TimeVector==-200):find(TimeVector==500),1),0,1)/sqrt(nSubjects),'b:','LineWidth',2);
xlabel('Time [ms]'); ylabel('x-position [cm]');
xline(onsetvalue14,'k:','linewidth',2); xline(0,'k--','linewidth',2);
xticks([-150 0 400]); xticklabels({'-0.15','0','0.4'});
yticks([-0.01 0.04 0.09]); yticklabels({'-10','-5','0'});
ylim([-0.015 0.1]);

timevector_test = 0:300;
pval_table = zeros(4,length(timevector_test));
for ii = 1 : size(pval_table,2)
   [pval_table(1,ii)] = ranksum(TotKine(NearTarget==1 & TotVectorTP==8, find(TimeVector==timevector_test(ii)),1),TotKine(NearTarget==2 & TotVectorTP==8, find(TimeVector==timevector_test(ii)),1));
   [pval_table(2,ii)] = ranksum(TotKine(NearTarget==3 & TotVectorTP==5, find(TimeVector==timevector_test(ii)),1),TotKine(NearTarget==2 & TotVectorTP==5, find(TimeVector==timevector_test(ii)),1));
   [pval_table(3,ii)] = ranksum(TotKine(NearTarget==1 & TotVectorTP==7, find(TimeVector==timevector_test(ii)),1),TotKine(NearTarget==2 & TotVectorTP==7, find(TimeVector==timevector_test(ii)),1));
   [pval_table(4,ii)] = ranksum(TotKine(NearTarget==3 & TotVectorTP==4, find(TimeVector==timevector_test(ii)),1),TotKine(NearTarget==2 & TotVectorTP==4, find(TimeVector==timevector_test(ii)),1));
end

figure('Name','Representing the decision time','units','normalized'); hold on;
plot(log10(pval_table(1,:)),'b','LineWidth',2);
plot(log10(pval_table(3,:)),'b:','LineWidth',2);
plot(log10(pval_table(2,:)),'g','LineWidth',2);
plot(log10(pval_table(4,:)),'g:','LineWidth',2);
plot(log10(0.05)*ones(1,length(timevector_test)),'r:');
plot(log10(0.005)*ones(1,length(timevector_test)),'r');

 
mat81 = TotKine(NearTarget==1 & TotVectorTP==8,find(TimeVector==timevector_test(1)):find(TimeVector==timevector_test(end)),1);
mat82 = TotKine(NearTarget==2 & TotVectorTP==8,find(TimeVector==timevector_test(1)):find(TimeVector==timevector_test(end)),1);
mat141 = TotKine(NearTarget==1 & TotVectorTP==14,find(TimeVector==timevector_test(1)):find(TimeVector==timevector_test(end)),1);
mat142 = TotKine(NearTarget==2 & TotVectorTP==14,find(TimeVector==timevector_test(1)):find(TimeVector==timevector_test(end)),1);
mat71 = TotKine(NearTarget==1 & TotVectorTP==7,find(TimeVector==timevector_test(1)):find(TimeVector==timevector_test(end)),1);
mat72 = TotKine(NearTarget==2 & TotVectorTP==7,find(TimeVector==timevector_test(1)):find(TimeVector==timevector_test(end)),1);
mat53 = TotKine(NearTarget==3 & TotVectorTP==5,find(TimeVector==timevector_test(1)):find(TimeVector==timevector_test(end)),1);
mat52 = TotKine(NearTarget==2 & TotVectorTP==5,find(TimeVector==timevector_test(1)):find(TimeVector==timevector_test(end)),1);
mat111 = TotKine(NearTarget==3 & TotVectorTP==11,find(TimeVector==timevector_test(1)):find(TimeVector==timevector_test(end)),1);
mat112 = TotKine(NearTarget==2 & TotVectorTP==11,find(TimeVector==timevector_test(1)):find(TimeVector==timevector_test(end)),1);
mat43 = TotKine(NearTarget==3 & TotVectorTP==4,find(TimeVector==timevector_test(1)):find(TimeVector==timevector_test(end)),1);
mat42 = TotKine(NearTarget==2 & TotVectorTP==4,find(TimeVector==timevector_test(1)):find(TimeVector==timevector_test(end)),1);
aucvalue8 = TotalAUC(mat81,mat82);
aucvalue14= TotalAUC(mat141,mat142);
aucvalue7 = TotalAUC(mat71,mat72);
aucvalue5 = TotalAUC(mat53,mat52);
aucvalue11 = TotalAUC(mat111,mat112);
aucvalue4 = TotalAUC(mat43,mat42);
close all;
figure('Name','ROC analysis','units','normalized'); hold on;
set(gca,'Color','none'); set(gca,'LineWidth',2); set(gca,'FontSize',14);
plot(aucvalue7,'r:','LineWidth',2); plot(aucvalue8,'r','LineWidth',2);
plot(aucvalue4,'b:','LineWidth',2); plot(aucvalue5,'b','LineWidth',2);
plot(aucvalue11,'b--','LineWidth',2); plot(aucvalue14,'r--','LineWidth',2);
plot([1 length(aucvalue4)],[0.75 0.75],'k:','LineWidth',2);

onsetvalue8 = find(aucvalue8>0.75,1);
onsetvalue14= find(aucvalue14>0.75,1);
onsetvalue7 = find(aucvalue7>0.75,1);
onsetvalue4 = find(aucvalue4>0.75,1);
onsetvalue5 = find(aucvalue5>0.75,1);
onsetvalue11 = find(aucvalue11>0.75,1);



diff_aucvalues8 = diff(aucvalue8(onsetvalue8-1:onsetvalue8));
diff_aucvalues7 = diff(aucvalue7(onsetvalue7-1:onsetvalue7));
diff_aucvalues4 = diff(aucvalue4(onsetvalue4-1:onsetvalue4));
diff_aucvalues5 = diff(aucvalue5(onsetvalue5-1:onsetvalue5));



close all; 
figure('Name','barplot of the timing','units','normalized'); 
subplot(1,2,1); hold on; set(gca,'Color','none'); set(gca,'LineWidth',2); set(gca,'FontSize',14);
bar([1 2 3],[onsetvalue7 onsetvalue8 onsetvalue14]);
xticks([1 2 3]); xticklabels({'Different rewards','Baseline','Smaller force'}); xtickangle(45);

subplot(1,2,2); hold on; set(gca,'Color','none'); set(gca,'LineWidth',2); set(gca,'FontSize',14);
bar([1 2 3],[onsetvalue4 onsetvalue5 onsetvalue11]);
xticks([1 2 3]); xticklabels({'Different rewards','Baseline','Smaller force'}); xtickangle(45);


figure('Name','plot for the time','units','normalized'); hold on;
subplot(1,2,1); hold on; set(gca,'Fontsize',14); set(gca,'Color','none'); set(gca,'LineWidth',2);
plot([1,2], [onsetvalue8, onsetvalue7],'k.-','LineWidth',2,'MarkerSize',25); %left
plot([1,2], [onsetvalue5, onsetvalue4],'k.:','LineWidth',2,'MarkerSize',25); %right
%plot([1 2], [mean([onsetvalue8 onsetvalue5]) mean([onsetvalue7 onsetvalue4])],'k.-','LineWidth',2,'MarkerSize',25);
title('sd'); xlim([0.7 2.3]); ylim([140 200]);
xticks([1 2]); xticklabels({'Same reward','Different rewards'}); xtickangle(45);

subplot(1,2,2); hold on; set(gca,'FontSize',14); set(gca,'Color','none'); set(gca,'LineWidth',2);
plot([1,2], [onsetvalue8,onsetvalue14],'k.-','LineWidth',2,'MarkerSize',25); %left
plot([1,2], [onsetvalue5,onsetvalue11],'k.:','LineWidth',2,'MarkerSize',25); %right
%plot([1 2], [mean([onsetvalue8 onsetvalue5]) mean([onsetvalue14 onsetvalue11])],'k.-','LineWidth',2,'MarkerSize',25);
title('sl'); xlim([0.7 2.3]);ylim([140 200]);
xticks([1 2]); xticklabels({'Large force','Small force'}); xtickangle(45);
