close all, clear all,  format bank
agvSim=[]; hPot=[];  hPot=[];hText1=[];hText2=[];




flagSafelocation=1;  % do we have safe location


%%%%%%%%%%%%%%% create map
agvSim = AGVSim(10);
agvSim.DRAW=1;
MapClass.createSimpleMap8a(agvSim);

agvSim.drawRoadLabels=0;  agvSim.drawNodeLabels=1;
agvSim.fontSize=200;
agvSim.fontSizeAGV=9;

% Additional graph connection
nID1=1; nID2=2; p1=[0;0]; p2=[0;4]; rID12=1200; twoway=0; rID21=25;
secShape=['A']; secParam=[0 0 -pi -pi pi*2];
agvSim.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )

figure(10); clf; axis equal; agvSim.precomputeMap(); agvSim.drawMap();

if flagSafelocation % do we have safe location
    % Safe location             
    [waiting_roadID, waiting_nodeID, entrance_roadID, entrance_nodeID]=addChargingStation_v2(agvSim, 407,0,1,'L',1,'S');
    figure(10); clf; axis equal; agvSim.precomputeMap(); agvSim.drawMap();
    % text(7.3,6.3,'SL');
end



L=0.24; W=0.12; rr=sqrt((L)^2+(W)^2); % polovicna dolzina in sirina  AGV-ja za namen detekcije trka
safetyMargin=1.6 ;
preventCollision=1;  %=0 ne preverja trkov, =1 preverja trke, =2 ignorira trke a jih belež
skipReplaneTime=1;  %Avoid successive unsuccessful replannings
agvSim.fontSize=180;
agvSim.fontSizeAGV=9;

 if flagSafelocation % do we have safe location
   % chargeStation=[chargeRoad,chargeEntranceNode,chargeEntranceRoad];
    chargeStation=[2324,4,423]; % vstop v polnilnico in polnilnica
    chargeRoad=chargeStation(1);
    goalFreeTime=0;
    agvSim.roadToRestNode=chargeRoad;
    agvSim.restNode=agvSim.roadID(chargeRoad).eNode;
    restLocationGoalData=[agvSim.roadToRestNode 0.98, 0,0,0]; % varno mesto je na cesti

  else
        chargeStation=[];
        goalFreeTime=inf;
        restLocationGoalData=[];  % brez varnega mesta
        chargeRoad=[];chargeEntranceNode=[];chargeEntranceRoad=[];
  end



    agvSim.setRobotHalfLengthWidth(L,W); % polovicna dolzina in sirina  AGV-ja za namen detekcije trka
    Ts=0.1; % Cas vzorcenja
    zamikIzris=0.2;

    set(gca,'Xlim',[get(gca,'XLim')]+[-1 1]);set(gca,'Ylim',[get(gca,'YLim')]+[-1 1]);
    figure(10),set(gcf,'position',[681.00        265.00       1028.00        834.00]);


drawnow();        

%%%%%%%%%%%%%%%%%% end -create map    
    
    
        
        








agvSim.DRAW =1;   % ali izrisujem, =0
flagSimulate=0;   % =0  % ali simuliram


Nstart = [7 11 2];

Tasks =[    3.00         13.00     
            6.00          5.00       
            3.00          5.00        
            1.00          5.00       
            1.00         14.00      
            3.00         13.00      
            6.00          5.00     
            6.00         13.00     
            6.00          5.00    
            1.00         14.00    ];



maxTask=size(Tasks,1);
[PSIPPL,taskSol]=fcnBench.PSIPPLLplanPickDropLifeLong(Nstart,Tasks,maxTask,agvSim,L,W,Ts,safetyMargin,skipReplaneTime,preventCollision,chargeStation,2,2);



Nagv=3;
%% Render timelines - draw only realised plan
for i = 1:Nagv
    tasks = taskSol([taskSol(:).agvID] == i);
    agvSim.AGV(i).lifeLongPlanHistory = [];
    for n = 1:size(tasks,2)
        % ???? erase part of the previous plan to SL which is not realised
        if (~isempty(agvSim.AGV(i).lifeLongPlanHistory))
            ii=find(agvSim.AGV(i).lifeLongPlanHistory(:,1) >= tasks(n).timePlanSinc(1,1));
            agvSim.AGV(i).lifeLongPlanHistory(ii,:)=[];
        end
        agvSim.AGV(i).lifeLongPlanHistory = [agvSim.AGV(i).lifeLongPlanHistory; tasks(n).timePlanSinc];
    end
    agvSim.AGV(i).lifeLongPlanHistory = [ agvSim.AGV(i).lifeLongPlanHistory zeros(size(agvSim.AGV(i).lifeLongPlanHistory,1), 1)];
end

maxt=0;
for (i=1:3)
    maxt=max(agvSim.AGV(i).lifeLongPlanHistory(end,1),maxt);
end
for (i=1:3)
    agvSim.AGV(i).lifeLongPlanHistory = [agvSim.AGV(i).lifeLongPlanHistory; agvSim.AGV(i).lifeLongPlanHistory(end,:)];
    agvSim.AGV(i).lifeLongPlanHistory(end,1)=maxt+5;
end

if flagSafelocation 
% % Render pick and drop locations in colors 
agvSim.AGV(1).lifeLongPlanHistory(2, 5) = 1;
agvSim.AGV(1).lifeLongPlanHistory(8, 5) = 2;
agvSim.AGV(1).lifeLongPlanHistory(18, 5) = 1;
agvSim.AGV(1).lifeLongPlanHistory(23, 5) = 2;
agvSim.AGV(1).lifeLongPlanHistory(29, 5) = 1;
agvSim.AGV(1).lifeLongPlanHistory(36, 5) = 2;
agvSim.AGV(1).lifeLongPlanHistory(47, 5) = 1;
agvSim.AGV(1).lifeLongPlanHistory(60, 5) = 2;
agvSim.AGV(1).lifeLongPlanHistory(end, 5) = 3;

agvSim.AGV(2).lifeLongPlanHistory(3, 5) = 1;
agvSim.AGV(2).lifeLongPlanHistory(10, 5) = 2;
agvSim.AGV(2).lifeLongPlanHistory(22, 5) = 1;
agvSim.AGV(2).lifeLongPlanHistory(32, 5) = 2;
agvSim.AGV(2).lifeLongPlanHistory(41, 5) = 1;
agvSim.AGV(2).lifeLongPlanHistory(48, 5) = 2;

agvSim.AGV(3).lifeLongPlanHistory(6, 5) = 1;
agvSim.AGV(3).lifeLongPlanHistory(15, 5) = 2;
agvSim.AGV(3).lifeLongPlanHistory(22, 5) = 1;
agvSim.AGV(3).lifeLongPlanHistory(28, 5) = 2;
agvSim.AGV(3).lifeLongPlanHistory(38, 5) = 1;
agvSim.AGV(3).lifeLongPlanHistory(43, 5) = 2;
% 
else
agvSim.AGV(1).lifeLongPlanHistory(2, 5) = 1;
agvSim.AGV(1).lifeLongPlanHistory(8, 5) = 2;
agvSim.AGV(1).lifeLongPlanHistory(18, 5) = 1;
agvSim.AGV(1).lifeLongPlanHistory(23, 5) = 2;
agvSim.AGV(1).lifeLongPlanHistory(31, 5) = 1;
agvSim.AGV(1).lifeLongPlanHistory(36, 5) = 2;

agvSim.AGV(2).lifeLongPlanHistory(3, 5) = 1;
agvSim.AGV(2).lifeLongPlanHistory(10, 5) = 2;
agvSim.AGV(2).lifeLongPlanHistory(18, 5) = 1;
agvSim.AGV(2).lifeLongPlanHistory(28, 5) = 2;
agvSim.AGV(2).lifeLongPlanHistory(35, 5) = 1;
agvSim.AGV(2).lifeLongPlanHistory(41, 5) = 2;

agvSim.AGV(3).lifeLongPlanHistory(6, 5) = 1;
agvSim.AGV(3).lifeLongPlanHistory(14, 5) = 2;
agvSim.AGV(3).lifeLongPlanHistory(21, 5) = 1;
agvSim.AGV(3).lifeLongPlanHistory(27, 5) = 2;
agvSim.AGV(3).lifeLongPlanHistory(32, 5) = 1;
agvSim.AGV(3).lifeLongPlanHistory(38, 5) = 2;
agvSim.AGV(3).lifeLongPlanHistory(43, 5) = 1;
agvSim.AGV(3).lifeLongPlanHistory(53, 5) = 2;
    
end

%%
MapClass.drawTimelines(agvSim, agvSim.AGV(1:3), 0, []);

%%
%set(gcf,'PaperPosition',[0 0 8 4]);
%print -depsc  planiLepljenjaTimelineSL;






