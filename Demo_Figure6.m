% Figure 6 experiments

close all
clear all
clc
format bank

% Additional 3 nodes on roads 204 and 402
close all, clear all, clc, format bank
agvSim = AGVSim(10);
agvSim.DRAW=1;
MapClass.createSimpleMap8a(agvSim);
agvSim.drawRoadLabels=0;  agvSim.drawNodeLabels=1;
agvSim.fontSize=140*1.7;
agvSim.fontSizeAGV=14;

% Additional graph connection
nID1=1; nID2=2; p1=[0;0]; p2=[0;4]; rID12=1200; twoway=0; rID21=25;
secShape=['A']; secParam=[0 0 -pi -pi pi*2];
agvSim.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )

figure(10); clf; axis equal; agvSim.precomputeMap(); agvSim.drawMap();


% AGV dimensions (half width/length)
L=0.24; 
W=0.12; 
agvSim.setRobotHalfLengthWidth(L,W); 

% AGV speed and safety margin
speed=1;
safetyMargin=1.6;

xlabel('$$x$$[m]','interpreter','latex','FontSize',14), ylabel('$$y$$[m]','interpreter','latex','FontSize',14)

% Zacetne == start locations
% Zacetne= [AGVid  rID loc L W ;]
Zacetne=[1 710  0 L W       
         2 1108 0 L W
         3 220  0 L W ];

% Initialize AGVs
for i=1:size(Zacetne,1)
    agvID = Zacetne(i,1); rid=Zacetne(i,2); loc=Zacetne(i,3); len=Zacetne(i,4); wid=Zacetne(i,5);  priority=0;
    agvSim.createAGVonRoadLW(agvID , rid, loc,len,wid, priority);
    ht(agvID) = text(5,-.5-0.5*agvID,' ');
end
Nagv = size(agvSim.AGV,2);

clearOccupancies(agvSim);
notFoundPaths=0;
h=gobjects(1,size(Zacetne,1));


%%%%%% Simulation parameters
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
zamikIzris=0.2; % Road plan graphics shifting
Ts=0.1;         % Sample time
agvSim.preventCollision=1;   % 0=ignore collisions, 1=block on collision, 2=ignore, but remember
%figure(10)
tmax=300;       % Simulation time
vmax=3;         % Maximum AGV speed
tSimDelay=Ts*.25*.25*.1; % Simulation display delay

% Tasks     PICK node    DROP node    start DELAY   WAIT pick      WAIT drop
Tasks =[    3.00         13.00             0          2.00          2.00
            6.00          5.00             0          2.00          2.00
            3.00          5.00             0          2.00          2.00
            1.00          5.00             0          2.00          2.00
            1.00         14.00             0          2.00          2.00
            3.00         13.00             0          2.00          2.00
            6.00          5.00             0          2.00          2.00
            6.00         13.00             0          2.00          2.00
            6.00          5.00             0          2.00          2.00
            1.00         14.00             0          2.00          2.00]

idxTasks=1; 
Ntasks=size(Tasks,1);
agvTask=zeros(size(agvSim.AGV,2),2);

% Simulation start
agvSim.intSimTime();
dAGV=zeros(Nagv,1); % Driven distances
t=Ts*0; i=1;

while(~(agvSim.AGVsPlanAchived && idxTasks>Ntasks) && t<tmax)
    % Find free AGVs
    prostiAgvji=find([agvSim.AGV(:).state] == 0);
    if ~isempty(prostiAgvji)
        for frejAgv=prostiAgvji
            if idxTasks<=Ntasks && t>=Tasks(idxTasks,3)
                agvSim.AGVsPlanAchived=0;

                safetyRadious = agvSim.AGV(frejAgv).AGVRadious*safetyMargin;
                priority=max([agvSim.AGV.priority])+1; % dam mu najmanjso prioriteto

                % Plan path
                timePlan=fcnSIPPContainer.planPathPickDrop(agvSim,t,speed,frejAgv,Tasks,idxTasks,safetyRadious,priority); % (startID, goalID, agvSpeed, startTime, safetyRadious,priority,replantime)

                if(~isempty(timePlan))
                    dAGV(frejAgv)=0;
                    agvSim.AGVsPlanAchived=0;

                    tw2=Tasks(idxTasks,4);
                    agvID=agvSim.AGV(frejAgv).id;

                    fcnSIPPContainer.setOccupancySIPP2(agvSim,timePlan,agvID,speed,safetyRadious,priority,t,tw2); % Set OIs

                    replanTimeTemp = timePlan(end,1);
                    tNadajujem = replanTimeTemp+tw2*0;

                    rid=timePlan(end,2);
                    start=agvSim.roadID(rid).eNode;

                    fcnSIPPContainer.addOccupancyForAgvStartDeadEndRoad(agvSim,agvID,replanTimeTemp,tNadajujem,safetyRadious,speed, start);

                    % Assign priority
                    agvSim.AGV(frejAgv).priority=priority;

                    [~,onRoadID,locStartTmp]=agvSim.getLocPoseAGV(agvID);
                    if onRoadID~=timePlan(1,2) % if not on the road (in the node, but on a different road...)
                        agvSim.replaceAGVonRoad(agvID,timePlan(1,2), 0);
                        locStart=0;
                    else
                        locStart=locStartTmp;
                    end
                    agvSim.setPlanRoadTimeSincLUT (agvID, timePlan,t);

                    % --------------------------------------------------------------------------
                    % Rendering
                    if(~isempty(h(frejAgv))), delete(h(frejAgv)); end
                    h(frejAgv)=agvSim.drawOnePlan(agvID,10,zamikIzris, locStart);


                    % le izpis plana na graf
                    if(~isempty(ht(agvID))), delete(ht(agvID)); end
                    ht(agvID)= text(5,-.1-0.5*agvID,strcat('AGV',num2str(agvID),', T=',num2str(idxTasks),' ,', num2str(Tasks(idxTasks,1)),' ->',num2str(Tasks(idxTasks,2))));

                    % hranim plane za vsak task
                    taskSol(idxTasks).agvID=agvID;
                    taskSol(idxTasks).pickDrop=Tasks(idxTasks,1:2);
                    taskSol(idxTasks).timePlan=timePlan;
                    taskSol(idxTasks).timePlanSinc=agvSim.AGV(agvID).planRoadTimeSinc;
                    taskSol(idxTasks).startRID=agvSim.AGV(frejAgv).onRoadID;
                    taskSol(idxTasks).startLOC=agvSim.AGV(frejAgv).onRoadLoc;
                    taskSol(idxTasks).startT=t;

                    idxTasks=idxTasks+1;  % naslednji task
                    % --------------------------------------------------------------------------

                else
                    % No path found
                    if(~isempty(h(frejAgv))), delete(h(frejAgv)); end
                    timePlan=[];
                    fprintf(1,' For AGV %d no path is found.\n',agvSim.AGV(frejAgv).id);
                    notFoundPaths=notFoundPaths+1;
                    agvSim.AGV(frejAgv).planRoadTimeSinc=[];
                end
            end
        end
    end

    t=t+Ts;
    dAGV=agvSim.moveAGVsOneSample(Ts,vmax,tSimDelay,dAGV,t); % Simulate one step
    i=i+1;
end

idxTasks
taskSol

% Results analysis
idleSumTime=notFoundPaths*Ts;
metrika=fcnSIPPContainer.metricsTasks(agvSim,Tasks,taskSol,notFoundPaths,idleSumTime);

agvSim.drawPlanAGVs(); % Draw plans

%% Render timelines
for i = 1:Nagv
    tasks = taskSol([taskSol(:).agvID] == i);
    agvSim.AGV(i).lifeLongPlanHistory = [];
    for n = 1:size(tasks,2)
        agvSim.AGV(i).lifeLongPlanHistory = [agvSim.AGV(i).lifeLongPlanHistory; tasks(n).timePlanSinc];
    end
    agvSim.AGV(i).lifeLongPlanHistory = [ agvSim.AGV(i).lifeLongPlanHistory zeros(size(agvSim.AGV(i).lifeLongPlanHistory,1), 1)];
end

% Render pick and drop locations in colors
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


%%
MapClass.drawTimelines(agvSim, agvSim.AGV(1:3), 0, []);

%%
%set(gcf,'PaperPosition',[0 0 8 4]);
%print -depsc  planiLepljenjaTimeline;

