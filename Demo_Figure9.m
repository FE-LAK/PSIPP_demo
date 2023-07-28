% Figure 9 experiments

close all, clear all, clc, format bank
agvSim = AGVSim(10); % simulator init
agvSim.DRAW=1;
agvSim.fontSize = 8;
agvSim.drawRoadLabels = 1;
agvSim.drawNodeLabels = 1;
agvSim.fontSize=140;
agvSim.fontSizeAGV=12;

% AGV parameters
L=0.24; W=0.12; agvSim.setRobotHalfLengthWidth(L,W);
safetyMargin=1.6;
s=1;

% Load map
map_figure9(agvSim);

% Add the safe location
[chargeRoad, waiting_nodeID, chargeEntranceRoad, chargeEntranceNode]=addChargingStation_v2(agvSim, 407,0,1,'L',1,'S');
figure(10); clf; axis equal; agvSim.precomputeMap(); agvSim.drawMap();
chargeStation=[chargeRoad,chargeEntranceNode,chargeEntranceRoad]; % Entrance to the safe location
goalFreeTime=0;
agvSim.roadToRestNode=chargeRoad;

agvSim.restNode=agvSim.roadID(chargeRoad).eNode;
restLocationGoalData=[agvSim.roadToRestNode 0.8, 0,0,0]; % Safe location is on the road

Ts=0.1; % Sampling time
zamikIzris=0.05;

% Load the example set of orders
Ns =  [18  32  29  28  27  26  25 ];
Ne =  [ 4  13  16  19  15   2   6 ];

Orders=[1 0 s 2 6 L W;     % Orders= [AGVid  tStart speed Ns Ne L W ; ]
        2 0 s 1 7 L W;
        3 0 s 9 14 L W;
        4 0 s 14 8 L W;
        5 0 s 13 2 L W;
        6 0 s 17 5 L W;
        7 0 s 18 4 L W];

Orders(:,4) = Ns';
Orders(:,5) = Ne';

casi=nan(size(Orders,1),1);

%%%%%%% Planning
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

clearOccupancies(agvSim);
replanTime=0;

% Check for dead ends
fcnSIPPContainer.adaptOccupanciesForAgvTrapedDeadEndRoad(agvSim,Orders,safetyMargin,replanTime);

AllTimePlans={};
notFoundPaths=0;
for i=1:size(Orders,1)
    agvID=Orders(i,1);startTime=Orders(i,2); speed=Orders(i,3); Ns= Orders(i,4); Ne= Orders(i,5); len=Orders(i,6); wid=Orders(i,7);
    safetyRadious=sqrt(len^2+wid^2)*safetyMargin;

    priority=i;
    tw=0;

    % goalFreeTime=inf;
    if ~isempty(restLocationGoalData)
        multiGoalData= {[Ne, tw,goalFreeTime,0],restLocationGoalData };
    else        
        multiGoalData= {[Ne, 0,goalFreeTime,0] };
    end

    [timePlan, times] = findPathSIPP_multiGoal(agvSim, Ns, multiGoalData, ...
        speed, startTime, safetyRadious, priority, replanTime, agvID, chargeStation);
    
    % Add the safe location entrance again
    if ~isempty(chargeStation)
        fcnSIPPContainer.addChargeEntranceRoad(agvSim, chargeEntranceNode, chargeEntranceRoad);
    end

    if(~isempty(timePlan))
        casi(i)= times(1);

        goalWait=goalFreeTime;
        fcnSIPPContainer.setOccupancySIPP2(agvSim,timePlan,agvID,speed,safetyRadious,priority,replanTime,goalWait); % nastavi OI-je

        agvSim.createAGVonRoadLW(agvID , timePlan(1,2), 0,len,wid,priority);

        fcnSIPPContainer.addChargeEntranceRoad(agvSim, chargeEntranceNode, chargeEntranceRoad);
        agvSim.setPlanRoadTimeSinc (agvID, timePlan);
        fcnSIPPContainer.removeChargeEntranceRoad(agvSim,chargeEntranceNode,chargeEntranceRoad);
    else
        timePlan=[];
        disp('For this AGV no path is found.');
        notFoundPaths=notFoundPaths+1;
        agvSim.createAGVonPoseLW(agvID , [2+i*len;-1;0],len,wid, priority);
    end
    AllTimePlans{i}=timePlan;
end

sumCost=0;
taskTime=0;
for i=1:length(AllTimePlans)
    if (~isempty(AllTimePlans{i}))
        sumCost=sumCost+(AllTimePlans{i}(end,1)-Orders(i,2)); % upoštevam se zacetni cas (Orders(i,2)), ce je >0
        taskTime=max(taskTime,AllTimePlans{i}(end,1));
    else
        sumCost=nan;
        taskTime=nan;
        break
    end
end
sumCost
taskTime
Nsipp=size(Orders,1)


sumCost2=sum(casi)
taskTime2=max(casi)
if sumCost2==NaN, taskTime2=NaN; end


%%%%%% Simulation
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
agvSim.recordMovie=0; % Movie recording flag



zamikIzris=0.2; % Road plan graphics shifting
Ts=0.1;         % Sample time
agvSim.preventCollision=1;   % 0=ignore collisions, 1=block on collision, 2=ignore, but remember
%figure(10)
tmax=80;        % Simulation time
vmax=3;         % Maximum AGV speed
tSimDelay=Ts*.25*.25; % Simulation display delay

figure(10);
agvSim.simulateAGVs( Ts, vmax, tmax,tSimDelay); % sample time, vmax, tmax
agvSim.drawPlanAGVs(); % Izris grafa analize
agvSim.drawPlanPaths(10,zamikIzris); %Izris poti

figure(10)
if(notFoundPaths==0 && agvSim.AGVsPlanAchived==1)
    title('SIPP: OK. Plan achieved. All AGVs reached goal. ');
elseif(notFoundPaths==0 && agvSim.AGVsPlanAchived==0)
    title('SIPP: PROBLEM. All paths computed but some AGVs did NOT reach goal. ');
elseif(notFoundPaths>0 && agvSim.AGVsPlanAchived==1)
    title('SIPP: CCA OK. Not all paths are found! Other vehicles have reched goal. (Vehicles without plan are not simulated.)');
    fprintf(1,' \n For %d AGVs path was not found. This AGVs are not simulated and drawn! \n',notFoundPaths);
end





