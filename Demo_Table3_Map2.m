% Simple example in Table3 map1

close all, clear all, clc, format bank

% ---- run options --
Nagv=5;  % chose 5 or 10
flag_method=1;   % =1 for PSIPP, =2 for PSIPP+, =3 for PSIPPs
%--------------------

flagSimulate=1;

agvSim = AGVSim(10);
agvSim.DRAW=1;

agvSim.drawRoadLabels = 1;
agvSim.drawNodeLabels = 1;
agvSim.fontSize=140;
agvSim.fontSizeAGV=12;

agvSim.fontSize = 8;
agvSim.drawRoadLabels = 0;
agvSim.drawNodeLabels = 1;
agvSim.fontSize=180;
agvSim.fontSizeAGV=12;

L=0.24; W=0.12;
agvSim.setRobotHalfLengthWidth(L,W);

s=1;

map_figure9(agvSim);

if flag_method==3 % Do we have safe location (PSIPPs)?
    [chargeRoad, waiting_nodeID, chargeEntranceRoad, chargeEntranceNode]=addChargingStation_v2(agvSim, 407,0,1,'L',1,'S');
    figure(10); clf; axis equal; agvSim.precomputeMap(); agvSim.drawMap();
    chargeStation=[chargeRoad,chargeEntranceNode,chargeEntranceRoad];
    goalFreeTime=0;
    agvSim.roadToRestNode=chargeRoad;


    agvSim.restNode=agvSim.roadID(chargeRoad).eNode;
    restLocationGoalData=[agvSim.roadToRestNode 0.8, 0,0,0];
else % just (PSIPP)
    chargeStation=[];
    goalFreeTime=inf;
    restLocationGoalData=[];
    chargeRoad=[];chargeEntranceNode=[];chargeEntranceRoad=[];
end

Ts=0.1; 
zamikIzris=0.05;

if Nagv==5 % 5 AGVs in Table 3
    SS=[ 32  26  18  25  24]; % start nodes
    EE=[  2   4  19  13   1]; % end nodes
    OrdPermBest=[2 1 3 4 5]; % best combination of priorities for PSSIP+
elseif Nagv==10 % 10 AGVs in Table 3
    SS=[ 27  23  26  28  24  29  32   9  18  25]; % start nodes
    EE=[ 16   5  13   3  19   6   2  15   1   4]; % end nodes
    OrdPermBest=[8 7 2 5 9 6 4 3 10 1]; % best combination of priorities for PSSIP+
end

Orders=[];
for j=1:length(SS)
    Orders=[Orders; j 0 s SS(j) EE(j) L W];
end

if flag_method==2 % get PSIPP+ (best combination of priorities)
    OrdersP= Orders(OrdPermBest,:); % permutiran vrstni red
    Orders=OrdersP;
end

casi=nan(size(Orders,1),1);

%%%%%%% Planning
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

clearOccupancies(agvSim);
safetyMargin=1.6;
replanTime=0;

fcnSIPPContainer.adaptOccupanciesForAgvTrapedDeadEndRoad(agvSim,Orders,safetyMargin,replanTime);

AllTimePlans={};
notFoundPaths=0;
for i=1:size(Orders,1)
    agvID=Orders(i,1);startTime=Orders(i,2); speed=Orders(i,3); Ns= Orders(i,4); Ne= Orders(i,5); len=Orders(i,6); wid=Orders(i,7);
    safetyRadious=sqrt(len^2+wid^2)*safetyMargin;

    priority=i;

    if ~isempty(restLocationGoalData)
        tw=0;
        multiGoalData= {[Ne, tw,goalFreeTime,0],restLocationGoalData };
    else
        tw=0;
        multiGoalData= {[Ne, 0,goalFreeTime,0] };
    end

    [timePlan, times] = findPathSIPP_multiGoal(agvSim, Ns, multiGoalData, ...
        speed, startTime, safetyRadious, priority, replanTime, agvID, chargeStation);

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
%sumCost
%taskTime
Nsipp=size(Orders,1);

sumCost2=sum(casi)
taskTime2=max(casi)
if sumCost2==NaN, taskTime2=NaN; end

if flagSimulate==0
    return
end

%%%%%% Simulation
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
agvSim.recordMovie=0;

agvSim.preventCollision=1;
figure(10),
tmax=80;
vmax=3;
tSimDelay=Ts*.25*.25;
agvSim.simulateAGVs( Ts, vmax, tmax,tSimDelay);
agvSim.drawPlanAGVs();
agvSim.drawPlanPaths(10,zamikIzris);

figure(10)
if(notFoundPaths==0 && agvSim.AGVsPlanAchived==1)
    title('SIPP: OK. Plan achieved. All AGVs reached goal. ');
elseif(notFoundPaths==0 && agvSim.AGVsPlanAchived==0)
    title('SIPP: PROBLEM. All paths computed but some AGVs did NOT reach goal. ');
elseif(notFoundPaths>0 && agvSim.AGVsPlanAchived==1)
    title('SIPP: CCA OK. Not all paths are found! Other vehicles have reched goal. (Vehicles without plan are not simulated.)');
    fprintf(1,' \n For %d AGVs path was not found. This AGVs are not simulated and drawn! \n',notFoundPaths);
end





