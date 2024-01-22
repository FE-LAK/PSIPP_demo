% Simple example in Table3 map1

close all, clear all, clc, format bank


% ----run options --
Nagv=3;  % chose 3, 4 or 5
flag_method=1;  % =1 for PSIPP, =2 for PSIPP+
%-------------------

flagSimulate=1;

agvSim = AGVSim(10);
agvSim.DRAW=1;

agvSim.drawRoadLabels = 1;
agvSim.drawNodeLabels = 1;
agvSim.fontSize=140;
agvSim.fontSizeAGV=12;


zamikIzris=0.05;
Ts=0.1;

agvSim.fontSize = 34;
L=.5/sqrt(2);
W=.5/sqrt(2);
agvSim.setRobotHalfLengthWidth(L,W);


%----- build map
nID1=1; nID2=2; p1=[1;5]; p2=[3;5]; rID12=102; twoway=1; rID21=201;secShape=['L'];  secParam=[p1(1) p1(2) p2(1) p2(2) norm(p1-p2)];agvSim.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
nID1=3; nID2=4; p1=[6;5]; p2=[7;5]; rID12=304; twoway=1; rID21=403;secShape=['L'];  secParam=[p1(1) p1(2) p2(1) p2(2) norm(p1-p2)]; agvSim.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
nID1=5; nID2=6; p1=[1;3]; p2=[3;3]; rID12=506; twoway=1; rID21=605;secShape=['L'];  secParam=[p1(1) p1(2) p2(1) p2(2) norm(p1-p2)]; agvSim.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
nID1=7; nID2=8; p1=[1;1]; p2=[3;1]; rID12=708; twoway=1; rID21=807;secShape=['L'];  secParam=[p1(1) p1(2) p2(1) p2(2) norm(p1-p2)]; agvSim.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
nID1=9; nID2=10; p1=[5;1]; p2=[7;1]; rID12=9010; twoway=1; rID21=1009;secShape=['L'];  secParam=[p1(1) p1(2) p2(1) p2(2) norm(p1-p2)]; agvSim.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
nID1=2; nID2=6; p1=[3;5]; p2=[3;3]; rID12=206; twoway=1; rID21=603;secShape=['L'];  secParam=[p1(1) p1(2) p2(1) p2(2) norm(p1-p2)]; agvSim.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )

agvSim.drawRoadLabels=0;
agvSim.drawNodeLabels=1;

flagNearbyNode=1; % with or withour near-by node. This solves correct planning (collision detection) in the connections intersection
if(~flagNearbyNode)
    nID1=6; nID2=9; p1=[3;3]; p2=[5;1]; rID12=609; twoway=1; rID21=906;secShape=['L'];  secParam=[p1(1) p1(2) p2(1) p2(2) norm(p1-p2)]; agvSim.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
    nID1=8; nID2=3; p1=[3;1]; p2=[6;5]; rID12=803; twoway=1; rID21=308;secShape=['L'];  secParam=[p1(1) p1(2) p2(1) p2(2) norm(p1-p2)]; agvSim.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
else
    nID1=6; nID2=11; p1=[3;3]; p2=[3.857;2.143]; rID12=6011; twoway=1; rID21=1106;secShape=['L'];  secParam=[p1(1) p1(2) p2(1) p2(2) norm(p1-p2)]; agvSim.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
    nID1=11; nID2=9; p1=[3.857;2.143]; p2=[5;1]; rID12=1109; twoway=1; rID21=9011;secShape=['L'];  secParam=[p1(1) p1(2) p2(1) p2(2) norm(p1-p2)]; agvSim.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
    % sekajoci povezavi, vpeljem dvojno vozlisce in nastavim bliznje ID-je
    nID1=8; nID2=12; p1=[3;1]; p2=[3.857;2.143]; rID12=8012; twoway=1; rID21=1208;secShape=['L'];  secParam=[p1(1) p1(2) p2(1) p2(2) norm(p1-p2)]; agvSim.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
    nID1=12; nID2=3; p1=[3.857;2.143]; p2=[6;5]; rID12=1203; twoway=1; rID21=3012;secShape=['L'];  secParam=[p1(1) p1(2) p2(1) p2(2) norm(p1-p2)]; agvSim.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
    agvSim.setNearbyNodes(11, [12]);
    agvSim.setNearbyNodes(12, [11])
end

nID1=10; nID2=4; p1=[7;1]; p2=[7;5]; rID12=1004; twoway=1; rID21=4010;secShape=['L'];  secParam=[p1(1) p1(2) p2(1) p2(2) norm(p1-p2)]; agvSim.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
nID1=5;  nID2=7; p1=[1;3]; p2=[1;1]; rID12=507; twoway=1; rID21=705;secShape=['L'];  secParam=[p1(1) p1(2) p2(1) p2(2) norm(p1-p2)]; agvSim.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
agvSim.precomputeMap(); agvSim.drawMap();

s=1.0; % speed


if Nagv==3  % delujoca komb.: pp=4
    Orders=[];
    Orders(1,:) = [1 0 s 1 9 L W];  % Orders= [AGVid  tStart speed Ns Ne L W; ]
    Orders(2,:) = [2 0 s 5 10 L W];
    Orders(3,:) = [3 0 s 7 4 L W];
    OrdPermBest=[2 3 1];
elseif Nagv==4 % delujoca komb.: pp=24
    Orders=[];
    Orders(1,:) = [1 0 s 1 9 L W];  % Orders= [AGVid  tStart speed Ns Ne L W; ]
    Orders(2,:) = [2 0 s 5 10 L W];
    Orders(3,:) = [3 0 s 7 4 L W];
    Orders(4,:) = [4 0 s 4 8 L W];
    OrdPermBest=[4 3 2 1];
elseif Nagv==5
    Orders=[];   % delujoca komb.: pp=87
    Orders(1,:) = [1 0 s 1 9 L W];  % Orders= [AGVid  tStart speed Ns Ne L W; ]
    Orders(2,:) = [2 0 s 5 10 L W];
    Orders(3,:) = [3 0 s 7 4 L W];
    Orders(4,:) = [4 0 s 4 8 L W];
    Orders(5,:) = [5 0 s 8 1 L W];
    OrdPermBest=[4 3 2 1 5];
end

if flag_method==2 % get PSIPP+ (best combination of priorities)
    OrdersP= Orders(OrdPermBest,:); % permutated order of priorities
    Orders=OrdersP;
end

Ts=0.01;
zamikIzris=0.05;

% Map 1 has no safe location (just PSIPP version or PSIPP+)
chargeStation=[];
goalFreeTime=inf;
restLocationGoalData=[];  % no safe location for map1
chargeRoad=[];chargeEntranceNode=[];chargeEntranceRoad=[];

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





