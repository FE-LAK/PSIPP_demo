% Simple example, build map and make single MAPF

close all
clear all
clc
format bank

% ------------------- DEMO options ----------------------
flag_SelectStartGoal=2; % set 0, 1, or 2 for different start/goal
% -------------------------------------------------------

agvSim = AGVSim(10); % Inicializacija simulatorja na zeleno okno
agvSim.DRAW=1;
agvSim.fontSize=70;
agvSim.fontSizeAGV=12;

%%%%% load simple map
simpleDemoMap;

% AGV configuration
L=0.24; W=0.12;
s=1;  % speed
safetyMargin=1.6; %
agvSim.setRobotHalfLengthWidth(L,W); % polovicna dolzina in sirina  AGV-ja za namen detekcije trka


% dolocim pot za vozili

%%%%%% Simulation parameters
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
zamikIzris=0.05;% Road plan graphics shifting
Ts=0.1;         % Sample time


% chose start/goal options
if flag_SelectStartGoal==0
    Orders=[ ...
        1 0 s 1 6 L*1 W;     % Orders= [AGVid  tStart speed Ns Ne L W; ]
        2 0 s 3 2 L*1 W;...
        ];
elseif flag_SelectStartGoal==1
    Orders=[ ...
        1 0 s 2 6 L*1 W;     % Orders= [AGVid  tStart speed Ns Ne L W; ]
        2 0 s 3 2 L*1 W;...
        ];
elseif flag_SelectStartGoal==2
    Orders=[ ...
        1 0 s 2 6 L*1 W;     % Orders= [AGVid  tStart speed Ns Ne L W; ]
        2 0 s 4 2 L*1 W;...
        ];
end
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

    disp(strcat('AGV',num2str(i)));
    priority=i;


    restLocationGoalData=[];  % without safe location
    %    restLocationGoalData=[agvSim.roadToRestNode 0.8, 0,0,0]; % Safe location on the road
    %restLocationGoalData=[agvSim.restNode, 0,0,0]; % Safe location in node
    %restLocationGoalData=[agvSim.roadToRestNode 1, 0,0,0];

    twait=0;
    goalFreeTime=inf;
    multiGoalData= {[Ne, twait,goalFreeTime,0] };
    chargeStation=[];

    [timePlan, times] = findPathSIPP_multiGoal(agvSim, Ns, multiGoalData, ...
        speed, startTime, safetyRadious, priority, replanTime, agvID, chargeStation);


    if(~isempty(timePlan))

        goalWait=goalFreeTime;
        fcnSIPPContainer.setOccupancySIPP2(agvSim,timePlan,agvID,speed,safetyRadious,priority,replanTime,goalWait); % nastavi OI-je

        agvSim.createAGVonRoadLW(agvID , timePlan(1,2), 0,len,wid,priority);
        agvSim.setPlanRoadTimeSinc (agvID, timePlan);
    else
        timePlan=[];
        disp('For this AGV no path is found.');
        notFoundPaths=notFoundPaths+1;
        agvSim.createAGVonPoseLW(agvID , [2+i*len;-1;0],len,wid, priority);
    end
    AllTimePlans{i}=timePlan;
end


if 1  % Criteria = sum cost
    sumCost=0;
    for i=1:length(AllTimePlans)
        if (~isempty(AllTimePlans{i}))
            sumCost=sumCost+(AllTimePlans{i}(end,1)-Orders(i,2)); % upoštevam se zacetni cas (Orders(i,2)), ce je >0
        end
    end
    sumCost
end





%%%%%% Simulation
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
agvSim.recordMovie=0; % Movie recording flag
agvSim.preventCollision=1;   % 0=ignore collisions, 1=block on collision, 2=ignore, but remember
figure(10),

tmax=20;        % Simulation time
vmax=3;         % Maximum AGV speed
tSimDelay=Ts*.25*.25*4; % Simulation display delay

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


