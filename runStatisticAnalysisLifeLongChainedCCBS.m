close all, clear all,  format bank
agvSim=[]; hPot=[];hText1=[]; hText2=[];  taskSolCCBS=[]; idxTasks=1; time=0;

mapChoice=1;  %map choices:  1= Mwrh1, 2=Mwrh2, 3= Mgame


flagStoreFile=0;

fidResultCCBS=[];
fileResultsCCBS=[];
flagSafelocation=0;

switch mapChoice
    case 1      % enostavna mapa skladisca do 10agv 30 ponovitev
        L=0.24; W=0.12; rr=sqrt((L)^2+(W)^2); % half length and width of AGV
        mapCCBS_xml='Maps/Mwrh1.xml';
        fcnBench.CCBSconfig(rr,30,0.1); %set: robotSize,timeLimit,precision
        
        fileScene =fileread('Scenarios/ScenarioLLC_Mwrh1.txt');
        fileResultsCCBS  ='Results/Test/ResultLLC_Mwrh1_M5.txt';
        
        % Lifelong Single CCBS:
        % tMksAll =1703.79
        % NitAll =4154110.00
        
        %===========================
    case 2      % Diganni 20 AGV 52 ponovitev iz slepih ulic
        L=0.24; W=0.12; rr=sqrt((L)^2+(W)^2); % half length and width of AGV
        mapCCBS_xml='Maps/Mwrh2_Digani.xml';
        fcnBench.CCBSconfig(rr,30,0.1); %set: robotSize,timeLimit,precision
        
        fileScene =fileread('Scenarios/ScenarioLLC_Mwrh2.txt');
        fileResultsCCBS  ='Results/Test/ResultLLC_Mwrh2_M5.txt';
        
        % tMksAll =7003.88
        % NitAll = 597270.00
        
        %===========================
    case 3 % den520d 20 agv 52 ponovitev iz slepih ulic
        mapCCBS_xml='Maps/Mgame_den520d.xml';
        L=0.24*3; W=0.12*3; rr=sqrt((L)^2+(W)^2); % half length and width of AGV
        fcnBench.CCBSconfig(rr,30,0.1); %set: robotSize,timeLimit,precision
        
        fileScene =fileread('Scenarios/ScenarioLLC_Mgame.txt');
        fileResultsCCBS  ='Results/Test/ResultLLC_Mgame_M5.txt';
        
        % successRate = 60.47
        % tMksAll = 21223.68
        % NitAll = 4048364.00
end
%=============================================================================
InitMapAgvSim; % init map  agvSim



agvSim.DRAW =0;   %ali izrisujem, =0
flagSimulate=0;   % =0  % ali simuliram
hPot=[];

if flagStoreFile
    if ~isempty(fileResultsCCBS),   fidResultCCBS=fopen(fileResultsCCBS,'w'); end
end



%================ CCBS ==============================================
tMksAll=0;
NitAll=0;

lines = strsplit(fileScene, '\n'); % Split the string into lines

values = str2double(strsplit(lines{1}, ';'));
% Remove any NaN values resulting from non-numeric entries
values = values(~isnan(values));
taskID=values(1);
Nagv=values(2);
Nstart=values(3:length(values));

for i = 2:length(lines)-1 % Process each line
    values = str2double(strsplit(lines{i}, ';')); % Split each line into individual numbers
    values = values(~isnan(values)); % Remove any NaN values resulting from non-numeric entries
    Nval= length(values)-2;
    
    taskID=values(1);
    Nagv=values(2);
    Ngoal=values(3:(end));
    
    %=====================
    [CCBS1,outputStructure,outputText,times1,plansDNN]=fcnBench.CCBSplan(Nstart,Ngoal,mapCCBS_xml);
    %=====================
    
    if 1 % store all plans in taskSolCCBS
        AllTimePlansCCBS=fcnBench.convertPlanCCBS2agvSim(agvSim,plansDNN);
        timeMax=time;
        for a=1:Nagv
            % store plan for each task
            timePlan=AllTimePlansCCBS{a};
            timePlan(:,1)=timePlan(:,1)+time;
            
            taskSolCCBS(idxTasks).agvID=a;
            taskSolCCBS(idxTasks).pickDrop=Ngoal(a);
            taskSolCCBS(idxTasks).timePlan=timePlan;
            taskSolCCBS(idxTasks).startRID=timePlan(1,2);
            taskSolCCBS(idxTasks).startLOC=0;
            taskSolCCBS(idxTasks).startT=time;
            taskSolCCBS(idxTasks).tDrop=timePlan(end,1);
            idxTasks=idxTasks+1;
            timeMax=max(timeMax,timePlan(end,1));
        end
        time=timeMax;
        
        if 0
            agvSim.DRAW =1;
            Ts=0.1;
            preventCollision=2;
            [hPot]=fcnBanch.simulateTimePlans(agvSim,AllTimePlansCCBS,hPot,L,W,Ts,preventCollision);
            pause
        end
    end
    
    
    times=tMksAll+times1;
    tMksAll=tMksAll+CCBS1(2);
    NitAll=NitAll+CCBS1(3);
    disp([i-1 , CCBS1(2:3),tMksAll,NitAll])
    
    if flagStoreFile     % store CCBS results to file
        if ~isempty(fidResultCCBS)
            string1='%d;%d;%12.4f;%12.4f;%10.1f;%12.4f;%10.1f;';
            string2= strcat(string1,repmat(' %f;',1,length(Ngoal)),'\n');
            
            fprintf(fidResultCCBS,   string2,...
                taskID,Nagv,CCBS1(1),CCBS1(2),CCBS1(3),tMksAll,NitAll,times');
        end
    end
    
    Nstart=Ngoal;
    
end

disp('Lifelong Single CCBS:')
tMksAll
NitAll

CCBS=[tMksAll, NitAll];


if flagStoreFile
    if ~isempty(fidResultCCBS), fclose(fidResultCCBS); end
    save(fileResultsCCBS(1:end-4), 'CCBS', 'taskSolCCBS','Nagv');
    % load(fileResultsCCBS(1:end-4))
end




if 0  % simulate entire plan in taskSol
    
    %load(fileResultsCCBS(1:end-4))
    
    switch mapChoice
        case 1
            load('Results/Chained/ResultLLC_Mwrh1_M5.mat')   
        case 2
            load('Results/Chained/ResultLLC_Mwrh2_M5.mat')
        case 3
            load('Results/Chained/ResultLLC_Mgame_M5.mat')
    end
    
    
    Ntasks=size(taskSolCCBS,2);
    Ntasks=100
   % Ntasks=Nagv

    agvSim.recordMovie=0;
    % agvSim.recordMovie=1;
    preventCollision=2;
    %hPot=[];hText1=[],hText2=[];
    showPaths=0;
    [hPot,hText1,hText2]=fcnBench.simulateLLPlansTaskSol(taskSolCCBS,Nagv,Ntasks,agvSim,L,W,2*Ts,preventCollision,'CCBS LL-ChainedSingleTasks',hPot,hText1,hText2,showPaths);

end



%=======================================================================
%=======================================================================
%==============generiram pregledni fajl        =========================
%=======================================================================
return
if 0
    
    script_PreglednaAnaliza
    %  Untitled_testDen520Map   % razne nastavitve
end





