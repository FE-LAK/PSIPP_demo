close all, clear all,  format bank
agvSim=[]; hPot=[]; hText1=[];hText2=[];   taskSolCCBS=[]; idxTasks=1;  time=0;

mapChoice=3;  %map choices:  1= Mwrh1, 2=Mwrh2, 3= Mgame


flagStoreFile=0;


fidResultCCBS=[]; fileResultsCCBS=[];
%ASTAR=[-1 -1 -1];PSIPP=[-1 -1 -1];PSIPPC=[-1 -1 -1];PSIPPL=[-1 -1 -1];CCBS=[-1 -1 -1];
flagSafelocation=0;

switch mapChoice
    
    case 1      % enostavna mapa skladisca do 10agv 52 ponovitev (iz json mape simple_test3.xml)
        L=0.24; W=0.12; rr=sqrt((L)^2+(W)^2); % half length and width of AGV
        mapCCBS_xml='Maps/Mwrh1.xml';
        fcnBench.CCBSconfig(rr,30,0.1); %set: robotSize,timeLimit,precision
        
        fileScene =fileread('Scenarios/ScenarioLLPD_Mwrh1.txt');
        fileResultsCCBS  ='Results/Test/ResultLLPD_Mwrh1_M5.txt';
        
        % Lifelong Pick-Drop CCBS:
        % tMksAll = 3387.48
        % NitAll = 17436166.00
        
        %===============================
    case 2   % Diganni 20AGV 10 ponovitev iz slepih ulic
        L=0.24; W=0.12; rr=sqrt((L)^2+(W)^2); % % half length and width of AGV
        mapCCBS_xml='Maps/Mwrh2_Digani.xml';
        fcnBench.CCBSconfig(rr,30,0.1); %set: robotSize,timeLimit,precision
        
        fileScene =fileread('Scenarios/ScenarioLLPD_Mwrh2.txt');
        fileResultsCCBS  ='Results/Test/ResultLLPD_Mwrh2_M5.txt';
        
        % Lifelong CCBS:
        % tMksAll =13953.94
        % NitAll =1039166.00
        
        %===============================
    case 3 % den520d 20 agv 52 ponovitev iz slepih ulic
        L=0.24*3; W=0.12*3; rr=sqrt((L)^2+(W)^2); % % half length and width of AGV
        mapCCBS_xml='Maps/Mgame_den520d.xml';
        fcnBench.CCBSconfig(rr,30,0.1); %set: robotSize,timeLimit,precision
        
        fileScene =fileread('Scenarios/ScenarioLLPD_Mgame.txt');
        fileResultsCCBS  ='Results/Test/ResultLLPD_Mgame_M5.txt';
         %===============================
      
end

InitMapAgvSim; % init map  agvSim




agvSim.DRAW =0;   %ali izrisujem, =0
flagSimulate=0;   % =0  % ali simuliram


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
    
    % Split each line into individual numbers
    values = str2double(strsplit(lines{i}, ';'));
    % Remove any NaN values resulting from non-numeric entries
    values = values(~isnan(values));
    Nval= length(values)-2;
    
    taskID=values(1);
    Nagv=values(2);
    Npick=values(3:(2+Nval/2));
    Ndrop=values((3+Nval/2):end);
    
    %=====================
    [CCBS1,outputStructure,outputText,times1,plansDNN1]=fcnBench.CCBSplan(Nstart,Npick,mapCCBS_xml); %disp(outputStructure);
    %=====================
    tMksAll=tMksAll+CCBS1(2);
    NitAll=NitAll+CCBS1(3);
    disp([i-1 , CCBS1(2:3),tMksAll,NitAll])
    
    %=====================
    [CCBS2,outputStructure,outputText,times2,plansDNN2]=fcnBench.CCBSplan(Npick,Ndrop,mapCCBS_xml); %disp(outputStructure);
    %=====================
    times=tMksAll+times2;        
    tMksAll=tMksAll+CCBS2(2);
    NitAll=NitAll+CCBS2(3);
    disp([i-1 , CCBS2(2:3),tMksAll,NitAll])
    
    
    
    if 1 % convert and put plans togeter in taskSol
        AllTimePlansCCBS1=fcnBench.convertPlanCCBS2agvSim(agvSim,plansDNN1);
        timeMax=time;
        for a=1:Nagv
            % hranim plane za vsak task
            timePlan=AllTimePlansCCBS1{a};
            timePlan(:,1)=timePlan(:,1)+time;
            
            taskSolCCBS(idxTasks).agvID=a; %agvID;
            taskSolCCBS(idxTasks).pickDrop=Npick(a);  %taskSolCCBS(idxTasks).pickDrop=Tasks(idxTasks,1:2);
            taskSolCCBS(idxTasks).timePlan=timePlan;
            taskSolCCBS(idxTasks).startRID=timePlan(1,2); %   agvSim.AGV(frejAgv).onRoadID;
            taskSolCCBS(idxTasks).startLOC=0; %agvSim.AGV(frejAgv).onRoadLoc;
            taskSolCCBS(idxTasks).startT=time;
            taskSolCCBS(idxTasks).tDrop=timePlan(end,1); %casDrop;
            idxTasks=idxTasks+1;
            timeMax=max(timeMax,timePlan(end,1));
        end
        time=timeMax;
        
        AllTimePlansCCBS2=fcnBench.convertPlanCCBS2agvSim(agvSim,plansDNN2);
        timeMax=time;
        for a=1:Nagv
            % hranim plane za vsak task
            timePlan=AllTimePlansCCBS2{a};
            timePlan(:,1)=timePlan(:,1)+time;
            
            taskSolCCBS(idxTasks).agvID=a; %agvID;
            taskSolCCBS(idxTasks).pickDrop=Ndrop(a);  %taskSolCCBS(idxTasks).pickDrop=Tasks(idxTasks,1:2);
            taskSolCCBS(idxTasks).timePlan=timePlan;
            taskSolCCBS(idxTasks).startRID=timePlan(1,2); %   agvSim.AGV(frejAgv).onRoadID;
            taskSolCCBS(idxTasks).startLOC=0; %agvSim.AGV(frejAgv).onRoadLoc;
            taskSolCCBS(idxTasks).startT=time;
            taskSolCCBS(idxTasks).tDrop=timePlan(end,1); %casDrop;
            idxTasks=idxTasks+1;
            timeMax=max(timeMax,timePlan(end,1));
        end
        time=timeMax;
        
        
    end
    
    
    
    
    % store CCBS in file
    if flagStoreFile
        if ~isempty(fidResultCCBS)
            string1='%d;%d;%12.4f;%12.4f;%10.1f;%12.4f;%12.4f;%10.1f;%12.4f;%10.1f;';
            string2= strcat(string1,repmat(' %f;',1,length(Ndrop)),'\n');
            fprintf(fidResultCCBS,   string2,...
                taskID,Nagv,CCBS1(1),CCBS1(2),CCBS1(3),CCBS2(1),CCBS2(2),CCBS2(3),tMksAll,NitAll,times');
        end
    end
    
  
    Nstart=Ndrop;
end

disp('Lifelong CCBS:')
tMksAll
NitAll

CCBS=[tMksAll, NitAll];



if flagStoreFile
    if ~isempty(fidResultCCBS), fclose(fidResultCCBS); end
    
    save(fileResultsCCBS(1:end-4), 'CCBS', 'taskSolCCBS','Nagv');
    % load(fileResultsCCBS(1:end-4))
end






if 0  % simulate  entire plan in taskSol
    
    %load(fileResultsCCBS(1:end-4))
    
    switch mapChoice
        case 1
            load('Results/PickDrop/ResultLLPD_Mwrh1_M5.mat')
        case 2
            load('Results/PickDrop/ResultLLPD_Mwrh2_M5.mat')  
        case 3
            load('Results/PickDrop/ResultLLPD_Mgame_M5.mat') 
    end
    
    
    Ntasks=size(taskSolCCBS,2);
    Ntasks=200   % to je 100 pick-drop taskov...
    % !!! taski so dvojni vsak pick in drop je svoj task
    agvSim.recordMovie=0;
    % agvSim.recordMovie=1;
    preventCollision=2;
    %hPot=[];hText1=[],hText2=[];
    showPaths=0;
    [hPot,hText1,hText2]=fcnBench.simulateLLPlansTaskSol(taskSolCCBS,Nagv,Ntasks,agvSim,L,W,Ts*2,preventCollision,'CCBS LL-PickDrop',hPot,hText1,hText2,showPaths);
    
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





