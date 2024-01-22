close all
clear all,  format bank
agvSim=[]; hPot=[]; hText1=[];hText2=[];

mapChoice=1;  %map choices:  1= Mwrh1, 2=Mwrh2, 3= Mgame


flagStoreFile=0;

fileResultsPSIPPL=[];
flagSafelocation=1;

switch mapChoice
    
    case 1      % enostavna mapa skladisca  10agv 52 ponovitev
        L=0.24; W=0.12; rr=sqrt((L)^2+(W)^2); % polovicna dolzina in sirina  AGV-ja za namen detekcije trka
        mapCCBS_xml='Maps/Mwrh1.xml';
        fcnBench.CCBSconfig(rr,30,0.1); %nastavis: robotSize,timeLimit,precision
        fileScene =fileread('Scenarios/ScenarioLLC_Mwrh1.txt');   
        fileResultsPSIPPL  ='Results/Test/ResultLLC_Mwrh1_M4.mat';  
        
        safetyMargin=1.6 ;
        preventCollision=1;  %=0 ne preverja trkov, =1 preverja trke, =2 ignorira trke a jih belež
        skipReplaneTime=1;  %Avoid successive unsuccessful replannings
        
        %    PSIPPl =1824.15       2090.00
        
        %===================
    case 2      % Diganni 20 AGV 52 ponovitev iz slepih ulic
        L=0.24; W=0.12; rr=sqrt((L)^2+(W)^2); % polovicna dolzina in sirina  AGV-ja za namen detekcije trka
        mapCCBS_xml='Maps/Mwrh2_Digani.xml';
        fcnBench.CCBSconfig(rr,30,0.1); %nastavis: robotSize,timeLimit,precision
        
        fileScene =fileread('Scenarios/ScenarioLLC_Mwrh2.txt'); 
        fileResultsPSIPPL='Results/Test/ResultLLC_Mwrh2_M4.mat';
        
        safetyMargin=1.6 ;
        preventCollision=2;  %=0 ne preverja trkov, =1 preverja trke, =2 ignorira trke a jih belež
        skipReplaneTime=1;  %Avoid successive unsuccessful replannings
        
        % PSIPPl = 4136.89       2080.00
        
        %===================
    case 3 % den520d 20 agv 52 ponovitev iz slepih ulic
        mapCCBS_xml='Maps/Mgame_den520d.xml';
        L=0.24*3; W=0.12*3; rr=sqrt((L)^2+(W)^2); % polovicna dolzina in sirina  AGV-ja za namen detekcije trka
        fcnBench.CCBSconfig(rr,30,0.1); %nastavis: robotSize,timeLimit,precision
        
        fileScene =fileread('Scenarios/ScenarioLLC_Mgame.txt'); 
        fileResultsPSIPPL  ='Results/Test/ResultLLC_Mgame_M4.mat'; 
        
        safetyMargin=1.6*0 +5; % vzamem velik margin, ker so ceste pod malim kotom
        preventCollision=2;  %=0 ne prevrja trkov, =1 preverja trke, =2 ignorira trke a jih belež
        skipReplaneTime=1;
        
        %    figure(10),set(gcf,'position',[62.00        470.00        973.00        577.00]);
        %get(gcf,'position')
        %   PSIPPl =
        %      14105.74       2616.00
             
end

InitMapAgvSim; % nalozim mapo v agvSim



Tasks=[];

lines = strsplit(fileScene, '\n'); % Split the string into lines

values = str2double(strsplit(lines{1}, ';'));
% Remove any NaN values resulting from non-numeric entries
values = values(~isnan(values));
Nstart=values(3:length(values));


for i = 2:length(lines)-1 % Process each line
    % Split each line into individual numbers
    values = str2double(strsplit(lines{i}, ';'));
    % Remove any NaN values resulting from non-numeric entries
    values = values(~isnan(values));
    Nval= length(values)-2;
    
    taskID=values(1);
    Nagv=values(2);
    Ngoal=values(3:end);
    Tasks=[Tasks; Ngoal'];
end

agvSim.DRAW =0;   % ali izrisujem, =0

maxTask=size(Tasks,1);
%maxTask=20;

tic
[PSIPPL,taskSol]=fcnBench.PSIPPLLplanSingleLifeLong(Nstart,Tasks,maxTask,agvSim,L,W,Ts,safetyMargin,skipReplaneTime,preventCollision,chargeStation);
toc


% store to file fajl
if flagStoreFile
    save(fileResultsPSIPPL, 'PSIPPL', 'taskSol','Nagv');
    % load(fileResultsPSIPPL)
end





if 0  % simulate entire stored plane in taskSol
    
    % load(fileResultsPSIPPL)
    
    switch mapChoice
        case 1
            load('Results/Chained/ResultLLC_Mwrh1_M4.mat')
        case 2
            load('Results/Chained/ResultLLC_Mwrh2_M4.mat')
        case 3
            load('Results/Chained/ResultLLC_Mgame_M4.mat')
    end
    
    fcnSIPPContainer.addChargeEntranceRoad(agvSim,chargeStation(2),chargeStation(3));
    Ntasks=size(taskSol,2);
    Ntasks=100
    %Ntasks=Nagv
    
    agvSim.recordMovie=0;
    % agvSim.recordMovie=1;
    
    %hPot=[];hText1=[],hText2=[];
    preventCollision=2;
    showPaths=0;
    [hPot,hText1,hText2]=fcnBench.simulateLLPlansTaskSol(taskSol,Nagv,Ntasks,agvSim,L,W,Ts*2,preventCollision,'PSIPPL LL-ChainedSingleTasks',hPot,hText1,hText2,showPaths);
    
end




%=======================================================================
%=======================================================================
%==============generiram pregledni fajl        =========================
%=======================================================================
return
if 0
    
    
    %  Untitled_testDen520Map   % razne nastavitve
    %  Untitled_SimpleMap   % razne nastavitve
    %  Untitled_DiganiMap   % razne nastavitve
    
end





