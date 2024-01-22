close all, clear all,  format bank
agvSim=[]; hPot=[];  hPot=[];hText1=[];hText2=[];

mapChoice=3;    %map choices:  1= Mwrh1, 2=Mwrh2, 3= Mgame


flagStoreFile=0;




flagSafelocation=1;

switch mapChoice
    case 1      % enostavna mapa skladisca  10agv 52 ponovitev
        L=0.24; W=0.12; rr=sqrt((L)^2+(W)^2); % polovicna dolzina in sirina  AGV-ja za namen detekcije trka
        fcnBench.CCBSconfig(rr,30,0.1); %nastavis: robotSize,timeLimit,precision
        mapCCBS_xml='Maps/simple_test3.xml';
        fileScene =fileread('Scenarios/clanekAnaliza_Msimple_LLScenarij.txt');
        fileResultsPSIPPL  ='Results/Test/clanekAnaliza_Msimple_LLM4_n.mat';  %
        
        safetyMargin=1.6 ;
        preventCollision=1;  %=0 ne preverja trkov, =1 preverja trke, =2 ignorira trke a jih belež
        skipReplaneTime=1;  %Avoid successive unsuccessful replannings
        
        %PSIPPl =2868.83       2922.00
        
        %==============
    case 2      % Diganni 20 AGV 52 ponovitev iz slepih ulic
        L=0.24; W=0.12; rr=sqrt((L)^2+(W)^2); % polovicna dolzina in sirina  AGV-ja za namen detekcije trka
        fcnBench.CCBSconfig(rr,30,0.1); %nastavis: robotSize,timeLimit,precision
        mapCCBS_xml='Maps/scenario_map_digani_01_01_out_fixedFinal2.xml';        
        fileScene =fileread('Scenarios/clanekAnaliza_Mdigani20_LLScenarij.txt');
        fileResultsPSIPPL='Results/Test/clanekAnaliza_Mdigani_LLM4.mat';
       
        safetyMargin=1.6 ; % bi bilo bolje dat malo vec zaradi malih ovinkov
        preventCollision=2;  %=0 ne preverja trkov, =1 preverja trke, =2 ignorira trke a jih belež
        skipReplaneTime=0.2;  %Avoid successive unsuccessful replannings
        
        %PSIPPl =7151.14       3120.00
        
        %==============
    case 3 % den520d 20 agv 52 ponovitev iz slepih ulic
        L=0.24*3; W=0.12*3; rr=sqrt((L)^2+(W)^2); % polovicna dolzina in sirina  AGV-ja za namen detekcije trka
        fcnBench.CCBSconfig(rr,30,0.1); %nastavis: robotSize,timeLimit,precision
        mapCCBS_xml='Maps/_Primerjave/den520d_test6g.xml';
        fileScene =fileread('Scenarios/clanekAnaliza_Mden520_LLScenarij.txt');
        fileResultsPSIPPL  ='Results/Test/clanekAnaliza_Mden520A_LLM4_nearNode.mat';  % ni ok je brez NearByNodes
                
        safetyMargin=1.6*0 +5; % vzamem velik margin, ker so ceste pod malim kotom
        preventCollision=2;  %=0 ne prevrja trkov, =1 preverja trke, =2 ignorira trke a jih belež
        skipReplaneTime=0.2;
                
        %     PSIPPl =24809.58       3261.00
end

InitMapAgvSim; % nalozim mapo v agvSim



agvSim.DRAW =0;   % ali izrisujem, =0
flagSimulate=0;   % =0  % ali simuliram




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
    Ngoal1=values(3:(2+Nval/2));
    Ngoal2=values((3+Nval/2):end);
    
    Tasks=[Tasks; [Ngoal1',Ngoal2']];
end


agvSim.DRAW =0;   % ali izrisujem, =0


maxTask=size(Tasks,1);
[PSIPPL,taskSol]=fcnBench.PSIPPLLplanPickDropLifeLong(Nstart,Tasks,maxTask,agvSim,L,W,Ts,safetyMargin,skipReplaneTime,preventCollision,chargeStation);


% store PEIPPL in file
if flagStoreFile
    save(fileResultsPSIPPL, 'PSIPPL', 'taskSol','Nagv');
    % load(fileResultsPSIPPL)
end



if 0  % simulacija celega niza planov v taskSol
    
    % load(fileResultsPSIPPL)
    
     switch mapChoice
        case 1
            load('Results/PickDrop/clanekAnaliza_Msimple_LLM4_n.mat')
        case 2
            load('Results/PickDrop/clanekAnaliza_Mdigani_LLM4.mat')
        case 3
            load('Results/PickDrop/clanekAnaliza_Mden520A_LLM4_nearNode.mat')
     end
   
    
    fcnSIPPContainer.addChargeEntranceRoad(agvSim,chargeStation(2),chargeStation(3));
    Ntasks=size(taskSol,2);
    Ntasks=100
    
    agvSim.recordMovie=0;
    % agvSim.recordMovie=1;
    preventCollision=2;
    %hPot=[];hText1=[],hText2=[];
    showPaths=0;
    [hPot,hText1,hText2]=fcnBench.simulateLLPlansTaskSol(taskSol,Nagv,Ntasks,agvSim,L,W,Ts*2,preventCollision,'PSIPP LL-PickDrop',hPot,hText1,hText2,showPaths);
end







%=======================================================================
%=======================================================================
%==============generiram scenarij fajl        =========================
%=======================================================================
return
if 0
    
    %  Untitled_testDen520Map   % razne nastavitve
    %  Untitled_SimpleMap   % razne nastavitve
end





