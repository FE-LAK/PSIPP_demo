close all, clear all,  format bank
agvSim=[]; hPot=[];

mapChoice=0;   %map choices: 0=Msimp, 1(-1)= Mwrh1, 2=Mwrh2, 3= Mgame


flagStoreFile=0; % do we store results in file

fidResultASTAR=[];fidResultPSIPP=[]; fidResultPSIPPS=[];fidResultPSIPPL=[]; fidResultCCBS=[];
fileResultsASTAR=[];fileResultsPSIPP=[];fileResultsPSIPPS=[];fileResultsPSIPPL=[];fileResultsCCBS=[];
ASTAR=[-1 -1 -1];PSIPP=[-1 -1 -1];PSIPPS=[-1 -1 -1];PSIPPL=[-1 -1 -1];CCBS=[-1 -1 -1];PSIPPPlus=[-1 -1 -1];
flagSafelocation=1;
tmax=230; % max simulation time

switch mapChoice

   case 0 % Msimp - simple map (3 tasks) 
         L=0.35; W=0.35; rr=sqrt((L)^2+(W)^2); % half length and width of AGV 
         safetyMargin=1.358; 
         mapCCBS_xml='Maps\Msimp.xml';
         fcnBench.CCBSconfig(rr,30,0.1); %set: robotSize,timeLimit,precision
         fileScene =fileread('Scenarios\Scenario_Msimp.txt');  
 
   %======================================================================
   case -1 % Mwrh1 -  (2 tasks) 
         L=0.24; W=0.12; rr=sqrt((L)^2+(W)^2); % half length and width of AGV 
         safetyMargin=1.6; 
         mapCCBS_xml='Maps\Mwrh1.xml'; 
         fcnBench.CCBSconfig(rr,30,0.1); %set: robotSize,timeLimit,precision
         fileScene =fileread('Scenarios\Scenario_Mwrh1_2t.txt');  
 
   %======================================================================
   case 1  % Mwrh1 - simple warehouse map (10 AGVs, 33 tasks per AGV) 
         L=0.24; W=0.12; rr=sqrt((L)^2+(W)^2); % half length and width of AGV 
         safetyMargin=1.6;
         mapCCBS_xml='Maps\Mwrh1.xml';
         fcnBench.CCBSconfig(rr,30,0.1); %set: robotSize,timeLimit,precision
         fileScene =fileread('Scenarios\Scenario_Mwrh1_d.txt');  
        
         fileResultsASTAR ='Results\Test\test_Mwrh1_M0.txt';
         fileResultsPSIPP ='Results\Test\test_Mwrh1_M1.txt';
         fileResultsPSIPPS='Results\Test\test_Mwrh1_M2.txt';
         fileResultsPSIPPL='Results\Test\test_Mwrh1_M4.txt';
         fileResultsCCBS = 'Results\Test\test_Mwrh1_M5.txt';
   
    %======================================================================
    case 2 % Mwrh2 - warehouse map from Diganni article (40 AGVs, 33 tasks per AGV)
        L=0.24; W=0.12; rr=sqrt((L)^2+(W)^2); % half length and width of AGV 
        safetyMargin=1.6;
        mapCCBS_xml='Maps\Mwrh2_Digani.xml';
        fcnBench.CCBSconfig(rr,30,0.1); %set: robotSize,timeLimit,precision
        fileScene =fileread('Scenarios\Scenario_Mwrh2.txt'); 
        
         fileResultsASTAR ='Results\Test\test_Mwrh2_M0.txt';
         fileResultsPSIPP ='Results\Test\test_Mwrh2_M1.txt';
         fileResultsPSIPPS='Results\Test\test_Mwrh2_M2.txt';
         fileResultsPSIPPL='Results\Test\test_Mwrh2_M4.txt';
         fileResultsCCBS = 'Results\Test\test_Mwrh2_M5.txt';
        
    %======================================================================
    case 3 % Mgame - den520d map (35 AGVs, 43 tasks per AGV)
        mapCCBS_xml='Maps\Mgame_den520d.xml';  
        L=0.24*3; W=0.12*3; rr=sqrt((L)^2+(W)^2); % half length and width of AGV 
        safetyMargin=1.6;
        fcnBench.CCBSconfig(rr,30,0.1); %set: robotSize,timeLimit,precision
        fileScene =fileread('Scenarios\Scenario_Mgame.txt'); 
        
        fileResultsASTAR ='Results\Test\test_Mgame_M0.txt';
        fileResultsPSIPP ='Results\Test\test_Mgame_M1.txt';
        fileResultsPSIPPS='Results\Test\test_Mgame_M2.txt';
        fileResultsPSIPPL='Results\Test\test_Mgame_M4.txt';
        fileResultsCCBS = 'Results\Test\test_Mgame_M5.txt';
end

InitMapAgvSim; % init map in agvSim



agvSim.DRAW =1;   % draw =1, no draw=0
flagSimulate=1;   % simulate=1, no simulation=0

if flagStoreFile
    if ~isempty(fileResultsASTAR),  fidResultASTAR=fopen(fileResultsASTAR,'w'); end
    if ~isempty(fileResultsPSIPP),  fidResultPSIPP=fopen(fileResultsPSIPP,'w'); end
    if ~isempty(fileResultsPSIPPS), fidResultPSIPPS=fopen(fileResultsPSIPPS,'w'); end
    if ~isempty(fileResultsPSIPPL), fidResultPSIPPL=fopen(fileResultsPSIPPL,'w'); end
    if ~isempty(fileResultsCCBS),   fidResultCCBS=fopen(fileResultsCCBS,'w'); end
end



lines = strsplit(fileScene, '\n'); % Split the string into lines
Ntask =length(lines)-1;



for i = 1:Ntask % Process each line  
%for i = ceil(Ntask/2):ceil(Ntask/2) % Process one line   

    values = str2double(strsplit(lines{i}, ';')); % Split each line into individual numbers
    values = values(~isnan(values)); % Remove any NaN values resulting from non-numeric entries
    Nval= length(values)-2;
    taskID=values(1);
    Nagv=values(2);
    Nstart=values(3:(2+Nval/2));
    Nend=values((3+Nval/2):end);

    
    if 0
        Nstart=[  1 27 30]
        Nend=  [  11 5 16]
        
        % mapChoice=-1, i=2, Nagv=10, optimal priorities for PSIPP (PSIPPPlus) 
        % running fcnBench.PSIPPPlusPlan takes too long therefore just set
        % start and goal
        Nstart=[9    32    23    24    18    29    28    26    25    27];
        Nend= [ 15     2     5    19     1     6     3    13     4    16];
    end
  
    %=====================
    [ASTAR,AllTimePlans,hPot]=fcnBench.PSIPPXplan(Nstart,Nend,agvSim,Ts,tmax,safetyMargin,L,W,hPot,flagSimulate,chargeStation,0,'ASTAR');
    %=====================
    [PSIPP,AllTimePlans,hPot]=fcnBench.PSIPPXplan(Nstart,Nend,agvSim,Ts,tmax,safetyMargin,L,W,hPot,flagSimulate,chargeStation,1,'PSIPP');
    %=====================
    if mapChoice==0
       [PSIPPPlus,AllTimePlans,hPot,NstartP,NendP]=fcnBench.PSIPPPlusPlan(Nstart,Nend,agvSim,Ts,tmax,safetyMargin,L,W,hPot,flagSimulate,'PSIPPPlus');
    else
    %=====================
      [PSIPPS,AllTimePlans,hPot]=fcnBench.PSIPPXplan(Nstart,Nend,agvSim,Ts,tmax,safetyMargin,L,W,hPot,flagSimulate,chargeStation,2,'PSIPPS');
    %=====================
      skipReplaneTime=Ts*2;
      [PSIPPL,AllTimePlans,hPot]=fcnBench.PSIPPLLplan(Nstart,Nend,agvSim,Ts,tmax,safetyMargin,L,W,hPot,skipReplaneTime,flagSimulate,chargeStation,'PSIPPL');
    end
    %=====================
    [CCBS,outputStructure,outputText,Times,AllPlanDNN]=fcnBench.CCBSplan(Nstart,Nend,mapCCBS_xml); 
    if agvSim.DRAW
        AllTimePlans=fcnBench.convertPlanCCBS2agvSim(agvSim,AllPlanDNN);
        if flagSimulate, [hPot]=fcnBench.simulateTimePlans(agvSim,AllTimePlans,hPot,L,W,Ts,1,'CCBS'); end
    end
    %=====================
    
    if mapChoice==0, RESULTS=[ASTAR;PSIPP;PSIPPPlus;CCBS];
    else, RESULTS=[ASTAR;PSIPP;PSIPPS;PSIPPL;CCBS]; end
    fprintf('id=%d\n',i);
    disp(RESULTS)
    
 
    if 0  % simulate obtained plan
         agvSim.recordMovie=0;
         %agvSim.recordMovie=1;
         agvSim.DRAW=1;
         sampleTime=Ts; 
         preventCollision=1;
         if ~isempty(chargeStation),fcnSIPPContainer.addChargeEntranceRoad(agvSim,chargeStation(2),chargeStation(3)); end
        [hPot]=fcnBench.simulateTimePlans(agvSim,AllTimePlans,hPot,L,W,sampleTime,preventCollision,'Algorithm');
    end
    
    % store in file
    if flagStoreFile
        if ~isempty(fidResultASTAR), fprintf(fidResultASTAR, '%d;%d;%12.4f;%12.4f;%10.1f;\n',taskID,Nagv,ASTAR(1),ASTAR(2),ASTAR(3)); end
        if ~isempty(fidResultPSIPP), fprintf(fidResultPSIPP, '%d;%d;%12.4f;%12.4f;%10.1f;\n',taskID,Nagv,PSIPP(1),PSIPP(2),PSIPP(3)); end
        if ~isempty(fidResultPSIPPS), fprintf(fidResultPSIPPS, '%d;%d;%12.4f;%12.4f;%10.1f;\n',taskID,Nagv,PSIPPS(1),PSIPPS(2),PSIPPS(3)); end
        if ~isempty(fidResultPSIPPL), fprintf(fidResultPSIPPL, '%d;%d;%12.4f;%12.4f;%10.1f;\n',taskID,Nagv,PSIPPL(1),PSIPPL(2),PSIPPL(3)); end
        if ~isempty(fidResultCCBS),   fprintf(fidResultCCBS,   '%d;%d;%12.4f;%12.4f;%10.1f;\n',taskID,Nagv,CCBS(1),CCBS(2),CCBS(3)); end
    end
    
end


if flagStoreFile
    if ~isempty(fidResultASTAR), fclose(fidResultASTAR); end
    if ~isempty(fidResultPSIPP), fclose(fidResultPSIPP); end
    if ~isempty(fidResultPSIPPS), fclose(fidResultPSIPPS); end
    if ~isempty(fidResultPSIPPL), fclose(fidResultPSIPPL); end
    if ~isempty(fidResultCCBS), fclose(fidResultCCBS); end
end


%=======================================================================
%=======================================================================
%==============generiram pregledni fajl        =========================
%=======================================================================
return
if 0
    
    script_PreglednaAnaliza
end







