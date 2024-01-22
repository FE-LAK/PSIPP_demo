close all, clear all,  format bank
agvSim=[]; hPot=[];


%===== set map parameters
L=0.24; W=0.12; rr=sqrt((L)^2+(W)^2); % half length and width of AGV
safetyMargin=1.6;
mapCCBS_xml='Maps\simple_test3.xml';
fcnBench.CCBSconfig(rr,30,0.1); %set: robotSize,timeLimit,precision
flagSafelocation=1;

mapChoice=1;
InitMapAgvSim; % init map in agvSim

agvSim.DRAW =1;   % draw =1, no draw=0
flagSimulate=1;   % simulate=1, no simulation=0

%===== define start and goal nodes 
Nstart=[  1 27 30 14 19];
Nend=  [  11 5 16 31 3];


%===== call plannnig algorithm

%=====================
[PSIPPS,AllTimePlans,hPot]=fcnBench.PSIPPXplan(Nstart,Nend,agvSim,Ts,safetyMargin,L,W,hPot,flagSimulate,chargeStation,2,'PSIPPS');
PSIPPS
%=====================
skipReplaneTime=Ts*2;
[PSIPPL,AllTimePlans,hPot]=fcnBench.PSIPPLLplan(Nstart,Nend,agvSim,Ts,safetyMargin,L,W,hPot,skipReplaneTime,flagSimulate,chargeStation,'PSIPPL');
PSIPPL
%=====================
[CCBS,outputStructure,outputText,Times,AllPlanDNN]=fcnBench.CCBSplan(Nstart,Nend,mapCCBS_xml);
if agvSim.DRAW
    AllTimePlans=fcnBench.convertPlanCCBS2agvSim(agvSim,AllPlanDNN);
    if flagSimulate, [hPot]=fcnBench.simulateTimePlans(agvSim,AllTimePlans,hPot,L,W,Ts,1,'CCBS'); end
end
CCBS
%=====================


