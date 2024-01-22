
if(isempty(agvSim))  % load if agvSim is empty
    
    switch mapChoice
        case {0}   % small map
            agvSim = AGVSim(10); % Inicializacija simulatorja na zeleno okno
            agvSim.DRAW =1; %ali izrisujem, =0
            flagSimulate=0;   % =0  % ali simuliram
            
            agvSim.fontSize = 8;
            
            agvSim.drawRoadLabels = 0;
            agvSim.drawNodeLabels = 1;
            agvSim.fontSize=60;
            agvSim.fontSizeAGV=9;
            
            graphSimIo = GraphSimIo(agvSim);
            %==================================================
            graphSimIo.forceTwoway=false;
            
            graphSimIo.loadScenario('Maps\Msimp.json');
            chargeStation=[];
            goalFreeTime=inf;
            restLocationGoalData=[];  % brez varnega mesta
            chargeRoad=[];chargeEntranceNode=[];chargeEntranceRoad=[];

            agvSim.setRobotHalfLengthWidth(L,W); % polovicna dolzina in sirina  AGV-ja za namen detekcije trka
            Ts=0.05; % Cas vzorcenja
            zamikIzris=0.05;
            
            set(gca,'Xlim',[get(gca,'XLim')]+[-1 1]);set(gca,'Ylim',[get(gca,'YLim')]+[-1 1]);
            figure(10),set(gcf,'position',[681.00        265.00       1028.00        834.00]);
            %get(gcf,'position')
            
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        
        case {1,-1}   %Mwrh1  enostavna mapa za clanek
            agvSim = AGVSim(10); % Inicializacija simulatorja na zeleno okno
            agvSim.DRAW =1; %ali izrisujem, =0
            flagSimulate=0;   % =0  % ali simuliram
            
            agvSim.fontSize = 8;
            
            agvSim.drawRoadLabels = 0;
            agvSim.drawNodeLabels = 1;
            agvSim.fontSize=180;
            agvSim.fontSizeAGV=9;
            
            graphSimIo = GraphSimIo(agvSim);
            %==================================================
            graphSimIo.forceTwoway=false;
            
            if flagSafelocation % do we have safe location
                graphSimIo.loadScenario('Maps\Mwrh1_SL.json');
                % chargeStation=[chargeRoad,chargeEntranceNode,chargeEntranceRoad];
                chargeStation=[39,4,37]; % vstop v polnilnico in polnilnica
                chargeRoad=chargeStation(1);
                goalFreeTime=0;
                agvSim.roadToRestNode=chargeRoad;
                agvSim.restNode=agvSim.roadID(chargeRoad).eNode;
                restLocationGoalData=[agvSim.roadToRestNode 0.8, 0,0,0]; % varno mesto je na cesti
                %restLocationGoalData=[agvSim.restNode, 0,0,0]; % podam vozliš?e
                %restLocationGoalData=[agvSim.roadToRestNode 1, 0,0,0]; % podam kot cesto in njen konec
            else
                graphSimIo.loadScenario('Maps\Mwrh1.json');
                chargeStation=[];
                goalFreeTime=inf;
                restLocationGoalData=[];  % brez varnega mesta
                chargeRoad=[];chargeEntranceNode=[];chargeEntranceRoad=[];
            end
            
            agvSim.setRobotHalfLengthWidth(L,W); % polovicna dolzina in sirina  AGV-ja za namen detekcije trka
            Ts=0.1; % Cas vzorcenja
            zamikIzris=0.05;
            
            set(gca,'Xlim',[get(gca,'XLim')]+[-1 1]);set(gca,'Ylim',[get(gca,'YLim')]+[-1 1]);
            figure(10),set(gcf,'position',[681.00        265.00       1028.00        834.00]);
            %get(gcf,'position')
            
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            
        case {2}   %Mwrh2 Digani mapa
            
            agvSim = AGVSim(10); % Inicializacija simulatorja na zeleno okno
            agvSim.DRAW =1; %ali izrisujem, =0
            flagSimulate=0;   % =0  % ali simuliram
            
            agvSim.fontSize = 8;
            
            agvSim.drawRoadLabels = 0;
            agvSim.drawNodeLabels = 1;
            agvSim.fontSize=200;
            agvSim.fontSizeAGV=12;
            
            zamikIzris = 0.1;
            graphSimIo = GraphSimIo(agvSim);
            
            %==================================================
            graphSimIo.forceTwoway=false;
            
            
            if flagSafelocation % ali imam varno lokacijo
                graphSimIo.loadScenario('Maps\Mwrh2_DiganiSL.json');
                
                % chargeStation=[chargeRoad,chargeEntranceNode,chargeEntranceRoad];
                chargeStation=[699,515,700]; % vstop v polnilnico in polnilnica
                chargeRoad=chargeStation(1);
                goalFreeTime=0;
                agvSim.roadToRestNode=chargeRoad;
                agvSim.restNode=agvSim.roadID(chargeRoad).eNode;
                restLocationGoalData=[agvSim.roadToRestNode 0.8, 0,0,0]; % varno mesto je na cesti
                %restLocationGoalData=[agvSim.restNode, 0,0,0]; % podam vozliš?e
                %restLocationGoalData=[agvSim.roadToRestNode 1, 0,0,0]; % podam kot cesto in njen konec
            else
                graphSimIo.loadScenario('Maps\Mwrh2_Digani.json');
                
                chargeStation=[];
                goalFreeTime=inf;
                restLocationGoalData=[];  % brez varnega mesta
                chargeRoad=[];chargeEntranceNode=[];chargeEntranceRoad=[];
            end
            agvSim.setRobotHalfLengthWidth(L,W); % polovicna dolzina in sirina  AGV-ja za namen detekcije trka
            
            set(gca,'Xlim',[get(gca,'XLim')]+[-1 1]);set(gca,'Ylim',[get(gca,'YLim')]+[-1 1]);
            figure(10),set(gcf,'position',[326.00         99.00       1382.00        911.00]);
            %get(gcf,'position')
            Ts=0.25; % Cas vzorcenja
            
            
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            
        case {3}   %Mgame den mapa
            agvSim = AGVSim(10); % Inicializacija simulatorja na zeleno okno
            agvSim.DRAW =1; %ali izrisujem, =0
            flagSimulate=1;   % =0  % ali simuliram
            
            agvSim.fontSize = 8;
            
            agvSim.drawRoadLabels = 0;
            agvSim.drawNodeLabels = 1;
            agvSim.fontSize=1500;
            agvSim.fontSizeAGV=12;
            
            zamikIzris = 0.1;
            graphSimIo = GraphSimIo(agvSim);
            
            %==================================================
            graphSimIo.forceTwoway=false;
                        
            if flagSafelocation % ali imam varno lokacijo
                graphSimIo.loadScenario('Maps\Mgame_den520dSL.json');
                % chargeStation=[chargeRoad,chargeEntranceNode,chargeEntranceRoad];
                chargeStation=[123,5,122]; % vstop v polnilnico in polnilnica
                chargeRoad=chargeStation(1);
                goalFreeTime=0;
                agvSim.roadToRestNode=chargeRoad;
                agvSim.restNode=agvSim.roadID(chargeRoad).eNode;
                restLocationGoalData=[agvSim.roadToRestNode 0.8, 0,0,0]; % varno mesto je na cesti
                %restLocationGoalData=[agvSim.restNode, 0,0,0]; % podam vozliš?e
                %restLocationGoalData=[agvSim.roadToRestNode 1, 0,0,0]; % podam kot cesto in njen konec
            else
                graphSimIo.loadScenario('Maps\Mgame_den520d.json');
                chargeStation=[];
                goalFreeTime=inf;
                restLocationGoalData=[];  % brez varnega mesta
                chargeRoad=[];chargeEntranceNode=[];chargeEntranceRoad=[];
            end
            
            agvSim.setRobotHalfLengthWidth(L,W); % polovicna dolzina in sirina  AGV-ja za namen detekcije trka
            
            nn=[ 173  174;175  176;183  184;185  186;189  190;191  192;193  194;195  202;197  198];    % near nodes
            for i=1:size(nn,1)
                agvSim.setNearbyNodes(nn(i,1), nn(i,2));
                agvSim.setNearbyNodes(nn(i,2), nn(i,1));
            end
            
            
            set(gca,'Xlim',[get(gca,'XLim')]+[-1 1]);set(gca,'Ylim',[get(gca,'YLim')]+[-1 1]);
            figure(10),set(gcf,'position',[483.00         86.00       1342.00        781.00]);
            %get(gcf,'position')
            
     
            Ts=0.5; % Cas vzorcenja
            
    end
end

drawnow();

