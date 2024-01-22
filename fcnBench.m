classdef fcnBench
    methods (Static)
        
        %====================================================
        function [result,AllTimePlans,hPot]=PSIPPXplan(Nstart,Nend,agvSim,Ts,tmax,safetyMargin,L,W,hPot,flagSimulate,chargeStation_,mode,titleAlg)
            %
            %    result=PSIPPXplan(Nstart,Nend,agvSim,L,W,mode)
            %
            %    Nstart,Nend            ..... row vector of starting and ending node for each agv
            %    agvSim                 ..... map structure
            %    Ts                     ..... sample time 
            %    tmax                   ..... max simulation time 
            %    safetyMargin           ..... safety margin (>1, typically 1.6)
            %    L,W                    ..... half robot dymensions
            %    hPot                   ..... handle to path drawings of the robot on plot
            %    flagSimulate           ..... ali simuliram ali ne
            %    chargeStation_         ..... [chargeRoad,chargeEntranceNode,chargeEntranceRoad]
            %    mode                   ..... 0=Astar, 1=PSIPP, 2=PSSIPC
            %
            %    result                 ..... [tSoC, tMks, Niter]
            %    AllTimePlans           .... plans for agents
            
            % z varno lokacijo PSIPPc
            %Ts=0.1; % Cas vzorcenja
            zamikIzris=0.05;
            
            title(titleAlg);
            
            if (~isempty(chargeStation_)&& mode ==2) % ali imam varno lokacijo
                %[chargeRoad, waiting_nodeID, chargeEntranceRoad, chargeEntranceNode]=addChargingStation_v2(agvSim, 407,0,1,'L',1,'S');
                %figure(10); clf; axis equal; agvSim.precomputeMap(); agvSim.drawMap();
                %  chargeStation=[chargeRoad,chargeEntranceNode,chargeEntranceRoad]; % vstop v polnilnico in polnilbca
                chargeStation=chargeStation_; % vstop v polnilnico in polnilbca
                goalFreeTime=0;
                agvSim.roadToRestNode=chargeStation(1);
                agvSim.restNode=agvSim.roadID(chargeStation(1)).eNode;
                
                chargeEntranceNode=chargeStation(2);chargeEntranceRoad=chargeStation(3);
                
                
                restLocationGoalData=[agvSim.roadToRestNode 0.8, 0,0,0]; % varno mesto je na cesti na 80% ceste
                %restLocationGoalData=[agvSim.restNode, 0,0,0]; % podam vozliš?e
                %restLocationGoalData=[agvSim.roadToRestNode 1, 0,0,0]; % podam kot cesto in njen konec
            else
                chargeStation=[];
                goalFreeTime=inf;
                restLocationGoalData=[];  % brez varnega mesta
                chargeRoad=[];chargeEntranceNode=[];chargeEntranceRoad=[];
                
                % da se ne vozimo cez varno lokacijo, tudi ce ta obstaja v zemljevidu
                if(length(chargeStation_)==3)
                    fcnSIPPContainer.removeChargeEntranceRoad(agvSim,chargeStation_(2),chargeStation_(3));
                end
                
            end
            
            
            % za multi goal specificiram zadnji cilj ki je safe location
            % GK 5may2023: opcija za dodat varno mesto pri planiranju
            %    restLocationGoalData=[];  % brez varnega mesta
            %    restLocationGoalData=[agvSim.roadToRestNode 0.8, 0,0,0]; % varno mesto je na cesti
            %restLocationGoalData=[agvSim.restNode, 0,0,0]; % podam vozliš?e
            %restLocationGoalData=[agvSim.roadToRestNode 1, 0,0,0]; % podam kot cesto in njen konec
            
            
            
            
            % pobrisem vse agv-je
            if ~isempty(agvSim.AGV)
                idd=[agvSim.AGV.id];
                for id=idd
                    agvSim.removeAGV(id);
                end
            end
            
            s=1;
            
            Nagv=length(Nstart);
            Orders=[(1:Nagv)', zeros(Nagv,1), ones(Nagv,1)*s,ones(Nagv,1),ones(Nagv,1), ones(Nagv,1)*L,ones(Nagv,1)*W ];
            
            Orders(:,4)=Nstart';Orders(:,5)=Nend';
            
            
            
            casi=nan(size(Orders,1),1);
            
            
            %%%%%%% planiranje
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            
            fcnBench.clearOccupancies(agvSim);
            % safetyMargin=1.6; % za koliko povecam varnostno razdaljo
            replanTime=0; % trenutek kdaj racunamo replaniranje
            
            
            % preverim ce kdo zacne iz vozlisca v slepi ulici in nastavim omejitve za vse ostale
            fcnSIPPContainer.adaptOccupanciesForAgvTrapedDeadEndRoad(agvSim,Orders,safetyMargin,replanTime);
            
            
            AllTimePlans={};
            notFoundPaths=0;
            for i=1:size(Orders,1)
                agvID=Orders(i,1);startTime=Orders(i,2); speed=Orders(i,3); Ns= Orders(i,4); Ne= Orders(i,5); len=Orders(i,6); wid=Orders(i,7);
                safetyRadious=sqrt(len^2+wid^2)*safetyMargin;
                
                %disp(strcat('AGV',num2str(i)));
                priority=i;
                
                % GK 5may2023: opcija za dodat varno mesto pri planiranju
                %    restLocationGoalData=[];  % brez varnega mesta
                %    restLocationGoalData=[agvSim.roadToRestNode 0.8, 0,0,0]; % varno mesto je na cesti
                %restLocationGoalData=[agvSim.restNode, 0,0,0]; % podam vozliš?e
                %restLocationGoalData=[agvSim.roadToRestNode 1, 0,0,0]; % podam kot cesto in njen konec
                
                if ~isempty(restLocationGoalData)
                    % multiGoalData= {[curTask(1), tw1], [curTask(2), tw2],  restLocationGoalData };
                    goalFreeTime=0;
                    tw=0;
                    multiGoalData= {[Ne, tw,goalFreeTime,0],restLocationGoalData };
                else
                    % multiGoalData= {[curTask(1), tw1], [curTask(2), tw2] };
                    goalFreeTime=inf;
                    tw=0;
                    multiGoalData= {[Ne, tw,goalFreeTime,0] };
                end
                
                
                [timePlan, times] = findPathSIPP_multiGoal(agvSim, Ns, multiGoalData, ...
                    speed, startTime, safetyRadious, priority, replanTime, agvID, chargeStation);
                
                %casDrop = times(end-1) + Tasks(idxTasks, 5);
                fcnSIPPContainer.addChargeEntranceRoad(agvSim, chargeEntranceNode, chargeEntranceRoad);
                
                
                %   fcnBench.clearOccupancies(agvSim); % ce zelis brez zasedenosti - podobno kot A*
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
                    
                    if agvSim.DRAW ==1
                        agvSim.createAGVonPoseLW(agvID , [2+i*len;-1;0],len,wid, priority);
                    end
                end
                AllTimePlans{i}=timePlan;
                
                if mode==0
                    fcnBench.clearOccupancies(agvSim);  % da ne pises zasedenosti in potem lahko pride do trka
                end
                
            end
            
            
            %return
            %     axis equal; xlabel('$$x [m]$$', 'interpreter', 'latex', 'FontSize', 12);  ylabel('$$y$$[m]', 'interpreter', 'latex', 'FontSize', 12);
            % set(gcf,'PaperPosition',[1 1 4 3]), print -depsc  map1; % ZanimiviScenariji=18;
            % set(gcf,'PaperPosition',[1 1 4 3]), print -depsc  map2; % ZanimiviScenariji=17;
            % set(gcf,'PaperPosition',[1 1 4 3]), print -depsc  map3; % ZanimiviScenariji=31;
            
            
            
            
            
            if 1  % kriterij=skupni cas voznje ?? lahko bi bil tudi najdaljsi cas voznje od vseh agv-jev
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
                Nsipp=size(Orders,1);
            end
            
            sumCost2=sum(casi);
            taskTime2=max(casi);
            if(mode==2), Nsipps=Nsipp*2; else Nsipps=Nsipp; end
            if isnan(sumCost2), taskTime2=NaN; end
            
            result=[sumCost2, taskTime2, Nsipps];
            
            
            if agvSim.DRAW ==1   % izrisem poti
                if(~isempty(hPot)), delete(hPot); end % le za izris, pobrisem prejsnjega
                hPot=gobjects(1,size(Orders,1)); % handle za izris poti
                
                for i=1:length(agvSim.AGV)  %grem cez vse AGV-je
                    agvID=agvSim.AGV(i).id;
                    if( ~isempty(agvSim.AGV(i).planRoadTimeSinc))  % ce ima plan
                        hPot(i)=agvSim.drawOnePlan(agvID,10,zamikIzris, 0);
                    end
                end
                %agvSim.drawPlanPaths(10,zamikIzris); %Izris vseh poti
            end
            
            
            
            
            
            if flagSimulate==0
                return
            end
            
            %%%%%% simulacija
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            %agvSim.recordMovie=0; % ali snemam film
            
            %agvSim.DRAW=0;
            agvSim.preventCollision=1; %=0 ne prevrja trkov, =1 preverja trke, =2 ignorira trke a jih beleži
            if mode==0, agvSim.preventCollision=2; end
            figure(10),
            %tmax=130; % max cas simulacije
            vmax=3; % max hitrost AGV-jev
            tSimDelay=Ts*.25*.25*.1; % za animacijo koliko pohitri
            %agvSim.drawPlanPaths(10,0.05); %Izris poti
            agvSim.simulateAGVs( Ts, vmax, tmax,tSimDelay); % sample time, vmax, tmax
            agvSim.drawPlanAGVs(); % Izris grafa analize
            %agvSim.drawPlanPaths(10,zamikIzris); %Izris poti
            figure(10)
            if(notFoundPaths==0 && agvSim.AGVsPlanAchived==1)
                title('SIPP: OK. Plan achieved. All AGVs reached goal. ');
            elseif(notFoundPaths==0 && agvSim.AGVsPlanAchived==0)
                title('SIPP: PROBLEM. All paths computed but some AGVs did NOT reach goal. ');
            elseif(notFoundPaths>0 && agvSim.AGVsPlanAchived==1)
                title('SIPP: CCA OK. Not all paths are found! Other vehicles have reched goal. (Vehicles without plan are not simulated.)');
                fprintf(1,' \n For %d AGVs path was not found. This AGVs are not simulated and drawn! \n',notFoundPaths);
            elseif(notFoundPaths>0 && agvSim.AGVsPlanAchived==1)
                title('SIPP: PROBLEM. Not all paths are found! Some of the vehicles with plan have NOT reched goal. (Vehicles without plan are not simulated.)');
                fprintf(1,' \n For %d AGVs path was not found. This AGVs are not simulated and drawn! \n',notFoundPaths);
            end
        end
        
        %==========================================================================================
        function [PSIPPl,AllTimePlans,hPot]=PSIPPLLplan(Nstart,Nend,agvSim,Ts,tmax,safetyMargin,L,W,hPot,skipReplaneTime,flagSimulate,chargeStation_,titleAlg)
            %
            %    result=PSIPPLLplan(Nstart,Nend,agvSim,L,W,mode)
            %
            %    Nstart,Nend            ..... row vector of starting and ending node for each agv
            %    agvSim                 ..... map structure
            %    L,W                    ..... half robot dymensions
            %    hPot                   ..... handle to path drawings of the robot on plot
            %    skipReplaneTime        ..... Avoid successive unsuccessful replannings
            %    flagSimulate           ..... ali simuliram ali ne
            %    chargeStation_         ..... [chargeRoad,chargeEntranceNode,chargeEntranceRoad]
            %
            %    PSIPPl                 ..... [tSoC, tMks, Niter]
            %    AllTimePlans           .... plans for agents
            
            
            % z varno lokacijo in zacetno rezervacijo PSIPPl
            %%%%%%% planiranje z zacetno rezervacijo
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            
            % % poskusim najprej PSIPPC
            %                     [PSIPPC,AllTimePlans,hPot]=PSIPPXplan(Nstart,Nend,agvSim,L,W,hPot,flagSimulate,chargeStation,2);
            %                     PSIPPl=PSIPPC;    % probam kar s PSIPPc
            %                     return
            %
            
            title(titleAlg);
            
            fcnBench.clearOccupancies(agvSim);
            %safetyMargin=1.6; % za koliko povecam varnostno razdaljo
            % Ts=0.1; % Cas vzorcenja
            zamikIzris=0.05;
            replanTime=0; % trenutek kdaj racunamo replaniranje
            
            
            % if any(Nstart==Nend) % plan ni izvedljiv z neskoncno zacetno rezervacijo
            %     % PSIPPC
            %     [PSIPPC,AllTimePlans,hPot]=PSIPPXplan(Nstart,Nend,agvSim,L,W,hPot,flagSimulate,chargeStation,2);
            %     PSIPPl=PSIPPC;    % probam kar s PSIPPc
            %     return
            % end
            
            
            if (~isempty(chargeStation_)) % ali imam varno lokacijo
                chargeStation=chargeStation_; % vstop v polnilnico in polnilnica
                goalFreeTime=0;
                agvSim.roadToRestNode=chargeStation(1);
                agvSim.restNode=agvSim.roadID(chargeStation(1)).eNode;
                
                chargeEntranceNode=chargeStation(2);chargeEntranceRoad=chargeStation(3);
                
                restLocationGoalData=[agvSim.roadToRestNode 0.8, 0,0,0]; % varno mesto je na cesti na 80% ceste
                %restLocationGoalData=[agvSim.restNode, 0,0,0]; % podam vozliš?e
                %restLocationGoalData=[agvSim.roadToRestNode 1, 0,0,0]; % podam kot cesto in njen konec
            else
                chargeStation=[];
                goalFreeTime=inf;
                restLocationGoalData=[];  % brez varnega mesta
                chargeRoad=[];chargeEntranceNode=[];chargeEntranceRoad=[];
                
                % da se ne vozimo ?ez varno lokacijo, tudi ce ta obstaja v zemljevidu
                if(length(chargeStation_)==3)
                    fcnSIPPContainer.removeChargeEntranceRoad(agvSim,chargeStation_(2),chargeStation_(3));
                end
            end
            
            
            
            
            % pobrisem vse agv-je
            if ~isempty(agvSim.AGV)
                idd=[agvSim.AGV.id];
                for id=idd
                    agvSim.removeAGV(id);
                end
            end
            
            s=1; speed=s;
            
            Nagv=length(Nstart);
            Orders=[(1:Nagv)', zeros(Nagv,1), ones(Nagv,1)*s,ones(Nagv,1),ones(Nagv,1), ones(Nagv,1)*L,ones(Nagv,1)*W ];
            Orders(:,4)=Nstart';Orders(:,5)=Nend';
            
            casi=nan(size(Orders,1),1);
            
            %tmax=1000; 
            vmax=3;
            tSimDelay=Ts*.25*.25*.1; % za animacijo koliko pohitri
            
            AllTimePlans={};
            notFoundPaths=0;
            
            dAGV=zeros(Nagv,1); % prevozene razdalje AGV-jev
            
            agentPlannedNum=0;
            agentPlanned=zeros(1,Nagv);
            prostiAgvji=1:Nagv;
            
            lastplantime=zeros(1,Nagv);
            %skipReplaneTime=2*Ts;
            
            
            if agvSim.DRAW ==1   % izrisem poti
                if(~isempty(hPot)), delete(hPot); end % le za izris, pobrisem prejsnjega
                hPot=gobjects(1,Nagv); % handle za izris poti
            end
            
            
            
            for i=1:Nagv
                agvID=Orders(i,1); Ns= Orders(i,4); len=Orders(i,6); wid=Orders(i,7);
                road=agvSim.nodeID(Ns).neighboarRoadIDs(1); % izberem eno cesto
                agvSim.createAGVonRoadLW(agvID , road, 0,len,wid,i);
                
                %  nastavim zacetne zasedenosti do Inf
                fcnBench.setStartNodeInitialReservation(agvSim,Ns, replanTime, agvID);
            end
            
            Nsipps=0;
            agvSim.AGVsPlanAchived=0;
            
            t=0;
            %while( agentPlannedNum<Nagv && t<tmax) %  max cas ali do dosezenega plana
            % while(~(idxTasks>Ntasks && (agvSim.AGVsPlanAchived || all(t>=casiVsehDrop)) ) && t<tmax) %  max cas ali do dosezenega plana
            while( ~((agvSim.AGVsPlanAchived )) && t<tmax ) %  max cas ali do dosezenega plana
                
                
                
                if ~isempty(prostiAgvji)   %&&  t==0
                    for i=prostiAgvji
                        
                        % Avoid successive unsuccessful replannings % ??? opcija, tiste v polnilnico bi lahko planiral bolj poredko?
                        % TODO: skrajsaj ali odstrani
                        if (t>0 && (lastplantime(i) + skipReplaneTime) > t )
                            continue
                        end
                        lastplantime(i) = t;
                        
                        
                        
                        % agvID=Orders(i,1);startTime=Orders(i,2); speed=Orders(i,3); Ns= Orders(i,4); Ne= Orders(i,5); len=Orders(i,6); wid=Orders(i,7);
                        agvID=Orders(i,1);
                        %startTime=t;
                        speed=Orders(i,3); Ns= Orders(i,4); Ne= Orders(i,5); len=Orders(i,6); wid=Orders(i,7);
                        
                        safetyRadious=sqrt(len^2+wid^2)*safetyMargin;
                        % priority=i;
                        priority=agentPlannedNum+1;
                        
                        Ns=Nstart(i);
                        Ne=Nend(i);
                        
                        if ~isempty(restLocationGoalData)
                            % multiGoalData= {[curTask(1), tw1], [curTask(2), tw2],  restLocationGoalData };
                            goalFreeTime=0;
                            tw=0;
                            multiGoalData= {[Ne, tw,goalFreeTime,0],restLocationGoalData };
                        else
                            % multiGoalData= {[curTask(1), tw1], [curTask(2), tw2] };
                            goalFreeTime=inf;
                            tw=0;
                            multiGoalData= {[Ne, 0,goalFreeTime,0] };
                        end
                        
                        
                        [timePlan, times] = findPathSIPP_multiGoal(agvSim, Ns, multiGoalData, ...
                            speed, t, safetyRadious, priority, t, agvID, chargeStation);
                        
                        
                        Nsipps=Nsipps+2;
                        
                        
                        fcnSIPPContainer.addChargeEntranceRoad(agvSim, chargeEntranceNode, chargeEntranceRoad);
                        
                        if(~isempty(timePlan))  % najden plan
                            agentPlannedNum=agentPlannedNum+1;
                            prostiAgvji=prostiAgvji(prostiAgvji~=i);
                            casi(i)= times(1);
                            
                            fcnBench.clearStartNodeInitialReservation(agvSim,Ns,agvID); % pobrisem svojo zacetno rezervacijo do Inf
                            
                            goalWait=goalFreeTime;
                            fcnSIPPContainer.setOccupancySIPP2(agvSim,timePlan,agvID,speed,safetyRadious,priority,t,goalWait); % nastavi OI-je
                            
                            fcnSIPPContainer.addChargeEntranceRoad(agvSim, chargeEntranceNode, chargeEntranceRoad);
                            
                            [~,onRoadID,locStartTmp]=agvSim.getLocPoseAGV(agvID);
                            if onRoadID~=timePlan(1,2) % ce ni na povezavi (je v vozliscu ampak na drugi poti...)
                                agvSim.replaceAGVonRoad(agvID,timePlan(1,2), 0);
                                locStart=0;
                            else
                                locStart=locStartTmp;
                            end
                            agvSim.setPlanRoadTimeSincLUT (agvID, timePlan,t);
                            
                            if(agvSim.DRAW ==1 && ~isempty(agvSim.AGV(i).planRoadTimeSinc))  % ce ima plan
                                hPot(i)=agvSim.drawOnePlan(agvID,10,zamikIzris, 0);
                            end
                            
                            fcnSIPPContainer.removeChargeEntranceRoad(agvSim,chargeEntranceNode,chargeEntranceRoad);
                        else
                            timePlan=[];
                            
                            izpis=strcat('For AGV=',num2str(i),' no path is found. Replanning again later.');
                            disp(izpis);
                            notFoundPaths=notFoundPaths+1;
                            %agvSim.createAGVonPoseLW(agvID , [2+i*len;-1;0],len,wid, priority);
                            
                            %  if(length(prostiAgvji)==Nagv && notFoundPaths>=Nagv) % vsi prosti a plana se nisem nasel, torej z zacetnimi rezervacijami ne gre
                            if(notFoundPaths>=Nagv*10) % vsi prosti a plana se nisem nasel, torej z zacetnimi rezervacijami ne gre
                                
                                % poskusim kar s PSIPPC, ce uspe, drugace vrnem NaN
                                % PSIPPC
                                [PSIPPC,AllTimePlans,hPot]=fcnBench.PSIPPXplan(Nstart,Nend,agvSim,Ts,L,W,hPot,flagSimulate,chargeStation,2,titleAlg);
                                PSIPPl=PSIPPC;    % probam kar s PSIPPc
                                return
                            end
                        end
                        AllTimePlans{i}=timePlan;
                        
                    end
                end
                
                
                if flagSimulate==0 && isempty(prostiAgvji)
                    break;
                end
                
                t=t+Ts;
                dAGV=agvSim.moveAGVsOneSample(Ts,vmax,tSimDelay,dAGV,t); % Simulacija premika
                if ~isempty(prostiAgvji)
                    agvSim.AGVsPlanAchived=0;
                end
            end
            
            
            
            
            
            if 1  % kriterij=skupni cas voznje ?? lahko bi bil tudi najdaljsi cas voznje od vseh agv-jev
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
                Nsipp=size(Orders,1);
            end
            
            sumCost2=sum(casi);
            taskTime2=max(casi);
            if isnan(sumCost2), taskTime2=NaN; end
            
            
            PSIPPl=[sumCost2, taskTime2, Nsipps];
            
            
            % if agvSim.DRAW ==1   % izrisem poti
            %     if(~isempty(hPot)), delete(hPot); end % le za izris, pobrisem prejsnjega
            %     hPot=gobjects(1,size(Orders,1)); % handle za izris poti
            %
            %     for i=1:length(agvSim.AGV)  %grem cez vse AGV-je
            %         agvID=agvSim.AGV(i).id;
            %         if( ~isempty(agvSim.AGV(i).planRoadTimeSinc))  % ce ima plan
            %             hPot(i)=agvSim.drawOnePlan(agvID,10,zamikIzris, 0);
            %         end
            %     end
            %     %agvSim.drawPlanPaths(10,zamikIzris); %Izris vseh poti
            % end
            %
            
            
            
            
            
            
            if flagSimulate==0
                return
            end
            
            
            
            
            %agvSim.drawPlanAGVs(); % Izris grafa analize
            %agvSim.drawPlanPaths(10,zamikIzris); %Izris poti
            figure(10)
            if(notFoundPaths==0 && agvSim.AGVsPlanAchived==1)
                title(' OK. Plan achieved. All AGVs reached goal. ');
            elseif(notFoundPaths==0 && agvSim.AGVsPlanAchived==0)
                title(' PROBLEM. All paths computed but some AGVs did NOT reach goal. ');
            elseif(notFoundPaths>0 && agvSim.AGVsPlanAchived==1 && isempty(prostiAgvji))
                title(' OK. All vehicles have reched goal. (Some vehicles were replanned)');
                fprintf(1,' \n For %d plannigs path was replanned. \n',notFoundPaths);
            elseif(notFoundPaths>0 && agvSim.AGVsPlanAchived==1)
                title(' PROBLEM. Not all paths are found! Some of the vehicles with plan have NOT reched goal. (Vehicles without plan are not simulated.)');
                fprintf(1,' \n For %d plannigs path was not found. \n',notFoundPaths);
            end
            
            
        end
        
        
        %====================================================
        
        function setStartNodeInitialReservation(obj,nodeID, replantime, agvID)
            % zasedem od zacetnega casa planiranja (? Ni potrebno še pred tem e.g. replantime-tVarn ) do inf s prioriteto 0
            occupancy= [replantime,Inf,0,agvID, 1];  %[ts, te, priority, agvID, pathSegment]
            fcnSIPPContainer.setNodeOccupancy(obj, nodeID, occupancy);  % zasedenost koncnega vozlisca [ts, te, priority, agvID, pathSegment]
        end
        %====================================================
        function clearStartNodeInitialReservation(obj,nodeID,agvID)
            st=obj.nodeID(nodeID);
            if isempty(st.occupancy)
                return
            end
            
            filt=((st.occupancy(:,4)==agvID) & (st.occupancy(:,2)==Inf));
            
            
            st.occupancy=st.occupancy(~filt,:); % ostranim mojo zasedenost do Inf
            obj.nodeID(nodeID)=st;
            
            %nastavim se zasedenosti drugih vozlisc, ki so prevec blizu temu vozliscu
            % ali nearbyNode dodat še varnostni faktor za koliko povecat
            % zasedenost dvojnemu vozliscu
            if(~isempty(st.nearbyNode))
                for i=1:size(st.nearbyNode,2)
                    nearID=st.nearbyNode(i);
                    st=obj.nodeID(nearID);
                    filt=((st.occupancy(:,4)==agvID) & (st.occupancy(:,2)==Inf));
                    st.occupancy=st.occupancy(~filt,:); % ostranim mojo zasedenost do Inf
                    obj.nodeID(nearID)=st;
                end
            end
        end
        %====================================================
        
        function [CCBS,outputStructure,capturedText,Times,AllPlanDNN]=CCBSplan(Ns,Ne,mapXML)
            Times=[]; AllPlanDNN={};
            % write task.xml
            fid = fopen('task.xml', 'w');
            
            fprintf(fid,'<?xml version="1.0" encoding="utf-8"?>\n');
            fprintf(fid,'<root>\n');
            for i=1:length(Ns)
                fprintf(fid,'<agent start_id="%d" goal_id="%d" />\n',Ns(i),Ne(i));
            end
            fprintf(fid,'</root>');
            fclose(fid);
            
            % call CCBS.exe
            inputParams=strcat(mapXML,' task.xml config.xml');
            fullCommand = ['CCBS.exe ' inputParams];  % call CCBS function
            capturedText = evalc(['!' fullCommand]);
            
            % inerprate results
            outputStructure = struct();  % Initialize a structure to store the parsed data
            if(strncmp(capturedText,'Error',5)||isempty(capturedText))
                outputStructure.Error=1;
                disp(capturedText);
                return;
            else
                outputStructure.Error=0;
            end
            lines = strsplit(capturedText, '\n'); % Split the lines of the captured text
            
            % iterprete result string:  Loop through each line and parse key-value pairs
            for i = 1:numel(lines)
                % Skip empty lines
                if isempty(lines{i})
                    continue;
                end
                
                % Split the line into key and value
                if strncmp(lines{i},'init_root',9)
                    keyValue = strsplit(lines{i}, ' ');
                else
                    keyValue = strsplit(lines{i}, ':');
                end
                
                % Remove leading and trailing whitespaces
                key = strtrim(keyValue{1});
                value = strtrim(keyValue{2});
                
                % Create the structure based on key-value pairs
                if(strcmp(key,'Soulution found'))
                    if strcmp(value,'true')
                        outputStructure.(genvarname(key)) =1;
                    else
                        outputStructure.(genvarname(key)) =0;
                    end
                elseif strcmp(key,'LL expanded(avg)')
                    key='LL expandedAvg';
                    outputStructure.(genvarname(key)) = str2double(value);
                else
                    outputStructure.(genvarname(key)) = str2double(value);
                end
            end
            
            
            Nagv=length(Ns);
            
            %disp('Parsed Output Structure:');
            %disp(outputStructure);
            CCBS=[NaN , NaN, NaN];
            if(outputStructure.Error==1)
                disp('Napaka CCBS iskanja.')
                disp(outputStructure);
            elseif(outputStructure.SoulutionFound==1)
                tSoc=outputStructure.Flowtime;
                tMks=outputStructure.Makespan;
                % Niter=(outputStructure.HLExpanded-1)*2+outputStructure.LLSearches*2*0+ Nagv;
                Niter=outputStructure.LLSearches+ Nagv;
                
                CCBS=[tSoc , tMks, Niter];
            elseif(outputStructure.SoulutionFound==0)
                Niter=outputStructure.LLSearches+ Nagv;
                CCBS=[NaN , NaN, Niter];
            end
            
            
            
            if(outputStructure.Error==0 && outputStructure.SoulutionFound==1)
                xmlFile = 'task_log.xml';
                xmlStruct = xml2struct(xmlFile);
                
                agents=xmlStruct.root.log.agent;
                for i = 1:length(agents)
                    if  length(agents)>1
                        Times=[Times;str2double(agents{i}.path.Attributes.duration)];
                        pathSections=agents{i}.path.section;
                    else
                        Times=[Times;str2double(agents.path.Attributes.duration)];
                        pathSections=agents.path.section;
                    end
                    
                    plan=[];
                    if(length(pathSections)>1)
                        for j=1:length(pathSections)
                            plan=[plan;...
                                str2double(pathSections{j}.Attributes.duration),...
                                str2double(pathSections{j}.Attributes.start_id),...
                                str2double(pathSections{j}.Attributes.goal_id)];
                        end
                    else
                        plan=[plan;...
                            str2double(pathSections.Attributes.duration),...
                            str2double(pathSections.Attributes.start_id),...
                            str2double(pathSections.Attributes.goal_id)];
                    end
                    AllPlanDNN{i}=plan;
                end
            end
            
            
            
            
            
            
            
        end
        
        %==================================================
        function AllTimePlans=convertPlanCCBS2agvSim(agvSim,plansDNN)
            
            AllTimePlans={};
            for i=1:size(plansDNN,2)
                TTs= plansDNN{i}(:,1);
                Ns=plansDNN{i}(:,2);
                Ne=plansDNN{i}(:,3);
                
                TP=[];
                t=0;
                twait=0;
                dtVmes=[];
                waitVmes=[];
                roadVmes=[];
                locVmes=0;
                NsPovezave=0;
                for j=1:length(Ns)
                    
                    if isKey(agvSim.nodeID, Ns(j))&& isKey(agvSim.nodeID, Ne(j))==0   % prvo vmesno vozlisce
                        dtVmes=[TTs(j)];
                        waitVmes=[0];
                        
                        
                        NsPovezave= Ns(j);
                        
                    elseif (isKey(agvSim.nodeID, Ns(j))==0 && isKey(agvSim.nodeID, Ne(j))==0)   %  vmesno vozlisce
                        dtVmes=[dtVmes,TTs(j)];
                        if Ns(j)==Ne(j)  % ali cakamo na vmesnem vozliscu
                            waitVmes=[waitVmes,1];
                        else
                            waitVmes=[waitVmes,0];
                        end
                        
                    elseif isKey(agvSim.nodeID, Ns(j))==0&& isKey(agvSim.nodeID, Ne(j))   % zadnje vozlisce sestavljene poezave
                        node=agvSim.nodeID(NsPovezave);
                        rids= node.neighboarRoadIDs;   % vsaka cesta ima samo eno zacetno vozlisce
                        
                        nasel=0;
                        for r=rids % poiscem pravo cesto
                            if (agvSim.roadID(r).eNode==Ne(j))
                                nasel=1;
                                roadVmes=r;
                                break;
                            end
                        end
                        if nasel==0
                            disp('napaka plana na vmesni povezavi pri pretvorbi iz CCBS v timePlan');
                        end
                        locVmes=0;
                        L=agvSim.roadID(roadVmes).length;
                        for v=1:length(dtVmes)
                            if waitVmes(v)
                                t=t+ dtVmes(v);
                                TP=[TP; t,roadVmes,locVmes];
                            else
                                locVmes=locVmes+dtVmes(v)/L;
                                t=t+ dtVmes(v);
                                TP=[TP; t,roadVmes,locVmes];
                            end
                        end
                        t=t+ TTs(j);
                        L=agvSim.roadID(roadVmes).length;
                        locVmes=locVmes+TTs(j)/L;
                        TP=[TP; t,roadVmes,locVmes];
                        
                        roadVmes=[];
                        %----------
                    elseif Ns(j)==Ne(j) % cakamo na glavnem vozliscu a se ne vem na kateri cesti
                        twait=twait+TTs(j);
                        continue;
                        
                    else % se vozim proti glavnemu vozliscu
                        node=agvSim.nodeID(Ns(j));
                        rids= node.neighboarRoadIDs;   % vsaka cesta ima samo eno startno vozlisce
                        
                        nasel=0;
                        for r=rids % poiscem pravo cesto
                            if (agvSim.roadID(r).eNode==Ne(j))
                                nasel=1;
                                break;
                            end
                        end
                        
                        if twait<1e-4
                            t=t+ TTs(j)+twait;
                            TP=[TP;t,r,1];
                        else
                            TP=[TP;t+twait,r,0]; % shranim prejsnej cakanje
                            
                            t=t+ TTs(j)+twait;
                            TP=[TP; t,r,1];      % shranim voznjo
                        end
                        twait=0;
                        if nasel==0
                            disp('napaka plana pri pretvorbi iz CCBS v timePlan');
                        end
                    end
                end
                AllTimePlans{i}=TP;
            end
            
        end
        %==================================================
        function [hPot]=simulateTimePlans(agvSim,AllTimePlans,hPot,len,wid,Ts,preventCollision,titleAlg)
            
            title(titleAlg);
            Nagv=size(AllTimePlans,2);
            zamikIzris=0.05;
            
            % pobrisem vse agv-je
            if ~isempty(agvSim.AGV)
                idd=[agvSim.AGV.id];
                for id=idd
                    agvSim.removeAGV(id);
                end
            end
            
            
            
            tmax=0;
            for i=1:Nagv
                priority=i;
                agvID=i;
                
                %  tu robota zmeraj premaknem na zacetek ceste, tudi ce je plan zacel
                %  na cesti (bi moral drugace podati se hitrosti voznje ali pa podat kot parameter zacetne lege...)
                agvSim.createAGVonRoadLW(agvID , AllTimePlans{i}(1,2), 0,len,wid,priority);
                
                
                %fcnSIPPContainer.addChargeEntranceRoad(agvSim, chargeEntranceNode, chargeEntranceRoad);
                agvSim.setPlanRoadTimeSinc (agvID, AllTimePlans{i});
                % fcnSIPPContainer.removeChargeEntranceRoad(agvSim,chargeEntranceNode,chargeEntranceRoad);
                tmax=max(tmax,AllTimePlans{i}(end,1));
            end
            
            tmax=tmax+10*Ts;
            
            
  %    tmax=42;
            
            
            
            if agvSim.DRAW ==1   % izrisem poti
                if(~isempty(hPot)), delete(hPot); end % le za izris, pobrisem prejsnjega
                hPot=gobjects(1,Nagv); % handle za izris poti
                
                for i=1:length(agvSim.AGV)  %grem cez vse AGV-je
                    agvID=agvSim.AGV(i).id;
                    if( ~isempty(agvSim.AGV(i).planRoadTimeSinc))  % ce ima plan
                        hPot(i)=agvSim.drawOnePlan(agvID,10,zamikIzris, 0);
                    end
                end
                %agvSim.drawPlanPaths(10,zamikIzris); %Izris vseh poti
            end
            
            
            
            
            %%%%%% simulacija
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            %agvSim.recordMovie=0; % ali snemam film
            
            %agvSim.DRAW=0;
            agvSim.preventCollision=preventCollision; %=0 ne prevrja trkov, =1 preverja trke, =2 ignorira trke a jih beleži
            figure(10),
            %tmax=130; % max cas simulacije
            vmax=3; % max hitrost AGV-jev
            tSimDelay=Ts*.25*.25*.01; % za animacijo koliko pohitri
            %agvSim.drawPlanPaths(10,0.05); %Izris poti
            agvSim.simulateAGVs( Ts, vmax, tmax,tSimDelay); % sample time, vmax, tmax
            agvSim.drawPlanAGVs(); % Izris grafa analize
            %agvSim.drawPlanPaths(10,zamikIzris); %Izris poti
            figure(10)
            notFoundPaths=0;
            if(notFoundPaths==0 && agvSim.AGVsPlanAchived==1)
                title(' OK. Plan achieved. All AGVs reached goal. ');
            elseif(notFoundPaths==0 && agvSim.AGVsPlanAchived==0)
                title(' PROBLEM. All paths computed but some AGVs did NOT reach goal. ');
            elseif(notFoundPaths>0 && agvSim.AGVsPlanAchived==1)
                title(' CCA OK. Not all paths are found! Other vehicles have reched goal. (Vehicles without plan are not simulated.)');
                fprintf(1,' \n For %d AGVs path was not found. This AGVs are not simulated and drawn! \n',notFoundPaths);
            elseif(notFoundPaths>0 && agvSim.AGVsPlanAchived==1)
                title(' PROBLEM. Not all paths are found! Some of the vehicles with plan have NOT reched goal. (Vehicles without plan are not simulated.)');
                fprintf(1,' \n For %d AGVs path was not found. This AGVs are not simulated and drawn! \n',notFoundPaths);
            end
            
            
            
        end
        %================================================
        function CCBSconfig(robotSize,timeLimit,precision)
            % write config.xml
            fid = fopen('config.xml', 'w');
            
            fprintf(fid,'<?xml version="1.0" ?>\n');
            fprintf(fid,'<root>\n');
            fprintf(fid,'	<algorithm>\n');
            fprintf(fid,'		<use_cardinal>true</use_cardinal>\n');
            fprintf(fid,'		<use_disjoint_splitting>true</use_disjoint_splitting>');
            fprintf(fid,'		<hlh_type>2</hlh_type>\n');
            fprintf(fid,'       <connectedness>3</connectedness>\n');
            fprintf(fid,'       <focal_weight>1.0</focal_weight>\n');
            fprintf(fid,'       <agent_size>%f</agent_size>\n',robotSize);
            fprintf(fid,'       <timelimit>%d</timelimit>\n',timeLimit);
            fprintf(fid,'       <precision>%f</precision>\n',precision);
            fprintf(fid,'	</algorithm>\n');
            fprintf(fid,'</root>');
            fclose(fid);
        end
        %================================================
        function clearOccupancies(obj)
            roads=obj.roadID.keys;
            nodes=obj.nodeID.keys;
            for i=1:obj.roadID.Count
                st=obj.roadID(roads{i});
                st.occupancy=[];
                st.occupancy2=[];
                obj.roadID(roads{i})=st;
            end
            for j=1:obj.nodeID.Count
                st=obj.nodeID(nodes{j});
                st.occupancy=[];
                obj.nodeID(nodes{j})=st;
            end
        end
        %================================================
        
        function [hPot,hText1,hText2]=simulateLLPlansTaskSol(taskSol,Nagv,Ntasks_,agvSim,L,W,Ts,preventCollision, title_,hPot,hText1,hText2,showPaths)
            %    taskSol                ..... all time plans and other data
            %    agvSim                 ..... map structure
            %    L,W                    ..... half robot dymensions
            %    Ts                     ..... sample time
            %    preventCollision       ..... =0 ne prevrja trkov, =1 preverja trke, =2 ignorira trke a jih beleži
            %
            
            t=0;
            Ntasks=min(Ntasks_,size(taskSol,2));
            
            
            oldDrAW=agvSim.DRAW;
            agvSim.DRAW=1;
            if ~isempty(hPot),  delete(hPot); end
            if ~isempty(hText1), delete(hText1); end
            if ~isempty(hText2), delete(hText2); end
            
            
            title(title_);
            
            hText2= text(1,-1,strcat('t=',num2str(t) ),'FontSize',14);
            agvSim.preventCollision=preventCollision; %=0 ne prevrja trkov, =1 preverja trke, =2 ignorira trke a jih beleži
            
            zamikIzris=0.05;
            
            tmax=0; tDropMAX=0;
            for i=(Ntasks-Nagv+1):Ntasks
                tmax=max(tmax,taskSol(i).timePlan(end,1) );
                tDropMAX=max( tDropMAX, taskSol(i).tDrop );
            end
            
            tmax=tmax+Ts*0;
            tDropMAX=tDropMAX+Ts*0;
            
            vmax=3;
            tSimDelay=Ts*.25*.25*.001; % za animacijo koliko pohitri
            speed=1;
            
            
            
            % pobrisem vse agv-je
            if ~isempty(agvSim.AGV)
                idd=[agvSim.AGV.id];
                for id=idd
                    agvSim.removeAGV(id);
                end
            end
            
            
            dAGV=zeros(Nagv,1); % prevozene razdalje AGV-jev
            
            agentPlannedNum=0;
            agentPlanned=zeros(1,Nagv);
            prostiAgvji=1:Nagv;
            
            
            hText1= [];hPot=[];
            
            for i=1:Nagv
                agvID=i; %Ns= Nstart(i);
                road=taskSol(i).timePlan(1,2); % izberem zacetno cesto
                agvSim.createAGVonRoadLW(agvID , road, 0,L,W,0); % id,onRoadID, onRoadLoc,length, width, priority
                
                %  nastavim zacetne zasedenosti do Inf
                %setStartNodeInitialReservation(agvSim,Ns, replanTime, agvID);
                if agvSim.DRAW ==1, hText1(agvID)= text(5,-.5-0.5*agvID,' '); end    % handli za tekst
            end
            if agvSim.DRAW ==1, hPot = gobjects(1,Nagv);end
            
            agvSim.AGVsPlanAchived=0;
            
            idxTasks=1;
            agvTask=zeros(size(agvSim.AGV,2),2); % za vsak agv si zapomnim kateri task izvaja in proti kateri lokaciji pickup=1, dropoff=2
            doneTasks=0;
            
            agvSim.intSimTime();
            dAGV=zeros(Nagv,1); % prevozene razdalje AGV-jev
            
            writerObj=[];
            
            casiVsehDrop=Inf(1,Nagv);
            distVsehDrop=Inf(1,Nagv);
            
            lastplantime=zeros(1,Nagv);
            
            while( t<tDropMAX )  % || t<=tmax) %  max cas ali do dosezenega plana
                
                while( idxTasks<=Ntasks && t>=taskSol(idxTasks).startT )   % nastavim nove plane agv-jem
                    
                    frejAgv=taskSol(idxTasks).agvID;
                    agvID=agvSim.AGV(frejAgv).id;
                    
                    dAGV(frejAgv)=0; % zacne nov plan zato resetiram prepotovano razdaljo
                    agvSim.AGVsPlanAchived=0;
                    
                    
                    %  agvSim.replaceAGVonRoad(agvID,taskSol(idxTasks).timePlan(1,2), 0); % ni ok, lahko zacne na cesti!
                    %  locStart=0;
                    
                    [~,onRoadID,locStartTmp]=agvSim.getLocPoseAGV(agvID);
                    if onRoadID~=taskSol(idxTasks).timePlan(1,2) % ce ni na povezavi (je v vozliscu ampak na drugi poti...)
                        agvSim.replaceAGVonRoad(agvID,taskSol(idxTasks).timePlan(1,2), 0);
                        locStart=0;
                    else
                        locStart=locStartTmp;
                    end
                    
                    
                    
                    
                    agvSim.setPlanRoadTimeSincLUT (agvID, taskSol(idxTasks).timePlan , t);
                    %distVsehDrop(frejAgv)= agvSim.AGV(frejAgv).planRoadTimeSinc(ind,4);    % prepisem pri kateri prepotovani razdalji ga dosezem
                    
                    % -------------- Izris planov
                    if agvSim.DRAW ==1
                      if showPaths  
                        if(~isempty(hPot(frejAgv))), delete(hPot(frejAgv)); end     % le za izris planov
                        hPot(frejAgv)=agvSim.drawOnePlan(agvID,10,zamikIzris, locStart);
                      end  
                        
                        startNode=agvSim.roadID(taskSol(idxTasks).timePlan(1,2)).sNode;
                        finNodes=taskSol(idxTasks).pickDrop;
                        
                        
                        if(~isempty(hText1(agvID))), delete(hText1(agvID)); end     % le za izpis plana
                         %%%% wrh1
                         % hText1(agvID)= text(5,-0.6*agvID,strcat('T=',num2str(idxTasks),',',' AGV',num2str(agvID),' ...',num2str(startNode),' ->',num2str(finNodes) ));
                         %%%% wrh2
                         % hText1(agvID)= text(70,35-2*agvID,strcat('T=',num2str(idxTasks),',',' AGV',num2str(agvID),' ...',num2str(startNode),' ->',num2str(finNodes) ));
                         %%%% game
                          hText1(agvID)= text(280,230-10*agvID,strcat('T=',num2str(idxTasks),',',' AGV',num2str(agvID),' ...',num2str(startNode),' ->',num2str(finNodes) ));
                        
                        
                        
                    end
                    
                    idxTasks=idxTasks+1;
                end
                
                
                t=t+Ts;
                dAGV=agvSim.moveAGVsOneSample(Ts,vmax,tSimDelay,dAGV,t); % Simulacija premika
                delete(hText2);
                hText2= text(1.5,-1,strcat('t=',num2str(t) ),'FontSize',14);
                
                % recordMovie;
                if agvSim.recordMovie
                    if (t==Ts)
                        % Set up the movie.
                        %axis([0 6 0 4]+[-1 1 -1 1]*0.5)
                        %axis([0 14 5 15]+[-1 1 -1 1]*0.5)
                        writerObj = VideoWriter('MovieOut.avi'); % Name it.
                        writerObj.FrameRate = 15; % How many frames per second.
                        open(writerObj);
                    else
                        frame = getframe(gcf); % 'gcf' can handle if you zoom in to take a movie.
                        writeVideo(writerObj, frame);
                    end %
                end
                
            end  % while(~agvSim.AGVsPlanAchived && t<tmax)
            
            if ~isempty(writerObj),   close(writerObj) ; end  % za snemanja videa
            
            
            
            agvSim.drawPlanAGVs(); % Izris grafa analize
            
            % if agvSim.DRAW ==1
            %     figure(10)
            %     if(agvSim.AGVsPlanAchived==1)
            %         title('OK. Plan achieved. All AGVs reached goal. ');
            %     elseif(agvSim.AGVsPlanAchived==0)
            %         title('PROBLEM. All paths computed but some AGVs did NOT reach goal. ');
            %     end
            % end
            
            agvSim.DRAW=oldDrAW;
            
        end % function
        
        %=========================================================
        
        
        function [PSIPPl,taskSol]=PSIPPLLplanSingleLifeLong(Nstart,Tasks_,maxTask_,agvSim,L,W,Ts,safetyMargin,skipReplaneTime,preventCollision,chargeStation_)
            %
            %    result=PSIPPLLplanLifeLong(Nstart,Nend,agvSim,L,W,mode)
            %
            %    Nstart                 ..... initial start locations
            %    Tasks                  ..... each row is pickup and dropoff node for one task
            %    agvSim                 ..... map structure
            %    L,W                    ..... half robot dymensions
            %    flagSimulate           ..... ali simuliram ali ne
            %    Ts                     ..... sample time
            %    safetyMargin           ..... safety margin
            %    skipReplaneTime        ..... Avoid successive unsuccessful replannings
            %    preventCollision       ..... =0 ne prevrja trkov, =1 preverja trke, =2 ignorira trke a jih beleži
            %    chargeStation_         ..... [chargeRoad,chargeEntranceNode,chargeEntranceRoad]
            %
            %    PSIPPl                 ..... [tMksAll, NitAll]
            %    taskSol                ..... plans for each task
            
            
            % z varno lokacijo in zacetno rezervacijo PSIPPl
            %%%%%%% planiranje z zacetno rezervacijo
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            agvSim.preventCollision=preventCollision; %=0 ne prevrja trkov, =1 preverja trke, =2 ignorira trke a jih beleži
            
            fcnBench.clearOccupancies(agvSim);
            %safetyMargin=1.6+0.4*1; % za koliko povecam varnostno razdaljo
            %Ts=0.1*10; % Cas vzorcenja
            zamikIzris=0.05;
            replanTime=0; % trenutek kdaj racunamo replaniranje
            
            tmax=50000; vmax=3;
            tSimDelay=Ts*.25*.25*.001; % za animacijo koliko pohitri
            speed=1;
            
            
            
            taskSol=[];
            Nsipps=0;
            
            if (~isempty(chargeStation_)) % ali imam varno lokacijo
                chargeStation=chargeStation_; % vstop v polnilnico in polnilnica
                goalFreeTime=0;
                agvSim.roadToRestNode=chargeStation(1);
                agvSim.restNode=agvSim.roadID(chargeStation(1)).eNode;
                
                
                restLocationGoalData=[agvSim.roadToRestNode 0.8, 0,0,0]; % varno mesto je na cesti na 80% ceste
                %restLocationGoalData=[agvSim.restNode, 0,0,0]; % podam vozliš?e
                %restLocationGoalData=[agvSim.roadToRestNode 1, 0,0,0]; % podam kot cesto in njen konec
                
                flagGoCharging=1; % z varnim mestom - ker se rezervira pot do polnilice je skupen cas lahko daljsi, a vedno najde pot
                
            else
                chargeStation=[];
                goalFreeTime=inf;
                restLocationGoalData=[];  % brez varnega mesta
                
                % da se ne vozimo cez varno lokacijo, tudi ce ta obstaja v zemljevidu
                if(length(chargeStation_)==3)
                    fcnSIPPContainer.removeChargeEntranceRoad(agvSim,chargeStation_(2),chargeStation_(3));
                end
                
                flagGoCharging=0; % brez varnega mesta
            end
            
            
            
            
            % pobrisem vse agv-je
            if ~isempty(agvSim.AGV)
                idd=[agvSim.AGV.id];
                for id=idd
                    agvSim.removeAGV(id);
                end
            end
            
            
            Nagv=length(Nstart);
            
            
            AllTimePlans={};
            notFoundPaths=0;
            
            dAGV=zeros(Nagv,1); % prevozene razdalje AGV-jev
            
            agentPlannedNum=0;
            agentPlanned=zeros(1,Nagv);
            prostiAgvji=1:Nagv;
            
            
            hText= [];hPot=[];
            
            for i=1:Nagv
                agvID=i; Ns= Nstart(i);
                road=agvSim.nodeID(Ns).neighboarRoadIDs(1); % izberem eno cesto
                agvSim.createAGVonRoadLW(agvID , road, 0,L,W,0); % id,onRoadID, onRoadLoc,length, width, priority
                
                %  nastavim zacetne zasedenosti do Inf
                %setStartNodeInitialReservation(agvSim,Ns, replanTime, agvID);
                if agvSim.DRAW ==1, hText(agvID)= text(5,-.5-0.5*agvID,' '); end    % handli za tekst
            end
            %if agvSim.DRAW ==1,
            hPot = gobjects(1,Nagv);
            %end
            
            
            
            
            agvSim.AGVsPlanAchived=0;
            
            Ntasks=size(Tasks_,1); tw1=0;
            Ntasks=min(maxTask_,Ntasks);
            %Tasks=[Tasks_(1:Ntasks,:), zeros(Ntasks,1),ones(Ntasks,1)*tw1];
            
            AgentTasks={};
            for i=1:Nagv
                AgentTasks{i}=[i:Nagv:Ntasks] ;
            end
            
            idxTasks=1;
            agvTask=zeros(size(agvSim.AGV,2),2); % za vsak agv si zapomnim kateri task izvaja in proti kateri lokaciji pickup=1, dropoff=2
            doneTasks=0;
            
            agvSim.intSimTime();
            
            Nagv=size(agvSim.AGV,2);
            dAGV=zeros(Nagv,1); % prevozene razdalje AGV-jev
            t=0; %i=1;
            writerObj=[];
            
            casiVsehDrop=Inf(1,Nagv);
            distVsehDrop=Inf(1,Nagv);
            
            lastplantime=zeros(1,Nagv);
            
            tSoc=0;
            
            while(~(idxTasks>Ntasks && (agvSim.AGVsPlanAchived || all(t>=casiVsehDrop)) ) && t<tmax) %  max cas ali do dosezenega plana
                
                % if(idxTasks>=59)
                %     g=123
                % end
                
                
                if( flagGoCharging && any(t>=casiVsehDrop)  ) % nastavim stanje za tiste ki so na poti v polnilnico
                    aa_=find(dAGV'>=distVsehDrop & [agvSim.AGV.state]==1); % agv-ji na poti v polnilnico
                    for a=aa_
                        agvSim.AGV(a).state=-1;  % stanje: na poti v polnilnico
                    end
                end
                
                
                
                %prostiAgvji=find([agvSim.AGV(:).state] == 0 );  % vsi gredo vedno v polnilnico in sele nato nadaljujejo
                prostiAgvji=find([agvSim.AGV(:).state] <= 0 );  % 0 ali -1  % lahko prekine pot v polnilnico
                % protiVarnemuMestuAgvji=find([agvSim.AGV(:).state] == -1);
                
                
                if ~isempty(prostiAgvji)   %&&  t==0
                    for frejAgv=prostiAgvji
                        
                        % Avoid successive unsuccessful replannings % ??? opcija, tiste v polnilnico bi lahko planiral bolj poredko?
                        % TODO: skrajsaj ali odstrani
                        %          if (idxTasks>1 && (lastplantime(frejAgv) + skipReplaneTime) > t )
                        if (t>0 && (lastplantime(frejAgv) + skipReplaneTime) > t )
                            continue
                        end
                        lastplantime(frejAgv) = t;
                        
                        
                        if idxTasks<=Ntasks && t>=0 && ~isempty(AgentTasks{frejAgv})
                            % New task available
                            agvSim.AGVsPlanAchived=0;
                            
                            safetyRadious = agvSim.AGV(frejAgv).AGVRadious*safetyMargin;
                            priority=max([agvSim.AGV.priority])+1; % dam mu najmanjso prioriteto
                            
                            
                            % chargeStation=[chargeRoad,chargeEntranceNode,chargeEntranceRoad];
                            
                            rid=agvSim.AGV(frejAgv).onRoadID;
                            loc=agvSim.AGV(frejAgv).onRoadLoc;
                            if loc==1
                                start=agvSim.roadID(rid).eNode;
                            elseif loc==0
                                start=agvSim.roadID(rid).sNode;
                            else
                                start=[rid, loc];
                            end
                            
                            agvID=agvSim.AGV(frejAgv).id;
                            
                            wait1=tw1;
                            Ne1=Tasks_(AgentTasks{frejAgv}(1));
                            
                            %               fcnSIPPContainer.addChargeEntranceRoad(agvSim,chargeStation(2),chargeStation(3));
                            
                            if flagGoCharging
                                multiGoalData= {[Ne1, wait1,goalFreeTime,0],restLocationGoalData };
                                Nsipps=Nsipps+2; % planira start->goal->SL
                            else
                                multiGoalData= {[Ne1, wait1,goalFreeTime,0]};
                                Nsipps=Nsipps+1; % planira start->goal
                            end
                            
                            %          if(idxTasks>=64)
                            %              g=1
                            %          end
                            
                            [timePlan, times] = findPathSIPP_multiGoal(agvSim, start, multiGoalData, ...
                                speed, t, safetyRadious, priority, t, agvID, chargeStation);
                            
                            if(~isempty(timePlan))
                                if flagGoCharging
                                    casDrop = times(1) + tw1;
                                else
                                    casDrop = 1e30;
                                end
                            end
                            if ~isempty(chargeStation)
                                fcnSIPPContainer.addChargeEntranceRoad(agvSim,chargeStation(2),chargeStation(3));
                            end
                            
                            
                            if(isempty(timePlan))
                                % ?? ce ni resitve je potrebno replanirat vse, ki so
                                % imeli prej manjso prioriteto od mene (so planirali za
                                % menoj). Visje prioritetne ni potrebno replanirat oz mogoce je potrebno?
                                if agvSim.DRAW ==1
                                    if(~isempty(hPot(frejAgv))), delete(hPot(frejAgv)); end     % le za izris
                                end
                                %timePlan=[];
                                if agvSim.AGV(frejAgv).state==-1
                                    fprintf(1,' V polnilnico: For AGV %d no path is found.\n',agvSim.AGV(frejAgv).id);
                                else
                                    fprintf(1,' Prost: For AGV %d no path is found.\n',agvSim.AGV(frejAgv).id);
                                    notFoundPaths=notFoundPaths+1;
                                    agvSim.AGV(frejAgv).planRoadTimeSinc=[]; % pobrisem mu plan, da se ne simulira??
                                end
                                
                                
                            else
                                AgentTasks{frejAgv}(1)=[]; % odstranim task agentu
                                
                                doneTasks=doneTasks+1;
                                agvID=agvSim.AGV(frejAgv).id;
                                
                                
                                fcnSIPPContainer.removeOccupanciesForUnfinishedPlan(agvSim,agvSim.AGV(frejAgv).planRoadTimeSinc,agvID,t);
                                % Manj prioritetni, ki so še na vožnji se bodo kljub temu
                                % izogibali prvotnemu (neizvršenemu planu) - cas bo torej malo daljsi
                                % Bolj prioritetnim od prej se izogiba ta AGV.
                                % Vsi novi AGV imajo nizjo prioriteto in upostevajo novi plan.
                                
                                
                                casiVsehDrop(frejAgv)= casDrop;
                                
                                dAGV(frejAgv)=0; % zacne nov plan zato resetiram prepotovano razdaljo
                                agvSim.AGVsPlanAchived=0;
                                
                                %tw2 = Task(end); % GK 24apr2023: to je za funkcijo set occupancy cakanje na zadnje na cilju. Uposteva se le ce v planu pri zadnji akciji ni cakanja.
                                %Ker je zadnja akcija varno mesto, ki je na cesti je to brezpredmetno. Ce je varno mesto vozlise pa se s tem dodda se do datno cakanje - zato je boje da je 0.
                                % kadar zadnja akcija ni varno mesto pa imaš tu možnost dodat dodaten cas cakanja v cilju, ki ga v planu ni. torej za normalno uporabo je to vecinoma 0.
                                tw2 = 0;
                                fcnSIPPContainer.setOccupancySIPP2(agvSim,timePlan,agvID,speed,safetyRadious,priority,t,tw2); % nastavi OI-je
                                
                                % TODO odstrani, za safe location se ne rabi
                                if flagGoCharging==0  % ?? ce koncamo v slepi ulici potem si zasedem tudi izhod iz nje   - zasedenost pomaga le za manj prioritetne, ki to upoštevajo
                                    replanTimeTemp = timePlan(end,1);
                                    tNadajujem = replanTimeTemp;
                                    rid=timePlan(end,2);
                                    start=agvSim.roadID(rid).eNode;
                                    fcnSIPPContainer.addOccupancyForAgvStartDeadEndRoad(agvSim,agvID,replanTimeTemp,tNadajujem,safetyRadious,speed, start);
                                end
                                
                                
                                % spremenim prioriteto AGV-ju
                                agvSim.AGV(frejAgv).priority=priority;
                                
                                [~,onRoadID,locStartTmp]=agvSim.getLocPoseAGV(agvID);
                                if onRoadID~=timePlan(1,2) % ce ni na povezavi (je v vozliscu ampak na drugi poti...)
                                    agvSim.replaceAGVonRoad(agvID,timePlan(1,2), 0);
                                    locStart=0;
                                else
                                    locStart=locStartTmp;
                                end
                                agvSim.setPlanRoadTimeSincLUT (agvID, timePlan,t);
                                
                                
                                if flagGoCharging
                                    ind=find(abs(timePlan(:,1)-casDrop)<1e-9); % dobim le indeks na plan
                                    %   if ~isnan(ind)
                                    distVsehDrop(frejAgv)= agvSim.AGV(frejAgv).planRoadTimeSinc(ind,4);    % prepisem pri kateri prepotovani razdalji ga dosezem
                                    %   else
                                    %      g=12
                                    %   end
                                end
                                
                                % Polnilnica: ko napisem plan odstranim povezavo. Simulacija bo vseeno delovala, ker preverja konzistenco plana le ob vpisu plana v agvSim
                                if( flagGoCharging && ~isempty(chargeStation) )
                                    fcnSIPPContainer.removeChargeEntranceRoad(agvSim,chargeStation(2),chargeStation(3));
                                end
                                
                                
                                %
                                % if(~isempty(hPot(frejAgv))), delete(hPot(frejAgv)); end
                                % hPot(frejAgv)=agvSim.drawOnePlan(agvID,10,zamikIzris, locStart);
                                % [agvID,frejAgv]
                                % [start,Ne1]
                                % pause
                                % if(~isempty(hPot(frejAgv))), delete(hPot(frejAgv)); end
                                
                                % -------------- Izris planov
                                if agvSim.DRAW ==1
                                    if(~isempty(hPot(frejAgv))), delete(hPot(frejAgv)); end     % le za izris planov
                                    hPot(frejAgv)=agvSim.drawOnePlan(agvID,10,zamikIzris, locStart);
                                    
                                    if(~isempty(hText(agvID))), delete(hText(agvID)); end     % le za izpis plana
                                    % hText(agvID)= text(5,-.1-0.5*agvID,strcat('AGV',num2str(agvID),', T=',num2str(idxTasks),' ,', num2str(Tasks(idxTasks,1)),' ->',num2str(Tasks(idxTasks,2))));
                                    % hText(agvID)= text(270,230-10*agvID,strcat('AGV',num2str(agvID),', T=',num2str(idxTasks),', ', num2str(Tasks(idxTasks,1)),' ->',num2str(Tasks(idxTasks,2))));
                                    %hText(agvID)= text(280,230-10*agvID,strcat('T=',num2str(idxTasks),',',' AGV',num2str(agvID),' ...',num2str(Ne1)));
                                    
                                    %  hText(agvID)= text(280,230-10*agvID,strcat('T=',num2str(idxTasks),',',' AGV',num2str(agvID),'..', num2str(agvSim.roadID(timePlan(1,2)).sNode),' ->',num2str(Ne1) ));
                                    hText(agvID)= text(25,16-.8*agvID,strcat('T=',num2str(idxTasks),',',' AGV',num2str(agvID),'..', num2str(agvSim.roadID(timePlan(1,2)).sNode),' ->',num2str(Ne1) ));
                                    
                                end
                                
                                % hranim plane za vsak task
                                taskSol(idxTasks).agvID=agvID;
                                taskSol(idxTasks).pickDrop=Ne1;
                                taskSol(idxTasks).timePlan=timePlan;
                                taskSol(idxTasks).startRID=agvSim.AGV(frejAgv).onRoadID;
                                taskSol(idxTasks).startLOC=agvSim.AGV(frejAgv).onRoadLoc;
                                taskSol(idxTasks).startT=t;
                                
                                taskSol(idxTasks).tDrop=casDrop;
                                % pri analizi glej za iste agv-je kako prejemajo taske in potem doloci TSoC, ,...
                                % Mks je dolocen kot max(taskSol(Ntasks-Nagv:Ntasks).tDrop)
                                
                                %tSoc=tSoc+ t-t
                                disp([idxTasks,t,Nsipps])
                                
                                
                                idxTasks=idxTasks+1;  % naslednji task
                                
                                
                                %------ enostavno pobrisem vse zasedenosti ponornega mesta
                                fcnSIPPContainer.clearNodeOccupancy(agvSim,agvSim.restNode)  ;
                            end
                        end  % if  length(noviCilji)>0
                    end
                end % if ~isempty(prostiAgv)
                
                t=t+Ts;
                
                % recordMovie;
                %     if agvSim.recordMovie
                %         if (t==Ts)
                %             % Set up the movie.
                %             %axis([0 6 0 4]+[-1 1 -1 1]*0.5)
                %             %axis([0 14 5 15]+[-1 1 -1 1]*0.5)
                %             writerObj = VideoWriter('MovieOut.avi'); % Name it.
                %             writerObj.FrameRate = 15; % How many frames per second.
                %             open(writerObj);
                %         else
                %             frame = getframe(gcf); % 'gcf' can handle if you zoom in to take a movie.
                %             writeVideo(writerObj, frame);
                %         end %
                %     end
                
                dAGV=agvSim.moveAGVsOneSample(Ts,vmax,tSimDelay,dAGV,t); % Simulacija premika
                
                %   i=i+1;
            end  % while(~agvSim.AGVsPlanAchived && t<tmax)
            
            if ~isempty(writerObj),   close(writerObj) ; end  % za snemanja videa
            
            idxTasks
            
            taskSol
            
            % analiza planiranja
            %fcnSIPPContainer.metricsTasks(agvSim,speed,Tasks,taskSol,notFoundPaths);
            idleSumTime=notFoundPaths*Ts;
            %metrika=fcnSIPPContainer.metricsTasks(agvSim,Tasks,taskSol,notFoundPaths,idleSumTime);
            % TODO ta metrika uposteva tudi voznjo v SL
            
            procentDelivered=doneTasks/Ntasks*100
            
            agvSim.drawPlanAGVs(); % Izris grafa analize
            
            if agvSim.DRAW ==1
                figure(10)
                if(notFoundPaths==0 && agvSim.AGVsPlanAchived==1)
                    title('SIPP: OK. Plan achieved. All AGVs reached goal. ');
                elseif(notFoundPaths==0 && agvSim.AGVsPlanAchived==0)
                    title('SIPP: PROBLEM. All paths computed but some AGVs did NOT reach goal. ');
                elseif(notFoundPaths>0 && agvSim.AGVsPlanAchived==1)
                    title('SIPP: CCA OK. Not all paths are found! Other vehicles have reched goal. (Vehicles without plan are not simulated.)');
                    fprintf(1,' \n For %d plannigs path was not found. \n',notFoundPaths);
                elseif(notFoundPaths>0 && agvSim.AGVsPlanAchived==1)
                    title('SIPP: PROBLEM. Not all paths are found! Some of the vehicles with plan have NOT reched goal. (Vehicles without plan are not simulated.)');
                    fprintf(1,' \n For %d plannigs path was not found. \n',notFoundPaths);
                end
            end
            
            
            tMks=max([taskSol((Ntasks-Nagv+1):Ntasks).tDrop]);
            
            PSIPPl=[tMks, Nsipps]
            PSIPPl_niOK=[t, Nsipps]
            
            
            
        end % function
        
        %=================================================================
        
        function [PSIPPPlus,AllTimePlans,hPot,NstartP,NendP]=PSIPPPlusPlan(Nstart,Nend,agvSim,Ts,tmax,safetyMargin,L,W,hPot,flagSimulate,titleAlg)
            
            DrawTmp=agvSim.DRAW;
            SaveTmp=agvSim.recordMovie;
            agvSim.recordMovie=0;
            agvSim.DRAW=0;
            flagSimulate_=0;
            
            
            Norder=length(Nstart);
            OrdPerm=flipud(perms(1:Norder));
            
            
            minSumCost=inf;
            minTaskTime=inf;
            ppMinTSOC=nan;
            ppMinTMKS=nan;
            
            chargeStation=[];
            
            for pp=1:size(OrdPerm,1)
                
                NstartP= Nstart(OrdPerm(pp,:)); % permutiran vrstni red
                NendP= Nend(OrdPerm(pp,:));   % permutiran vrstni red
                
                [PSIPP,AllTimePlans,hPot]=fcnBench.PSIPPXplan(NstartP,NendP,agvSim,Ts,tmax,safetyMargin,L,W,hPot,flagSimulate_,chargeStation,1,titleAlg);
                
                if PSIPP(1)<minSumCost  % SOC
                    minSumCost=PSIPP(1);
                    ppMinTSOC=[pp,PSIPP(1),PSIPP(2)];
                end
                
                if PSIPP(2)<minTaskTime %MKS
                    minTaskTime=PSIPP(2);
                    ppMinTMKS=[pp,PSIPP(1),PSIPP(2)];
                end
                
            end
            
            
            
            % return solution with minimum tSOC
            agvSim.DRAW=DrawTmp;
            agvSim.recordMovie=SaveTmp;
            pp=ppMinTSOC(1);
            NstartP= Nstart(OrdPerm(pp,:)); % permutiran vrstni red
            NendP= Nend(OrdPerm(pp,:));   % permutiran vrstni red
            [PSIPPPlus,AllTimePlans,hPot]=fcnBench.PSIPPXplan(NstartP,NendP,agvSim,Ts,tmax,safetyMargin,L,W,hPot,flagSimulate,chargeStation,1,titleAlg);
            
            
            PSIPPPlus(3)=round(exp(1)*factorial(Norder)-1);
            %PSIPPPlus(3)=factorial(Norder)*Norder;
            
            
        end
        
        %=================================================
        
        
        function [PSIPPl,taskSol]=PSIPPLLplanPickDropLifeLong(Nstart,Tasks_,maxTask_,agvSim,L,W,Ts,safetyMargin,skipReplaneTime,preventCollision,chargeStation_)
            %
            %    result=PSIPPLLplanLifeLong(Nstart,Nend,agvSim,L,W,mode)
            %
            %    Nstart                 ..... initial start locations
            %    Tasks                  ..... each row is pickup and dropoff node for one task
            %    agvSim                 ..... map structure
            %    L,W                    ..... half robot dymensions
            %    flagSimulate           ..... ali simuliram ali ne
            %    Ts                     ..... sample time
            %    safetyMargin           ..... safety margin
            %    skipReplaneTime        ..... Avoid successive unsuccessful replannings
            %    preventCollision       ..... =0 ne prevrja trkov, =1 preverja trke, =2 ignorira trke a jih beleži
            %    chargeStation_         ..... [chargeRoad,chargeEntranceNode,chargeEntranceRoad]
            %
            %    PSIPPl                 ..... [tMksAll, NitAll]
            %    taskSol                ..... plans for each task
            
            
            % z varno lokacijo in zacetno rezervacijo PSIPPl
            %%%%%%% planiranje z zacetno rezervacijo
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            agvSim.preventCollision=preventCollision; %=0 ne prevrja trkov, =1 preverja trke, =2 ignorira trke a jih beleži
            
            fcnBench.clearOccupancies(agvSim);
            %safetyMargin=1.6+0.4*1; % za koliko povecam varnostno razdaljo
            %Ts=0.1*10; % Cas vzorcenja
            zamikIzris=0.05;
            replanTime=0; % trenutek kdaj racunamo replaniranje
            
            tmax=50000; vmax=3;
            tSimDelay=Ts*.25*.25*.001; % za animacijo koliko pohitri
            speed=1;
            
            
            
            taskSol=[];
            Nsipps=0;
            
            if (~isempty(chargeStation_)) % ali imam varno lokacijo
                chargeStation=chargeStation_; % vstop v polnilnico in polnilnica
                goalFreeTime=0;
                agvSim.roadToRestNode=chargeStation(1);
                agvSim.restNode=agvSim.roadID(chargeStation(1)).eNode;
                
                
                restLocationGoalData=[agvSim.roadToRestNode 0.8, 0,0,0]; % varno mesto je na cesti na 80% ceste
                %restLocationGoalData=[agvSim.restNode, 0,0,0]; % podam vozliš?e
                %restLocationGoalData=[agvSim.roadToRestNode 1, 0,0,0]; % podam kot cesto in njen konec
                
                flagGoCharging=1; % z varnim mestom - ker se rezervira pot do polnilice je skupen cas lahko daljsi, a vedno najde pot
                
            else
                chargeStation=[];
                goalFreeTime=inf;
                restLocationGoalData=[];  % brez varnega mesta
                
                % da se ne vozimo cez varno lokacijo, tudi ce ta obstaja v zemljevidu
                if(length(chargeStation_)==3)
                    fcnSIPPContainer.removeChargeEntranceRoad(agvSim,chargeStation_(2),chargeStation_(3));
                end
                
                flagGoCharging=0; % brez varnega mesta
            end
            
            
            
            
            % pobrisem vse agv-je
            if ~isempty(agvSim.AGV)
                idd=[agvSim.AGV.id];
                for id=idd
                    agvSim.removeAGV(id);
                end
            end
            
            
            Nagv=length(Nstart);
            
            
            AllTimePlans={};
            notFoundPaths=0;
            
            dAGV=zeros(Nagv,1); % prevozene razdalje AGV-jev
            
            agentPlannedNum=0;
            agentPlanned=zeros(1,Nagv);
            prostiAgvji=1:Nagv;
            
            
            hText= [];hPot=[];
            
            for i=1:Nagv
                agvID=i; Ns= Nstart(i);
                road=agvSim.nodeID(Ns).neighboarRoadIDs(1); % izberem eno cesto
                agvSim.createAGVonRoadLW(agvID , road, 0,L,W,0); % id,onRoadID, onRoadLoc,length, width, priority
                
                %  nastavim zacetne zasedenosti do Inf
                %setStartNodeInitialReservation(agvSim,Ns, replanTime, agvID);
                if agvSim.DRAW ==1, hText(agvID)= text(5,-.5-0.5*agvID,' '); end    % handli za tekst
            end
            if agvSim.DRAW ==1, hPot = gobjects(1,Nagv);end
            
            
            
            
            
            agvSim.AGVsPlanAchived=0;
            
            Ntasks=size(Tasks_,1); tw1=0; tw2=0;
            Ntasks=min(maxTask_,Ntasks);
            Tasks=[Tasks_(1:Ntasks,:), zeros(Ntasks,1),ones(Ntasks,1)*tw1,ones(Ntasks,1)*tw2];
            
            idxTasks=1;
            agvTask=zeros(size(agvSim.AGV,2),2); % za vsak agv si zapomnim kateri task izvaja in proti kateri lokaciji pickup=1, dropoff=2
            doneTasks=0;
            
            agvSim.intSimTime();
            
            Nagv=size(agvSim.AGV,2);
            dAGV=zeros(Nagv,1); % prevozene razdalje AGV-jev
            t=0; %i=1;
            writerObj=[];
            
            casiVsehDrop=Inf(1,Nagv);
            distVsehDrop=Inf(1,Nagv);
            
            lastplantime=zeros(1,Nagv);
            
            tSoc=0;
            
            while(~(idxTasks>Ntasks && (agvSim.AGVsPlanAchived || all(t>=casiVsehDrop)) ) && t<tmax) %  max cas ali do dosezenega plana
                
                
                
                if( flagGoCharging && any(t>=casiVsehDrop)  ) % nastavim stanje za tiste ki so na poti v polnilnico
                    aa_=find(dAGV'>=distVsehDrop & [agvSim.AGV.state]==1); % agv-ji na poti v polnilnico
                    for a=aa_
                        agvSim.AGV(a).state=-1;  % stanje: na poti v polnilnico
                    end
                end
                
                
                
                %prostiAgvji=find([agvSim.AGV(:).state] == 0 );  % vsi gredo vedno v polnilnico in sele nato nadaljujejo
                prostiAgvji=find([agvSim.AGV(:).state] <= 0 );  % 0 ali -1  % lahko prekine pot v polnilnico
                % protiVarnemuMestuAgvji=find([agvSim.AGV(:).state] == -1);
                
                
                if ~isempty(prostiAgvji)   %&&  t==0
                    for frejAgv=prostiAgvji
                        
                        % Avoid successive unsuccessful replannings % ??? opcija, tiste v polnilnico bi lahko planiral bolj poredko?
                        %if  lastplantime(frejAgv) + skipReplaneTime > t
                        if  t>0 && lastplantime(frejAgv) + skipReplaneTime > t
                            continue
                        end
                        lastplantime(frejAgv) = t;
                        
                        
                        if idxTasks<=Ntasks && t>=Tasks(idxTasks,3)
                            % New task available
                            agvSim.AGVsPlanAchived=0;
                            
                            safetyRadious = agvSim.AGV(frejAgv).AGVRadious*safetyMargin;
                            priority=max([agvSim.AGV.priority])+1; % dam mu najmanjso prioriteto
                            
                                
                            rid=agvSim.AGV(frejAgv).onRoadID;
                            loc=agvSim.AGV(frejAgv).onRoadLoc;
                            if loc==1
                                start=agvSim.roadID(rid).eNode;
                            elseif loc==0
                                start=agvSim.roadID(rid).sNode;
                            else
                                start=[rid, loc];
                            end

                            agvID=agvSim.AGV(frejAgv).id;
                            Ne1=Tasks(idxTasks,1); Ne2=Tasks(idxTasks,2);
                            wait1=Tasks(idxTasks,4); wait2=Tasks(idxTasks,5);
                            if flagGoCharging
                                multiGoalData= {[Ne1, wait1,goalFreeTime,0],[Ne2, wait2,goalFreeTime,0],restLocationGoalData };
                                Nsipps=Nsipps+3; % planira start->pick->drop->SL
                            else
                                multiGoalData= {[Ne1, wait1,goalFreeTime,0],[Ne2, wait2,goalFreeTime,0]};
                                Nsipps=Nsipps+2; % planira start->pick->drop
                            end

                            [timePlan, times] = findPathSIPP_multiGoal(agvSim, start, multiGoalData, ...
                                speed, t, safetyRadious, priority, t, agvID, chargeStation);
                    
                            if(~isempty(timePlan))
                               if flagGoCharging
                                   casDrop = times(2) + Tasks(idxTasks, 5);
                               else
                                   casDrop=1e30;
                               end
                            end
                            
                            if ~isempty(chargeStation)
                                fcnSIPPContainer.addChargeEntranceRoad(agvSim,chargeStation(2),chargeStation(3));
                            end   
                               

                            if(isempty(timePlan))
                                % ?? ce ni resitve je potrebno replanirat vse, ki so
                                % imeli prej manjso prioriteto od mene (so planirali za
                                % menoj). Visje prioritetne ni potrebno replanirat oz mogoce je potrebno?
                                if agvSim.DRAW ==1
                                    if(~isempty(hPot(frejAgv))), delete(hPot(frejAgv)); end     % le za izris
                                end
                                %timePlan=[];
                                if agvSim.AGV(frejAgv).state==-1
                                    fprintf(1,' V polnilnico: For AGV %d no path is found.\n',agvSim.AGV(frejAgv).id);
                                else
                                    fprintf(1,' Prost: For AGV %d no path is found.\n',agvSim.AGV(frejAgv).id);
                                    notFoundPaths=notFoundPaths+1;
                                    agvSim.AGV(frejAgv).planRoadTimeSinc=[]; % pobrisem mu plan, da se ne simulira??
                                end
                                
                                
                            else
                                doneTasks=doneTasks+1;
                                agvID=agvSim.AGV(frejAgv).id;
                                
                                
                                fcnSIPPContainer.removeOccupanciesForUnfinishedPlan(agvSim,agvSim.AGV(frejAgv).planRoadTimeSinc,agvID,t);
                                % Manj prioritetni, ki so še na vožnji se bodo kljub temu
                                % izogibali prvotnemu (neizvršenemu planu) - cas bo torej malo daljsi
                                % Bolj prioritetnim od prej se izogiba ta AGV.
                                % Vsi novi AGV imajo nizjo prioriteto in upostevajo novi plan.
                                
                                
                                casiVsehDrop(frejAgv)= casDrop;
                                
                                dAGV(frejAgv)=0; % zacne nov plan zato resetiram prepotovano razdaljo
                                agvSim.AGVsPlanAchived=0;
                                
                                %tw2 = Task(end); % GK 24apr2023: to je za funkcijo set occupancy cakanje na zadnje na cilju. Uposteva se le ce v planu pri zadnji akciji ni cakanja.
                                %Ker je zadnja akcija varno mesto, ki je na cesti je to brezpredmetno. Ce je varno mesto vozlise pa se s tem dodda se do datno cakanje - zato je boje da je 0.
                                % kadar zadnja akcija ni varno mesto pa imaš tu možnost dodat dodaten cas cakanja v cilju, ki ga v planu ni. torej za normalno uporabo je to vecinoma 0.
                                tw2 = 0;
                                fcnSIPPContainer.setOccupancySIPP2(agvSim,timePlan,agvID,speed,safetyRadious,priority,t,tw2); % nastavi OI-je
                                
                                % TODO odstrani, za safe location se ne rabi
                                if flagGoCharging==0  % ?? ce koncamo v slepi ulici potem si zasedem tudi izhod iz nje   - zasedenost pomaga le za manj prioritetne, ki to upoštevajo
                                    replanTimeTemp = timePlan(end,1);
                                    tNadajujem = replanTimeTemp;
                                    rid=timePlan(end,2);
                                    start=agvSim.roadID(rid).eNode;
                                    fcnSIPPContainer.addOccupancyForAgvStartDeadEndRoad(agvSim,agvID,replanTimeTemp,tNadajujem,safetyRadious,speed, start);
                                end
                                
                                
                                % spremenim prioriteto AGV-ju
                                agvSim.AGV(frejAgv).priority=priority;
                                
                                [~,onRoadID,locStartTmp]=agvSim.getLocPoseAGV(agvID);
                                if onRoadID~=timePlan(1,2) % ce ni na povezavi (je v vozliscu ampak na drugi poti...)
                                    agvSim.replaceAGVonRoad(agvID,timePlan(1,2), 0);
                                    locStart=0;
                                else
                                    locStart=locStartTmp;
                                end
                                agvSim.setPlanRoadTimeSincLUT (agvID, timePlan,t);
                                
                                
                                if flagGoCharging
                                    ind=find(abs(timePlan(:,1)-casDrop)<1e-9); % dobim le indeks na plan
                                    distVsehDrop(frejAgv)= agvSim.AGV(frejAgv).planRoadTimeSinc(ind,4);    % prepisem pri kateri prepotovani razdalji ga dosezem
                                end
                                
                                % Polnilnica: ko napisem plan odstranim povezavo. Simulacija bo vseeno delovala, ker preverja konzistenco plana le ob vpisu plana v agvSim
                                if( flagGoCharging && ~isempty(chargeStation) )
                                    fcnSIPPContainer.removeChargeEntranceRoad(agvSim,chargeStation(2),chargeStation(3));
                                end
                                
                                % -------------- Izris planov
                                if agvSim.DRAW ==1
                                    if(~isempty(hPot(frejAgv))), delete(hPot(frejAgv)); end     % le za izris planov
                                    hPot(frejAgv)=agvSim.drawOnePlan(agvID,10,zamikIzris, locStart);
                                    
                                    if(~isempty(hText(agvID))), delete(hText(agvID)); end     % le za izpis plana
                                    % hText(agvID)= text(5,-.1-0.5*agvID,strcat('AGV',num2str(agvID),', T=',num2str(idxTasks),' ,', num2str(Tasks(idxTasks,1)),' ->',num2str(Tasks(idxTasks,2))));
                                    % hText(agvID)= text(270,230-10*agvID,strcat('AGV',num2str(agvID),', T=',num2str(idxTasks),', ', num2str(Tasks(idxTasks,1)),' ->',num2str(Tasks(idxTasks,2))));
                                    hText(agvID)= text(280,230-10*agvID,strcat('T=',num2str(idxTasks),',',' AGV',num2str(agvID),' ...',num2str(Tasks(idxTasks,1)),'->',num2str(Tasks(idxTasks,2))));
                                    
                                end
                                
                                % hranim plane za vsak task
                                taskSol(idxTasks).agvID=agvID;
                                taskSol(idxTasks).pickDrop=Tasks(idxTasks,1:2);
                                taskSol(idxTasks).timePlan=timePlan;
                                taskSol(idxTasks).startRID=agvSim.AGV(frejAgv).onRoadID;
                                taskSol(idxTasks).startLOC=agvSim.AGV(frejAgv).onRoadLoc;
                                taskSol(idxTasks).startT=t;
                                
                                taskSol(idxTasks).tDrop=casDrop;
                                % pri analizi glej za iste agv-je kako prejemajo taske in potem doloci TSoC, ,...
                                % Mks je dolocen kot max(taskSol(Ntasks-Nagv:Ntasks).tDrop)
                                
                                %tSoc=tSoc+ t-t
                                disp([idxTasks,t,Nsipps])
                                
                                
                                idxTasks=idxTasks+1;  % naslednji task
                                
                                
                                %------ enostavno pobrisem vse zasedenosti ponornega mesta
                                fcnSIPPContainer.clearNodeOccupancy(agvSim,agvSim.restNode)  ;
                            end
                        end  % if  length(noviCilji)>0
                    end
                end % if ~isempty(prostiAgv)
                
                t=t+Ts;
                
                % recordMovie;
                %     if agvSim.recordMovie
                %         if (t==Ts)
                %             % Set up the movie.
                %             %axis([0 6 0 4]+[-1 1 -1 1]*0.5)
                %             %axis([0 14 5 15]+[-1 1 -1 1]*0.5)
                %             writerObj = VideoWriter('MovieOut.avi'); % Name it.
                %             writerObj.FrameRate = 15; % How many frames per second.
                %             open(writerObj);
                %         else
                %             frame = getframe(gcf); % 'gcf' can handle if you zoom in to take a movie.
                %             writeVideo(writerObj, frame);
                %         end %
                %     end
                
                dAGV=agvSim.moveAGVsOneSample(Ts,vmax,tSimDelay,dAGV,t); % Simulacija premika
                
                %   i=i+1;
            end  % while(~agvSim.AGVsPlanAchived && t<tmax)
            
            if ~isempty(writerObj),   close(writerObj) ; end  % za snemanja videa
            
            idxTasks
            
            taskSol
            
            % analiza planiranja
            %fcnSIPPContainer.metricsTasks(agvSim,speed,Tasks,taskSol,notFoundPaths);
            idleSumTime=notFoundPaths*Ts;
            metrika=fcnSIPPContainer.metricsTasks(agvSim,Tasks,taskSol,notFoundPaths,idleSumTime);
            % TODO ta metrika uposteva tudi voznjo v SL
            
            procentDelivered=doneTasks/Ntasks*100
            
            agvSim.drawPlanAGVs(); % Izris grafa analize
            
            if agvSim.DRAW ==1
                figure(10)
                if(notFoundPaths==0 && agvSim.AGVsPlanAchived==1)
                    title('SIPP: OK. Plan achieved. All AGVs reached goal. ');
                elseif(notFoundPaths==0 && agvSim.AGVsPlanAchived==0)
                    title('SIPP: PROBLEM. All paths computed but some AGVs did NOT reach goal. ');
                elseif(notFoundPaths>0 && agvSim.AGVsPlanAchived==1)
                    title('SIPP: CCA OK. Not all paths are found! Other vehicles have reched goal. (Vehicles without plan are not simulated.)');
                    fprintf(1,' \n For %d plannigs path was not found. \n',notFoundPaths);
                elseif(notFoundPaths>0 && agvSim.AGVsPlanAchived==1)
                    title('SIPP: PROBLEM. Not all paths are found! Some of the vehicles with plan have NOT reched goal. (Vehicles without plan are not simulated.)');
                    fprintf(1,' \n For %d plannigs path was not found. \n',notFoundPaths);
                end
            end
            
            
            tMks=max([taskSol((Ntasks-Nagv+1):Ntasks).tDrop]);
            
            PSIPPl=[tMks, Nsipps]
            PSIPPl_niOK=[t, Nsipps]
            
            
            
        end % function
        
        
        
        
        
        
        
        
        
    end
end