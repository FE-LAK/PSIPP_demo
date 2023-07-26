classdef AGVSim < handle
properties
    nodeID = [];   % map container of nodes structures
    roadID = [];   % map container of road structures 
  %  nodeID = []; roadID = [];  % array of structures
    allNodes=0; allRoads=0; % unique keys

    mainFig=10;
    hMainFig=[];
    hhh=[];
    
    hMap=[]; hArrow=[];

    AGV=[];  allAGV=0;   % naj bo AGV kar class in vse narišem
    
    DRAW=1; % enable animation  
    
    nodePoses=[];
    nodeLabels=[];

    drawNodeLabels=1;
    drawRoadLabels=1;
    
   % za ponorno vozlisce
   restNode=[]; 
   roadToRestNode=[]; 
   roadToRestNodeLoc=1; 
    
    
   roadLabels=[];
   roadLabelPos=[];
   roadPoints=[];
   
   AGVsPlanAchived=0;
   
   AGVLength=0.24; % polovicna dolzina AGV-ja za namen detekcije trka
   AGVWidth = 0.12; % polovicna sirina AGV-ja za namen detekcije trka
   AGVRadious =sqrt(0.24^2+0.12^2);
   
   preventCollision = 1; % ali vkljucim detekcijo trka v funkciji moveAGVs (drugje ni upoštevano)
   
   
   fontSize=60; % velikost oznacb na zemljevidu
   fontSizeAGV=12; % velikost oznacb na zemljevidu
  
   tabelResolution=20; %okvirna resolucija za tabeliranje cest, 30 razdelkov na enoto
   
   % razdalje in prepotovane razdalje AGV-jev (inicializira z zadnjim klicem setPlanAGV(.), logira se v moveAGVs(.), izrišeš z drawPlanAGVs(.))
   maxAGVRec=20; tp=0; dp=[]; kp=[]; TP=[]; DP=[]; KP=[];    % vzamem belezenje za prvih maxAGVRec=10 agv-jev maks
 
   
   recordMovie=0; writerMovieObj=[];
   
   % LUT tabelo (look up tabel) za simulacijo voznje
    LUt_dAGV={}; % Lookup tabela glede na cas in vrne prepotovano razdaljo AGV-ja
    LUd_rid={}; % Lookup tabela glede na razdaljo in vrne id poti
    LUd_loc={}; % Lookup tabela glede na razdaljo in vrne loc poti

   
end
    
methods
 %=====================================================================================    
    function obj = AGVSim(val)   % konstruktor
        if nargin == 1
            obj.mainFig = val;
        end

        obj.nodeID = containers.Map('KeyType','int64','ValueType','any');   % map container of nodes structures
        obj.roadID = containers.Map('KeyType','int64','ValueType','any');   % map container of road structures

        obj.hMainFig=figure(obj.mainFig); clf;
  %      set(obj.hMainFig, 'renderermode', 'manual', 'renderer', 'opengl'); %AZ
   %     obj.hMainFig.CloseRequestFcn=[]; % da se okno ne da zapret, nazaj popraviš ce daš na 'closereq'
        hold on;
        zoom on;
        title('AGV simulation');xlabel('x (m)','FontSize', 8);ylabel('y (m)','FontSize', 8);
        axis equal
      %  axis;
        obj.hMainFig.CurrentAxes.FontSize = 14;
        obj.hArrow=plot(nan,nan,'Color','r','LineWidth',1) ; % arrow
        hold off;    
        
        obj.intSimTime(); % da spremenljivke strukture za belezenje 
    end
%=====================================================================================    
    function delete(obj)    %destruktor
       if(~isempty(obj.mainFig)) % ?? ni najbolje
        %   delete(obj.mainFig);  % zapre glavno okno, ki ima prepreceno zapiranje obj.hMainFig.CloseRequestFcn=[];
       end
    end
%=====================================================================================    
function moveAGVs(obj, sampleTime) % premakne AGV-je,ki imajo plan, za en vzorec
 
        allAGVsWithPlan=0; allAGVsWithReachedPlan=0;
        
        for i=1:size(obj.AGV,2)     % grem cez vse AGVje 
            if(~isempty(obj.AGV(i).planRoad)) % jih premaknem le ce imajo plan
                allAGVsWithPlan=allAGVsWithPlan+1;
                if ~obj.AGV(i).wait% NEJC premaknem le ce niso v stanju cakanja
                    idxP=obj.AGV(i).planRoadIdx;  % kje na planiranem seznamu sem?? na zacetku ko AGV zgeneriran naj bo ta 1
                    if(obj.AGV(i).onRoadID~=obj.AGV(i).planRoad(idxP)), error('AGVSim::moveAGVs: planned and current node not consistent??'); end %??

                    idRoad=obj.AGV(i).onRoadID;

                    if(idxP< length(obj.AGV(i).planRoad)) % ni zadnja planirana pot
                        distRoadToGo=obj.roadID(idRoad).length*(1-obj.AGV(i).onRoadLoc) ; % preostanek poti za prevozit
                    else
                        if(obj.AGV(i).goalRoadLoc < obj.AGV(i).onRoadLoc)
                            error('AGVSim::moveAGVs: on final planned path onRoadLoc should not be more than goalRoadLoc');
                        else
                            distRoadToGo=obj.roadID(idRoad).length*(obj.AGV(i).goalRoadLoc - obj.AGV(i).onRoadLoc);
                        end
                    end
                    %%%%%%%%% NEJC -> dodano "(idxP)"
                    dd=obj.AGV(i).velocity(idxP)*sampleTime; % nameravana prevozena razdalja AGV-ja v sampleTimu
                    ddstart=dd;
         
                    
         % preveri test trka           
                    
             collision=obj.checkCollisionAGV(i,dd);  % preveri trk za i-tega AGV za razdaljo dd v smeri voznje        
    
              if(~collision)
                    while(dd>distRoadToGo && idxP< length(obj.AGV(i).planRoad)) % gremo na naslednjo pot v planu 
                        
                           idxP=idxP+1; % gremo na naslednjo planirano pot
                           endNode=obj.roadID(idRoad).eNode;
                           if(any(obj.nodeID(endNode).neighboarRoadIDs==obj.AGV(i).planRoad(idxP))) % je pot dosegljiva  
                               dd=dd-distRoadToGo;
                               idRoad=obj.AGV(i).planRoad(idxP);
                               obj.AGV(i).onRoadID=idRoad;  % AGV je na zacetku naslednje poti
                       %        obj.AGV(i).onRoadLoc=0; % ?? zakaj 0? - se nastavi kasneje (vrstica cca. 133)
                                                             
                               obj.AGV(i).planRoadIdx = idxP;
                               if(idxP < length(obj.AGV(i).planRoad)) % ni zadnja planirana pot
                                   
                                   if( dd < obj.roadID(idRoad).length )
                                    obj.AGV(i).onRoadLoc=dd/obj.roadID(idRoad).length; % ?? zakaj 0? - se nastavi kasneje (vrstica cca. 133)
                                   else
                                    obj.AGV(i).onRoadLoc=1;
                                   end
                                    distRoadToGo=obj.roadID(idRoad).length*(1-obj.AGV(i).onRoadLoc) ; % preostanek naslednje poti za prevozit
                                   % distRoadToGo= min(dd,obj.roadID(idRoad).length) ; %GK_19jan2022 popravek: preostanek naslednje poti za prevozit
                               else
                                   
                                   if( dd < obj.AGV(i).goalRoadLoc*obj.roadID(idRoad).length )
                                    obj.AGV(i).onRoadLoc=dd/obj.roadID(idRoad).length; % ?? zakaj 0? - se nastavi kasneje (vrstica cca. 133)
                                   else
                                    obj.AGV(i).onRoadLoc=obj.AGV(i).goalRoadLoc;
                                   end

                                   
                                    if(obj.AGV(i).goalRoadLoc >= obj.AGV(i).onRoadLoc)
                                        distRoadToGo=obj.roadID(idRoad).length*(obj.AGV(i).goalRoadLoc - obj.AGV(i).onRoadLoc);
                                        %distRoadToGo=obj.roadID(idRoad).length*(obj.AGV(i).goalRoadLoc); %GK_19jan2022 popravek: preostanek naslednje poti za prevozit
                                    else
                                        error('AGVSim::moveAGVs: on final planned path goalRoadLoc should not be more than onRoadLoc');
                                    end
                               end

                           else
                               error('AGVSim::moveAGVs: next road can not be reached from the current one, wrong AGV plan?');
                           end
                    end
              end
                    
                    
                    % dolocim nove lege AGV-jev po premiku
                    sampleDistance=0; 
                    if(i<=obj.maxAGVRec) obj.kp(i)=1; end  % za izris razdalj
    %                if(distSensorMin>distSafe)
                     if(~collision)
                        if(idxP == length(obj.AGV(i).planRoad) && dd>distRoadToGo ) % ce je zadnja in sem že blizu cilja
                                obj.AGV(i).onRoadLoc=obj.AGV(i).goalRoadLoc;
                                allAGVsWithReachedPlan=allAGVsWithReachedPlan+1;
                                obj.AGV(i).state=0; % prost
                                sampleDistance=distRoadToGo; % nova prepotovana razdalja
                        else
                               obj.AGV(i).onRoadLoc= obj.AGV(i).onRoadLoc + dd/obj.roadID(idRoad).length;
                               sampleDistance=ddstart;
                        end
                        obj.AGV(i).pose = obj.getPoseOnRoad(obj.AGV(i).onRoadID, obj.AGV(i).onRoadLoc);
                        if(i<=obj.maxAGVRec), obj.kp(i)=0; end
                    end
                    % nova prepotovana razdalja
                    if(i<=obj.maxAGVRec) obj.dp(i)=obj.dp(i)+sampleDistance; end
                end
            end
        end
        
        % izris analize plana
        obj.tp=obj.tp+sampleTime; % za izris voženj
        obj.TP=[obj.TP;obj.tp]; % cas
        obj.DP=[obj.DP;obj.dp]; % prepotovane razdalje AGV-jev
        obj.KP=[obj.KP;obj.kp];
        
        
        if(allAGVsWithReachedPlan==allAGVsWithPlan)
            obj.AGVsPlanAchived=1;
        else
            obj.AGVsPlanAchived=0;
        end
              
      
      if(obj.DRAW)  
        for i=[obj.AGV(:).id] %1:size(obj.AGV,2)     % grem cez vse AGVje 
            drawAGV(obj,i);           
        end      
      end
      
       
end

%=====================================================================================

function setRobotHalfLengthWidth(obj,L,W)
    obj.AGVLength =L; % dolzina AGV-ja za namen detekcije trka
    obj.AGVWidth  = W; % sirina AGV-ja za namen detekcije trka
    obj.AGVRadious=sqrt(L^2+W^2);
end   
%=====================================================================================
function simulateAGVsOld(obj, Ts, vMax, finTime,tSimDelay) %simulira AGV-je, ki imajo plan podan v 'planRoadTimeSinc'

    obj.intSimTime(); % inicializiram cas
    t=0;  % cas simulacije
    
    % zgradi tabelo preslikav (casi->indekse tabele) za izvajanje posameznih akcij
    allAGVsWithPlan=0; 
    allAGVsWithReachedPlan=0;
    
    idxAllAGV=[]; % shranim agv-je z indeksi
    for i=1:size(obj.AGV,2)     % grem cez vse AGVje in prestejem koliko jih ima plane 
        if(~isempty(obj.AGV(i).planRoadTimeSinc))
            if ( obj.AGV(i).planRoadTimeSinc(1,2)~=obj.AGV(i).onRoadID )
                error('AGVSim::simulateAGVs: First road in plan is not the same as current road of AGV?');
            end
            allAGVsWithPlan=allAGVsWithPlan+1;  
            idxAllAGV=[idxAllAGV,i];
        end            
    end

    dAGV=zeros(allAGVsWithPlan,1); % prevozene razdalje AGV-jev
    p=ones(allAGVsWithPlan,1); % pointerji AGV-jev na tabele
    pMax=p;   % maximumi akcij v tabeli oz. dimenzija tabele akcij
    P=cell(allAGVsWithPlan,1); % cell array planov P za lazji dostop po vrsticah: cas, roadID,roadLoc, komulativna razdalja
    for j=1:allAGVsWithPlan          
       i=idxAllAGV(j);
       pMax(j)=size(obj.AGV(i).planRoadTimeSinc,1);
       P{j}=obj.AGV(i).planRoadTimeSinc;
    end
    
    
    while(allAGVsWithReachedPlan~=allAGVsWithPlan && t < finTime)
     for i=1:allAGVsWithPlan          % grem cez vse AGVje 
      if(dAGV(i)<P{i}(pMax(i),4))   % ce se ni prisel do cilja
      
        a=idxAllAGV(i);   % a = indeks na AGV, i=indeks na plan (npr laho je prvi plan podan z 2. robota)
        while ( (P{i}(p(i),1)-t) < Ts && p(i)<pMax(i)) % povecam indeks
                p(i)=p(i)+1;
        end
          
        dt= P{i}(p(i),1)-t;
        dd= P{i}(p(i),4)-dAGV(i);
        if(dt>1e-8)
            v= dd/dt;
        else
          % dt= P{1}(pMax(i),1)-P{1}(1,1);  %???
          % dd= P{i}(pMax(i),4)-P{1}(1,4);
           dt= P{i}(pMax(i),1)-P{i}(1,1); % ?? GK: popravek 23.3.2022
           dd= P{i}(pMax(i),4)-P{i}(1,4);
            
           if(dt>0.0001)
             v= dd/dt;% neka povprecna hitrost
           else
             v=vMax*0.5;  % nekaj nastavim
           end
        end
        
        if(v>vMax),
            v=vMax; 
        end
        
        dAGVinc=v*Ts;
        % preveri test trka           
        collision=obj.checkCollisionAGV(a,dAGVinc);  % preveri trk za a-tega AGV za razdaljo dd v smeri voznje        
        
     if(~collision)        
         dAGV(i)=dAGV(i)+dAGVinc;    
   
         d2=P{i}(p(i),4);
         if p(i)>1
            d1=P{i}(p(i)-1,4);
            
            %  if(d1<=dAGV(i)&& dAGV(i)<=d2)
              if( (dAGV(i)-d1)>-1e-14 && (d2-dAGV(i))>-1e-14 )
                 if(P{i}(p(i)-1,2)==P{i}(p(i),2)) % akciji sta na isti poti?
                    loc1=P{i}(p(i)-1,3);
                    L1=obj.roadID(P{i}(p(i)-1,2)).length;
                    loc=(dAGV(i)-d1)/L1 + loc1;
                    rid=P{i}(p(i)-1,2);
                 else % akciji sta na razlicnih poteh
                     loc1=P{i}(p(i)-1,3); 
                     L1=obj.roadID(P{i}(p(i)-1,2)).length;
                     L2=obj.roadID(P{i}(p(i),2)).length;
                     if( dAGV(i)<=(d1+(1-loc1)*L1) ) % agv je se na 1. poti?
                        loc=(dAGV(i)-d1)/L1 + loc1;
                      
             
                       
                       rid=P{i}(p(i)-1,2);
                     else
                        loc=(dAGV(i)-d1-(1-loc1)*L1)/L2 ; 
                        rid=P{i}(p(i),2);
                     end
                 end
              elseif (d1-dAGV(i))>-1e-14 % se zmeraj je pred prvo ozna?bo ??
                   loc1=P{i}(p(i)-1,3); 
                   L1=obj.roadID(P{i}(p(i)-1,2)).length;
                   loc=loc1-(d1-dAGV(i))/L1 ; 
                   rid=P{i}(p(i),2);
              else
                 cudno=1
              end
           
            
            
         else
            L1=obj.roadID(P{i}(p(i),2)).length; 
            loc1=P{i}(p(i),3);
            loc= loc1-(d2-dAGV(i))/L1;
            rid=P{i}(p(i),2);
         end
        
         
        
         % shranim lokacijo AGV-ja
         obj.AGV(a).onRoadID=rid;
         if(dAGV(i)>= P{i}(pMax(i),4)) % ce prisel do konca
                obj.AGV(a).onRoadLoc=P{i}(pMax(i),3);
                allAGVsWithReachedPlan=allAGVsWithReachedPlan+1;
                obj.AGV(a).state=0; % prost
                dAGV(i)= P{i}(pMax(i),4);
         else
               obj.AGV(a).onRoadLoc= loc;
         end
  if(obj.AGV(a).onRoadLoc<0)
      g=1
  end
  if(length(obj.AGV(a).onRoadLoc)>1)
      g=1
  end
         
         obj.AGV(a).pose = obj.getPoseOnRoad(obj.AGV(a).onRoadID, obj.AGV(a).onRoadLoc);
         % sharani % nova prepotovana razdalja in trk 
         if(i<=obj.maxAGVRec) obj.dp(a)=dAGV(i); end
         if(i<=obj.maxAGVRec), obj.kp(a)=0; end % ni trka

     else
        if(i<=obj.maxAGVRec) obj.dp(a)=obj.dp(i); end 
        if(i<=obj.maxAGVRec) obj.kp(a)=1; end  % za izris razdalj
     end % if(~collision)
        
      end     
     end
      

        t=t+Ts;
    
        % izris analize plana
        obj.tp=t; % za izris voženj
        obj.TP=[obj.TP;obj.tp]; % cas
        obj.DP=[obj.DP;obj.dp]; % prepotovane razdalje AGV-jev
        obj.KP=[obj.KP;obj.kp]; % trki robotov
    
      
      
      if(obj.DRAW)  
        for i=[obj.AGV(:).id] %1:size(obj.AGV,2)     % grem cez vse AGVje 
            obj.drawAGV(i);  
            pause(tSimDelay);
          %  pause
        end      
      end


      
    end

    
    
    if(allAGVsWithReachedPlan==allAGVsWithPlan)
        obj.AGVsPlanAchived=1;
    else
        obj.AGVsPlanAchived=0;
    end
   
        
end
%=====================================================================================
 function simulateAGVs(obj, Ts, vMax, finTime,tSimDelay) %simulira AGV-je, ki imajo plan podan v 'planRoadTimeSinc'

    obj.intSimTime(); % inicializiram cas
    t=0;  % cas simulacije
    
    % zgradi tabelo preslikav (casi->indekse tabele) za izvajanje posameznih akcij
    allAGVsWithPlan=0; 
    allAGVsWithReachedPlan=0;
    
    idxAllAGV=[]; % shranim agv-je z indeksi
    for i=1:size(obj.AGV,2)     % grem cez vse AGVje in prestejem koliko jih ima plane 
        if(~isempty(obj.AGV(i).planRoadTimeSinc))
            if ( obj.AGV(i).planRoadTimeSinc(1,2)~=obj.AGV(i).onRoadID )
                error('AGVSim::simulateAGVs: First road in plan is not the same as current road of AGV?');
            end
            allAGVsWithPlan=allAGVsWithPlan+1;  
            idxAllAGV=[idxAllAGV,i];
        end            
    end

    dAGV=zeros(allAGVsWithPlan,1); % prevozene razdalje AGV-jev
    pMax=ones(allAGVsWithPlan,1);   % maximumi akcij v tabeli oz. dimenzija tabele akcij
    P=cell(allAGVsWithPlan,1); % cell array planov P za lazji dostop po vrsticah: cas, roadID,roadLoc, komulativna razdalja
    
    LUt_dAGV=cell(allAGVsWithPlan,1); % Lookup tabela glede na cas in vrne prepotovano razdaljo AGV-ja

    LUd_rid=cell(allAGVsWithPlan,1); % Lookup tabela glede na razdaljo in vrne id poti
    LUd_loc=cell(allAGVsWithPlan,1); % Lookup tabela glede na razdaljo in vrne loc poti
   
    %%%% sestava tabel za voznjo
    for j=1:allAGVsWithPlan          
       a=idxAllAGV(j);
       pMax(j)=size(obj.AGV(a).planRoadTimeSinc,1);
       P{j}=obj.AGV(a).planRoadTimeSinc;  % plan
       
       LUt_dAGV{j}=[[0; P{j}(:,1)],[0;P{j}(:,4)]];  % Lookup tabela glede na cas in prepotovano razdaljo AGV-ja

       LUd_rid{j}=[0,obj.AGV(a).onRoadID];  % Lookup tabela glede na prepotovano razdaljo in vrne id poti
       LUd_loc{j}=[0,obj.AGV(a).onRoadLoc];  % Lookup tabela glede na prepotovano razdaljo in vrne loc poti

       dCumCest=(1-obj.AGV(a).onRoadLoc)*obj.roadID(obj.AGV(a).onRoadID).length;
       
       iii = 1; %AZ
       for ak=1:pMax(j)  % grem cez vse akcije
           
         if(P{j}(ak,4)>LUd_rid{j}(end,1)) % naslednja razdalja je vecja
           if(P{j}(ak,2)~=LUd_rid{j}(end,2)) % gre na drugo cesto zato vstavimo konec prejsnje in zacetek nove ceste v tabelo
               % gre za nezvezno funkcijo, kjer pri istem d-ju imam preskok
               % vrednosti ,resim z dodajanjem 1e-14... Pri animaciji lahko
               % vozilo trzne, ?e pride ravno v ta interval... To rešim
               % kasneje z detekcijo tega.
              % LUd_rid{j}=[LUd_rid{j}; dCumCest, LUd_rid{j}(end,2);  dCumCest+1e-14, P{j}(ak,2)  ]; 
              % LUd_loc{j}=[LUd_loc{j}; dCumCest, 1 ; dCumCest+1e-14, 0 ]; 
               
         LUd_rid{j}=[LUd_rid{j};  dCumCest, P{j}(ak,2)  ];  %AZ
         LUd_loc{j}=[LUd_loc{j};  dCumCest, iii ];  %AZ
         iii = iii + 1; %AZ

               
               
               
               rid=LUd_rid{j}(end,2);
               dCumCest = dCumCest+ obj.roadID(rid).length;
           end
         end
       end
       % dodam se zadnje akcije
       LUd_rid{j}=[LUd_rid{j}; dCumCest, P{j}(ak,2) ]; 
      % LUd_loc{j}=[LUd_loc{j}; dCumCest, 1 ];
       LUd_loc{j}=[LUd_loc{j}; dCumCest, iii ]; %AZ
    end

     
    %%%% simulacija vozil
    t=Ts;
    
    while(allAGVsWithReachedPlan~=allAGVsWithPlan && t < finTime)
     for i=1:allAGVsWithPlan        % grem cez vse AGVje 
      if(dAGV(i)<P{i}(pMax(i),4))   % ce se ni prisel do cilja
      
        a=idxAllAGV(i);   % a = indeks na AGV, i=indeks na plan (npr. lahko je prvi plan podan z 2. robota)
        
        dAGVNew=interp1(LUt_dAGV{i}(:,1),LUt_dAGV{i}(:,2),t,'linear','extrap');  % dAGV=prepotovana razdalja  (tabela dAGV=fun(t))
        if(dAGVNew>LUd_rid{i}(end,1)), dAGVNew= LUt_dAGV{i}(end,2); end

        v=(dAGVNew-dAGV(i))/Ts;
        
        if(v>vMax)
            v=vMax; 
        end

        dAGVinc=v*Ts;   % nameravan premik

        % preveri test trka           
        %collision=obj.checkCollisionAGV(a,dAGVinc);  % preveri trk za a-tega AGV za razdaljo dd v smeri voznje        
        
      %--- 18.4.2023: novo za preverit trk, dolocim predikcijo lokacije vzdolz poti ppNext     
        dAGVTmp=dAGV(i)+dAGVinc;   
        if(find( abs( dAGVTmp- LUd_rid{i}(:,1))<1e-14)) % ali sem zelo blizu nezveznega intervala ali v njem
            dAGVSample=dAGVTmp+2*1e-14; % da preskocim nezvezni interval in se izognem problemom hipnih skokov agv-ja pri animaciji
        else
            dAGVSample=dAGVTmp;
        end
        rid=interp1(LUd_rid{i}(:,1),LUd_rid{i}(:,2),dAGVSample,'previous','extrap');  
        % loc=interp1(LUd_loc{i}(:,1),LUd_loc{i}(:,2),dAGVSample,'linear');  
        loc=interp1(LUd_loc{i}(:,1),LUd_loc{i}(:,2),dAGVSample,'linear'); loc = loc - floor(loc); %AZ
        
        if(dAGVTmp>= P{i}(pMax(i),4)) % ce prisel do konca
                loc=P{i}(pMax(i),3);
                dAGVTmp= P{i}(pMax(i),4);
        end  
        ppNext = obj.getPoseOnRoad(rid, loc);     
        collision=obj.checkCollisionAGV2(a,dAGVinc, ppNext);  % preveri trk za a-tega AGV za razdaljo dd v smeri voznje        

    
      %------ dodano da ignorira trke v ponornem vozliscu   
     if(~isempty(obj.roadToRestNode) && obj.AGV(i).onRoadID == obj.roadToRestNode )
          % zelo poenostavim in kar recem, da se na cesti do ponora ignorira trke 
       %   if(obj.AGV(i).onRoadLoc > obj.roadToRestNodeLoc)
            collision=0;
       %   end
     end

        
        
        
        
     if(~collision || obj.preventCollision==2)        
          
%       if 0   
%         dAGV(i)=dAGV(i)+dAGVinc; 
%         %GK dodano 22maj2022 : reševanje preskokov pri animaciji zaradi
%         %aproksimacije nezveznih preskokov (lege in id-ja ceste) in interpolacije z interp1
%         if(find( abs( dAGV(i)- LUd_rid{i}(:,1))<1e-14)) % ali sem zelo blizu nezveznega intervala ali v njem
%             dAGVSample=dAGV(i)+2*1e-14; % da preskocim nezvezni interval in se izognem problemom hipnih skokov agv-ja pri animaciji
%         else
%             dAGVSample=dAGV(i);
%         end
%         rid=interp1(LUd_rid{i}(:,1),LUd_rid{i}(:,2),dAGVSample,'previous','extrap');  
%        % loc=interp1(LUd_loc{i}(:,1),LUd_loc{i}(:,2),dAGVSample,'linear');  
%         loc=interp1(LUd_loc{i}(:,1),LUd_loc{i}(:,2),dAGVSample,'linear'); loc = loc - floor(loc); %AZ
% 
%         obj.AGV(a).onRoadLoc = loc;
%         obj.AGV(a).onRoadID  = rid;
%         
%          if(dAGV(i)>= P{i}(pMax(i),4)) % ce prisel do konca
%                 obj.AGV(a).onRoadLoc=P{i}(pMax(i),3);
%                 allAGVsWithReachedPlan=allAGVsWithReachedPlan+1;
%                 obj.AGV(a).state=0; % prost
%                 dAGV(i)= P{i}(pMax(i),4);
%          end
%          
%          obj.AGV(a).pose = obj.getPoseOnRoad(obj.AGV(a).onRoadID, obj.AGV(a).onRoadLoc);
% 
%      else
     %--- 18.4.2023: novo za preverit trk, dolocim predikcijo lokacije vzdolz poti ppNext 
         dAGV(i)= dAGVTmp;
         obj.AGV(a).onRoadLoc = loc;
         obj.AGV(a).onRoadID  = rid;
        
         if(dAGV(i)>= P{i}(pMax(i),4)) % ce prisel do konca
               % obj.AGV(a).onRoadLoc=P{i}(pMax(i),3);
                allAGVsWithReachedPlan=allAGVsWithReachedPlan+1;
                obj.AGV(a).state=0; % prost
              %  dAGV(i)= P{i}(pMax(i),4);
         end

         obj.AGV(a).pose = ppNext;
%      end
     
     
         % shrani % nova prepotovana razdalja in trk 
         if(i<=obj.maxAGVRec)  obj.dp(a)=dAGV(i); end
         %if(i<=obj.maxAGVRec), obj.kp(a)=0; end % ni trka
         
         if( obj.preventCollision==2 && collision )  % trke le zaznavam a ne ustavljam vozil
            if(i<=obj.maxAGVRec), obj.kp(a)=1; end % je trk a ga le zabeležimo vozilo pa vseeno premaknemo
         else
            if(i<=obj.maxAGVRec), obj.kp(a)=0; end % ni trka
         end
       
         
         
         
         

     else
 %        if(i<=obj.maxAGVRec) obj.dp(a)=obj.dp(i); end   %?? sumljivo
        if(i<=obj.maxAGVRec) obj.dp(a)=obj.dp(a); end   %?? sumljivo
        if(i<=obj.maxAGVRec) obj.kp(a)=1; end  % za izris razdalj
     end % if(~collision)
        
   end     
  end
      

   %     t=t+Ts;
    
        % izris analize plana
        obj.tp=t; % za izris voženj
        obj.TP=[obj.TP;obj.tp]; % cas
        obj.DP=[obj.DP;obj.dp]; % prepotovane razdalje AGV-jev
        obj.KP=[obj.KP;obj.kp]; % trki robotov
    
      
      
       if(obj.DRAW)  
         for i=[obj.AGV(:).id] %1:size(obj.AGV,2)     % grem cez vse AGVje 
            obj.drawAGV(i);  
           % if(tSimDelay>0)
          %     pause(tSimDelay);
           % end
           % drawnow
          %  pause
         end 
             pause(tSimDelay);  % GK 6maj2022:  prestavil pause
       end

       
        % snemanje filma ??
        if obj.recordMovie
            if (t==Ts) 
                %% Set up the movie.
                %axis([0 6 0 4]+[-1 1 -1 1]*0.5)
                %axis([0 14 5 15]+[-1 1 -1 1]*0.5)
                obj.writerMovieObj = VideoWriter('MovieOut.avi'); % Name it.
                obj.writerMovieObj.FrameRate = 30; % How many frames per second.
                open(obj.writerMovieObj);
            else
                frame = getframe(gcf); % 'gcf' can handle if you zoom in to take a movie.
                writeVideo(obj.writerMovieObj, frame);
            end % 
        end
   
      
        t=t+Ts;
        
    end % while

    
    % snemanje filma ??
    if obj.recordMovie
        close(obj.writerMovieObj); % Saves the movie.             
    end
    
    if(allAGVsWithReachedPlan==allAGVsWithPlan)
        obj.AGVsPlanAchived=1;
    else
        obj.AGVsPlanAchived=0;
    end
   
        
end
%=====================================================================================
 



function drawPlanAGVs(obj)
       
       N=min(size(obj.AGV,2),obj.maxAGVRec);
       
        for i = 1:N
            legendCell{i} = num2str(obj.AGV(i).id,'AGV %-d');
        end
        
        newDefaultColors = lines(obj.maxAGVRec);       
       
        figure(1), 
        subplot(2,1,1),
        set(gca,'ColorOrder', newDefaultColors, 'NextPlot', 'replacechildren');
        plot(obj.TP,obj.DP); %xlabel('t[s]'), 
        title('Traveled distance to goal for AGVs.')
       % legend(legendCell)
        xlabel('$$t$$[s]', 'interpreter', 'latex', 'FontSize', 14);  ylabel('$$d$$[m]', 'interpreter', 'latex', 'FontSize', 14);

        subplot(2,1,2),
        set(gca,'ColorOrder', newDefaultColors, 'NextPlot', 'replacechildren');      
        stairs(obj.TP,obj.KP,'linewidth',2); xlabel('t[s]'), 
        title('Delays due to collision prevention.')
        xlabel('$$t$$[s]', 'interpreter', 'latex', 'FontSize', 14);  %ylabel('$$conflict$$[]', 'interpreter', 'latex', 'FontSize', 14);
        legend(legendCell)
        
        
%         figure(2)
%         set(gca,'ColorOrder', newDefaultColors, 'NextPlot', 'replacechildren');
%         plot(obj.TP,obj.DP);legend(legendCell)
%         xlabel('$$t$$[s]', 'interpreter', 'latex', 'FontSize', 14);  ylabel('$$d$$[m]', 'interpreter', 'latex', 'FontSize', 14);
       % print -depsc  zaporedne3TimePlan;   
       % print -depsc  sestavljeneTimePlan;   
       % print -depsc  slepaUlica;   
       % print -depsc  lepjenjeSestavTimePlan;   

        
        if obj.AGVsPlanAchived
            disp('Plan achieved. All AGVs reached goal.');
        else
            disp('Plan NOT achieved. Some AGVs did not reach goal.');
        end
        
   end
%=====================================================================================

function collision=checkCollisionAGV(obj,i,dd)   % za i-ti AGV preveri trk z vsemi drugimi za razdaljo dd naprej v smeri voznje
    
   collision=0;
 
    pp=obj.AGV(i).pose + dd*[cos(obj.AGV(i).pose(3));sin(obj.AGV(i).pose(3));0]; % moja nova poza 
    
    % pazi da premikas vzdolz planiranih poti - mogo?e raje vzami iz LUT tabele 
    
    
    
    if obj.preventCollision
        for j=1:size(obj.AGV,2)
            if j==i, continue; end 
            
            eee=obj.AGV(j).pose-pp;
            
%            distSafe=(obj.AGV(i).AGVRadious+ obj.AGV(j).AGVRadious)^2; %vsota radijev vozil na kvadrat, da ni potrebno korenit

            % prilagoditev cca.: da je manj obcutlivo na trk od drugega jemljem le širino ali višino glede na medsebojni kot           
              dimDrugi=abs(obj.AGV(j).AGVWidth*sin(eee(3))) + abs(obj.AGV(j).AGVLength*cos(eee(3)));
              distSafe=(obj.AGV(i).AGVRadious + dimDrugi )^2; % vsota radijev vozil na kvadrat 

            dsx=eee(1);dsy=eee(2);
            distSensor=dsx^2+dsy^2;
            angleSensor=abs(wrapToPi(pp(3)-atan2(dsy,dsx)));  % je robot pred mano?
            if(distSensor<distSafe && angleSensor<pi/4)
                collision=1; 
                break;  
            end
        end
    end
                      
end
%=====================================================================================

function collision=checkCollisionAGV2(obj,i,dInc,ppNext)   % za i-ti AGV preveri trk z vsemi drugimi za razdaljo dd naprej v smeri voznje
    
   collision=0;
 
  % flagOriginal=0;
   
 %  if flagOriginal==1
 %       pp=obj.AGV(i).pose + 1*dInc*[cos(obj.AGV(i).pose(3));sin(obj.AGV(i).pose(3));0]; % moja nova poza 
 %  else
       % dolocim naslednj lego robota, ki je ok tudi ce menja ceste pod
        pp = ppNext;
 %  end

    
    % pazi da premikas vzdolz planiranih poti - mogo?e raje vzami iz LUT tabele 
    
 
    if obj.preventCollision
        for j=1:size(obj.AGV,2)
            if j==i, continue; end 
            
            eee=obj.AGV(j).pose-pp;
            
%            distSafe=(obj.AGV(i).AGVRadious+ obj.AGV(j).AGVRadious)^2; %vsota radijev vozil na kvadrat, da ni potrebno korenit

            % prilagoditev cca.: da je manj obcutlivo na trk od drugega jemljem le širino ali višino glede na medsebojni kot           
              dimDrugi=abs(obj.AGV(j).AGVWidth*sin(eee(3))) + abs(obj.AGV(j).AGVLength*cos(eee(3)));
        %      distSafe=(obj.AGV(i).AGVRadious + dimDrugi )^2; % vsota radijev vozil na kvadrat 
              distSafe=(obj.AGV(i).AGVLength + dimDrugi )^2; % vsota dimenzij vozil na kvadrat 

            dsx=eee(1);dsy=eee(2);
            distSensor=dsx^2+dsy^2;
            angleSensor=abs(wrapToPi(pp(3)-atan2(dsy,dsx)));  % je robot pred mano?
            if(distSensor<distSafe && angleSensor<pi/4)
                collision=1; 
                break;  
            end
        end
    end
                      
end
%=====================================================================================
    function setPlanAGV(obj,id, planRoad,velocity, goalRoadLoc)
        if(obj.allAGV>0)
            r=[obj.AGV(:).id]; i=find(r==id,1); % najdemo pravo strukturo z iskanim id-jem
            if(isempty(i)),disp('setPlanAGV:??? AGV ne obstaja??'), return; end
        end
        if (~all( isKey(obj.roadID,num2cell(planRoad)) ))    % ali so vse poti s pravimi id-ji?
           error('AGVSim::setPlanAGV: no such road id ?'); 
        end
        if (~isempty(planRoad))
            if ( planRoad(1)~=obj.AGV(i).onRoadID )
                error('AGVSim::setPlanAGV: First road in plan is not the same as current road of AGV?');
            end
        end
        
        obj.AGVsPlanAchived=0; %
        
        obj.AGV(i).planRoad=planRoad;
        obj.AGV(i).planRoadIdx=1;     % prvi index na planRoad, prva planirana pot se mora ujemat z dejansko
        obj.AGV(i).velocity=velocity;
        obj.AGV(i).goalRoadLoc=goalRoadLoc;     
        obj.AGV(i).state = 1; % je zaseden    
    end
%=====================================================================================    
    function setStateAGV(obj,agvID, state)  % nastavis poljuben state AGV-ju
        if(obj.allAGV>0)
            r=[obj.AGV(:).id]; i=find(r==agvID,1); % najdemo pravo strukturo z iskanim id-jem
            if(isempty(i)),disp('setPlanRoadTimeSinc:??? AGV does not exist??'), return; end
        end
        obj.AGV(i).state = state; % 1-je zaseden, 0=prost, ostalo po izbiri npr -1 grem v polnilnico 
    end
%=====================================================================================    
    function setPlanRoadTimeSinc(obj,agvID, timePlan)  % 2feb2022, GK: dodan plan akcij podan s casovnimi trenutki
        % timePlan=[Time1, roadID1, roadLoc1; Time2, roadID2, roadLoc2; .... ]
        
        tPlan=zeros(size(timePlan,1),4);
        
        if(obj.allAGV>0)
            r=[obj.AGV(:).id]; i=find(r==agvID,1); % najdemo pravo strukturo z iskanim id-jem
            if(isempty(i)),disp('setPlanRoadTimeSinc:??? AGV does not exist??'), return; end
        end
        if (~all( isKey(obj.roadID,num2cell(timePlan(:,2))) ))    % ali so vse poti s pravimi id-ji?
           error('AGVSim::setPlanRoadTimeSinc: no such road id ?'); 
        end
        if (~isempty(timePlan))
            if ( timePlan(1,2)~=obj.AGV(i).onRoadID )
                error('AGVSim::setPlanRoadTimeSinc: First road in plan is not the same as current road of AGV?');
            end
            if ( timePlan(1,2)==obj.AGV(i).onRoadID && obj.AGV(i).onRoadLoc>timePlan(1,3))
                error('AGVSim::setPlanRoadTimeSinc: AGV location of the first road in plan need to be less than the location in the plan');
            end
        else
            return;
        end
        
        % preveri ce je plan pravilen in dodaj komulativno razdaljo kot 4.parameter
        
        d= (timePlan(1,3)-obj.AGV(i).onRoadLoc)* obj.roadID(timePlan(1,2)).length;
        tPlan(1,:)=[timePlan(1,:), d];
        
        for j=1:(size(timePlan,1)-1)
             idRoad= timePlan(j,2); 
             endNode=obj.roadID(idRoad).eNode;
             if(idRoad~=timePlan(j+1,2) && (~any(obj.nodeID(endNode).neighboarRoadIDs==timePlan(j+1,2))) ) % je nasledna pot dosegljiva  
                error('AGVSim::setPlanRoadTimeSinc: next road can not be reached from the current one, wrong AGV plan?');
             elseif (idRoad==timePlan(j+1,2) && timePlan(j+1,3)< timePlan(j,3) )
                 error('AGVSim::setPlanRoadTimeSinc: plan wrong - it can not move in reverse');
             end
             
             if( timePlan(j+1)>=timePlan(j,1) )
             else
               error('AGVSim::setPlanRoadTimeSinc: time in table need to increase monotonically');    
             end
             
             if(timePlan(j,2)==timePlan(j+1,2) ) % ostane na isti poti
                d1=(timePlan(j+1,3)-timePlan(j,3))* obj.roadID(timePlan(j+1,2)).length;
                d=d+d1;
             elseif(timePlan(j,2)~=timePlan(j+1,2) ) % gre na drugo pot
                d1=(1-timePlan(j,3))* obj.roadID(timePlan(j,2)).length; % preostanek poti na prejsnji poti
                d2=(timePlan(j+1,3))* obj.roadID(timePlan(j+1,2)).length; % razdalja na drugi poti
                d=d+d1+d2;
             end
             tPlan(j+1,:)=[timePlan(j+1,:), d];
        end
        
        obj.AGVsPlanAchived=0; %
        obj.AGV(i).planRoadTimeSinc=tPlan;
        obj.AGV(i).state = 1; % je zaseden    
        
        
        %if tPlan(end,3)<1
            obj.AGV(i).endRoadLoc=tPlan(end,3);
        %end
    end
%=====================================================================================    
%=====================================================================================    

    function setPlanRoadTimeSincLUT(obj,agvID, timePlan,replanTime)  % 16okt2022, GK: dodal se LUT tabele
        % timePlan=[Time1, roadID1, roadLoc1; Time2, roadID2, roadLoc2; .... ]
        
        tPlan=zeros(size(timePlan,1),4);
        
        if(obj.allAGV>0)
            r=[obj.AGV(:).id]; i=find(r==agvID,1); % najdemo pravo strukturo z iskanim id-jem
            if(isempty(i)),disp('setPlanRoadTimeSinc:??? AGV does not exist??'), return; end
        end
        if (~all( isKey(obj.roadID,num2cell(timePlan(:,2))) ))    % ali so vse poti s pravimi id-ji?
           error('AGVSim::setPlanRoadTimeSinc: no such road id ?'); 
        end
        if (~isempty(timePlan))
            if ( timePlan(1,2)~=obj.AGV(i).onRoadID )
                error('AGVSim::setPlanRoadTimeSinc: First road in plan is not the same as current road of AGV?');
            end
            if ( timePlan(1,2)==obj.AGV(i).onRoadID && obj.AGV(i).onRoadLoc>timePlan(1,3))
                error('AGVSim::setPlanRoadTimeSinc: AGV location of the first road in plan need to be less than the location in the plan');
            end
        else
            return;
        end
        
        % preveri ce je plan pravilen in dodaj komulativno razdaljo kot 4.parameter
        d= (timePlan(1,3)-obj.AGV(i).onRoadLoc)* obj.roadID(timePlan(1,2)).length;
        tPlan(1,:)=[timePlan(1,:), d];
        
        for j=1:(size(timePlan,1)-1)
             idRoad= timePlan(j,2); 
             endNode=obj.roadID(idRoad).eNode;
             if(idRoad~=timePlan(j+1,2) && (~any(obj.nodeID(endNode).neighboarRoadIDs==timePlan(j+1,2))) ) % je nasledna pot dosegljiva  
                error('AGVSim::setPlanRoadTimeSinc: next road can not be reached from the current one, wrong AGV plan?');
             elseif (idRoad==timePlan(j+1,2) && timePlan(j+1,3)< timePlan(j,3) )
                 error('AGVSim::setPlanRoadTimeSinc: plan wrong - it can not move in reverse');
             end
             
             if( timePlan(j+1)>=timePlan(j,1) )
             else
               error('AGVSim::setPlanRoadTimeSinc: time in table need to increase monotonically');    
             end
             
             if(timePlan(j,2)==timePlan(j+1,2) ) % ostane na isti poti
                d1=(timePlan(j+1,3)-timePlan(j,3))* obj.roadID(timePlan(j+1,2)).length;
                d=d+d1;
             elseif(timePlan(j,2)~=timePlan(j+1,2) ) % gre na drugo pot
                d1=(1-timePlan(j,3))* obj.roadID(timePlan(j,2)).length; % preostanek poti na prejsnji poti
                d2=(timePlan(j+1,3))* obj.roadID(timePlan(j+1,2)).length; % razdalja na drugi poti
                d=d+d1+d2;
             end
             tPlan(j+1,:)=[timePlan(j+1,:), d];
        end
        
        obj.AGVsPlanAchived=0; %
        obj.AGV(i).planRoadTimeSinc=tPlan;
        obj.AGV(i).state = 1; % je zaseden    
        
        obj.AGV(i).endRoadLoc=tPlan(end,3);
        
     % 16okt2022: se LUT tabele za simulacijo poti
      %%%% sestava tabel za voznjo   
   %  for j=1:allAGVsWithPlan          
   %    a=idxAllAGV(j);
       a=i; 
       j=i;
       
       pMax=size(obj.AGV(a).planRoadTimeSinc,1);
       P=obj.AGV(a).planRoadTimeSinc;  % plan
       
       obj.LUt_dAGV{j}=[[replanTime; P(:,1)],[0;P(:,4)]];  % Lookup tabela glede na cas in prepotovano razdaljo AGV-ja

       obj.LUd_rid{j}=[0,obj.AGV(a).onRoadID];  % Lookup tabela glede na prepotovano razdaljo in vrne id poti
       obj.LUd_loc{j}=[0,obj.AGV(a).onRoadLoc]; % Lookup tabela glede na prepotovano razdaljo in vrne loc poti

       dCumCest=(1-obj.AGV(a).onRoadLoc)*obj.roadID(obj.AGV(a).onRoadID).length;
       
       iii = 1; %AZ
       for ak=1:pMax  % grem cez vse akcije
           
         if( P(ak,4)>obj.LUd_rid{j}(end,1) ) % naslednja razdalja je vecja
           if(P(ak,2)~=obj.LUd_rid{j}(end,2)) % gre na drugo cesto zato vstavimo konec prejsnje in zacetek nove ceste v tabelo
               % gre za nezvezno funkcijo, kjer pri istem d-ju imam preskok
               % vrednosti ,resim z dodajanjem 1e-14... Pri animaciji lahko
               % vozilo trzne, ce pride ravno v ta interval... To rešim
               % kasneje z detekcijo tega.
       %       obj.LUd_rid{j}=[obj.LUd_rid{j}; dCumCest, obj.LUd_rid{j}(end,2);  dCumCest+1e-14, P(ak,2)  ]; 
       %       obj.LUd_loc{j}=[obj.LUd_loc{j}; dCumCest, 1 ; dCumCest+1e-14, 0 ]; 
               
 

         obj.LUd_rid{j}=[obj.LUd_rid{j};  dCumCest, P(ak,2)  ];  %AZ
         obj.LUd_loc{j}=[obj.LUd_loc{j};  dCumCest, iii ];  %AZ
         iii = iii + 1; %AZ


               rid=obj.LUd_rid{j}(end,2);
               dCumCest = dCumCest+ obj.roadID(rid).length;
           end
         end
       end
       % dodam se zadnje akcije
       obj.LUd_rid{j}=[obj.LUd_rid{j}; dCumCest, P(ak,2) ]; 
   %    obj.LUd_loc{j}=[obj.LUd_loc{j}; dCumCest, 1 ]; 
       obj.LUd_loc{j}=[obj.LUd_loc{j}; dCumCest, iii ]; %AZ
 
   %   end   
   % 
    
%    % ?? kdaj imam se vedno dvojnike v razdalji
%     d=obj.LUd_rid{j}(:,1);
%     if(~isempty( find(diff(d)==0) ) ) % ce so dvojniki
%         d = d + (0:size(d,1)-1)'*1e-14;
%        
%        obj.LUd_rid{j}(:,1)=d; 
%        obj.LUd_loc{j}(:,1)=d;
% 
%        g=1
%     end
    

%     if( min(diff(obj.LUd_rid{j}(:,1))) <1e-14 )
%         
%         g_LUTtabela=1
%         
%  %       obj.LUd_rid{j}(:,1)=d; 
%  %       obj.LUd_loc{j}(:,1)=d;
% 
%     end




    
  end
    
%=====================================================================================    

function intSimTime(obj)   % le ponastavi spremenjlivke za snemanje
         % izris analize plana
        obj.tp=0; 
        obj.dp=zeros(1,obj.maxAGVRec); % inicializiraj prepotovane razdalje AGV-jev
        obj.kp=zeros(1,obj.maxAGVRec);
        obj.TP=[]; obj.DP=[]; obj.KP=[];
        obj.TP=[obj.TP;obj.tp]; % cas
        obj.DP=[obj.DP;obj.dp]; % prepotovane razdalje AGV-jev
        obj.KP=[obj.KP;obj.kp]; % konflikt = trk
end
%=====================================================================================    

function replaceAGVonRoad(obj,id,onRoadID, onRoadLoc)
        if(obj.allAGV>0)
            r=[obj.AGV(:).id]; i=find(r==id,1); % najdemo pravo strukturo z iskanim id-jem
            if(isempty(i)),disp('replaceAGVonRoad:??? AGV ne obstaja??'), return; end
        end
        if( ~isKey(obj.roadID,onRoadID) ) % pot že obstaja
            error('AGVSim::createAGV: no such road id ?');
        else
            pose=obj.getPoseOnRoad(onRoadID, onRoadLoc);
        end
        obj.AGV(i).pose=pose;
        obj.AGV(i).onRoadID=onRoadID;
        obj.AGV(i).onRoadLoc=onRoadLoc;
        obj.AGV(i).startRoadLoc=onRoadLoc;
        
        drawAGV(obj,id);           
    end
%=====================================================================================    
function [pose,onRoadID,onRoadLoc]=getLocPoseAGV(obj,id)
        if(obj.allAGV>0)
            r=[obj.AGV(:).id]; i=find(r==id,1); % najdemo pravo strukturo z iskanim id-jem
            if(isempty(i)),disp('getLocPoseAGV:??? AGV ne obstaja??'), return; end
        end
        pose      =obj.AGV(i).pose;
        onRoadID  =obj.AGV(i).onRoadID;
        onRoadLoc =obj.AGV(i).onRoadLoc;
end
%=====================================================================================    
function pose=getPoseOnRoad(obj, rID, locRoad)
 %%% brez uporabe tabelirane poti
%        if( ~isKey(obj.roadID,rID) ) % pot že obstaja
%             error('AGVSim::getPoseOnRoad: no such road id ?');
%         elseif(locRoad>1)
%             error('AGVSim::getPoseOnRoad: relative loc on segment must be in range [0, 1] ?');
%         else
%             
%             distAll=obj.roadID(rID).length;
%             loc=(obj.roadID(rID).param(:,5))/distAll;
%             locRel=cumsum(loc);
%             s=find(locRoad<=locRel,1,'first');
%             if(abs(locRoad-locRel(s))<1e-15)
%                 segRelPart=1;  % je na koncu s-tega sektorja
%             else
%                 segRelPart=1-(locRel(s)-locRoad)/loc(s);  %delež segmenta 
%             end                        
%             xx=0;yy=0;ff=0;
%              
%             if(obj.roadID(rID).secShape(s)=='L')
%                 pp=obj.roadID(rID).param(s,1:4);
%                 dx=pp(3)-pp(1); dy=pp(4)-pp(2);
%                 xx=pp(1)+dx*segRelPart;
%                 yy=pp(2)+dy*segRelPart;
%                 ff=atan2(dy,dx);
%             elseif (obj.roadID(rID).secShape(s)=='A')
%                 par=obj.roadID(rID).param(s,:);   %A=[x0,y0,f0,df,dd]  
%                 par(4)=par(4)*segRelPart; 
%                 par(5)=par(5)*segRelPart;
%                 [xx,yy]=obj.computeArc(par,1);  
%                 ff=par(3)+par(4);
%             else
%                 error('AGVSim: Unknown sector shape'),
%             end
%             pose=[xx;yy;ff];
            
   %%% s tabelirano potjo       
        L=obj.roadID(rID).length;
        N=size(obj.roadID(rID).tableXYF,1);
        travelDist=L*locRoad;
        Dinc=L/(N-1); % increment razdalje med tockami
        
        ic=travelDist/Dinc+1; % indeks na tabelo nezaokrozen
        i=fix(ic); % indeks na tabelo 
        
        if i<N
          dpose=(obj.roadID(rID).tableXYF(i+1,1:3)'-obj.roadID(rID).tableXYF(i,1:3)');
          dpose(3)=wrapToPi(dpose(3));  % da ne skoci po kotu pri kakih spojih poti - za izris
          pose= obj.roadID(rID).tableXYF(i,1:3)' + dpose*(ic-i); 
        else
          %pose= obj.roadID(rID).tableXYF(i,1:3)';
          pose= obj.roadID(rID).tableXYF(N,1:3)';  % GK 5maj2022: to spremenil preveri, ce je bila tu napaka -> naj bi bilo isto
        end
        
    end 
%=====================================================================================    
    function createAGVonRoad(obj,id,onRoadID, onRoadLoc)
        figure(obj.mainFig); 
        
        if(obj.allAGV>0)
            r=[obj.AGV(:).id]; i=find(r==id,1); % najdemo pravo strukturo z iskanim id-jem
            if(~isempty(i)),disp('createAGVonRoad:??? AGV ze obstaja'), return; end
        end
        if( ~isKey(obj.roadID,onRoadID) ) % ali pot obstaja
            error('AGVSim::createAGV: no such road id ?');
        else
            pose=obj.getPoseOnRoad(onRoadID, onRoadLoc);
        end

        h=plot(nan,nan,'b','LineWidth',1) ;     % handle na AGV 
        htext=text(nan,nan,'','Color','b');
        hFill=fill(nan,nan,'b'); %NEJC
        obj.allAGV=obj.allAGV+1;
        new_Agv=struct('id',id,...            
                       'pose',pose,... %  [x;y;fi]
                       'onRoadID',onRoadID,... % roadID
                       'onRoadLoc',onRoadLoc,... % lokacija na cesti 0-1
                       'startRoadLoc',onRoadLoc,... % zapomnim si zacetno lego na cesti - rabim za izris poti
                       'endRoadLoc',nan,... % zapomnim si koncno lego na cesti - rabim za izris poti                      
                       'planRoad',[],...% spisek id-jev cest od zacetne do koncne
                       'planRoadIdx',[],...% index kje na planu sem  ??                     
                       'velocity',0,...% hitrost voznje %NEJC hitrost lahko vektor hitrosti za vsako planirano pot
                       'planRoadTimeSinc',[],...% [Time, roadID, roadLoc], vsaka vrstica nova akcija - dodatna opcija za vnos plana, ki je podan s casovnimi trenutki
                       'goalRoadLoc',[],... % kje na zadnji cesti se ustavi 
                       'hAGV',h,...
                       'hLabel',htext,...
                       'hFill',hFill,...
                       'state',0,... % stanje vozila 0=prosto, 1=na poti do prevzema paketa (prazno), 2=polno, dostava paketa
                       'wait',false,... % NEJC
                       'AGVLength',obj.AGVLength,...
                       'AGVWidth',obj.AGVWidth,...
                       'AGVRadious',sqrt(obj.AGVLength^2+obj.AGVWidth^2),...
                       'priority',0); 

        obj.AGV=[obj.AGV,new_Agv];
        drawAGV(obj,id);           
    end
    
    
%=====================================================================================        
       function createAGVonRoadLW(obj,id,onRoadID, onRoadLoc,length, width, priority)
        figure(obj.mainFig); 
        
        if(obj.allAGV>0)
            r=[obj.AGV(:).id]; i=find(r==id,1); % najdemo pravo strukturo z iskanim id-jem
            if(~isempty(i)),disp('createAGVonRoad:??? AGV ze obstaja'), return; end
        end
        if( ~isKey(obj.roadID,onRoadID) ) % ali pot obstaja
            error('AGVSim::createAGV: no such road id ?');
        else
            pose=obj.getPoseOnRoad(onRoadID, onRoadLoc);
        end

        h=plot(nan,nan,'b','LineWidth',1) ;     % handle na AGV 
        htext=text(nan,nan,'','Color','b');
        hFill=fill(nan,nan,'b'); %NEJC
        obj.allAGV=obj.allAGV+1;
        new_Agv=struct('id',id,...            
                       'pose',pose,... %  [x;y;fi]
                       'onRoadID',onRoadID,... % roadID
                       'onRoadLoc',onRoadLoc,... % lokacija na cesti 0-1
                       'startRoadLoc',onRoadLoc,... % zapomnim si zacetno lego na cesti - rabim za izris poti
                       'endRoadLoc',nan,... % zapomnim si koncno lego na cesti - rabim za izris poti                      
                       'planRoad',[],...% spisek id-jev cest od zacetne do koncne
                       'planRoadIdx',[],...% index kje na planu sem  ??                     
                       'velocity',0,...% hitrost voznje %NEJC hitrost lahko vektor hitrosti za vsako planirano pot
                       'planRoadTimeSinc',[],...% [Time, roadID, roadLoc], vsaka vrstica nova akcija - dodatna opcija za vnos plana, ki je podan s casovnimi trenutki
                       'goalRoadLoc',[],... % kje na zadnji cesti se ustavi 
                       'hAGV',h,...
                       'hLabel',htext,...
                       'hFill',hFill,...
                       'state',0,... % stanje vozila 0=prosto, 1=na poti do prevzema paketa (prazno), 2=polno, dostava paketa
                       'wait',false,...   %NEJC
                       'AGVLength',length,...
                       'AGVWidth',width,...
                       'AGVRadious',sqrt(length^2+width^2),...
                       'priority',priority); 
        obj.AGV=[obj.AGV,new_Agv];
        drawAGV(obj,id);           
    end
%=====================================================================================        
   function createAGVonPose(obj,id,pose)
        if(obj.allAGV>0)
            r=[obj.AGV(:).id]; i=find(r==id,1); % najdemo pravo strukturo z iskanim id-jem
            if(~isempty(i)),disp('createAGVonRoad:??? AGV ze obstaja'), return; end
        end
        
        figure(obj.mainFig); 
        h=plot(nan,nan,'b','LineWidth',2) ;     % handle na AGV 
        htext=text(nan,nan,'','Color','b');
        hFill=fill(nan,nan,'b'); %NEJC
        obj.allAGV=obj.allAGV+1;
        new_Agv=struct('id',id,...            
                       'pose',pose,... %  [x;y;fi]
                       'onRoadID',nan,... % id ceste  
                       'onRoadLoc',[nan],... % [roadID, lokacija],lokacija na cesti 0-1
                       'startRoadLoc',onRoadLoc,... % zapomnim si zacetno lego na cesti - rabim za izris poti
                       'endRoadLoc',nan,... % zapomnim si koncno lego na cesti - rabim za izris poti
                       'planRoad',[],...% spisek id-jev cest
                       'planRoadIdx',[],...% index kje na planu sem  ??                                            
                       'velocity',0,...% hitrost vozneje
                       'planRoadTimeSinc',[],...% [Time, roadID, roadLoc], vsaka vrstica nova akcija - dodatna opcija za vnos plana, ki je podan s casovnimi trenutki
                       'goalRoadLoc',[],... % kje na zadnji cesti se ustavi 
                       'hAGV',h,...
                       'hLabel',htext,...
                       'hFill',hFill,...
                       'state',0,... % stanje vozila 0=prosto, 1=na poti do prevzema paketa (prazno), 2=polno, dostava paketa
                       'wait',false,...
                       'AGVLength',obj.AGVLength,...
                       'AGVWidth',obj.AGVWidth,...
                       'AGVRadious',sqrt(obj.AGVLength^2+obj.AGVWidth^2),...
                       'priority',priority); 

        obj.AGV=[obj.AGV,new_Agv];
        drawAGV(obj,id);           
    end
%=====================================================================================        
   function createAGVonPoseLW(obj,id,pose,length,width,priority)
        if(obj.allAGV>0)
            r=[obj.AGV(:).id]; i=find(r==id,1); % najdemo pravo strukturo z iskanim id-jem
            if(~isempty(i)),disp('createAGVonRoad:??? AGV ze obstaja'), return; end
        end
        
        figure(obj.mainFig); 
        h=plot(nan,nan,'b','LineWidth',2) ;     % handle na AGV 
        htext=text(nan,nan,'','Color','b');
        hFill=fill(nan,nan,'b'); %NEJC
        obj.allAGV=obj.allAGV+1;
        new_Agv=struct('id',id,...            
                       'pose',pose,... %  [x;y;fi]
                       'onRoadID',nan,... % id ceste  
                       'onRoadLoc',[nan],... % [roadID, lokacija],lokacija na cesti 0-1
                       'startRoadLoc',nan,... % zapomnim si zacetno lego na cesti - rabim za izris poti
                       'endRoadLoc',nan,... % zapomnim si koncno lego na cesti - rabim za izris poti
                       'planRoad',[],...% spisek id-jev cest
                       'planRoadIdx',[],...% index kje na planu sem  ??                                            
                       'velocity',0,...% hitrost vozneje
                       'planRoadTimeSinc',[],...% [Time, roadID, roadLoc], vsaka vrstica nova akcija - dodatna opcija za vnos plana, ki je podan s casovnimi trenutki
                       'goalRoadLoc',[],... % kje na zadnji cesti se ustavi 
                       'hAGV',h,...
                       'hLabel',htext,...
                       'hFill',hFill,...
                       'state',0,... % stanje vozila 0=prosto, 1=na poti do prevzema paketa (prazno), 2=polno, dostava paketa
                       'wait',false,...
                       'AGVLength',length,...
                       'AGVWidth',width,...
                       'AGVRadious',sqrt(length^2+width^2),...
                       'priority',priority); 
        obj.AGV=[obj.AGV,new_Agv];
        drawAGV(obj,id);           
    end
%==========================================================================
    function removeAGV(obj,idd)
        r=[obj.AGV(:).id]; i=find(r==idd,1); % najdemo pravo strukturo z iskanim id-jem
        if(isempty(i)),disp('??? neveljaven id v removeAGV'), return; end
        delete(obj.AGV(i).hAGV);
        delete(obj.AGV(i).hLabel);
        delete(obj.AGV(i).hFill);
        if(i<length(r))
            obj.AGV=[obj.AGV(1:i-1), obj.AGV(i+1:end)];
        else
            obj.AGV=obj.AGV(1:i-1);
        end
        obj.allAGV=obj.allAGV-1;
    end
%==========================================================================
    function drawAGV(obj,idd)
        if(~obj.DRAW),return;end;
        r=[obj.AGV(:).id]; i=find(r==idd,1); % najdemo pravo strukturo z iskanim id-jem
        if(isempty(i))
            disp('??? neveljaven id v drawAGV'); 
            return; 
        end
        Xr=obj.AGV(i).pose;
       
        L=obj.AGV(i).AGVLength;
        W=obj.AGV(i).AGVWidth;

        P=[-L L L*1.25 L -L -L;...  % oblika robota
           -W -W 0 W W -W];   
       
        theta = Xr(3); 
        R=[cos(theta) -sin(theta); sin(theta) cos(theta)];
        T=repmat([Xr(1);Xr(2)],1,size(P,2)) ;
        % tocke obrisa robota transliramo in rotiramo
        P=R*P+T; 
        set(obj.AGV(i).hAGV,'XData',P(1,:),'YData',P(2,:))   % izris dejanskega robota
      %  COLORS=jet(length(obj.AGV))*0.8;
        COLORS=[[0 0.4470 0.7410]	; [0.8500 0.3250 0.0980];	[0.9290 0.6940 0.1250]	;[0.4940 0.1840 0.5560]	;[0.4660 0.6740 0.1880]	;[0.3010 0.7450 0.9330]	;[0.6350 0.0780 0.1840]	];
        iddc=i; %popravek: 23sep20022   iddc=idd;% le za izbiro barve vozila
        if (iddc>size(COLORS,1)), iddc=size(COLORS,1); end;
        % [0 .3 0.8]; [0 0.8 0]; [.8 0 0];
%         agvSim.AGV(2).hFill.FaceColor=[0 0.8 0];
%         agvSim.AGV(2).hLabel.Color=[0 0.8 0];
%         agvSim.AGV(1).hFill.FaceColor=[0 .3 0.8];
%         agvSim.AGV(1).hLabel.Color=[0 .3 0.8];
        set(obj.AGV(i).hFill,'XData',P(1,:),'YData',P(2,:),'FaceColor',COLORS(iddc,:),'facealpha',.8);   % NEJC  

%         fill(P(1,:),P(2,:),'r'); %NEJC Pobarvaj AGV
                                                                                                            %Xr(3)*180/pi
 %       set(obj.AGV(i).hLabel,'String',num2str(obj.AGV(i).id),'position',[Xr(1)+0.25*obj.fontSize/60,Xr(2)-0.25*obj.fontSize/60], 'Rotation',0, 'FontSize', obj.fontSize,'color',COLORS(iddc,:))
 %%       set(obj.AGV(i).hLabel,'String',num2str(obj.AGV(i).id),'position',[Xr(1)+0.25*L,Xr(2)-L], 'Rotation',0, 'FontSize', L*10,'color',COLORS(iddc,:))
        set(obj.AGV(i).hLabel,'String',num2str(obj.AGV(i).id),'position',[Xr(1)+L ,Xr(2)-L ], 'Rotation',0, 'FontSize', obj.fontSizeAGV ,'color',COLORS(iddc,:))

 
    end
 %=====================================================================================   
   function drawMapHeatOccupancy(obj,fig) 
        figure(fig); hold on,
        % draw all nodes 
        plot(obj.nodePoses(1,:),obj.nodePoses(2,:),'ko') % draw all nodes at once 
    %    text(obj.nodePoses(1,:)+0.1,obj.nodePoses(2,:)+0.1,obj.nodeLabels)

        % draw all roads
        plot(obj.roadPoints(1,:),obj.roadPoints(2,:),'r') % draw all roads at once
    %    text(obj.roadLabelPos(1,:),obj.roadLabelPos(2,:)+.1,obj.roadLabels,'Color','r')
       % obj.drawRoadArrow(obj.roadLabelPos);   
       
       axis equal
   end    
 %=====================================================================================   
%    function drawMap(obj) 
%         figure(obj.mainFig); hold on,
%         % draw all nodes 
%         plot(obj.nodePoses(1,:),obj.nodePoses(2,:),'ko') % draw all nodes at once 
%         text(obj.nodePoses(1,:)+0.1,obj.nodePoses(2,:)+0.1,obj.nodeLabels)
% 
%         % draw all roads
%         plot(obj.roadPoints(1,:),obj.roadPoints(2,:),'r') % draw all roads at once
%         text(obj.roadLabelPos(1,:),obj.roadLabelPos(2,:)+.1,obj.roadLabels,'Color','r')
%         obj.drawRoadArrow(obj.roadLabelPos);   
%    end    
    
   function drawMap(obj) 
       
      if(~obj.DRAW),return; end; 
              
        figure(obj.mainFig); hold on,
        % draw all nodes 
        plot(obj.nodePoses(1,:),obj.nodePoses(2,:),'ko') % draw all nodes at once 
   %  plot(obj.nodePoses(1,:),obj.nodePoses(2,:),'k.') % draw all nodes at once 

        ax=axis;
        
        d1=obj.fontSize/600;d2=obj.fontSize/400;
       % d1=obj.fontSize/50;d2=obj.fontSize/20;
        
        if obj.drawNodeLabels
          txt1=text(obj.nodePoses(1,:)+d1,obj.nodePoses(2,:)+d2,obj.nodeLabels,'fontsize',obj.fontSize/(ax(4)-ax(3)));
 %          txt1=text(obj.nodePoses(1,:)+d1,obj.nodePoses(2,:)+d2,obj.nodeLabels,'fontsize',obj.fontSize);
        end
        % draw all roads
     plot(obj.roadPoints(1,:),obj.roadPoints(2,:),'r') % draw all roads at once
        if obj.drawRoadLabels
          txt2=text(obj.roadLabelPos(1,:)-d1,obj.roadLabelPos(2,:)-d2,obj.roadLabels,'Color','r','fontsize',obj.fontSize/(ax(4)-ax(3))); %NEJC
  %         txt2=text(obj.roadLabelPos(1,:)-d1,obj.roadLabelPos(2,:)-d2,obj.roadLabels,'Color','r','fontsize',obj.fontSize); %NEJC
        end 
        obj.drawRoadArrow(obj.roadLabelPos); % NEJC
        
% odkomentiraj ce zelis da se fonti povecajo pri zoomu        
        h = zoom; % get handle to zoom utility
        set(h,'ActionPostCallback',@zoomCallBack);
        set(h,'Enable','on');
        % everytime you zoom in, this function is executed
        function zoomCallBack(~, evd)      
            % Since I expect to zoom in ax(4)-ax(3) gets smaller, so fontsize
            % gets bigger.
            ax = axis(evd.Axes); % get axis size
            % change font size accordingly      
            %set(txt1,'FontSize',12);%80/(ax(4)-ax(3))/2); 
            if obj.drawNodeLabels, set(txt1,'FontSize',obj.fontSize/(ax(4)-ax(3))); end
            if obj.drawRoadLabels, set(txt2,'FontSize',obj.fontSize/(ax(4)-ax(3))); end  
        end
        
    end 
%==========================================================================
function drawPlanPaths(obj,fig,offset) 
  
  COLORS=[[0 0.4470 0.7410]	; [0.8500 0.3250 0.0980];	[0.9290 0.6940 0.1250]	;[0.4940 0.1840 0.5560]	;[0.4660 0.6740 0.1880]	;[0.3010 0.7450 0.9330]	;[0.6350 0.0780 0.1840]	];

  figure(fig),hold on
  for i=1:size(obj.AGV,2)     % grem cez vse AGVje in za tiste, ki imajo plan izrisem pot 
        if(~isempty(obj.AGV(i).planRoadTimeSinc))
          
           iddc=i;% le za izbiro barve vozila
           if (iddc>size(COLORS,1)), iddc=size(COLORS,1); end;
           N=size(obj.AGV(i).planRoadTimeSinc,1);
           PP=[];
           roadOld=-1;
           for j=1:N
              
              if j==1  % pri prvi poti lahko zacnemo na povezavi
                 locOffset=obj.AGV(i).startRoadLoc; 
              else
                  locOffset=0;
              end
              
              if j==N && ~isnan(obj.AGV(i).endRoadLoc)  % zadnjo pot lahko koncamo na povezavi
                  locFin=obj.AGV(i).endRoadLoc;
              else
                  locFin=1;
              end
              
               road=obj.AGV(i).planRoadTimeSinc(j,2);
               
               if(road~=roadOld)
                for l=locOffset:.02:locFin
                 pp=obj.getPoseOnRoad(road, l);
                 PP=[PP,pp(1:2)];
                end
               end
               roadOld=road;
           end
           plot(PP(1,:)+i*offset,PP(2,:)+i*offset,'Color',COLORS(iddc,:),'linewidth',1.5);
        end            
    end

  hold off  
end
%==========================================================================
function h=drawOnePlan(obj,agvID,fig,offset,locStart) 
  
  COLORS=[[0 0.4470 0.7410]	; [0.8500 0.3250 0.0980];	[0.9290 0.6940 0.1250]	;[0.4940 0.1840 0.5560]	;[0.4660 0.6740 0.1880]	;[0.3010 0.7450 0.9330]	;[0.6350 0.0780 0.1840]	];
 h=[];
  figure(fig),hold on
   i=find([obj.AGV.id]==agvID);
  if(~isempty(obj.AGV(i).planRoadTimeSinc))
            
           iddc=i;% le za izbiro barve vozila
           if (iddc>size(COLORS,1)), iddc=size(COLORS,1); end;
           N=size(obj.AGV(i).planRoadTimeSinc,1);
           PP=[];
           roadOld=-1;
           for j=1:N   
                
              if j==N && ~isnan(obj.AGV(i).endRoadLoc)  % zadnjo pot lahko koncamo na povezavi
                  locFin=obj.AGV(i).endRoadLoc;
              else
                  locFin=1;
              end
               
               
               road=obj.AGV(i).planRoadTimeSinc(j,2);
               if(road~=roadOld)
                  if j==1, locDraw=locStart; else locDraw=0; end; 
                  for l=locDraw:.02:locFin
                     pp=obj.getPoseOnRoad(road, l);
                     PP=[PP,pp(1:2)];
                  end
               end
               roadOld=road;
           end
           h=plot(PP(1,:)+i*offset,PP(2,:)+i*offset,'Color',COLORS(iddc,:),'linewidth',2);
        end            

  hold off  
end
%==========================================================================
function h=drawOnePlan2(obj,agvID,fig,offset,locStart,tMax) 
  % dodal se cas do kje rise plan
  COLORS=[[0 0.4470 0.7410]	; [0.8500 0.3250 0.0980];	[0.9290 0.6940 0.1250]	;[0.4940 0.1840 0.5560]	;[0.4660 0.6740 0.1880]	;[0.3010 0.7450 0.9330]	;[0.6350 0.0780 0.1840]	];
 h=[];
  figure(fig),hold on
   i=find([obj.AGV.id]==agvID);
  if(~isempty(obj.AGV(i).planRoadTimeSinc))
            
           iddc=i;% le za izbiro barve vozila
           if (iddc>size(COLORS,1)), iddc=size(COLORS,1); end;
           %N=size(obj.AGV(i).planRoadTimeSinc,1);
   
           N= sum( obj.AGV(i).planRoadTimeSinc(:,1)<= tMax );     
           
           
           PP=[];
           roadOld=-1;
           for j=1:N   
                
              if j==N && ~isnan(obj.AGV(i).endRoadLoc)  % zadnjo pot lahko koncamo na povezavi
                  locFin=obj.AGV(i).endRoadLoc;
              else
                  locFin=1;
              end
               
               
               road=obj.AGV(i).planRoadTimeSinc(j,2);
               if(road~=roadOld)
                  if j==1, locDraw=locStart; else locDraw=0; end; 
                  for l=locDraw:.02:locFin
                     pp=obj.getPoseOnRoad(road, l);
                     PP=[PP,pp(1:2)];
                  end
               end
               roadOld=road;
           end
           h=plot(PP(1,:)+i*offset,PP(2,:)+i*offset,'Color',COLORS(iddc,:),'linewidth',2);
        end            

  hold off  
end

%==========================================================================



function precomputeMap(obj) 
        %nodes
        allKeys=cell2mat(keys(obj.nodeID));
        obj.nodePoses=nan(2,obj.nodeID.Count);
        obj.nodeLabels=cell(1,obj.nodeID.Count);
        for i=1:obj.nodeID.Count
            obj.nodePoses(:,i)=obj.nodeID(allKeys(i)).pose;
%             obj.nodeLabels{i}= ['N',int2str(obj.nodeID(allKeys(i)).id)];
            obj.nodeLabels{i}= [int2str(obj.nodeID(allKeys(i)).id)]; %NEJC
        end
        
        %roads
        allKeysR=cell2mat(keys(obj.roadID));
        obj.roadLabels=cell(1,obj.roadID.Count);
        obj.roadLabelPos=nan(3,obj.roadID.Count);        
        obj.roadPoints=[]; % GK 21dec2022 dodal naknadno
        for i=1:obj.roadID.Count
            for s=1:size(obj.roadID(allKeysR(i)).secShape,2) % through all sectors in the road
                if(obj.roadID(allKeysR(i)).secShape(s)=='L')
                    sec_x=obj.roadID(allKeysR(i)).param(s,[1,3]);
                    sec_y=obj.roadID(allKeysR(i)).param(s,[2,4]);
                    
                    if(s==1)
                        dx=sec_x(2)-sec_x(1); dy=sec_y(2)-sec_y(1);
                        fi_lab=atan2(dy,dx);
                        obj.roadLabelPos(:,i)=[sec_x(1)+0.3*dx ; sec_y(1)+0.3*dy;fi_lab];
                    end
                elseif (obj.roadID(allKeysR(i)).secShape(s)=='A')
                   N=20;
                   [sec_x,sec_y]=obj.computeArc(obj.roadID(allKeysR(i)).param(s,:),N);  % draw arc using 20 points
                   if(s==1)
                       par=obj.roadID(allKeysR(i)).param(s,:);
                       fi_lab=par(3)+par(4)/3;
                       obj.roadLabelPos(:,i)=[sec_x(ceil(N/3)); sec_y(ceil(N/3));fi_lab];
                   end
                else
                    error('AGVSim: Unknown sector shape'),
                end

                obj.roadPoints=[obj.roadPoints,[sec_x,nan;sec_y,nan]];
            end
%             obj.roadLabels{i}= ['r',int2str(obj.roadID(allKeysR(i)).id)];
            obj.roadLabels{i}= [int2str(obj.roadID(allKeysR(i)).id)]; %NEJC
        end
    end        
%==========================================================================
function drawRoadArrow(obj,Xr)
     N=size(Xr,2); % number of arrows
     xx=[]; yy=[];
     for n=1:N   
      %  P=[-1 1 -1 ;-.8 0  .8 ]*0.06;  % oblika puscice
         P=[-1 1 -1 ;-.8 0  .8 ]*obj.fontSize/2000;  % oblika puscice
     %    P=[-1 1 -1 ;-.8 0  .8 ]*obj.fontSize/50;  % oblika puscice

        R=[cos(Xr(3,n)) -sin(Xr(3,n)); sin(Xr(3,n)) cos(Xr(3,n))];
        T=repmat([Xr(1,n);Xr(2,n)],1,size(P,2)) ;
        % tocke obrisa puscice transliramo in rotiramo
        P=R*P+T; 
        xx=[xx,P(1,:),nan];
        yy=[yy,P(2,:),nan];
     end
     figure(obj.mainFig); hold on, plot(xx,yy,'Color','r')
end
%==========================================================================
 function [xA,yA]=computeArc(obj,par,N)  
       Sx=par(1); Sy=par(2);fi0=par(3);dfi=par(4);dd=par(5);
       FI=fi0+linspace(0, dfi , N);  % draw circular arc using N points
       if(abs(dfi)>0.000001)
            R=dd/dfi;
            xA=Sx-R*sin(fi0)+R*sin(FI);  yA=Sy+R*cos(fi0)-R*cos(FI);
       else
            DD =linspace(0, dd , N);
            xA=Sx+DD*cos(fi0);  yA=Sy+DD*sin(fi0);
       end
 end
%===================================================================
%==========================================================================
 function addReverseRoad(obj,rID,rIDreverse,deadEndRoad)     
      
     if( ~isKey(obj.roadID,rID)) % pot že obstaja
         error('AVGSim::addReverseRoad: road does not exist');
     end
        
       N=size(obj.roadID(rID).secShape,2);
        
        secShape= fliplr(obj.roadID(rID).secShape);
        
        param=nan(size(obj.roadID(rID).param));
        for i=1:N
            p=obj.roadID(rID).param(i,:);
            if(obj.roadID(rID).secShape(i)=='L')
                param(i,:)=[p(3:4),p(1:2),p(5)];
            elseif(obj.roadID(rID).secShape(i)=='A')
                if(abs(p(4))>0.0001)
                    x0=p(1)+p(5)/p(4)*(sin(p(3)+p(4))-sin(p(3))); % new start is previous end
                    y0=p(2)-p(5)/p(4)*(cos(p(3)+p(4))-cos(p(3))); % new start is previous end
                else
                    x0=p(1)+p(5)*cos(p(3)); % new start is previous end
                    y0=p(2)+p(5)*sin(p(3)); % new start is previous end
                end
                param(i,:)=[x0, y0, wrapToPi(p(3)+p(4)-pi),-p(4),p(5)];
            else
                error('AVGSim::addReverseRoad: not existing shape parameter');
            end
        end
        
        param=flipud(param);
        
        tableXYF=obj.computeTableXYF(obj.roadID(rID).length,secShape,param); 
        obj.roadID(rIDreverse) = struct('id',rIDreverse ,...
                            'length', obj.roadID(rID).length, ... %
                            'secShape',secShape,...  % sections: L= line, A=arc
                            'param',param,...  % parameters of sections L=[x1,y1,x2,y2,d], A=[x0,y0,f0,df,dd]  
                            'sNode', obj.roadID(rID).eNode,... % start noad id
                            'eNode',obj.roadID(rID).sNode,...
                            'occupancy',[],... % road Occupancy Nejc
                            'occupancy2',[],...
                            'deadEndRoad',deadEndRoad, ...                            
                            'unsafeInterval',[],... % za CCBS pri move akcijah (kdaj ne sme za?et premika po povezavi)
                            'twoWayID',rID,...
                            'waitInterval',[1-obj.roadID(rID).waitInterval(2) 1-obj.roadID(rID).waitInterval(1)],... % interval na cesti kjer dovoljeno cakanje 0-1
                            'tableXYF',tableXYF);  % look-up table x(d),y(d),phi(d) (d-lendth allong the road )
     
 end
%===================================================================
        
%==========================================================================
 function addTwoNodesAndConnection(obj,nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )


 % nID1, nID2 : indexes of road nodes
 % rID12: road indeks from nodes nID1 to nID2
 % secShape: line vector of 'L'= line, 'A'=arc
 % secParam: parameters of sections. for ech section new line of 5 parameters for line of arc: L=[x1,y1,x2,y2,d], A=[x0,y0,f0,df,dd] 
 % twoway: 0=oneway, 1=twoway
 % rID21: if twoway==1  then define road indeks from nodes nID1 to nID2 else if twoway==0 then rID21 is irelevant and can be rID21=nan
    if( isKey(obj.nodeID,nID1)) % novd že obstaja
        %if(any(obj.nodeID(nID1).pose~=p1)) % opozori pozicija ni ista
        if(any(abs(obj.nodeID(nID1).pose-p1) > 1e-3)) % MB: opozori pozicija ni ista
            error('AGVSim::addTwoNodesAndConnection: not the same position1?');
        end
        
        
        if(~any(obj.nodeID(nID1).neighboarRoadIDs==rID12)) % pot ne obstaja, dodam pot in koncno vozlisce (tudi ce ze obstaja)         
            b=obj.nodeID(nID1); b.neighboarRoadIDs=[b.neighboarRoadIDs,rID12];
            obj.nodeID(nID1)=b; % ?? širjene elementov strukture pri containers.Map je problem
            
            a=obj.nodeID(nID1); a.neighboarNodeIDs=[a.neighboarNodeIDs,nID2];
            obj.nodeID(nID1)=a; % ?? širjene elementov strukture pri containers.Map je problem

            
        end




    else
            obj.nodeID(nID1) = struct('id', nID1,... 
                                    'pose',p1,...
                                    'neighboarNodeIDs',nID2, ... % reachable neighbors 
                                    'neighboarRoadIDs',rID12,... % possible roads
                                    'occupancy',[],...
                                    'unsafeInterval',[],... % za CCBS
                                    'nearbyNode',[]);  % nodes that are tooclose and need to be occupied at the same time
                                %    'nearbyNodeMargin',[]);            
                                  % 'safeIntervals',[0,inf],...

    end
    
    
     if( isKey(obj.nodeID,nID2)) % novd že obstaja
        %if(any(obj.nodeID(nID2).pose~=p2)) % opozori pozicija ni ista
        if(any(abs(obj.nodeID(nID2).pose-p2) > 1e-3)) % opozori pozicija ni ista
            error('AGVSim::addTwoNodesAndConnection: not the same position2?');
        end
        
            
        if(twoway)        
            if(~any(obj.nodeID(nID2).neighboarRoadIDs==rID21)) % pot ne obstaja          
                b=obj.nodeID(nID2); b.neighboarRoadIDs=[b.neighboarRoadIDs,rID21];
                obj.nodeID(nID2)=b; % ?? širjene elementov strukture pri containers.Map je problem
                
                a= obj.nodeID(nID2); a.neighboarNodeIDs=[a.neighboarNodeIDs,nID1];
                obj.nodeID(nID2)=a;  % ?? širjene elementov strukture pri containers.Map je problem           
            end
        end
        
     else
           if(twoway) % je dvosmerna
               nr=rID21;
               nn=nID1;
           else
               nr=[];
               nn=[];
           end
           obj.nodeID(nID2) = struct('id', nID2,... 
                                    'pose',p2,...
                                    'neighboarNodeIDs',nn, ... % reachable neighbors 
                                    'neighboarRoadIDs',nr,... % possible roads
                                    'occupancy',[],...         %node occupancy Nejc
                                    'unsafeInterval',[],... % za CCBS
                                    'nearbyNode',[]);  % nodes that are tooclose and need to be occupied at the same time
                              %      'nearbyNodeMargin',[]);            
                             %       'safeIntervals',[0,inf],... 

     end
   
    
    if( isKey(obj.roadID,rID12)) % pot 12 že obstaja
        error('AGVSim::addTwoNodesAndConnection: path ID12 already exist ?');
    else
        
        for i=1:length(secShape)% preverim razdalje za sektorje 'L'
            if(secShape(i)=='L')
                p=secParam(i,:);
                secParam(i,5)=norm([p(1)-p(3);p(2)-p(4)]);
            end
        end
        len=sum(secParam(:,end));
  
        if(twoway), rID21_=rID21; else rID21_=0; end
    
  
        tableXYF=obj.computeTableXYF(len,secShape,secParam); 
        
        obj.roadID(rID12) = struct('id', rID12,...
                            'length', len, ... %
                            'secShape',secShape,...  % sections: L= line, A=arc
                            'param',secParam,... % parameters of sections L=[x1,y1,x2,y2,d], A=[x0,y0,f0,df,dd]  
                            'sNode',nID1,... % start node id
                            'eNode',nID2,... % end node id
                            'occupancy',[],...%road occupancy %%%%%%%%%%%%%%%%%%%%%%%%%%%%% Nejc
                            'occupancy2',[],...
                            'deadEndRoad',0, ...
                            'unsafeInterval',[],... % za CCBS pri move akcijah (kdaj ne sme za?et premika po povezavi)                            
                            'twoWayID',rID21_,...
                            'waitInterval',[0 1],... % interval na cesti kjer dovoljeno cakanje 0-1                            
                            'tableXYF',tableXYF);  % look-up table x(d),y(d),phi(d) (d-lendth allong the road )
  
    end
     
    if(twoway) 
        if( isKey(obj.roadID,rID21)) % pot 21 že obstaja
            error('AGVSim::addTwoNodesAndConnection: path ID21 already exist ?');
        else
            obj.addReverseRoad(rID12,rID21,0);               
        end
    end

     
 end
 
%===================================================================      
 function addTwoNodesAndConnectionOptionDeadEnd(obj,nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21, deadEndRoad )


 % nID1, nID2 : indexes of road nodes
 % rID12: road indeks from nodes nID1 to nID2
 % secShape: line vector of 'L'= line, 'A'=arc
 % secParam: parameters of sections. for ech section new line of 5 parameters for line of arc: L=[x1,y1,x2,y2,d], A=[x0,y0,f0,df,dd] 
 % twoway: 0=oneway, 1=twoway
 % rID21: if twoway==1  then define road indeks from nodes nID1 to nID2 else if twoway==0 then rID21 is irelevant and can be rID21=nan
    if( isKey(obj.nodeID,nID1)) % novd že obstaja
        %if(any(obj.nodeID(nID1).pose~=p1)) % opozori pozicija ni ista
        if(any(abs(obj.nodeID(nID1).pose-p1) > 1e-3)) % MB: opozori pozicija ni ista
            error('AGVSim::addTwoNodesAndConnection: not the same position1?');
        end
        
        
        if(~any(obj.nodeID(nID1).neighboarRoadIDs==rID12)) % pot ne obstaja, dodam pot in koncno vozlisce (tudi ce ze obstaja)         
            b=obj.nodeID(nID1); b.neighboarRoadIDs=[b.neighboarRoadIDs,rID12];
            obj.nodeID(nID1)=b; % ?? širjene elementov strukture pri containers.Map je problem
            
            a=obj.nodeID(nID1); a.neighboarNodeIDs=[a.neighboarNodeIDs,nID2];
            obj.nodeID(nID1)=a; % ?? širjene elementov strukture pri containers.Map je problem

            
        end




    else
            obj.nodeID(nID1) = struct('id', nID1,... 
                                    'pose',p1,...
                                    'neighboarNodeIDs',nID2, ... % reachable neighbors 
                                    'neighboarRoadIDs',rID12,... % possible roads
                                    'occupancy',[],...
                                    'unsafeInterval',[],... % za CCBS
                                    'nearbyNode',[]);  % nodes that are tooclose and need to be occupied at the same time
                                %    'nearbyNodeMargin',[]);            
                                  % 'safeIntervals',[0,inf],...

    end
    
    
     if( isKey(obj.nodeID,nID2)) % novd že obstaja
        %if(any(obj.nodeID(nID2).pose~=p2)) % opozori pozicija ni ista
        if(any(abs(obj.nodeID(nID2).pose-p2) > 1e-3)) % opozori pozicija ni ista
            error('AGVSim::addTwoNodesAndConnection: not the same position2?');
        end
        
            
        if(twoway)        
            if(~any(obj.nodeID(nID2).neighboarRoadIDs==rID21)) % pot ne obstaja          
                b=obj.nodeID(nID2); b.neighboarRoadIDs=[b.neighboarRoadIDs,rID21];
                obj.nodeID(nID2)=b; % ?? širjene elementov strukture pri containers.Map je problem
                
                a= obj.nodeID(nID2); a.neighboarNodeIDs=[a.neighboarNodeIDs,nID1];
                obj.nodeID(nID2)=a;  % ?? širjene elementov strukture pri containers.Map je problem           
            end
        end
        
     else
           if(twoway) % je dvosmerna
               nr=rID21;
               nn=nID1;
           else
               nr=[];
               nn=[];
           end
           obj.nodeID(nID2) = struct('id', nID2,... 
                                    'pose',p2,...
                                    'neighboarNodeIDs',nn, ... % reachable neighbors 
                                    'neighboarRoadIDs',nr,... % possible roads
                                    'occupancy',[],...         %node occupancy Nejc
                                    'unsafeInterval',[],... % za CCBS
                                    'nearbyNode',[]);  % nodes that are tooclose and need to be occupied at the same time
                              %      'nearbyNodeMargin',[]);            
                             %       'safeIntervals',[0,inf],... 

     end
   
    
    if( isKey(obj.roadID,rID12)) % pot 12 že obstaja
        error('AGVSim::addTwoNodesAndConnection: path ID12 already exist ?');
    else
        
        for i=1:length(secShape)% preverim razdalje za sektorje 'L'
            if(secShape(i)=='L')
                p=secParam(i,:);
                secParam(i,5)=norm([p(1)-p(3);p(2)-p(4)]);
            end
        end
        len=sum(secParam(:,end));
  
        if(twoway), rID21_=rID21; else rID21_=0; end
    
  
        tableXYF=obj.computeTableXYF(len,secShape,secParam); 
        
        obj.roadID(rID12) = struct('id', rID12,...
                            'length', len, ... %
                            'secShape',secShape,...  % sections: L= line, A=arc
                            'param',secParam,... % parameters of sections L=[x1,y1,x2,y2,d], A=[x0,y0,f0,df,dd]  
                            'sNode',nID1,... % start node id
                            'eNode',nID2,... % end node id
                            'occupancy',[],...%road occupancy %%%%%%%%%%%%%%%%%%%%%%%%%%%%% Nejc
                            'occupancy2',[],...
                            'deadEndRoad',deadEndRoad, ...
                            'unsafeInterval',[],... % za CCBS pri move akcijah (kdaj ne sme za?et premika po povezavi)                            
                            'twoWayID',rID21_,...
                            'waitInterval',[0 1],... % interval na cesti kjer dovoljeno cakanje 0-1
                            'tableXYF',tableXYF);  % look-up table x(d),y(d),phi(d) (d-lendth allong the road )
  
    end
     
    if(twoway) 
        if( isKey(obj.roadID,rID21)) % pot 21 že obstaja
            error('AGVSim::addTwoNodesAndConnection: path ID21 already exist ?');
        else
            obj.addReverseRoad(rID12,rID21,deadEndRoad);               
        end
    end

     
 end
 
%===================================================================      
 

function setNearbyNodes(obj,nodeID, nearNodeIDs) 
%function setNearbyNodes(obj,nodeID, nearNodeIDs, marginsNearNodes) 
  % nearNodeIDs ... a row of node IDs which are too close and need to be occupied at the same time  
  %%% marginsNearNodes ... a row vector how much extra to enlarge occupancy of the near node 
  %
  % (ce se cesti sekata pod kotom se cas zasedenosti lahko podaljša ali skrajsa za dvojno vozlisce odvisno od kota pod katerim se sekata (za 90° cca 1.4, ))
         tt=obj.nodeID(nodeID); 
      %   tt.nearbyNode=[tt.nearbyNode, nearNodeIDs]; 
          tt.nearbyNode= nearNodeIDs; 
  %        tt.nearbyNodeMargin= marginsNearNodes; 
          obj.nodeID(nodeID)=tt;   
 end
%==========================================================================
%  function tableXYF=computeTableXYF(obj,len,secShape,secParam) 
%           
%         N=ceil(len*obj.tabelResolution); % stevilo tock v tabeli za pot
%         
%         %tableXYF=zeros(N,4);
%         
%         
%         incD=len/N; % popravljen inkrement, da se dolzina poti izzide
%                    
%         tableXYF=[];
%         dOst=incD; % ostanek razdalje prejsnjega sektorja, da zacne prvi sektor z d=0
%         for i=1:length(secShape)% tabeliram pot
%            p=secParam(i,:); 
%            switch(secShape(i))
%              case 'L'      %L=[x0,y0,x1,x2,d]  
%                    d=((incD-dOst):incD:p(5))';
%                    segRelPart=d/p(5);
%                    dx=p(3)-p(1); dy=p(4)-p(2);
%                    xx=p(1)+dx*segRelPart;
%                    yy=p(2)+dy*segRelPart;
%                    ff=atan2(dy,dx)*ones(size(xx));
%                    dOst=p(5)-d(end); % pristejem naslednjemu sektorju
%              case 'A'        %A=[x0,y0,f0,df,dd]  
%                    d=((incD-dOst):incD:p(5))';
%                    segRelPart=d/p(5);
%                    d_ff=p(4)*segRelPart; % delezi inkrementa kota
%                    d_dd=p(5)*segRelPart; % delez inkrementa razdalje po loku 
% 
%                    Sx=p(1); Sy=p(2);Fi0=p(3);dFi=p(4);dD=p(5);
%                    ff=Fi0+d_ff;
%                    if(abs(dFi)>0.00000001)
%                         R=dD/dFi;
%                         xx=Sx-R*sin(Fi0)+R*sin(ff);  yy=Sy+R*cos(Fi0)-R*cos(ff);
%                    else
%                         DD =d_dd;
%                         xx =Sx+DD*cos(Fi0);  yy=Sy+DD*sin(Fi0);
%                    end
%                    dOst=p(5)-d(end); % pristejem naslednjemu sektorju
%              case 'B'
%            end
%            tableXYF = [tableXYF; xx,yy,ff];%,d ]; 
%         end
%         
%  end
%  %===================================================================      


 %===================================================================      
 function tableXYF=computeTableXYF(obj,len,secShape,secParam) 
        N=ceil(len*obj.tabelResolution)+1; % stevilo tock v tabeli za pot
        %N=ceil(len*obj.tabelResolution)+2; % stevilo tock v tabeli za pot GK 6maj2022, dodal +2 za vsak primer??

        incD=len/(N-1); % inkrement, da se dolzina poti izzide
        idxD=0; % trenutni odmik indeksa na tabelo
        oldD=0; % trenutna razdalja vzdolz poti
                
        minSectDist=min(secParam(:,5)); % GK 11may2022: preverim in popravim ce je incD vecji od najkrajšega segmenta 
        if (minSectDist>1e-3)
            if (minSectDist<incD)               
              N=ceil(len/minSectDist)+1; 
              incD=len/(N-1);
            end
        else
            error('AGVSim::computeTableXYF: sector length in a connection is too small - less than 1e-3?');
        end
        
        tableXYF=zeros(N,3);

        tol=1e-5; % dodano da gre do zadnjega elementa
        
        dOst=incD; % ostanek razdalje prejsnjega sektorja, da zacne prvi sektor z d=0
        for i=1:length(secShape)% tabeliram pot
           p=secParam(i,:); 
           switch(secShape(i))
             case 'L'        %L=[x0,y0,x1,x2,d]                     
                   d=((incD-dOst):incD:(p(5)+tol))';
                   segRelPart=d/p(5);
                   dx=p(3)-p(1); dy=p(4)-p(2);
                   xx=p(1)+dx*segRelPart;
                   yy=p(2)+dy*segRelPart;
                   ff=atan2(dy,dx)*ones(size(xx));
                   dOst=p(5)-d(end); % pristejem naslednjemu sektorju
             case 'A'        %A=[x0,y0,f0,df,dd]  
                   d=((incD-dOst):incD:(p(5)+tol))';
                   segRelPart=d/p(5);
                   d_ff=p(4)*segRelPart; % delezi inkrementa kota
                   Sx=p(1);Sy=p(2);Fi0=p(3);dFi=p(4);dD=p(5);
                   ff=Fi0+d_ff;
                   if(abs(dFi)>0.0000000001)
                        R=dD/dFi;
                        xx=Sx-R*sin(Fi0)+R*sin(ff);  yy=Sy+R*cos(Fi0)-R*cos(ff);
                   else
                        d_dd=p(5)*segRelPart; % delez inkrementa razdalje po loku 
                        xx=Sx+d_dd*cos(Fi0);  yy=Sy+d_dd*sin(Fi0);
                   end
                   dOst=p(5)-d(end); % pristejem naslednjemu sektorju
             case 'B'
                 %
           end
           tableXYF (idxD+(1:length(d)),:) = [ xx,yy,ff];%,d+oldD ]; 
           idxD=idxD+length(d);
           oldD=oldD+dOst+d(end);
        end
                
 end
 %===================================================================      
 %=====================================================================================    
    function createSimpleMap2(obj)
         nID1=1; nID2=2; p1=[0;0];p2=[4;3.5]; rID12=12; twoway=1; rID21=21;
         secShape=['A','L','A','L','A'];
         secParam=[  0 0 0 pi/2 pi/2;... %  A=[x0,y0,fi_start,delta_fi,razdaljaPoKroznemLoku] 
                     1 1 1 1.5 0.5;... % L=[x1,y1,x2,y2,razdalja],
                     1 1.5 pi/2 -pi/2 pi/2;...
                     2 2.5 3 2.5 1;...
                     3 2.5 0 pi/2 pi/2];
         obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )

         nID1=1; nID2=3; p1=[0;0];p2=[5;0]; rID12=13; twoway=0; rID21=nan;
         secShape='L';
         secParam=[0 0 5 0 5];
         obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )


         nID1=2; nID2=3; p1=[4;3.5];p2=[5;0]; rID12=23; twoway=0; rID21=nan;
         secShape=['L','A'];
         secParam=[4 3.5 4 1 2.5;...
                   4 1 -pi/2 +pi/2 pi/2];
         obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )

         nID1=3; nID2=4; p1=[5;0];p2=[6;0]; rID12=34; twoway=0; rID21=nan;
         secShape='L';
         secParam=[5 0 6 0 1];
         obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
         
%          nID1=5; nID2=2; p1=[2;3.5]; p2=[4;3.5]; rID12=52; twoway=1; rID21=25;
%          secShape=['L']; secParam=[2 3.5 4 3.5 2];
%          obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )

         
         obj.precomputeMap();
         obj.drawMap();
    end
    
  %===================================================================      
 %=====================================================================================    
function dAGV=moveAGVsOneSample(obj, Ts,vMax,tSimDelay,dAGV,t) % premakne AGV-je,ki imajo plan, za en vzorec
 
  allAGVsWithPlan=0; allAGVsWithReachedPlan=0;
        
  for i=1:size(obj.AGV,2)     % grem cez vse AGVje 
    if(~isempty(obj.AGV(i).planRoadTimeSinc)) % jih premaknem le ce imajo plan
       
      a=i; % indeks na plan je isto kot indeks na agv ???
      allAGVsWithPlan=allAGVsWithPlan+1;
      P=obj.AGV(i).planRoadTimeSinc;  % plan
            
      if(dAGV(i)<P(end,4)  || P(end,1)-t>Ts) %-Ts  )  % ce se ni prisel do cilja ?? ali ce prisel prehitro potem caka, ni ok ce ima plan cakanjena koncu
      
        if obj.LUt_dAGV{i}(end,2)==0
           dAGVNew=0; 
        else
           dAGVNew=interp1(obj.LUt_dAGV{i}(:,1),obj.LUt_dAGV{i}(:,2),t,'linear','extrap');  % dAGV=prepotovana razdalja  (tabela dAGV=fun(t))
        end
        if(dAGVNew>obj.LUd_rid{i}(end,1)), dAGVNew= obj.LUt_dAGV{i}(end,2); end

        v=(dAGVNew-dAGV(i))/Ts;
        
        if(v>vMax)
            v=vMax; 
        end

        dAGVinc=v*Ts;   % nameravan premik

        % preveri test trka           
     %  collision=obj.checkCollisionAGV(a,dAGVinc);  % preveri trk za a-tega AGV za razdaljo dd v smeri voznje        
    %--- 18.4.2023: novo za preverit trk, dolocim predikcijo lokacije vzdolz poti ppNext 
        dAGVTmp=dAGV(i)+dAGVinc;   
        if(find( abs( dAGVTmp- obj.LUd_rid{i}(:,1))<1e-14)) % ali sem zelo blizu nezveznega intervala ali v njem
            dAGVSample=dAGVTmp+2*1e-14; % da preskocim nezvezni interval in se izognem problemom hipnih skokov agv-ja pri animaciji
        else
            dAGVSample=dAGVTmp;
        end
        rid=interp1(obj.LUd_rid{i}(:,1),obj.LUd_rid{i}(:,2),dAGVSample,'previous','extrap');  
        % loc=interp1(LUd_loc{i}(:,1),LUd_loc{i}(:,2),dAGVSample,'linear');  
        loc=interp1(obj.LUd_loc{i}(:,1),obj.LUd_loc{i}(:,2),dAGVSample,'linear'); loc = loc - floor(loc); %AZ
        
         if(dAGVTmp>= P(end,4))% && t>=P(end,1)) % ce prisel do konca po razdalji
             loc=P(end,3);
%              if(t-P(end,1)>Ts)  
%                 allAGVsWithReachedPlan=allAGVsWithReachedPlan+1;
%                 obj.AGV(a).state=0; % prost
%              end  
             dAGVTmp = P(end,4);
         end  
        ppNext = obj.getPoseOnRoad(rid, loc);     
        collision=obj.checkCollisionAGV2(a,dAGVinc, ppNext);  % preveri trk za a-tega AGV za razdaljo dd v smeri voznje        
      
       
       
       
       
       
       
       
        
     %------ dodano da ignorira trke v ponornem vozliscu   
     if(~isempty(obj.roadToRestNode) && obj.AGV(i).onRoadID == obj.roadToRestNode )
          % zelo poenostavim in kar recem, da se na cesti do ponora ignorira trke 
       %   if(obj.AGV(i).onRoadLoc > obj.roadToRestNodeLoc)
            collision=0;
       %   end
     end
        
        
        
     if(~collision && dAGVinc>0 || obj.preventCollision==2 )        
       
%        if 0  
%          dAGV(i)=dAGV(i)+dAGVinc;   
%         %GK dodano 22maj2022 : reševanje preskokov pri animaciji zaradi
%         %aproksimacije nezveznih preskokov (lege in id-ja ceste) in interpolacije z interp1
%         if(find( abs( dAGV(i)- obj.LUd_rid{i}(:,1))<1e-14)) % ali sem zelo blizu nezveznega intervala ali v njem
%             dAGVSample=dAGV(i)+2*1e-14; % da preskocim nezvezni interval in se izognem problemom hipnih skokov agv-ja pri animaciji
%         else
%             dAGVSample=dAGV(i);
%         end
%         
% 
%         rid=interp1(obj.LUd_rid{i}(:,1),obj.LUd_rid{i}(:,2),dAGVSample,'previous','extrap');  
%      %   loc=interp1(obj.LUd_loc{i}(:,1),obj.LUd_loc{i}(:,2),dAGVSample,'linear');  
%         loc=interp1(obj.LUd_loc{i}(:,1),obj.LUd_loc{i}(:,2),dAGVSample,'linear'); loc = loc - floor(loc); %AZ
% 
%         obj.AGV(a).onRoadLoc = loc;
%         obj.AGV(a).onRoadID  = rid;
%         
%          if(dAGV(i)>= P(end,4))% && t>=P(end,1)) % ce prisel do konca po razdalji
%              obj.AGV(a).onRoadLoc=P(end,3);
%              if(t-P(end,1)>Ts)  
%                 allAGVsWithReachedPlan=allAGVsWithReachedPlan+1;
%                 obj.AGV(a).state=0; % prost
%              end  
%              dAGV(i)= P(end,4);
%          end  
%          obj.AGV(a).pose = obj.getPoseOnRoad(obj.AGV(a).onRoadID, obj.AGV(a).onRoadLoc);
%        
%        else
     %--- 18.4.2023: novo za preverit trk, dolocim predikcijo lokacije vzdolz poti ppNext 
         dAGV(i)= dAGVTmp;
         obj.AGV(a).onRoadLoc = loc;
         obj.AGV(a).onRoadID  = rid;
        
         if(dAGV(i)>= P(end,4))% && t>=P(end,1)) % ce prisel do konca po razdalji
             %obj.AGV(a).onRoadLoc=P(end,3);
             if(t-P(end,1)>Ts)  
                allAGVsWithReachedPlan=allAGVsWithReachedPlan+1;
                obj.AGV(a).state=0; % prost
             end  
             %dAGV(i)= P(end,4);
         end  

         obj.AGV(a).pose = ppNext;                   
 %      end
         
         
         
         
         % shrani % nova prepotovana razdalja in trk 
         if(i<=obj.maxAGVRec)  obj.dp(a)=dAGV(i); end
         
         if( obj.preventCollision==2 && collision )  % trke le zaznavam a ne ustavljam vozil
            if(i<=obj.maxAGVRec), obj.kp(a)=1; end % je trk a ga le zabeležimo vozilo pa vseeno premaknemo
         else
            if(i<=obj.maxAGVRec), obj.kp(a)=0; end % ni trka
         end
     elseif(~collision && dAGVinc<=0)      % ni trka in se ne premikam  
         if(i<=obj.maxAGVRec)  obj.dp(a)=dAGV(i); end  % nova razdalja
         if(i<=obj.maxAGVRec), obj.kp(a)=0; end % ni trka
     else % trk
       % if(i<=obj.maxAGVRec) obj.dp(a)=obj.dp(a); end   %?? sumljivo
        if(i<=obj.maxAGVRec) obj.kp(a)=1; end  % za izris razdalj
     end % if(~collision)
   else
      allAGVsWithReachedPlan=allAGVsWithReachedPlan+1;   
      obj.AGV(a).state=0; % prost
   end                 
                
  end
 end
        
        % izris analize plana
        obj.tp=t; % za izris voženj
        obj.TP=[obj.TP;obj.tp]; % cas
        obj.DP=[obj.DP;obj.dp]; % prepotovane razdalje AGV-jev
        obj.KP=[obj.KP;obj.kp];
        
        
        if(allAGVsWithReachedPlan==allAGVsWithPlan)
            obj.AGVsPlanAchived=1;
        else
            obj.AGVsPlanAchived=0;
        end
              
      
      if(obj.DRAW)  
        for i=[obj.AGV(:).id] %1:size(obj.AGV,2)     % grem cez vse AGVje 
            drawAGV(obj,i);           
        end    
        pause(tSimDelay);
      end
      
end

%=====================================================================================
%=====================================================================================
%=====================================================================================
%=====================================================================================
%=====================================================================================

  
  
  
  
  
  
  
  
  
  
 function createSimpleMap3(obj)
         coor=nan(2,100);
         coor(:,2)=[4;3];
         coor(:,3)=[8;3];
         coor(:,4)=[5;10];
         coor(:,5)=[16;3];
         coor(:,6)=[22;1];
         coor(:,55)=[4;5];
         coor(:,57)=[9;14];
         coor(:,60)=[7;10];
         coor(:,64)=[8;5];
         coor(:,66)=[13;10];
         coor(:,68)=[12;7];
         coor(:,69)=[11;6];
         coor(:,70)=[13;6];
         coor(:,71)=[12;5];
         coor(:,72)=[16;2];
         coor(:,73)=[15;1];
         coor(:,74)=[17;1];
         coor(:,75)=[16;6];
         coor(:,76)=[18;6];
         coor(:,77)=[17;5];
         coor(:,78)=[20;11];
         coor(:,79)=[19;10];
         coor(:,80)=[20;9];
         coor(:,81)=[20;7];
         coor(:,82)=[19;6];
         coor(:,83)=[20;5];
         coor(:,84)=[20;2];
         coor(:,85)=[19;1];
         coor(:,86)=[21;1];
         
         
         
         
         
         nID1=1; nID2=50; p1=[0;0];p2=[0;5/2]; rID12=150; twoway=1; rID21=5001;
         secShape='L';
         secParam=[  0 0 0 5 5 ]/2; % last parameter is sector length
         obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
         
         nID1=2; nID2=55; p1=coor(:,nID1)/2;p2=coor(:,nID2)/2; rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
         secShape='L';
         secParam=[ p1' p2' 2/2]; % last parameter is sector length
         obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
         
         nID1=3; nID2=64; p1=coor(:,nID1)/2;p2=coor(:,nID2)/2; rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
         secShape='L';
         secParam=[ p1' p2' 2/2]; % last parameter is sector length
         obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
         
         nID1=4; nID2=60; p1=coor(:,nID1)/2;p2=coor(:,nID2)/2; rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
         secShape='L';
         secParam=[ p1' p2' 2/2]; % last parameter is sector length
         obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
         
         nID1=5; nID2=72; p1=coor(:,nID1)/2;p2=coor(:,nID2)/2; rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
         secShape='L';
         secParam=[ p1' p2' 2/2]; % last parameter is sector length
         obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
         
         nID1=6; nID2=86; p1=coor(:,nID1)/2;p2=coor(:,nID2)/2; rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
         secShape='L';
         secParam=[ p1' p2' 2/2]; % last parameter is sector length
         obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )

         nID1=50; nID2=51; p1=[0;5/2];p2=[0;7/2]; rID12=5051; twoway=1; rID21=5150;
         secShape='L';
         secParam=[  0 5 0 7 2 ]/2; % last parameter is sector length
         obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
         
         nID1=50; nID2=52; p1=[0;5/2];p2=[1/2;6/2]; rID12=5052; twoway=1; rID21=5250;
         secShape='A';
         secParam=[  0 5/2 pi/2 -pi/2 pi/2/2]; % last parameter is sector length
         obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
        
         nID1=51; nID2=52; p1=[0;7/2];p2=[1/2;6/2]; rID12=5152; twoway=1; rID21=5251;
         secShape='A';
         secParam=[  0 7/2 -pi/2 pi/2 pi/2/2 ]; % last parameter is sector length
         obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
         
         nID1=52; nID2=53; p1=[1/2;6/2];p2=[3/2;6/2]; rID12=5253; twoway=1; rID21=5352;
         secShape='L';
         secParam=[ 1/2 6/2 3/2 6/2 2/2 ]; % last parameter is sector length
         obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
         
         nID1=53; nID2=54; p1=[3/2;6/2];p2=[5/2;6/2]; rID12=5354; twoway=1; rID21=5453;
         secShape='L';
         secParam=[ 3/2 6/2 5/2 6/2 2/2 ]; % last parameter is sector length
         obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
         
         nID1=53; nID2=55; p1=[3/2;6/2];p2=[4/2;5/2]; rID12=5355; twoway=1; rID21=5553;
         secShape='A';
         secParam=[ 3/2 6/2 0 -pi/2 pi/4 ]; % last parameter is sector length
         obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
         
         nID1=55; nID2=54; p1=[4/2;5/2];p2=[5/2;6/2]; rID12=5554; twoway=1; rID21=5455;
         secShape='A';
         secParam=[ 4/2 5/2 pi/2 -pi/2 pi/4 ]; % last parameter is sector length
         obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
         
         nID1=51; nID2=56; p1=[0;7]/2;p2=[7;14]/2; rID12=5156; twoway=1; rID21=5651;
         secShape=['L','A','L'];
         secParam=[ 0 7/2 0/2 13/2 6/2 ;...
                    0/2 13/2 pi/2 -pi/2 pi/4;...
                    1/2 14/2 7/2 14/2 6/2]; % last parameter is sector length
         obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
         
         nID1=56; nID2=57; p1=[7;14]/2;p2=[9;14]/2; rID12=5657; twoway=1; rID21=5756;
         secShape='L';
         secParam=[ 7 14 9 14 2 ]/2; % last parameter is sector length
         obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
         
         nID1=56; nID2=58; p1=[7/2;14/2];p2=[8/2;13/2]; rID12=5658; twoway=1; rID21=5856;
         secShape='A';
         secParam=[ 7/2 14/2 0 -pi/2 pi/4 ]; % last parameter is sector length
         obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
         
         nID1=58; nID2=57; p1=[8/2;13/2];p2=[9/2;14/2]; rID12=5857; twoway=1; rID21=5758;
         secShape='A';
         secParam=[ 8/2 13/2 pi/2 -pi/2 pi/4 ]; % last parameter is sector length
         obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
         
         nID1=58; nID2=59; p1=[8;13]/2;p2=[8;11]/2; rID12=5859; twoway=1; rID21=5958;
         secShape='L';
         secParam=[ 8 13 8 11 2 ]/2; % last parameter is sector length
         obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
         
         nID1=59; nID2=60; p1=[8;11]/2 ;p2=[7;10]/2; rID12=5960; twoway=1; rID21=6059;
         secShape='A';
         secParam=[ 8/2 11/2 -pi/2 -pi/2 pi/4 ]; % last parameter is sector length
         obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
         
         nID1=59; nID2=61; p1=[8;11]/2 ;p2=[9;10]/2; rID12=5961; twoway=1; rID21=6159;
         secShape='A';
         secParam=[ 8/2 11/2 -pi/2 pi/2 pi/4 ]; % last parameter is sector length
         obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
         
         nID1=60; nID2=61; p1=[7;10]/2 ;p2=[9;10]/2; rID12=6061; twoway=1; rID21=6160;
         secShape='L';
         secParam=[ 7 10 9 10 2 ]/2; % last parameter is sector length
         obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
         
         nID1=54; nID2=62; p1=[5;6]/2;p2=[7;6]/2; rID12=5462; twoway=1; rID21=6254;
         secShape='L';
         secParam=[ 5 6 7 6 2 ]/2; % last parameter is sector length
         obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
         
         nID1=62; nID2=63; p1=[7;6]/2;p2=[9;6]/2; rID12=6263; twoway=1; rID21=6362;
         secShape='L';
         secParam=[ 7 6 9 6 2 ]/2; % last parameter is sector length
         obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
         
         nID1=62; nID2=64; p1=[7/2;6/2];p2=[8/2;5/2]; rID12=6264; twoway=1; rID21=6462;
         secShape='A';
         secParam=[ 7/2 6/2 0 -pi/2 pi/4 ]; % last parameter is sector length
         obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
         
         nID1=64; nID2=63; p1=[8/2;5/2];p2=[9/2;6/2]; rID12=6463; twoway=1; rID21=6364;
         secShape='A';
         secParam=[ 8/2 5/2 pi/2 -pi/2 pi/4 ]; % last parameter is sector length
         obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
         
         nID1=61; nID2=65; p1=[9;10]/2;p2=[11;10]/2; rID12=6165; twoway=1; rID21=6561;
         secShape='L';
         secParam=[ 9 10 11 10 2 ]/2; % last parameter is sector length
         obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
         
         nID1=65; nID2=66; p1=[11;10]/2;p2=[13;10]/2; rID12=6566; twoway=1; rID21=6665;
         secShape='L';
         secParam=[ 11 10 13 10 2 ]/2; % last parameter is sector length
         obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
         
         nID1=65; nID2=67; p1=[11;10]/2;p2=[12;9]/2; rID12=6567; twoway=1; rID21=6765;
         secShape='A';
         secParam=[ 11/2 10/2 0 -pi/2 pi/4 ]; % last parameter is sector length
         obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
         
         nID1=67; nID2=66; p1=[12;9]/2;p2=[13;10]/2; rID12=6766; twoway=1; rID21=6667;
         secShape='A';
         secParam=[ 12/2 9/2 pi/2 -pi/2 pi/4 ]; % last parameter is sector length
         obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
         
         nID1=67; nID2=68; p1=[12;9]/2;p2=[12;7]/2; rID12=6768; twoway=1; rID21=6867;
         secShape='L';
         secParam=[ 12 9 12 7 2]/2; % last parameter is sector length
         obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
         
         nID1=63; nID2=69; p1=[9;6]/2;p2=[11;6]/2; rID12=6369; twoway=1; rID21=6963;
         secShape='L';
         secParam=[ 9 6 11 6 2]/2; % last parameter is sector length
         obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
         
         nID1=68; nID2=69; p1=[12;7]/2;p2=[11;6]/2; rID12=6869; twoway=1; rID21=6968;
         secShape='A';
         secParam=[ 12/2 7/2 -pi/2 -pi/2 pi/4]; % last parameter is sector length
         obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
         
         nID1=68; nID2=70; p1=coor(:,nID1)/2;p2=coor(:,nID2)/2; rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
         secShape='A';
         secParam=[ p1' -pi/2 pi/2 pi/4]; % last parameter is sector length
         obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
         
         nID1=69; nID2=70; p1=coor(:,nID1)/2;p2=coor(:,nID2)/2; rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
         secShape='L';
         secParam=[ p1' p2' 2/2]; % last parameter is sector length
         obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
         
         nID1=68; nID2=71; p1=coor(:,nID1)/2;p2=coor(:,nID2)/2; rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
         secShape='L';
         secParam=[ p1' p2' 2/2]; % last parameter is sector length
         obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )

         nID1=69; nID2=71; p1=coor(:,nID1)/2;p2=coor(:,nID2)/2; rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
         secShape='A';
         secParam=[ p1' 0 -pi/2 pi/4]; % last parameter is sector length
         obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
         
         nID1=70; nID2=71; p1=coor(:,nID1)/2;p2=coor(:,nID2)/2; rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
         secShape='A';
         secParam=[ p1' -pi pi/2 pi/4]; % last parameter is sector length
         obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
         
         nID1=71; nID2=73; p1=coor(:,nID1)/2;p2=coor(:,nID2)/2; rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
         secShape=['L','A','L'];
         secParam=[ p1' p1'-[0,3]/2 2/2;...
                    p1'-[0,3]/2 -pi/2 pi/2 pi/4;...
                    p2'-[2,0]/2 p2' 2/2]; % last parameter is sector length
         obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
         
         nID1=72; nID2=73; p1=coor(:,nID1)/2;p2=coor(:,nID2)/2; rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
         secShape='A';
         secParam=[ p1' -pi/2 -pi/2 pi/4]; % last parameter is sector length
         obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
         
         nID1=72; nID2=74; p1=coor(:,nID1)/2;p2=coor(:,nID2)/2; rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
         secShape='A';
         secParam=[ p1' -pi/2 pi/2 pi/4]; % last parameter is sector length
         obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
         
         nID1=73; nID2=74; p1=coor(:,nID1)/2;p2=coor(:,nID2)/2; rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
         secShape='L';
         secParam=[ p1' p2' 2/2]; % last parameter is sector length
         obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
         
         nID1=70; nID2=75; p1=coor(:,nID1)/2;p2=coor(:,nID2)/2; rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
         secShape='L';
         secParam=[ p1' p2' 2/2]; % last parameter is sector length
         obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
         
         nID1=75; nID2=76; p1=coor(:,nID1)/2;p2=coor(:,nID2)/2; rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
         secShape='L';
         secParam=[ p1' p2' 2/2]; % last parameter is sector length
         obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
         
         nID1=75; nID2=77; p1=coor(:,nID1)/2;p2=coor(:,nID2)/2; rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
         secShape='A';
         secParam=[ p1' 0 -pi/2 pi/4]; % last parameter is sector length
         obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
         
         nID1=76; nID2=77; p1=coor(:,nID1)/2;p2=coor(:,nID2)/2; rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
         secShape='A';
         secParam=[ p1' -pi pi/2 pi/4]; % last parameter is sector length
         obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
         
         nID1=57; nID2=78; p1=coor(:,nID1)/2;p2=coor(:,nID2)/2; rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
         secShape=['L','A','L'];
         secParam=[ p1' p1'+[10,0]/2 10/2;...
                    p1'+[10,0]/2 0 -pi/2 pi/4;...
                    p2'+[0,2]/2 p2' 2]; % last parameter is sector length
         obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
         
         nID1=66; nID2=79; p1=coor(:,nID1)/2;p2=coor(:,nID2)/2; rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
         secShape='L';
         secParam=[ p1' p2' 6/2]; % last parameter is sector length
         obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
         
         nID1=78; nID2=79; p1=coor(:,nID1)/2;p2=coor(:,nID2)/2; rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
         secShape='A';
         secParam=[ p1' -pi/2 -pi/2 pi/4]; % last parameter is sector length
         obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
         
         nID1=78; nID2=80; p1=coor(:,nID1)/2;p2=coor(:,nID2)/2; rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
         secShape='L';
         secParam=[ p1' p2' 2/2]; % last parameter is sector length
         obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
         
         nID1=79; nID2=80; p1=coor(:,nID1)/2;p2=coor(:,nID2)/2; rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
         secShape='A';
         secParam=[ p1' 0 -pi/2 pi/4]; % last parameter is sector length
         obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
         
         nID1=80; nID2=81; p1=coor(:,nID1)/2;p2=coor(:,nID2)/2; rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
         secShape='L';
         secParam=[ p1' p2' 2/2]; % last parameter is sector length
         obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
         
         nID1=76; nID2=82; p1=coor(:,nID1)/2;p2=coor(:,nID2)/2; rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
         secShape='L';
         secParam=[ p1' p2' 1/2]; % last parameter is sector length
         obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
         
         nID1=81; nID2=82; p1=coor(:,nID1)/2;p2=coor(:,nID2)/2; rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
         secShape='A';
         secParam=[ p1' -pi/2 -pi/2 pi/4]; % last parameter is sector length
         obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
         
         nID1=81; nID2=83; p1=coor(:,nID1)/2;p2=coor(:,nID2)/2; rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
         secShape='L';
         secParam=[ p1' p2' 2/2]; % last parameter is sector length
         obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
         
         nID1=82; nID2=83; p1=coor(:,nID1)/2;p2=coor(:,nID2)/2; rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
         secShape='A';
         secParam=[ p1' 0 -pi/2 pi/4]; % last parameter is sector length
         obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
         
         nID1=83; nID2=84; p1=coor(:,nID1)/2;p2=coor(:,nID2)/2; rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
         secShape='L';
         secParam=[ p1' p2' 3/2]; % last parameter is sector length
         obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
         
         nID1=74; nID2=85; p1=coor(:,nID1)/2;p2=coor(:,nID2)/2; rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
         secShape='L';
         secParam=[ p1' p2' 2/2]; % last parameter is sector length
         obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
         
         nID1=84; nID2=85; p1=coor(:,nID1)/2;p2=coor(:,nID2)/2; rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
         secShape='A';
         secParam=[ p1' -pi/2 -pi/2 pi/4]; % last parameter is sector length
         obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
         
         nID1=84; nID2=86; p1=coor(:,nID1)/2;p2=coor(:,nID2)/2; rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
         secShape='A';
         secParam=[ p1' -pi/2 pi/2 pi/4]; % last parameter is sector length
         obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
         
         nID1=85; nID2=86; p1=coor(:,nID1)/2;p2=coor(:,nID2)/2; rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
         secShape='L';
         secParam=[ p1' p2' 2/2]; % last parameter is sector length
         obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
         
         
         obj.precomputeMap();
         obj.drawMap();

 end   

function createSimpleMap4(obj)
     coor=nan(2,1000);
     coor(:,01)=[0;0];
     coor(:,02)=[0.5;0];
     coor(:,03)=[0;3];
     coor(:,04)=[0.5;3];
     coor(:,05)=[1;2.5];
     coor(:,06)=[1;2];
     coor(:,07)=[0.5;1.5];
     coor(:,08)=[0;1.5];
     coor(:,10)=[2.5;0.5];
     coor(:,11)=[2;2.5];
     coor(:,12)=[3;2.5];
     coor(:,13)=[3;2];
     coor(:,14)=[2.5;1.5];
     coor(:,15)=[2;2];
     coor(:,20)=[3;4];
     coor(:,30)=[4.5;0.5];
     coor(:,31)=[4;2.5];
     coor(:,32)=[5;2.5];
     coor(:,33)=[5;2];
     coor(:,34)=[4.5;1.5];
     coor(:,35)=[4;2];
     coor(:,21)=[4.5;4.5];
     coor(:,22)=[5;4];
     coor(:,23)=[4;4];
     coor(:,24)=[4;7];
     coor(:,25)=[5;7];
     coor(:,26)=[5;6.5];
     coor(:,27)=[4.5;6];
     coor(:,28)=[4;6.5];
     coor(:,70)=[6.5;3];
     coor(:,71)=[7;1.5];
     coor(:,72)=[6.5;1.5];
     coor(:,73)=[6;2];
     coor(:,74)=[6;2.5];
     coor(:,50)=[7.5;4.5];
     coor(:,51)=[7.5;4];
     coor(:,52)=[6.5;3.5];
     coor(:,53)=[6;4];
     coor(:,40)=[7;5.5];
     coor(:,41)=[6.5;7];
     coor(:,42)=[7.5;7];
     coor(:,43)=[7.5;6.5];
     coor(:,44)=[7;6];
     coor(:,45)=[6.5;6.5];
     coor(:,60)=[8.5;0.5];
     coor(:,61)=[9;1];
     coor(:,62)=[9.5;0.5];
     coor(:,63)=[9.5;0];
     coor(:,64)=[8.5;0];
     coor(:,65)=[9;2];
     coor(:,54)=[8.5;2.5];
     coor(:,55)=[8;4.5];
     coor(:,56)=[9;4.5];
     coor(:,57)=[9;4];
     coor(:,58)=[8.5;3.5];
     coor(:,59)=[8;4];
     coor(:,90)=[12.5;1];
     coor(:,91)=[13;1];
     coor(:,92)=[13.5;0.5];
     coor(:,93)=[13.5;0];
     coor(:,94)=[12;0];
     coor(:,95)=[12;0.5];
     coor(:,80)=[12.5;5];
     coor(:,81)=[13;5];
     coor(:,82)=[13;3.5];
     coor(:,83)=[12.5;3.5];
     coor(:,84)=[12;4];
     coor(:,85)=[12;4.5];
     %1-2
     nID1=01; nID2=08; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
     secShape='L';
     secParam=[ p1' p2' norm(p1'-p2')]; % last parameter is sector length
     obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
     nID1=02; nID2=07; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
     secShape='L';
     secParam=[ p1' p2' norm(p1'-p2')]; % last parameter is sector length
     obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
     %2
     nID1=03; nID2=05; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
     secShape=['A','L'];
     secParam=[ p1' -pi/2 pi/2 pi/4;...
                p2'-[0.5 0] p2' 0.5]; % last parameter is sector length
     obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
     nID1=03; nID2=06; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
     secShape='A';
     secParam=[ p1' -pi/2 pi/2 pi/2]; % last parameter is sector length
     obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
     nID1=03; nID2=07; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
     secShape='L';
     secParam=[ p1' p2' norm(p1'-p2')]; % last parameter is sector length
     obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
     nID1=03; nID2=08; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
     secShape='L';
     secParam=[ p1' p2' norm(p1'-p2')]; % last parameter is sector length
     obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
     nID1=04; nID2=05; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
     secShape='A';
     secParam=[ p1' -pi/2 pi/2 pi/4]; % last parameter is sector length
     obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
     nID1=04; nID2=06; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
     secShape=['L','A'];
     secParam=[ p1' p1'-[0,0.5] 0.5;...
                p1'-[0,0.5] -pi/2 pi/2 pi/4]; % last parameter is sector length
     obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
     nID1=04; nID2=07; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
     secShape='L';
     secParam=[ p1' p2' norm(p1'-p2')]; % last parameter is sector length
     obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
     nID1=04; nID2=08; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
     secShape='L';
     secParam=[ p1' p2' norm(p1'-p2')]; % last parameter is sector length
     obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
     nID1=05; nID2=07; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
     secShape=['A','L'];
     secParam=[ p1' pi pi/2 pi/4;...
                p2'+[0,0.5] p2' 0.5]; % last parameter is sector length
     obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
     nID1=05; nID2=08; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
     secShape='A';
     secParam=[ p1' pi pi/2 pi/2]; % last parameter is sector length
     obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
     nID1=06; nID2=07; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
     secShape='A';
     secParam=[ p1' pi pi/2 pi/4]; % last parameter is sector length
     obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
     nID1=06; nID2=08; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
     secShape=['L','A'];
     secParam=[ p1' p1'-[0.5,0] 0.5;...
                p1'-[0.5,0] pi pi/2 pi/4]; % last parameter is sector length
     obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
     %2-4
     nID1=05; nID2=11; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
     secShape='L';
     secParam=[ p1' p2' norm(p1'-p2')]; % last parameter is sector length
     obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
     nID1=06; nID2=15; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
     secShape='L';
     secParam=[ p1' p2' norm(p1'-p2')]; % last parameter is sector length
     obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
     %4
     nID1=11; nID2=12; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
     secShape='L';
     secParam=[ p1' p2' norm(p1'-p2')]; % last parameter is sector length
     obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
     nID1=11; nID2=13; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
     secShape='L';
     secParam=[ p1' p2' norm(p1'-p2')]; % last parameter is sector length
     obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
     nID1=11; nID2=14; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
     secShape=['A','L'];
     secParam=[ p1' 0 -pi/2 pi/4;...
                p2'+[0,0.5] p2' 0.5]; % last parameter is sector length
     obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
     nID1=12; nID2=14; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
     secShape=['A','L'];
     secParam=[ p1' pi pi/2 pi/4;...
                p2'+[0,0.5] p2' 0.5]; % last parameter is sector length
     obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
     nID1=12; nID2=15; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
     secShape='L';
     secParam=[ p1' p2' norm(p1'-p2')]; % last parameter is sector length
     obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
     nID1=13; nID2=14; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
     secShape='A';
     secParam=[ p1' pi pi/2 pi/4]; % last parameter is sector length
     obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
     nID1=13; nID2=15; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
     secShape='L';
     secParam=[ p1' p2' norm(p1'-p2')]; % last parameter is sector length
     obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
     nID1=14; nID2=15; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
     secShape='A';
     secParam=[ p1' pi/2 pi/2 pi/4]; % last parameter is sector length
     obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
     %3-4
     nID1=10; nID2=14; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
     secShape='L';
     secParam=[ p1' p2' norm(p1'-p2')]; % last parameter is sector length
     obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
     %4-7
     nID1=12; nID2=31; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
     secShape='L';
     secParam=[ p1' p2' norm(p1'-p2')]; % last parameter is sector length
     obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
     nID1=13; nID2=35; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
     secShape='L';
     secParam=[ p1' p2' norm(p1'-p2')]; % last parameter is sector length
     obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
     %7
     nID1=31; nID2=32; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
     secShape='L';
     secParam=[ p1' p2' norm(p1'-p2')]; % last parameter is sector length
     obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
     nID1=31; nID2=33; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
     secShape='L';
     secParam=[ p1' p2' norm(p1'-p2')]; % last parameter is sector length
     obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
     nID1=31; nID2=34; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
     secShape=['A','L'];
     secParam=[ p1' 0 -pi/2 pi/4;...
                p2'+[0,0.5] p2' 0.5]; % last parameter is sector length
     obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
     nID1=32; nID2=34; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
     secShape=['A','L'];
     secParam=[ p1' pi pi/2 pi/4;...
                p2'+[0,0.5] p2' 0.5]; % last parameter is sector length
     obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
     nID1=32; nID2=35; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
     secShape='L';
     secParam=[ p1' p2' norm(p1'-p2')]; % last parameter is sector length
     obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
     nID1=33; nID2=34; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
     secShape='A';
     secParam=[ p1' pi pi/2 pi/4]; % last parameter is sector length
     obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
     nID1=33; nID2=35; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
     secShape='L';
     secParam=[ p1' p2' norm(p1'-p2')]; % last parameter is sector length
     obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
     nID1=34; nID2=35; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
     secShape='A';
     secParam=[ p1' pi/2 pi/2 pi/4]; % last parameter is sector length
     obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
     %6-7
     nID1=30; nID2=34; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
     secShape='L';
     secParam=[ p1' p2' norm(p1'-p2')]; % last parameter is sector length
     obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
     %2-9
     nID1=03; nID2=24; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
     secShape=['L','A','L'];
     secParam=[ p1' p1'+[0,3] 3;...
                p1'+[0,3] pi/2 -pi/2 pi/2;...
                p2'-[3,0] p2' 3]; % last parameter is sector length
     obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
     nID1=04; nID2=28; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
     secShape=['L','A','L'];
     secParam=[ p1' p1'+[0,3] 3;...
                p1'+[0,3] pi/2 -pi/2 pi/4;...
                p2'-[3,0] p2' 3]; % last parameter is sector length
     obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
     %9
     nID1=24; nID2=25; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
     secShape='L';
     secParam=[ p1' p2' norm(p1'-p2')]; % last parameter is sector length
     obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
     nID1=24; nID2=26; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
     secShape='L';
     secParam=[ p1' p2' norm(p1'-p2')]; % last parameter is sector length
     obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
     nID1=24; nID2=27; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
     secShape=['A','L'];
     secParam=[ p1' 0 -pi/2 pi/4;...
                p2'+[0,0.5] p2' 0.5]; % last parameter is sector length
     obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
     nID1=25; nID2=27; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
     secShape=['A','L'];
     secParam=[ p1' pi pi/2 pi/4;...
                p2'+[0,0.5] p2' 0.5]; % last parameter is sector length
     obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
     nID1=25; nID2=28; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
     secShape='L';
     secParam=[ p1' p2' norm(p1'-p2')]; % last parameter is sector length
     obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
     nID1=26; nID2=27; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
     secShape='A';
     secParam=[ p1' pi pi/2 pi/4]; % last parameter is sector length
     obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
     nID1=26; nID2=28; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
     secShape='L';
     secParam=[ p1' p2' norm(p1'-p2')]; % last parameter is sector length
     obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
     nID1=27; nID2=28; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
     secShape='A';
     secParam=[ p1' pi/2 pi/2 pi/4]; % last parameter is sector length
     obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
     %9-8
     nID1=27; nID2=21; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
     secShape='L';
     secParam=[ p1' p2' norm(p1'-p2')]; % last parameter is sector length
     obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
     %8
     nID1=21; nID2=22; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
     secShape='A';
     secParam=[ p1' -pi/2 pi/2 pi/4]; % last parameter is sector length
     obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
     nID1=22; nID2=23; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
     secShape='L';
     secParam=[ p1' p2' norm(p1'-p2')]; % last parameter is sector length
     obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
     nID1=21; nID2=23; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
     secShape='A';
     secParam=[ p1' -pi/2 -pi/2 pi/4]; % last parameter is sector length
     obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
     %8-5
     nID1=23; nID2=20; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
     secShape='L';
     secParam=[ p1' p2' norm(p1'-p2')]; % last parameter is sector length
     obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
     %7-10
     nID1=32; nID2=74; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
     secShape='L';
     secParam=[ p1' p2' norm(p1'-p2')]; % last parameter is sector length
     obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
     nID1=33; nID2=73; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
     secShape='L';
     secParam=[ p1' p2' norm(p1'-p2')]; % last parameter is sector length
     obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
     %10
     nID1=70; nID2=71; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
     secShape='L';
     secParam=[ p1' p2' norm(p1'-p2')]; % last parameter is sector length
     obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
     nID1=70; nID2=72; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
     secShape='L';
     secParam=[ p1' p2' norm(p1'-p2')]; % last parameter is sector length
     obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
     nID1=70; nID2=73; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
     secShape=['L','A'];
     secParam=[ p1' p1'-[0,0.5] 0.5;...
                p1'-[0,0.5] -pi/2 -pi/2 pi/4]; % last parameter is sector length
     obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
     nID1=70; nID2=74; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
     secShape='A';
     secParam=[ p1' -pi/2 -pi/2 pi/4]; % last parameter is sector length
     obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
     nID1=71; nID2=73; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
     secShape=['A','L'];
     secParam=[ p1' pi/2 pi/2 pi/4;...
                p2'+[0.5,0] p2' 0.5]; % last parameter is sector length
     obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
     nID1=71; nID2=74; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
     secShape='A';
     secParam=[ p1' pi/2 pi/2 pi/2]; % last parameter is sector length
     obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
     nID1=72; nID2=73; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
     secShape='A';
     secParam=[ p1' pi/2 pi/2 pi/4]; % last parameter is sector length
     obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
     nID1=72; nID2=74; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
     secShape=['L','A'];
     secParam=[ p1' p1'+[0,0.5] 0.5;...
                p1'+[0,0.5] pi/2 pi/2 pi/4]; % last parameter is sector length
     obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
     %8-11
     nID1=22; nID2=53; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
     secShape='L';
     secParam=[ p1' p2' norm(p1'-p2')]; % last parameter is sector length
     obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
     %11
     nID1=53; nID2=50; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
     secShape='L';
     secParam=[ p1' p2' norm(p1'-p2')]; % last parameter is sector length
     obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
%      nID1=50; nID2=51; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
%      secShape='L';
%      secParam=[ p1' p2' norm(p1'-p2')]; % last parameter is sector length
%      obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
     nID1=53; nID2=52; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
     secShape='A';
     secParam=[ p1' 0 -pi/2 pi/4]; % last parameter is sector length
     obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
     nID1=50; nID2=52; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
     secShape='A';
     secParam=[ p1' pi pi/2 pi/2]; % last parameter is sector length
     obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
     nID1=51; nID2=52; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
     secShape=['L','A'];
     secParam=[ p1' p1'-[0.5,0] 0.5;...
                p1'-[0.5,0] pi pi/2 pi/4]; % last parameter is sector length
     obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
     %10-11
     nID1=70; nID2=52; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
     secShape='L';
     secParam=[ p1' p2' norm(p1'-p2')]; % last parameter is sector length
     obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
     %9-13
     nID1=25; nID2=41; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
     secShape='L';
     secParam=[ p1' p2' norm(p1'-p2')]; % last parameter is sector length
     obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
     nID1=26; nID2=45; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
     secShape='L';
     secParam=[ p1' p2' norm(p1'-p2')]; % last parameter is sector length
     obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
     %13
     nID1=41; nID2=42; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
     secShape='L';
     secParam=[ p1' p2' norm(p1'-p2')]; % last parameter is sector length
     obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
     nID1=41; nID2=43; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
     secShape='L';
     secParam=[ p1' p2' norm(p1'-p2')]; % last parameter is sector length
     obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
     nID1=41; nID2=44; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
     secShape=['A','L'];
     secParam=[ p1' 0 -pi/2 pi/4;...
                p2'+[0,0.5] p2' 0.5]; % last parameter is sector length
     obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
     nID1=42; nID2=44; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
     secShape=['A','L'];
     secParam=[ p1' pi pi/2 pi/4;...
                p2'+[0,0.5] p2' 0.5]; % last parameter is sector length
     obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
     nID1=42; nID2=45; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
     secShape='L';
     secParam=[ p1' p2' norm(p1'-p2')]; % last parameter is sector length
     obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
     nID1=43; nID2=44; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
     secShape='A';
     secParam=[ p1' pi pi/2 pi/4]; % last parameter is sector length
     obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
     nID1=43; nID2=45; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
     secShape='L';
     secParam=[ p1' p2' norm(p1'-p2')]; % last parameter is sector length
     obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
     nID1=44; nID2=45; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
     secShape='A';
     secParam=[ p1' pi/2 pi/2 pi/4]; % last parameter is sector length
     obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
     %12-13
     nID1=40; nID2=44; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
     secShape='L';
     secParam=[ p1' p2' norm(p1'-p2')]; % last parameter is sector length
     obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
     %10-14
     nID1=71; nID2=60; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
     secShape=['L','A','L'];
     secParam=[ p1' p1'-[0,0.5] 0.5;...
                p1'-[0,0.5] -pi/2 pi/2 pi/4;...
                p2'-[1,0] p2' 1]; % last parameter is sector length
     obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
     nID1=72; nID2=64; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
     secShape=['L','A','L'];
     secParam=[ p1' p1'-[0,0.5] 0.5;...
                p1'-[0,0.5] -pi/2 pi/2 pi/2;...
                p2'-[1,0] p2' 1]; % last parameter is sector length
     obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
     %14
     nID1=60; nID2=61; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
     secShape='A';
     secParam=[ p1' 0 pi/2 pi/4]; % last parameter is sector length
     obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
     nID1=60; nID2=62; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
     secShape='L';
     secParam=[ p1' p2' norm(p1'-p2')]; % last parameter is sector length
     obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
     nID1=60; nID2=63; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
     secShape='L';
     secParam=[ p1' p2' norm(p1'-p2')]; % last parameter is sector length
     obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
     nID1=61; nID2=62; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
     secShape='A';
     secParam=[ p1' -pi/2 pi/2 pi/4]; % last parameter is sector length
     obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
     nID1=61; nID2=63; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
     secShape=['L','A'];
     secParam=[ p1' p1'-[0,0.5] 0.5;...
                p1'-[0,0.5] -pi/2 pi/2 pi/4]; % last parameter is sector length
     obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
     nID1=61; nID2=64; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
     secShape=['L','A'];
     secParam=[ p1' p1'-[0,0.5] 0.5;...
                p1'-[0,0.5] -pi/2 -pi/2 pi/4]; % last parameter is sector length
     obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
     nID1=62; nID2=64; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
     secShape='L';
     secParam=[ p1' p2' norm(p1'-p2')]; % last parameter is sector length
     obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
     nID1=63; nID2=64; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
     secShape='L';
     secParam=[ p1' p2' norm(p1'-p2')]; % last parameter is sector length
     obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
     %14-15
     nID1=61; nID2=65; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
     secShape='L';
     secParam=[ p1' p2' norm(p1'-p2')]; % last parameter is sector length
     obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
     %11-17
     nID1=50; nID2=55; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
     secShape='L';
     secParam=[ p1' p2' norm(p1'-p2')]; % last parameter is sector length
     obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
     nID1=51; nID2=59; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
     secShape='L';
     secParam=[ p1' p2' norm(p1'-p2')]; % last parameter is sector length
     obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
     %17
     nID1=55; nID2=56; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
     secShape='L';
     secParam=[ p1' p2' norm(p1'-p2')]; % last parameter is sector length
     obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
     nID1=55; nID2=57; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
     secShape='L';
     secParam=[ p1' p2' norm(p1'-p2')]; % last parameter is sector length
     obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
     nID1=55; nID2=58; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
     secShape=['A','L'];
     secParam=[ p1' 0 -pi/2 pi/4;...
                p2'+[0,0.5] p2' 0.5]; % last parameter is sector length
     obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
     nID1=56; nID2=58; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
     secShape=['A','L'];
     secParam=[ p1' pi pi/2 pi/4;...
                p2'+[0,0.5] p2' 0.5]; % last parameter is sector length
     obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
     nID1=56; nID2=59; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
     secShape='L';
     secParam=[ p1' p2' norm(p1'-p2')]; % last parameter is sector length
     obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
     nID1=57; nID2=58; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
     secShape='A';
     secParam=[ p1' pi pi/2 pi/4]; % last parameter is sector length
     obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
     nID1=57; nID2=59; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
     secShape='L';
     secParam=[ p1' p2' norm(p1'-p2')]; % last parameter is sector length
     obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
     nID1=58; nID2=59; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
     secShape='A';
     secParam=[ p1' pi/2 pi/2 pi/4]; % last parameter is sector length
     obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
     %16-17
     nID1=54; nID2=58; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
     secShape='L';
     secParam=[ p1' p2' norm(p1'-p2')]; % last parameter is sector length
     obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
     %14-18
     nID1=62; nID2=95; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
     secShape='L';
     secParam=[ p1' p2' norm(p1'-p2')]; % last parameter is sector length
     obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
     nID1=63; nID2=94; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
     secShape='L';
     secParam=[ p1' p2' norm(p1'-p2')]; % last parameter is sector length
     obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
     %18
     nID1=90; nID2=92; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
     secShape=['A','L'];
     secParam=[ p1' -pi/2 pi/2 pi/4;...
                p2'-[0.5,0] p2' 0.5]; % last parameter is sector length
     obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
     nID1=90; nID2=93; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
     secShape='A';
     secParam=[ p1' -pi/2 pi/2 pi/2]; % last parameter is sector length
     obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
     nID1=90; nID2=94; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
     secShape=['L','A'];
     secParam=[ p1' p1'-[0,0.5] 0.5;...
                p1'-[0,0.5] -pi/2 -pi/2 pi/4]; % last parameter is sector length
     obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
     nID1=90; nID2=95; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
     secShape='A';
     secParam=[ p1' -pi/2 -pi/2 pi/4]; % last parameter is sector length
     obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
     nID1=91; nID2=92; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
     secShape='A';
     secParam=[ p1' -pi/2 pi/2 pi/4]; % last parameter is sector length
     obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
     nID1=91; nID2=93; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
     secShape=['L','A'];
     secParam=[ p1' p1'-[0,0.5] 0.5;...
                p1'-[ 0,0.5] -pi/2 pi/2 pi/4]; % last parameter is sector length
     obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
     nID1=91; nID2=94; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
     secShape='A';
     secParam=[ p1' -pi/2 -pi/2 pi/2]; % last parameter is sector length
     obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
     nID1=91; nID2=95; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
     secShape=['A','L'];
     secParam=[ p1' -pi/2 -pi/2 pi/4;...
                p2'+[0.5,0] p2' 0.5]; % last parameter is sector length
     obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
     nID1=92; nID2=94; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
     secShape='L';
     secParam=[ p1' p2' norm(p1'-p2')]; % last parameter is sector length
     obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
     nID1=92; nID2=95; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
     secShape='L';
     secParam=[ p1' p2' norm(p1'-p2')]; % last parameter is sector length
     obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
     nID1=93; nID2=94; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
     secShape='L';
     secParam=[ p1' p2' norm(p1'-p2')]; % last parameter is sector length
     obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
     nID1=93; nID2=95; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
     secShape='L';
     secParam=[ p1' p2' norm(p1'-p2')]; % last parameter is sector length
     obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
     %13-19
     nID1=42; nID2=81; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
     secShape=['L','A','L'];
     secParam=[ p1' p1'+[4.5,0] 4.5;...
                p1'+[4.5,0] 0 -pi/2 pi/2;...
                p2'+[0,1] p2' 1]; % last parameter is sector length
     obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
     nID1=43; nID2=80; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
     secShape=['L','A','L'];
     secParam=[ p1' p1'+[4.5,0] 4.5;...
                p1'+[4.5,0] 0 -pi/2 pi/4;...
                p2'+[0,1] p2' 1]; % last parameter is sector length
     obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
     %19
     nID1=80; nID2=82; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
     secShape='L';
     secParam=[ p1' p2' norm(p1'-p2')]; % last parameter is sector length
     obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
     nID1=80; nID2=83; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
     secShape='L';
     secParam=[ p1' p2' norm(p1'-p2')]; % last parameter is sector length
     obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
     nID1=80; nID2=84; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
     secShape=['L','A'];
     secParam=[ p1' p1'-[0,0.5] 0.5;...
                p1'-[0,0.5] -pi/2 -pi/2 pi/4]; % last parameter is sector length
     obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
     nID1=80; nID2=85; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
     secShape='A';
     secParam=[ p1' -pi/2 -pi/2 pi/4]; % last parameter is sector length
     obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
     nID1=81; nID2=82; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
     secShape='L';
     secParam=[ p1' p2' norm(p1'-p2')]; % last parameter is sector length
     obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
     nID1=81; nID2=83; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
     secShape='L';
     secParam=[ p1' p2' norm(p1'-p2')]; % last parameter is sector length
     obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
     nID1=81; nID2=84; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
     secShape='A';
     secParam=[ p1' -pi/2 -pi/2 pi/2]; % last parameter is sector length
     obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
     nID1=81; nID2=85; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
     secShape=['A','L'];
     secParam=[ p1' -pi/2 -pi/2 pi/4;...
                p2'+[0.5,0] p2' 0.5]; % last parameter is sector length
     obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
     nID1=82; nID2=84; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
     secShape=['A','L'];
     secParam=[ p1' pi/2 pi/2 pi/4;...
                p2'+[0.5,0] p2' 0.5]; % last parameter is sector length
     obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
     nID1=82; nID2=85; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
     secShape='A';
     secParam=[ p1' pi/2 pi/2 pi/2]; % last parameter is sector length
     obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
     nID1=83; nID2=84; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
     secShape='A';
     secParam=[ p1' pi/2 pi/2 pi/4]; % last parameter is sector length
     obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
     nID1=83; nID2=85; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
     secShape=['L','A'];
     secParam=[ p1' p1'+[0,0.5] 0.5;...
                p1'+[0,0.5] pi/2 pi/2 pi/4]; % last parameter is sector length
     obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
     %17-19
     nID1=56; nID2=85; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
     secShape='L';
     secParam=[ p1' p2' norm(p1'-p2')]; % last parameter is sector length
     obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
     nID1=57; nID2=84; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
     secShape='L';
     secParam=[ p1' p2' norm(p1'-p2')]; % last parameter is sector length
     obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
     %18-19
     nID1=90; nID2=83; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
     secShape='L';
     secParam=[ p1' p2' norm(p1'-p2')]; % last parameter is sector length
     obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
     nID1=91; nID2=82; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
     secShape='L';
     secParam=[ p1' p2' norm(p1'-p2')]; % last parameter is sector length
     obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
     %

     obj.precomputeMap();
     obj.drawMap();
end

function createSimpleMap5(obj)
     coor=nan(2,100);
     coor(:,1)=[0;0]*2;
     coor(:,2)=[1;0]*2;
     coor(:,3)=[2;0]*2;
     coor(:,4)=[0;1]*2;
     coor(:,5)=[1;1]*2;
     coor(:,6)=[2;1]*2;
     coor(:,7)=[0;2]*2;
     coor(:,8)=[1;2]*2;
     coor(:,9)=[2;2]*2;
     
     nID1=1; nID2=4; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
     secShape='L';
     secParam=[ p1' p2' norm(p1'-p2')]; % last parameter is sector length
     obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
     nID1=2; nID2=5; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
     secShape='L';
     secParam=[ p1' p2' norm(p1'-p2')]; % last parameter is sector length
     obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
     nID1=3; nID2=6; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
     secShape='L';
     secParam=[ p1' p2' norm(p1'-p2')]; % last parameter is sector length
     obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
     nID1=4; nID2=5; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
     secShape='L';
     secParam=[ p1' p2' norm(p1'-p2')]; % last parameter is sector length
     obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
     nID1=5; nID2=6; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
     secShape='L';
     secParam=[ p1' p2' norm(p1'-p2')]; % last parameter is sector length
     obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
     nID1=4; nID2=7; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
     secShape='L';
     secParam=[ p1' p2' norm(p1'-p2')]; % last parameter is sector length
     obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
     nID1=5; nID2=8; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
     secShape='L';
     secParam=[ p1' p2' norm(p1'-p2')]; % last parameter is sector length
     obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
     nID1=6; nID2=9; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
     secShape='L';
     secParam=[ p1' p2' norm(p1'-p2')]; % last parameter is sector length
     obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
     nID1=7; nID2=8; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
     secShape='L';
     secParam=[ p1' p2' norm(p1'-p2')]; % last parameter is sector length
     obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
     nID1=8; nID2=9; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
     secShape='L';
     secParam=[ p1' p2' norm(p1'-p2')]; % last parameter is sector length
     obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
     nID1=1; nID2=2; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
     secShape='L';
     secParam=[ p1' p2' norm(p1'-p2')]; % last parameter is sector length
     obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
     nID1=2; nID2=3; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
     secShape='L';
     secParam=[ p1' p2' norm(p1'-p2')]; % last parameter is sector length
     obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
     
     obj.fontSize=20;
     obj.precomputeMap();
     obj.drawMap();
end

function createSimpleMap6(obj)
     coor=nan(2,100);
     coor(:,1)=[0;0];
     coor(:,2)=[1;0];
     coor(:,3)=[2;0];
     coor(:,4)=[3;0];
     coor(:,5)=[4;0];
     coor(:,6)=[5;0];
     coor(:,7)=[6;0];
     coor(:,8)=[7;0];
     coor(:,9)=[8;0];
     coor(:,10)=[9;0];
     coor(:,11)=[0;1];
     coor(:,12)=[1;1];
     coor(:,13)=[2;1];
     coor(:,14)=[3;1];
     coor(:,15)=[4;1];
     coor(:,16)=[5;1];
     coor(:,17)=[6;1];
     coor(:,18)=[7;1];
     coor(:,19)=[8;1];
     coor(:,20)=[9;1];
     coor(:,21)=[0;2];
     coor(:,22)=[1;2];
     coor(:,23)=[2;2];
     coor(:,24)=[3;2];
     coor(:,25)=[4;2];
     coor(:,26)=[5;2];
     coor(:,27)=[6;2];
     coor(:,28)=[7;2];
     coor(:,29)=[8;2];
     coor(:,30)=[9;2];
     coor(:,31)=[0;3];
     coor(:,32)=[1;3];
     coor(:,33)=[2;3];
     coor(:,34)=[3;3];
     coor(:,35)=[4;3];
     coor(:,36)=[5;3];
     coor(:,37)=[6;3];
     coor(:,38)=[7;3];
     coor(:,39)=[8;3];
     coor(:,40)=[9;3];
     coor(:,41)=[0;4];
     coor(:,42)=[1;4];
     coor(:,43)=[2;4];
     coor(:,44)=[3;4];
     coor(:,45)=[4;4];
     coor(:,46)=[5;4];
     coor(:,47)=[6;4];
     coor(:,48)=[7;4];
     coor(:,49)=[8;4];
     coor(:,50)=[9;4];
     coor(:,51)=[0;5];
     coor(:,52)=[1;5];
     coor(:,53)=[2;5];
     coor(:,54)=[3;5];
     coor(:,55)=[4;5];
     coor(:,56)=[5;5];
     coor(:,57)=[6;5];
     coor(:,58)=[7;5];
     coor(:,59)=[8;5];
     coor(:,60)=[9;5];
     coor(:,61)=[0;6];
     coor(:,62)=[1;6];
     coor(:,63)=[2;6];
     coor(:,64)=[3;6];
     coor(:,65)=[4;6];
     coor(:,66)=[5;6];
     coor(:,67)=[6;6];
     coor(:,68)=[7;6];
     coor(:,69)=[8;6];
     coor(:,70)=[9;6];
     coor(:,71)=[0;7];
     coor(:,72)=[1;7];
     coor(:,73)=[2;7];
     coor(:,74)=[3;7];
     coor(:,75)=[4;7];
     coor(:,76)=[5;7];
     coor(:,77)=[6;7];
     coor(:,78)=[7;7];
     coor(:,79)=[8;7];
     coor(:,80)=[9;7];
     coor(:,81)=[0;8];
     coor(:,82)=[1;8];
     coor(:,83)=[2;8];
     coor(:,84)=[3;8];
     coor(:,85)=[4;8];
     coor(:,86)=[5;8];
     coor(:,87)=[6;8];
     coor(:,88)=[7;8];
     coor(:,89)=[8;8];
     coor(:,90)=[9;8];
     coor(:,91)=[0;9];
     coor(:,92)=[1;9];
     coor(:,93)=[2;9];
     coor(:,94)=[3;9];
     coor(:,95)=[4;9];
     coor(:,96)=[5;9];
     coor(:,97)=[6;9];
     coor(:,98)=[7;9];
     coor(:,99)=[8;9];
     coor(:,100)=[9;9];
     
     nID1=1; nID2=11; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
     secShape='L';
     secParam=[ p1' p2' norm(p1'-p2')]; % last parameter is sector length
     obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
     nID1=2; nID2=12; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
     secShape='L';
     secParam=[ p1' p2' norm(p1'-p2')]; % last parameter is sector length
     obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
     nID1=3; nID2=13; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
     secShape='L';
     secParam=[ p1' p2' norm(p1'-p2')]; % last parameter is sector length
     obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
     nID1=4; nID2=14; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
     secShape='L';
     secParam=[ p1' p2' norm(p1'-p2')]; % last parameter is sector length
     obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
     nID1=5; nID2=15; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
     secShape='L';
     secParam=[ p1' p2' norm(p1'-p2')]; % last parameter is sector length
     obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
     nID1=6; nID2=16; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
     secShape='L';
     secParam=[ p1' p2' norm(p1'-p2')]; % last parameter is sector length
     obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
     nID1=7; nID2=17; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
     secShape='L';
     secParam=[ p1' p2' norm(p1'-p2')]; % last parameter is sector length
     obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
     nID1=8; nID2=18; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
     secShape='L';
     secParam=[ p1' p2' norm(p1'-p2')]; % last parameter is sector length
     obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
     nID1=9; nID2=19; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
     secShape='L';
     secParam=[ p1' p2' norm(p1'-p2')]; % last parameter is sector length
     obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
     
     nID1=10; nID2=20; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
     secShape='L';
     secParam=[ p1' p2' norm(p1'-p2')]; % last parameter is sector length
     obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
     nID1=11; nID2=21; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
     secShape='L';
     secParam=[ p1' p2' norm(p1'-p2')]; % last parameter is sector length
     obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
     nID1=12; nID2=22; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
     secShape='L';
     secParam=[ p1' p2' norm(p1'-p2')]; % last parameter is sector length
     obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
     nID1=13; nID2=23; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
     secShape='L';
     secParam=[ p1' p2' norm(p1'-p2')]; % last parameter is sector length
     obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
     nID1=14; nID2=24; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
     secShape='L';
     secParam=[ p1' p2' norm(p1'-p2')]; % last parameter is sector length
     obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
     nID1=15; nID2=25; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
     secShape='L';
     secParam=[ p1' p2' norm(p1'-p2')]; % last parameter is sector length
     obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
     nID1=16; nID2=26; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
     secShape='L';
     secParam=[ p1' p2' norm(p1'-p2')]; % last parameter is sector length
     obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
     nID1=17; nID2=27; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
     secShape='L';
     secParam=[ p1' p2' norm(p1'-p2')]; % last parameter is sector length
     obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
     nID1=18; nID2=28; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
     secShape='L';
     secParam=[ p1' p2' norm(p1'-p2')]; % last parameter is sector length
     obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
     nID1=19; nID2=29; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
     secShape='L';
     secParam=[ p1' p2' norm(p1'-p2')]; % last parameter is sector length
     obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
     
          nID1=20; nID2=30; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
     secShape='L';
     secParam=[ p1' p2' norm(p1'-p2')]; % last parameter is sector length
     obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
     nID1=21; nID2=31; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
     secShape='L';
     secParam=[ p1' p2' norm(p1'-p2')]; % last parameter is sector length
     obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
     nID1=22; nID2=32; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
     secShape='L';
     secParam=[ p1' p2' norm(p1'-p2')]; % last parameter is sector length
     obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
     nID1=23; nID2=33; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
     secShape='L';
     secParam=[ p1' p2' norm(p1'-p2')]; % last parameter is sector length
     obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
     nID1=24; nID2=34; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
     secShape='L';
     secParam=[ p1' p2' norm(p1'-p2')]; % last parameter is sector length
     obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
     nID1=25; nID2=35; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
     secShape='L';
     secParam=[ p1' p2' norm(p1'-p2')]; % last parameter is sector length
     obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
     nID1=26; nID2=36; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
     secShape='L';
     secParam=[ p1' p2' norm(p1'-p2')]; % last parameter is sector length
     obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
     nID1=27; nID2=37; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
     secShape='L';
     secParam=[ p1' p2' norm(p1'-p2')]; % last parameter is sector length
     obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
     nID1=28; nID2=38; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
     secShape='L';
     secParam=[ p1' p2' norm(p1'-p2')]; % last parameter is sector length
     obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
     nID1=29; nID2=39; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
     secShape='L';
     secParam=[ p1' p2' norm(p1'-p2')]; % last parameter is sector length
     obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
     
          nID1=30; nID2=40; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
     secShape='L';
     secParam=[ p1' p2' norm(p1'-p2')]; % last parameter is sector length
     obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
     nID1=31; nID2=41; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
     secShape='L';
     secParam=[ p1' p2' norm(p1'-p2')]; % last parameter is sector length
     obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
     nID1=32; nID2=42; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
     secShape='L';
     secParam=[ p1' p2' norm(p1'-p2')]; % last parameter is sector length
     obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
     nID1=33; nID2=43; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
     secShape='L';
     secParam=[ p1' p2' norm(p1'-p2')]; % last parameter is sector length
     obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
     nID1=34; nID2=44; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
     secShape='L';
     secParam=[ p1' p2' norm(p1'-p2')]; % last parameter is sector length
     obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
     nID1=35; nID2=45; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
     secShape='L';
     secParam=[ p1' p2' norm(p1'-p2')]; % last parameter is sector length
     obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
     nID1=36; nID2=46; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
     secShape='L';
     secParam=[ p1' p2' norm(p1'-p2')]; % last parameter is sector length
     obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
     nID1=37; nID2=47; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
     secShape='L';
     secParam=[ p1' p2' norm(p1'-p2')]; % last parameter is sector length
     obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
     nID1=38; nID2=48; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
     secShape='L';
     secParam=[ p1' p2' norm(p1'-p2')]; % last parameter is sector length
     obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
     nID1=39; nID2=49; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
     secShape='L';
     secParam=[ p1' p2' norm(p1'-p2')]; % last parameter is sector length
     obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
     
          nID1=40; nID2=50; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
     secShape='L';
     secParam=[ p1' p2' norm(p1'-p2')]; % last parameter is sector length
     obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
     nID1=41; nID2=51; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
     secShape='L';
     secParam=[ p1' p2' norm(p1'-p2')]; % last parameter is sector length
     obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
     nID1=42; nID2=52; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
     secShape='L';
     secParam=[ p1' p2' norm(p1'-p2')]; % last parameter is sector length
     obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
     nID1=43; nID2=53; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
     secShape='L';
     secParam=[ p1' p2' norm(p1'-p2')]; % last parameter is sector length
     obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
     nID1=44; nID2=54; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
     secShape='L';
     secParam=[ p1' p2' norm(p1'-p2')]; % last parameter is sector length
     obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
     nID1=45; nID2=55; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
     secShape='L';
     secParam=[ p1' p2' norm(p1'-p2')]; % last parameter is sector length
     obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
     nID1=46; nID2=56; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
     secShape='L';
     secParam=[ p1' p2' norm(p1'-p2')]; % last parameter is sector length
     obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
     nID1=47; nID2=57; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
     secShape='L';
     secParam=[ p1' p2' norm(p1'-p2')]; % last parameter is sector length
     obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
     nID1=48; nID2=58; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
     secShape='L';
     secParam=[ p1' p2' norm(p1'-p2')]; % last parameter is sector length
     obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
     nID1=49; nID2=59; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
     secShape='L';
     secParam=[ p1' p2' norm(p1'-p2')]; % last parameter is sector length
     obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
     
          nID1=50; nID2=60; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
     secShape='L';
     secParam=[ p1' p2' norm(p1'-p2')]; % last parameter is sector length
     obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
     nID1=51; nID2=61; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
     secShape='L';
     secParam=[ p1' p2' norm(p1'-p2')]; % last parameter is sector length
     obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
     nID1=52; nID2=62; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
     secShape='L';
     secParam=[ p1' p2' norm(p1'-p2')]; % last parameter is sector length
     obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
     nID1=53; nID2=63; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
     secShape='L';
     secParam=[ p1' p2' norm(p1'-p2')]; % last parameter is sector length
     obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
     nID1=54; nID2=64; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
     secShape='L';
     secParam=[ p1' p2' norm(p1'-p2')]; % last parameter is sector length
     obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
     nID1=55; nID2=65; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
     secShape='L';
     secParam=[ p1' p2' norm(p1'-p2')]; % last parameter is sector length
     obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
     nID1=56; nID2=66; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
     secShape='L';
     secParam=[ p1' p2' norm(p1'-p2')]; % last parameter is sector length
     obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
     nID1=57; nID2=67; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
     secShape='L';
     secParam=[ p1' p2' norm(p1'-p2')]; % last parameter is sector length
     obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
     nID1=58; nID2=68; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
     secShape='L';
     secParam=[ p1' p2' norm(p1'-p2')]; % last parameter is sector length
     obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
     nID1=59; nID2=69; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
     secShape='L';
     secParam=[ p1' p2' norm(p1'-p2')]; % last parameter is sector length
     obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
     
          nID1=60; nID2=70; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
     secShape='L';
     secParam=[ p1' p2' norm(p1'-p2')]; % last parameter is sector length
     obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
     nID1=61; nID2=71; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
     secShape='L';
     secParam=[ p1' p2' norm(p1'-p2')]; % last parameter is sector length
     obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
     nID1=62; nID2=72; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
     secShape='L';
     secParam=[ p1' p2' norm(p1'-p2')]; % last parameter is sector length
     obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
     nID1=63; nID2=73; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
     secShape='L';
     secParam=[ p1' p2' norm(p1'-p2')]; % last parameter is sector length
     obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
     nID1=64; nID2=74; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
     secShape='L';
     secParam=[ p1' p2' norm(p1'-p2')]; % last parameter is sector length
     obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
     nID1=65; nID2=75; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
     secShape='L';
     secParam=[ p1' p2' norm(p1'-p2')]; % last parameter is sector length
     obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
     nID1=66; nID2=76; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
     secShape='L';
     secParam=[ p1' p2' norm(p1'-p2')]; % last parameter is sector length
     obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
     nID1=67; nID2=77; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
     secShape='L';
     secParam=[ p1' p2' norm(p1'-p2')]; % last parameter is sector length
     obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
     nID1=68; nID2=78; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
     secShape='L';
     secParam=[ p1' p2' norm(p1'-p2')]; % last parameter is sector length
     obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
     nID1=69; nID2=79; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
     secShape='L';
     secParam=[ p1' p2' norm(p1'-p2')]; % last parameter is sector length
     obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
     
          nID1=70; nID2=80; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
     secShape='L';
     secParam=[ p1' p2' norm(p1'-p2')]; % last parameter is sector length
     obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
     nID1=71; nID2=81; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
     secShape='L';
     secParam=[ p1' p2' norm(p1'-p2')]; % last parameter is sector length
     obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
     nID1=72; nID2=82; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
     secShape='L';
     secParam=[ p1' p2' norm(p1'-p2')]; % last parameter is sector length
     obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
     nID1=73; nID2=83; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
     secShape='L';
     secParam=[ p1' p2' norm(p1'-p2')]; % last parameter is sector length
     obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
     nID1=74; nID2=84; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
     secShape='L';
     secParam=[ p1' p2' norm(p1'-p2')]; % last parameter is sector length
     obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
     nID1=75; nID2=85; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
     secShape='L';
     secParam=[ p1' p2' norm(p1'-p2')]; % last parameter is sector length
     obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
     nID1=76; nID2=86; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
     secShape='L';
     secParam=[ p1' p2' norm(p1'-p2')]; % last parameter is sector length
     obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
     nID1=77; nID2=87; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
     secShape='L';
     secParam=[ p1' p2' norm(p1'-p2')]; % last parameter is sector length
     obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
     nID1=78; nID2=88; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
     secShape='L';
     secParam=[ p1' p2' norm(p1'-p2')]; % last parameter is sector length
     obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
     nID1=79; nID2=89; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
     secShape='L';
     secParam=[ p1' p2' norm(p1'-p2')]; % last parameter is sector length
     obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
     
     nID1=80; nID2=90; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
     secShape='L';
     secParam=[ p1' p2' norm(p1'-p2')]; % last parameter is sector length
     obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
     nID1=81; nID2=91; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
     secShape='L';
     secParam=[ p1' p2' norm(p1'-p2')]; % last parameter is sector length
     obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
     nID1=82; nID2=92; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
     secShape='L';
     secParam=[ p1' p2' norm(p1'-p2')]; % last parameter is sector length
     obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
     nID1=83; nID2=93; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
     secShape='L';
     secParam=[ p1' p2' norm(p1'-p2')]; % last parameter is sector length
     obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
     nID1=84; nID2=94; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
     secShape='L';
     secParam=[ p1' p2' norm(p1'-p2')]; % last parameter is sector length
     obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
     nID1=85; nID2=95; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
     secShape='L';
     secParam=[ p1' p2' norm(p1'-p2')]; % last parameter is sector length
     obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
     nID1=86; nID2=96; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
     secShape='L';
     secParam=[ p1' p2' norm(p1'-p2')]; % last parameter is sector length
     obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
     nID1=87; nID2=97; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
     secShape='L';
     secParam=[ p1' p2' norm(p1'-p2')]; % last parameter is sector length
     obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
     nID1=88; nID2=98; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
     secShape='L';
     secParam=[ p1' p2' norm(p1'-p2')]; % last parameter is sector length
     obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
     nID1=89; nID2=99; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
     secShape='L';
     secParam=[ p1' p2' norm(p1'-p2')]; % last parameter is sector length
     obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
    
%      nID1=90; nID2=100; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
%      secShape='L';
%      secParam=[ p1' p2' norm(p1'-p2')]; % last parameter is sector length
%      obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
     
     
     nID1=01; nID2=2; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
     secShape='L';
     secParam=[ p1' p2' norm(p1'-p2')]; % last parameter is sector length
     obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
     nID1=02; nID2=3; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
     secShape='L';
     secParam=[ p1' p2' norm(p1'-p2')]; % last parameter is sector length
     obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
     nID1=03; nID2=4; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
     secShape='L';
     secParam=[ p1' p2' norm(p1'-p2')]; % last parameter is sector length
     obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
     nID1=04; nID2=5; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
     secShape='L';
     secParam=[ p1' p2' norm(p1'-p2')]; % last parameter is sector length
     obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
     nID1=05; nID2=6; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
     secShape='L';
     secParam=[ p1' p2' norm(p1'-p2')]; % last parameter is sector length
     obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
     nID1=06; nID2=7; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
     secShape='L';
     secParam=[ p1' p2' norm(p1'-p2')]; % last parameter is sector length
     obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
     nID1=07; nID2=8; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
     secShape='L';
     secParam=[ p1' p2' norm(p1'-p2')]; % last parameter is sector length
     obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
     nID1=8; nID2=9; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
     secShape='L';
     secParam=[ p1' p2' norm(p1'-p2')]; % last parameter is sector length
     obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
     nID1=9; nID2=10; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
     secShape='L';
     secParam=[ p1' p2' norm(p1'-p2')]; % last parameter is sector length
     obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
     
     nID1=11; nID2=12; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
     secShape='L';
     secParam=[ p1' p2' norm(p1'-p2')]; % last parameter is sector length
     obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
     nID1=12; nID2=13; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
     secShape='L';
     secParam=[ p1' p2' norm(p1'-p2')]; % last parameter is sector length
     obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
     nID1=13; nID2=14; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
     secShape='L';
     secParam=[ p1' p2' norm(p1'-p2')]; % last parameter is sector length
     obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
     nID1=14; nID2=15; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
     secShape='L';
     secParam=[ p1' p2' norm(p1'-p2')]; % last parameter is sector length
     obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
     nID1=15; nID2=16; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
     secShape='L';
     secParam=[ p1' p2' norm(p1'-p2')]; % last parameter is sector length
     obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
     nID1=16; nID2=17; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
     secShape='L';
     secParam=[ p1' p2' norm(p1'-p2')]; % last parameter is sector length
     obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
     nID1=17; nID2=18; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
     secShape='L';
     secParam=[ p1' p2' norm(p1'-p2')]; % last parameter is sector length
     obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
     nID1=18; nID2=19; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
     secShape='L';
     secParam=[ p1' p2' norm(p1'-p2')]; % last parameter is sector length
     obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
     nID1=19; nID2=20; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
     secShape='L';
     secParam=[ p1' p2' norm(p1'-p2')]; % last parameter is sector length
     obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
     
     nID1=21; nID2=22; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
     secShape='L';
     secParam=[ p1' p2' norm(p1'-p2')]; % last parameter is sector length
     obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
     nID1=22; nID2=23; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
     secShape='L';
     secParam=[ p1' p2' norm(p1'-p2')]; % last parameter is sector length
     obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
     nID1=23; nID2=24; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
     secShape='L';
     secParam=[ p1' p2' norm(p1'-p2')]; % last parameter is sector length
     obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
     nID1=24; nID2=25; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
     secShape='L';
     secParam=[ p1' p2' norm(p1'-p2')]; % last parameter is sector length
     obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
     nID1=25; nID2=26; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
     secShape='L';
     secParam=[ p1' p2' norm(p1'-p2')]; % last parameter is sector length
     obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
     nID1=26; nID2=27; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
     secShape='L';
     secParam=[ p1' p2' norm(p1'-p2')]; % last parameter is sector length
     obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
     nID1=27; nID2=28; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
     secShape='L';
     secParam=[ p1' p2' norm(p1'-p2')]; % last parameter is sector length
     obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
     nID1=28; nID2=29; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
     secShape='L';
     secParam=[ p1' p2' norm(p1'-p2')]; % last parameter is sector length
     obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
     nID1=29; nID2=30; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
     secShape='L';
     secParam=[ p1' p2' norm(p1'-p2')]; % last parameter is sector length
     obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
     
     nID1=31; nID2=32; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
     secShape='L';
     secParam=[ p1' p2' norm(p1'-p2')]; % last parameter is sector length
     obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
     nID1=32; nID2=33; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
     secShape='L';
     secParam=[ p1' p2' norm(p1'-p2')]; % last parameter is sector length
     obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
     nID1=33; nID2=34; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
     secShape='L';
     secParam=[ p1' p2' norm(p1'-p2')]; % last parameter is sector length
     obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
     nID1=34; nID2=35; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
     secShape='L';
     secParam=[ p1' p2' norm(p1'-p2')]; % last parameter is sector length
     obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
     nID1=35; nID2=36; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
     secShape='L';
     secParam=[ p1' p2' norm(p1'-p2')]; % last parameter is sector length
     obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
     nID1=36; nID2=37; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
     secShape='L';
     secParam=[ p1' p2' norm(p1'-p2')]; % last parameter is sector length
     obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
     nID1=37; nID2=38; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
     secShape='L';
     secParam=[ p1' p2' norm(p1'-p2')]; % last parameter is sector length
     obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
     nID1=38; nID2=39; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
     secShape='L';
     secParam=[ p1' p2' norm(p1'-p2')]; % last parameter is sector length
     obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
     nID1=39; nID2=40; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
     secShape='L';
     secParam=[ p1' p2' norm(p1'-p2')]; % last parameter is sector length
     obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
     
     nID1=41; nID2=42; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
     secShape='L';
     secParam=[ p1' p2' norm(p1'-p2')]; % last parameter is sector length
     obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
     nID1=42; nID2=43; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
     secShape='L';
     secParam=[ p1' p2' norm(p1'-p2')]; % last parameter is sector length
     obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
     nID1=43; nID2=44; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
     secShape='L';
     secParam=[ p1' p2' norm(p1'-p2')]; % last parameter is sector length
     obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
     nID1=44; nID2=45; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
     secShape='L';
     secParam=[ p1' p2' norm(p1'-p2')]; % last parameter is sector length
     obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
     nID1=45; nID2=46; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
     secShape='L';
     secParam=[ p1' p2' norm(p1'-p2')]; % last parameter is sector length
     obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
     nID1=46; nID2=47; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
     secShape='L';
     secParam=[ p1' p2' norm(p1'-p2')]; % last parameter is sector length
     obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
     nID1=47; nID2=48; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
     secShape='L';
     secParam=[ p1' p2' norm(p1'-p2')]; % last parameter is sector length
     obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
     nID1=48; nID2=49; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
     secShape='L';
     secParam=[ p1' p2' norm(p1'-p2')]; % last parameter is sector length
     obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
     nID1=49; nID2=50; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
     secShape='L';
     secParam=[ p1' p2' norm(p1'-p2')]; % last parameter is sector length
     obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
     
     nID1=51; nID2=52; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
     secShape='L';
     secParam=[ p1' p2' norm(p1'-p2')]; % last parameter is sector length
     obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
     nID1=52; nID2=53; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
     secShape='L';
     secParam=[ p1' p2' norm(p1'-p2')]; % last parameter is sector length
     obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
     nID1=53; nID2=54; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
     secShape='L';
     secParam=[ p1' p2' norm(p1'-p2')]; % last parameter is sector length
     obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
     nID1=54; nID2=55; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
     secShape='L';
     secParam=[ p1' p2' norm(p1'-p2')]; % last parameter is sector length
     obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
     nID1=55; nID2=56; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
     secShape='L';
     secParam=[ p1' p2' norm(p1'-p2')]; % last parameter is sector length
     obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
     nID1=56; nID2=57; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
     secShape='L';
     secParam=[ p1' p2' norm(p1'-p2')]; % last parameter is sector length
     obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
     nID1=57; nID2=58; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
     secShape='L';
     secParam=[ p1' p2' norm(p1'-p2')]; % last parameter is sector length
     obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
     nID1=58; nID2=59; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
     secShape='L';
     secParam=[ p1' p2' norm(p1'-p2')]; % last parameter is sector length
     obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
     nID1=59; nID2=60; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
     secShape='L';
     secParam=[ p1' p2' norm(p1'-p2')]; % last parameter is sector length
     obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
     
     nID1=61; nID2=62; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
     secShape='L';
     secParam=[ p1' p2' norm(p1'-p2')]; % last parameter is sector length
     obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
     nID1=62; nID2=63; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
     secShape='L';
     secParam=[ p1' p2' norm(p1'-p2')]; % last parameter is sector length
     obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
     nID1=63; nID2=64; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
     secShape='L';
     secParam=[ p1' p2' norm(p1'-p2')]; % last parameter is sector length
     obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
     nID1=64; nID2=65; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
     secShape='L';
     secParam=[ p1' p2' norm(p1'-p2')]; % last parameter is sector length
     obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
     nID1=65; nID2=66; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
     secShape='L';
     secParam=[ p1' p2' norm(p1'-p2')]; % last parameter is sector length
     obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
     nID1=66; nID2=67; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
     secShape='L';
     secParam=[ p1' p2' norm(p1'-p2')]; % last parameter is sector length
     obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
     nID1=67; nID2=68; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
     secShape='L';
     secParam=[ p1' p2' norm(p1'-p2')]; % last parameter is sector length
     obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
     nID1=68; nID2=69; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
     secShape='L';
     secParam=[ p1' p2' norm(p1'-p2')]; % last parameter is sector length
     obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
     nID1=69; nID2=70; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
     secShape='L';
     secParam=[ p1' p2' norm(p1'-p2')]; % last parameter is sector length
     obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
     
     nID1=71; nID2=72; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
     secShape='L';
     secParam=[ p1' p2' norm(p1'-p2')]; % last parameter is sector length
     obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
     nID1=72; nID2=73; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
     secShape='L';
     secParam=[ p1' p2' norm(p1'-p2')]; % last parameter is sector length
     obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
     nID1=73; nID2=74; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
     secShape='L';
     secParam=[ p1' p2' norm(p1'-p2')]; % last parameter is sector length
     obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
     nID1=74; nID2=75; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
     secShape='L';
     secParam=[ p1' p2' norm(p1'-p2')]; % last parameter is sector length
     obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
     nID1=75; nID2=76; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
     secShape='L';
     secParam=[ p1' p2' norm(p1'-p2')]; % last parameter is sector length
     obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
     nID1=76; nID2=77; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
     secShape='L';
     secParam=[ p1' p2' norm(p1'-p2')]; % last parameter is sector length
     obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
     nID1=77; nID2=78; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
     secShape='L';
     secParam=[ p1' p2' norm(p1'-p2')]; % last parameter is sector length
     obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
     nID1=78; nID2=79; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
     secShape='L';
     secParam=[ p1' p2' norm(p1'-p2')]; % last parameter is sector length
     obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
     nID1=79; nID2=80; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
     secShape='L';
     secParam=[ p1' p2' norm(p1'-p2')]; % last parameter is sector length
     obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
     
     nID1=81; nID2=82; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
     secShape='L';
     secParam=[ p1' p2' norm(p1'-p2')]; % last parameter is sector length
     obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
     nID1=82; nID2=83; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
     secShape='L';
     secParam=[ p1' p2' norm(p1'-p2')]; % last parameter is sector length
     obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
     nID1=83; nID2=84; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
     secShape='L';
     secParam=[ p1' p2' norm(p1'-p2')]; % last parameter is sector length
     obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
     nID1=84; nID2=85; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
     secShape='L';
     secParam=[ p1' p2' norm(p1'-p2')]; % last parameter is sector length
     obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
     nID1=85; nID2=86; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
     secShape='L';
     secParam=[ p1' p2' norm(p1'-p2')]; % last parameter is sector length
     obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
     nID1=86; nID2=87; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
     secShape='L';
     secParam=[ p1' p2' norm(p1'-p2')]; % last parameter is sector length
     obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
     nID1=87; nID2=88; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
     secShape='L';
     secParam=[ p1' p2' norm(p1'-p2')]; % last parameter is sector length
     obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
     nID1=88; nID2=89; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
     secShape='L';
     secParam=[ p1' p2' norm(p1'-p2')]; % last parameter is sector length
     obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
     nID1=89; nID2=90; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
     secShape='L';
     secParam=[ p1' p2' norm(p1'-p2')]; % last parameter is sector length
     obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
     
     nID1=91; nID2=92; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
     secShape='L';
     secParam=[ p1' p2' norm(p1'-p2')]; % last parameter is sector length
     obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
     nID1=92; nID2=93; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
     secShape='L';
     secParam=[ p1' p2' norm(p1'-p2')]; % last parameter is sector length
     obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
     nID1=93; nID2=94; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
     secShape='L';
     secParam=[ p1' p2' norm(p1'-p2')]; % last parameter is sector length
     obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
     nID1=94; nID2=95; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
     secShape='L';
     secParam=[ p1' p2' norm(p1'-p2')]; % last parameter is sector length
     obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
     nID1=95; nID2=96; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
     secShape='L';
     secParam=[ p1' p2' norm(p1'-p2')]; % last parameter is sector length
     obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
     nID1=96; nID2=97; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
     secShape='L';
     secParam=[ p1' p2' norm(p1'-p2')]; % last parameter is sector length
     obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
     nID1=97; nID2=98; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
     secShape='L';
     secParam=[ p1' p2' norm(p1'-p2')]; % last parameter is sector length
     obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
     nID1=98; nID2=99; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
     secShape='L';
     secParam=[ p1' p2' norm(p1'-p2')]; % last parameter is sector length
     obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
%      nID1=99; nID2=100; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
%      secShape='L';
%      secParam=[ p1' p2' norm(p1'-p2')]; % last parameter is sector length
%      obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
     
     obj.precomputeMap();
     obj.drawMap();
     
     
end

function createSimpleMap7(obj)
    coor=nan(2,100);
    coor(:,1)=[0;1];
    coor(:,2)=[1;1];
    coor(:,3)=[2;1];
    coor(:,4)=[3;1];
    coor(:,5)=[4;1];
    coor(:,6)=[5;1];
    coor(:,7)=[6;1];
    coor(:,8)=[7;1];
    coor(:,9)=[0;2];
    coor(:,10)=[5;2];
    coor(:,11)=[5;0];
    coor(:,12)=[6;0];
    coor(:,99)=[-11;1];
    coor(:,98)=[6;2];
    coor(:,97)=[-6;1];



    nID1=97; nID2=1; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
    secShape='L';
    secParam=[ p1' p2' norm(p1'-p2')]; % last parameter is sector length
    obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
    nID1=7; nID2=98; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
    secShape='L';
    secParam=[ p1' p2' norm(p1'-p2')]; % last parameter is sector length
    obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
    nID1=97; nID2=99; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
    secShape='L';
    secParam=[ p1' p2' norm(p1'-p2')]; % last parameter is sector length
    obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
    nID1=1; nID2=2; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
    secShape='L';
    secParam=[ p1' p2' norm(p1'-p2')]; % last parameter is sector length
    obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
    nID1=2; nID2=3; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
    secShape='L';
    secParam=[ p1' p2' norm(p1'-p2')]; % last parameter is sector length
    obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
    nID1=3; nID2=4; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
    secShape='L';
    secParam=[ p1' p2' norm(p1'-p2')]; % last parameter is sector length
    obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
    nID1=4; nID2=5; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
    secShape='L';
    secParam=[ p1' p2' norm(p1'-p2')]; % last parameter is sector length
    obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
    nID1=5; nID2=6; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
    secShape='L';
    secParam=[ p1' p2' norm(p1'-p2')]; % last parameter is sector length
    obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
    nID1=6; nID2=7; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
    secShape='L';
    secParam=[ p1' p2' norm(p1'-p2')]; % last parameter is sector length
    obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
    nID1=7; nID2=8; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
    secShape='L';
    secParam=[ p1' p2' norm(p1'-p2')]; % last parameter is sector length
    obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
    nID1=9; nID2=10; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
    secShape='L';
    secParam=[ p1' p2' norm(p1'-p2')]; % last parameter is sector length
    obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
    nID1=10; nID2=6; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
    secShape='L';
    secParam=[ p1' p2' norm(p1'-p2')]; % last parameter is sector length
    obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
    nID1=6; nID2=11; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
    secShape='L';
    secParam=[ p1' p2' norm(p1'-p2')]; % last parameter is sector length
    obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
    nID1=7; nID2=12; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
    secShape='L';
    secParam=[ p1' p2' norm(p1'-p2')]; % last parameter is sector length
    obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )

    obj.precomputeMap();
    obj.drawMap();


end


function createSimpleMap11(obj)
    coor=nan(2,100);
    coor(:,1)=[2;0];
    coor(:,2)=[2;2];
    coor(:,3)=[4;1];
    coor(:,4)=[4;2];
    coor(:,5)=[6;1];
    

    nID1=1; nID2=2; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
    secShape=['L','L','L'];
    secParam=[ p1' p1'-[2,0] norm(p1'-(p1'-[2,0]));...
               p1'-[2,0] p2'-[2,0] norm((p1'-[2,0])-(p2'-[2,0]));...
               p2'-[2,0] p2' norm((p2'-[2,0])-p2')]; % last parameter is sector length
    obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
    nID1=1; nID2=3; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
    secShape=['L','L'];
    secParam=[ p1' (p1'+[2,0]) norm(p1'-(p1'+[2,0]));...
               (p1'+[2,0]) p2' norm((p1'+[2,0])-p2')]; % last parameter is sector length
    obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
    nID1=2; nID2=4; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
    secShape='L';
    secParam=[ p1' p2' norm(p1'-p2')]; % last parameter is sector length
    obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
    nID1=3; nID2=4; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
    secShape='L';
    secParam=[ p1' p2' norm(p1'-p2')]; % last parameter is sector length
    obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
    nID1=3; nID2=5; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
    secShape='L';
    secParam=[ p1' p2' norm(p1'-p2')]; % last parameter is sector length
    obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
    
    obj.precomputeMap();
    obj.drawMap();
end

function createSimpleMap12(obj)
    coor=nan(2,100);
    coor(:,1)=[0;2];
    coor(:,2)=[0;4];
    coor(:,3)=[2;0];
    coor(:,4)=[2;2];
    coor(:,5)=[2;4];
    coor(:,6)=[4;2];

    nID1=1; nID2=2; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
    secShape='L';
    secParam=[ p1' p2' norm(p1'-p2')]; % last parameter is sector length
    obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
    nID1=1; nID2=4; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
    secShape='L';
    secParam=[ p1' p2' norm(p1'-p2')]; % last parameter is sector length
    obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
    nID1=2; nID2=5; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
    secShape='L';
    secParam=[ p1' p2' norm(p1'-p2')]; % last parameter is sector length
    obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
    nID1=3; nID2=4; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
    secShape='L';
    secParam=[ p1' p2' norm(p1'-p2')]; % last parameter is sector length
    obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
    nID1=4; nID2=5; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
    secShape='L';
    secParam=[ p1' p2' norm(p1'-p2')]; % last parameter is sector length
    obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
    nID1=4; nID2=6; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
    secShape='L';
    secParam=[ p1' p2' norm(p1'-p2')]; % last parameter is sector length
    obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
    
    obj.precomputeMap();
    obj.drawMap();
end
function createSimpleMap13(obj)
    coor=nan(2,100);
    coor(:,1)=[0;2];
    coor(:,2)=[2;0];
    coor(:,3)=[2;2];
    coor(:,4)=[2;4];
    coor(:,5)=[4;2];

    nID1=1; nID2=3; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
    secShape='L';
    secParam=[ p1' p2' norm(p1'-p2')]; % last parameter is sector length
    obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
    nID1=2; nID2=3; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
    secShape='L';
    secParam=[ p1' p2' norm(p1'-p2')]; % last parameter is sector length
    obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
    nID1=3; nID2=4; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
    secShape='L';
    secParam=[ p1' p2' norm(p1'-p2')]; % last parameter is sector length
    obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
    nID1=3; nID2=5; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
    secShape='L';
    secParam=[ p1' p2' norm(p1'-p2')]; % last parameter is sector length
    obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
    
    obj.precomputeMap();
    obj.drawMap();
end
function createSimpleMap14(obj)
    coor=nan(2,100);
    coor(:,1)=[0;0];
    coor(:,2)=[2;0];
    coor(:,3)=[2;2];
    coor(:,4)=[4;0];
    coor(:,5)=[4;2];
    coor(:,6)=[6;0];


    nID1=1; nID2=2; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
    secShape='L';
    secParam=[ p1' p2' norm(p1'-p2')]; % last parameter is sector length
    obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
    nID1=2; nID2=3; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
    secShape='L';
    secParam=[ p1' p2' norm(p1'-p2')]; % last parameter is sector length
    obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
    nID1=2; nID2=4; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=0; rID21=nID2*100+nID1;
    secShape='L';
    secParam=[ p1' p2' norm(p1'-p2')]; % last parameter is sector length
    obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
    nID1=3; nID2=5; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
    secShape='L';
    secParam=[ p1' p2' norm(p1'-p2')]; % last parameter is sector length
    obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
    nID1=4; nID2=5; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
    secShape='L';
    secParam=[ p1' p2' norm(p1'-p2')]; % last parameter is sector length
    obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
    nID1=4; nID2=6; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
    secShape='L';
    secParam=[ p1' p2' norm(p1'-p2')]; % last parameter is sector length
    obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
   
    obj.precomputeMap();
    obj.drawMap();
end
function createSimpleMap15(obj)
    coor=nan(2,100);
    coor(:,1)=[0;0];
    coor(:,2)=[2;0];
    coor(:,3)=[8;0];
    coor(:,4)=[2;2];
    coor(:,5)=[2;4];

    nID1=1; nID2=2; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
    secShape='L';
    secParam=[ p1' p2' norm(p1'-p2')]; % last parameter is sector length
    obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
    nID1=2; nID2=3; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
    secShape='L';
    secParam=[ p1' p2' norm(p1'-p2')]; % last parameter is sector length
    obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
    nID1=2; nID2=4; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
    secShape='L';
    secParam=[ p1' p2' norm(p1'-p2')]; % last parameter is sector length
    obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
    nID1=4; nID2=5; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
    secShape='L';
    secParam=[ p1' p2' norm(p1'-p2')]; % last parameter is sector length
    obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
    
    obj.precomputeMap();
    obj.drawMap();
end
function createSimpleMap16(obj)
    coor=nan(2,100);
    coor(:,1)=[0;2]/2;
    coor(:,2)=[10;2]/2;
    coor(:,3)=[12;0]/2;
    coor(:,4)=[12;2]/2;
    coor(:,5)=[12;4]/2;
    coor(:,6)=[10;4]/2;

    nID1=1; nID2=2; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
    secShape='L';
    secParam=[ p1' p2' norm(p1'-p2')]; % last parameter is sector length
    obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
%     nID1=2; nID2=6; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
%     secShape='L';
%     secParam=[ p1' p2' norm(p1'-p2')]; % last parameter is sector length
%     obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
    nID1=2; nID2=4; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
    secShape='L';
    secParam=[ p1' p2' norm(p1'-p2')]; % last parameter is sector length
    obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
    nID1=3; nID2=4; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
    secShape='L';
    secParam=[ p1' p2' norm(p1'-p2')]; % last parameter is sector length
    obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
    nID1=4; nID2=5; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
    secShape='L';
    secParam=[ p1' p2' norm(p1'-p2')]; % last parameter is sector length
    obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
    
    obj.precomputeMap();
    obj.drawMap();
end
function createSimpleMap17(obj)
    coor=nan(2,100);
    coor(:,1)=[0;2];
    coor(:,2)=[2;0];
    coor(:,3)=[2;2];
    coor(:,4)=[2;4];

    nID1=1; nID2=3; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
    secShape='L';
    secParam=[ p1' p2' norm(p1'-p2')]; % last parameter is sector length
    obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
    nID1=2; nID2=3; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
    secShape='L';
    secParam=[ p1' p2' norm(p1'-p2')]; % last parameter is sector length
    obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
    nID1=3; nID2=4; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
    secShape='L';
    secParam=[ p1' p2' norm(p1'-p2')]; % last parameter is sector length
    obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
    
    obj.precomputeMap();
    obj.drawMap();
end

function createSimpleMap8(obj)
    coor=nan(2,100);
    coor(:,1)=[0;0]*2;
    coor(:,2)=[0;2]*2;
    coor(:,3)=[2;1]*2;
    coor(:,4)=[2;2]*2;
    coor(:,5)=[3;4]*2;
    coor(:,6)=[4;1]*2;
    coor(:,7)=[4;2]*2;
    coor(:,8)=[4;4]*2;
    coor(:,9)=[4;6]*2;
    coor(:,10)=[6;2]*2;
    coor(:,11)=[6;4]*2;
    coor(:,12)=[8;0]*2;
    coor(:,13)=[8;1]*2;
    coor(:,14)=[10;1]*2;
    coor(:,15)=[10;2]*2;
    coor(:,16)=[12;0]*2;
    coor(:,17)=[12;2]*2;
    coor(:,18)=[12;4]*2;
    coor(:,19)=[14;0]*2;

    nID1=1; nID2=2; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
    secShape='L';
    secParam=[ p1' p2' norm(p1'-p2')]; % last parameter is sector length
    obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
    nID1=2; nID2=4; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
    secShape='L';
    secParam=[ p1' p2' norm(p1'-p2')]; % last parameter is sector length
    obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
    nID1=2; nID2=9; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
    secShape=['L','A','L'];
    secParam=[ p1' p1'+[0,4-1]*2 norm(p1'-(p1'+[0,4-1]*2));...
               p1'+[0,4-1]*2 pi/2 -pi/2 pi;...
               p1'+[0+1,4]*2 p2' norm(p2'-(p1'+[0+1,4]*2))]; % last parameter is sector length
    obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
    nID1=3; nID2=4; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
    secShape='L';
    secParam=[ p1' p2' norm(p1'-p2')]; % last parameter is sector length
    obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
    nID1=4; nID2=7; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
    secShape='L';
    secParam=[ p1' p2' norm(p1'-p2')]; % last parameter is sector length
    obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
    nID1=6; nID2=7; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
    secShape='L';
    secParam=[ p1' p2' norm(p1'-p2')]; % last parameter is sector length
    obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
    nID1=7; nID2=10; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
    secShape='L';
    secParam=[ p1' p2' norm(p1'-p2')]; % last parameter is sector length
    obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
    nID1=5; nID2=8; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
    secShape='L';
    secParam=[ p1' p2' norm(p1'-p2')]; % last parameter is sector length
    obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
    nID1=8; nID2=9; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
    secShape='L';
    secParam=[ p1' p2' norm(p1'-p2')]; % last parameter is sector length
    obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
    nID1=8; nID2=11; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
    secShape='L';
    secParam=[ p1' p2' norm(p1'-p2')]; % last parameter is sector length
    obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
    nID1=9; nID2=18; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
    secShape=['L','A','L'];
    secParam=[ p1' p1'+[8-1,0]*2 norm(p1'-(p1'+[8-1,0]*2));...
               p1'+[8-1,0]*2 0 -pi/2 pi;...
               p1'+[8,0-1]*2 p2' norm(p2'-(p1'+[8,0-1]*2))]; % last parameter is sector length
    obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
    nID1=10; nID2=11; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
    secShape='L';
    secParam=[ p1' p2' norm(p1'-p2')]; % last parameter is sector length
    obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
    nID1=10; nID2=15; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
    secShape='L';
    secParam=[ p1' p2' norm(p1'-p2')]; % last parameter is sector length
    obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
    nID1=10; nID2=12; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
    secShape=['L','A','L'];
    secParam=[ p1' p1'+[0,-2+1]*2 norm(p1'-(p1'+[0,-2+1]*2));...
               p1'+[0,-2+1]*2 -pi/2 pi/2 pi;...
               p1'+[0+1,-2]*2 p2' norm(p2'-(p1'+[0+1,-2]*2))]; % last parameter is sector length
    obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
    nID1=11; nID2=18; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
    secShape='L';
    secParam=[ p1' p2' norm(p1'-p2')]; % last parameter is sector length
    obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
    nID1=12; nID2=13; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
    secShape='L';
    secParam=[ p1' p2' norm(p1'-p2')]; % last parameter is sector length
    obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
    nID1=12; nID2=16; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
    secShape='L';
    secParam=[ p1' p2' norm(p1'-p2')]; % last parameter is sector length
    obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
    nID1=14; nID2=15; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
    secShape='L';
    secParam=[ p1' p2' norm(p1'-p2')]; % last parameter is sector length
    obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
    nID1=15; nID2=17; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
    secShape='L';
    secParam=[ p1' p2' norm(p1'-p2')]; % last parameter is sector length
    obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
    nID1=16; nID2=17; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
    secShape='L';
    secParam=[ p1' p2' norm(p1'-p2')]; % last parameter is sector length
    obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
    nID1=17; nID2=18; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
    secShape='L';
    secParam=[ p1' p2' norm(p1'-p2')]; % last parameter is sector length
    obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
    nID1=16; nID2=19; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
    secShape='L';
    secParam=[ p1' p2' norm(p1'-p2')]; % last parameter is sector length
    obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
   
    %obj.fontSize=140;
    obj.precomputeMap();
    obj.drawMap();
end

function createSimpleMap8a(obj)
    coor=nan(2,100);
    coor(:,1)=[0;0]*2;
    coor(:,2)=[0;2]*2;
    coor(:,3)=[2;1]*2;
    coor(:,4)=[2;2]*2;
    coor(:,5)=[3;4]*2;
    coor(:,6)=[4;1]*2;
    coor(:,7)=[4;2]*2;
    coor(:,8)=[4;4]*2;
    coor(:,9)=[4;6]*2;
    coor(:,10)=[6;2]*2;
    coor(:,11)=[6;4]*2;
    coor(:,12)=[8;0]*2;
    coor(:,13)=[8;1]*2;
    coor(:,14)=[10;1]*2;
    coor(:,15)=[10;2]*2;
    coor(:,16)=[12;0]*2;
    coor(:,17)=[12;2]*2;
    coor(:,18)=[12;4]*2;
    coor(:,19)=[14;0]*2;

    coor(:,20)=[1;4];
    coor(:,21)=[2;4];
    coor(:,22)=[3;4];
  
    
    nID1=1; nID2=2; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
    secShape='L';
    secParam=[ p1' p2' norm(p1'-p2')]; % last parameter is sector length
    obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
    
    % dodam vozlisca
%     nID1=2; nID2=4; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
%     secShape='L';
%     secParam=[ p1' p2' norm(p1'-p2')]; % last parameter is sector length
%     obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
    nID1=2; nID2=20; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
    secShape='L';
    secParam=[ p1' p2' norm(p1'-p2')]; % last parameter is sector length
    obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
    
    nID1=20; nID2=21; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
    secShape='L';
    secParam=[ p1' p2' norm(p1'-p2')]; % last parameter is sector length
    obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
    
    nID1=21; nID2=22; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
    secShape='L';
    secParam=[ p1' p2' norm(p1'-p2')]; % last parameter is sector length
    obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )

    nID1=22; nID2=4; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
    secShape='L';
    secParam=[ p1' p2' norm(p1'-p2')]; % last parameter is sector length
    obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
    %
    
    nID1=2; nID2=9; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
    secShape=['L','A','L'];
    secParam=[ p1' p1'+[0,4-1]*2 norm(p1'-(p1'+[0,4-1]*2));...
               p1'+[0,4-1]*2 pi/2 -pi/2 pi;...
               p1'+[0+1,4]*2 p2' norm(p2'-(p1'+[0+1,4]*2))]; % last parameter is sector length
    obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
    nID1=3; nID2=4; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
    secShape='L';
    secParam=[ p1' p2' norm(p1'-p2')]; % last parameter is sector length
    obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
    nID1=4; nID2=7; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
    secShape='L';
    secParam=[ p1' p2' norm(p1'-p2')]; % last parameter is sector length
    obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
    nID1=6; nID2=7; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
    secShape='L';
    secParam=[ p1' p2' norm(p1'-p2')]; % last parameter is sector length
    obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
    nID1=7; nID2=10; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
    secShape='L';
    secParam=[ p1' p2' norm(p1'-p2')]; % last parameter is sector length
    obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
    nID1=5; nID2=8; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
    secShape='L';
    secParam=[ p1' p2' norm(p1'-p2')]; % last parameter is sector length
    obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
    nID1=8; nID2=9; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
    secShape='L';
    secParam=[ p1' p2' norm(p1'-p2')]; % last parameter is sector length
    obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
    nID1=8; nID2=11; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
    secShape='L';
    secParam=[ p1' p2' norm(p1'-p2')]; % last parameter is sector length
    obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
    nID1=9; nID2=18; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
    secShape=['L','A','L'];
    secParam=[ p1' p1'+[8-1,0]*2 norm(p1'-(p1'+[8-1,0]*2));...
               p1'+[8-1,0]*2 0 -pi/2 pi;...
               p1'+[8,0-1]*2 p2' norm(p2'-(p1'+[8,0-1]*2))]; % last parameter is sector length
    obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
    nID1=10; nID2=11; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
    secShape='L';
    secParam=[ p1' p2' norm(p1'-p2')]; % last parameter is sector length
    obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
    nID1=10; nID2=15; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
    secShape='L';
    secParam=[ p1' p2' norm(p1'-p2')]; % last parameter is sector length
    obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
    nID1=10; nID2=12; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
    secShape=['L','A','L'];
    secParam=[ p1' p1'+[0,-2+1]*2 norm(p1'-(p1'+[0,-2+1]*2));...
               p1'+[0,-2+1]*2 -pi/2 pi/2 pi;...
               p1'+[0+1,-2]*2 p2' norm(p2'-(p1'+[0+1,-2]*2))]; % last parameter is sector length
    obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
    nID1=11; nID2=18; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
    secShape='L';
    secParam=[ p1' p2' norm(p1'-p2')]; % last parameter is sector length
    obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
    nID1=12; nID2=13; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
    secShape='L';
    secParam=[ p1' p2' norm(p1'-p2')]; % last parameter is sector length
    obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
    nID1=12; nID2=16; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
    secShape='L';
    secParam=[ p1' p2' norm(p1'-p2')]; % last parameter is sector length
    obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
    nID1=14; nID2=15; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
    secShape='L';
    secParam=[ p1' p2' norm(p1'-p2')]; % last parameter is sector length
    obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
    nID1=15; nID2=17; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
    secShape='L';
    secParam=[ p1' p2' norm(p1'-p2')]; % last parameter is sector length
    obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
    nID1=16; nID2=17; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
    secShape='L';
    secParam=[ p1' p2' norm(p1'-p2')]; % last parameter is sector length
    obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
    nID1=17; nID2=18; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
    secShape='L';
    secParam=[ p1' p2' norm(p1'-p2')]; % last parameter is sector length
    obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
    nID1=16; nID2=19; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
    secShape='L';
    secParam=[ p1' p2' norm(p1'-p2')]; % last parameter is sector length
    obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
   
    obj.fontSize=140;
    obj.precomputeMap();
    obj.drawMap();
end

function createSimpleMap8b(obj)
    coor=nan(2,100);
    coor(:,1)=[0;0]*2;
    coor(:,2)=[0;2]*2;
    coor(:,3)=[2;0]*2;
    coor(:,4)=[2;2]*2;
    coor(:,5)=[3;4]*2;
    coor(:,6)=[4;1]*2;
    coor(:,7)=[4;2]*2;
    coor(:,8)=[4;4]*2;
    coor(:,9)=[4;6]*2;
    coor(:,10)=[6;2]*2;
    coor(:,11)=[6;4]*2;
    coor(:,12)=[8;0]*2;
    coor(:,13)=[8;1]*2;
    coor(:,14)=[10;1]*2;
    coor(:,15)=[10;2]*2;
    coor(:,16)=[12;0]*2;
    coor(:,17)=[12;2]*2;
    coor(:,18)=[12;4]*2;
    coor(:,19)=[14;0]*2;
    
    nID1=20; nID2=16; p1=[24;-2]; p2=[24;0]; rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
    secShape='L'; secParam=[ p1' p2' norm(p1'-p2')];
    obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )

    nID1=1; nID2=2; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
    secShape='L';
    secParam=[ p1' p2' norm(p1'-p2')]; % last parameter is sector length
    obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
    
    % Vzporedna cesta prejsnji cesti (krozni odsek)
    nID1=1; nID2=2; p1=[0;0]; p2=[0;4]; rID12=1200; twoway=0; rID21=0012;
    secShape=['A']; secParam=[0 0 -pi -pi pi*2];
    obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )

    nID1=2; nID2=4; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
    secShape='L';
    secParam=[ p1' p2' norm(p1'-p2')]; % last parameter is sector length
    obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )

    nID1=2; nID2=9; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
    secShape=['L','A','L'];
    secParam=[ p1' p1'+[0,4-1]*2 norm(p1'-(p1'+[0,4-1]*2));...
               p1'+[0,4-1]*2 pi/2 -pi/2 pi;...
               p1'+[0+1,4]*2 p2' norm(p2'-(p1'+[0+1,4]*2))]; % last parameter is sector length
    obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )

    nID1=3; nID2=4; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
    secShape='L';
    secParam=[ p1' p2' norm(p1'-p2')]; % last parameter is sector length
    obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )

    nID1=4; nID2=7; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
    secShape='L';
    secParam=[ p1' p2' norm(p1'-p2')]; % last parameter is sector length
    obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
    nID1=6; nID2=7; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
    secShape='L';
    secParam=[ p1' p2' norm(p1'-p2')]; % last parameter is sector length
    obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
    nID1=7; nID2=10; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
    secShape='L';
    secParam=[ p1' p2' norm(p1'-p2')]; % last parameter is sector length
    obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
    nID1=5; nID2=8; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
    secShape='L';
    secParam=[ p1' p2' norm(p1'-p2')]; % last parameter is sector length
    obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
    nID1=8; nID2=9; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
    secShape='L';
    secParam=[ p1' p2' norm(p1'-p2')]; % last parameter is sector length
    obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
    nID1=8; nID2=11; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
    secShape='L';
    secParam=[ p1' p2' norm(p1'-p2')]; % last parameter is sector length
    obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
    nID1=9; nID2=18; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
    secShape=['L','A','L'];
    secParam=[ p1' p1'+[8-1,0]*2 norm(p1'-(p1'+[8-1,0]*2));...
               p1'+[8-1,0]*2 0 -pi/2 pi;...
               p1'+[8,0-1]*2 p2' norm(p2'-(p1'+[8,0-1]*2))]; % last parameter is sector length
    obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
    nID1=10; nID2=11; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
    secShape='L';
    secParam=[ p1' p2' norm(p1'-p2')]; % last parameter is sector length
    obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
    nID1=10; nID2=15; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=0; rID21=nID2*100+nID1;
    secShape='L';
    secParam=[ p1' p2' norm(p1'-p2')]; % last parameter is sector length
    obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
    nID1=10; nID2=12; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
    secShape=['L','A','L'];
    secParam=[ p1' p1'+[0,-2+1]*2 norm(p1'-(p1'+[0,-2+1]*2));...
               p1'+[0,-2+1]*2 -pi/2 pi/2 pi;...
               p1'+[0+1,-2]*2 p2' norm(p2'-(p1'+[0+1,-2]*2))]; % last parameter is sector length
    obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
    nID1=11; nID2=18; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=0; rID21=nID2*100+nID1;
    secShape='L';
    secParam=[ p1' p2' norm(p1'-p2')]; % last parameter is sector length
    obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
    nID1=12; nID2=13; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
    secShape='L';
    secParam=[ p1' p2' norm(p1'-p2')]; % last parameter is sector length
    obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
    nID1=12; nID2=16; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
    secShape='L';
    secParam=[ p1' p2' norm(p1'-p2')]; % last parameter is sector length
    obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
    nID1=14; nID2=15; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
    secShape='L';
    secParam=[ p1' p2' norm(p1'-p2')]; % last parameter is sector length
    obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
    nID1=15; nID2=17; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
    secShape='L';
    secParam=[ p1' p2' norm(p1'-p2')]; % last parameter is sector length
    obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
    nID1=16; nID2=17; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
    secShape='L';
    secParam=[ p1' p2' norm(p1'-p2')]; % last parameter is sector length
    obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
    nID1=17; nID2=18; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
    secShape='L';
    secParam=[ p1' p2' norm(p1'-p2')]; % last parameter is sector length
    obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
    nID1=16; nID2=19; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
    secShape='L';
    secParam=[ p1' p2' norm(p1'-p2')]; % last parameter is sector length
    obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )

    nID1=1; nID2=12; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
    secShape=['A','L','A'];
    secParam=[ p1' -pi/2 pi/2 pi;...
               [2,-2] [14,-2] norm([14,-2]-[2,-2]);...
               [14,-2] 0 pi/2 pi]; % last parameter is sector length
    obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
   
    obj.fontSize=140;
    obj.precomputeMap();
    obj.drawMap();
end
function createSimpleMap9(obj)
    coor=nan(2,100);
    coor(:,1)=[0;0];
    coor(:,2)=[0;2];
    coor(:,3)=[0;3];
    coor(:,4)=[4;2];
    coor(:,5)=[4;1];
    coor(:,6)=[4;3];


    nID1=1; nID2=2; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
    secShape='L';
    secParam=[ p1' p2' norm(p1'-p2')]; % last parameter is sector length
    obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
    nID1=2; nID2=3; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
    secShape='L';
    secParam=[ p1' p2' norm(p1'-p2')]; % last parameter is sector length
    obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
    nID1=2; nID2=4; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
    secShape='L';
    secParam=[ p1' p2' norm(p1'-p2')]; % last parameter is sector length
    obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
    nID1=4; nID2=5; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
    secShape='L';
    secParam=[ p1' p2' norm(p1'-p2')]; % last parameter is sector length
    obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
    nID1=4; nID2=6; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
    secShape='L';
    secParam=[ p1' p2' norm(p1'-p2')]; % last parameter is sector length
    obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
    
    obj.precomputeMap();
    obj.drawMap();
end

function createSimpleMap19(obj)
    coor=nan(2,100);
    coor(:,1)=[0;0];
    coor(:,2)=[2;0];
    coor(:,3)=[5;0];
    coor(:,4)=[0;2];
    coor(:,5)=[2;2];
    coor(:,6)=[4;2];
    coor(:,7)=[2;6.5];


    nID1=1; nID2=2; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
    secShape='L';
    secParam=[ p1' p2' norm(p1'-p2')]; % last parameter is sector length
    obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
    nID1=3; nID2=2; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
    secShape='L';
    secParam=[ p1' p2' norm(p1'-p2')]; % last parameter is sector length
    obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
    nID1=2; nID2=5; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
    secShape='L';
    secParam=[ p1' p2' norm(p1'-p2')]; % last parameter is sector length
    obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
    nID1=4; nID2=5; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
    secShape='L';
    secParam=[ p1' p2' norm(p1'-p2')]; % last parameter is sector length
    obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
    nID1=5; nID2=6; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
    secShape='L';
    secParam=[ p1' p2' norm(p1'-p2')]; % last parameter is sector length
    obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
    nID1=5; nID2=7; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
    secShape='L';
    secParam=[ p1' p2' norm(p1'-p2')]; % last parameter is sector length
    obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
    
    obj.precomputeMap();
    obj.drawMap();
end

function createSimpleMap20(obj)
    coor=nan(2,100);
    coor(:,1)=[0;2];
    coor(:,2)=[0;4];
    coor(:,3)=[2;0];
    coor(:,4)=[2;2];
    coor(:,5)=[4;2];
    coor(:,6)=[7;2];
    coor(:,7)=[10;2];
%     coor=[coor(2,:);coor(1,:)];


    nID1=1; nID2=4; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
    secShape='L';
    secParam=[ p1' p2' norm(p1'-p2')]; % last parameter is sector length
    obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
    nID1=3; nID2=4; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
    secShape='L';
    secParam=[ p1' p2' norm(p1'-p2')]; % last parameter is sector length
    obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
    nID1=4; nID2=5; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
    secShape='L';
    secParam=[ p1' p2' norm(p1'-p2')]; % last parameter is sector length
    obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
    nID1=2; nID2=5; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
    secShape=['L','L'];
    secParam=[ p1' p1'+[4;0]' norm(p1'-(p1'+[4;0]'));...
               p1'+[4;0]' p2' norm((p1'+[4;0]')-p2')]; % last parameter is sector length
    obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
    nID1=5; nID2=6; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
    secShape='L';
    secParam=[ p1' p2' norm(p1'-p2')]; % last parameter is sector length
    obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
    nID1=6; nID2=7; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
    secShape='L';
    secParam=[ p1' p2' norm(p1'-p2')]; % last parameter is sector length
    obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
%     nID1=7; nID2=9; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
%     secShape='L';
%     secParam=[ p1' p2' norm(p1'-p2')]; % last parameter is sector length
%     obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
%     nID1=9; nID2=10; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
%     secShape='L';
%     secParam=[ p1' p2' norm(p1'-p2')]; % last parameter is sector length
%     obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
   
    obj.precomputeMap();
    obj.drawMap();
end

function createSimpleMap21(obj)
    coor=nan(2,100);
    coor(:,1)=[0;0]*2;
    coor(:,2)=[0;1]*2;
    coor(:,3)=[0;2]*2;
    coor(:,4)=[0;3]*2;
    coor(:,5)=[0;4]*2;
    coor(:,6)=[0;5]*2;
    coor(:,7)=[0;6]*2;
    coor(:,8)=[1+0.2;0]*2;
    coor(:,9)=[1+0.2;1]*2;
    coor(:,10)=[1+0.2;2]*2;
    coor(:,11)=[1+0.2;3]*2;
    coor(:,12)=[1+0.2;4]*2;
    coor(:,13)=[1+0.2;5]*2;
    coor(:,14)=[1+0.2;6]*2;
    coor(:,15)=[2+0.4;0]*2;
    coor(:,16)=[2+0.4;1]*2;
    coor(:,17)=[2+0.4;2]*2;
    coor(:,18)=[2+0.4;3]*2;
    coor(:,19)=[2+0.4;4]*2;
    coor(:,20)=[2+0.4;5]*2;
    coor(:,21)=[2+0.4;6]*2;
    coor(:,22)=[3+0.6;0]*2;
    coor(:,23)=[3+0.6;1]*2;
    coor(:,24)=[3+0.6;2]*2;
    coor(:,25)=[3+0.6;3]*2;
    coor(:,26)=[3+0.6;4]*2;
    coor(:,27)=[3+0.6;5]*2;
    coor(:,28)=[3+0.6;6]*2;
    coor=[coor(2,:);coor(1,:)];


    nID1=1; nID2=2; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
    secShape='L';
    secParam=[ p1' p2' norm(p1'-p2')]; % last parameter is sector length
    obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
    nID1=2; nID2=3; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
    secShape='L';
    secParam=[ p1' p2' norm(p1'-p2')]; % last parameter is sector length
    obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
    nID1=3; nID2=4; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
    secShape='L';
    secParam=[ p1' p2' norm(p1'-p2')]; % last parameter is sector length
    obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
    nID1=4; nID2=5; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
    secShape='L';
    secParam=[ p1' p2' norm(p1'-p2')]; % last parameter is sector length
    obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
    nID1=5; nID2=6; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
    secShape='L';
    secParam=[ p1' p2' norm(p1'-p2')]; % last parameter is sector length
    obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
    nID1=6; nID2=7; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
    secShape='L';
    secParam=[ p1' p2' norm(p1'-p2')]; % last parameter is sector length
    obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
    nID1=8; nID2=9; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
    secShape='L';
    secParam=[ p1' p2' norm(p1'-p2')]; % last parameter is sector length
    obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
    nID1=9; nID2=10; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
    secShape='L';
    secParam=[ p1' p2' norm(p1'-p2')]; % last parameter is sector length
    obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
    nID1=10; nID2=11; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
    secShape='L';
    secParam=[ p1' p2' norm(p1'-p2')]; % last parameter is sector length
    obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
    nID1=11; nID2=12; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
    secShape='L';
    secParam=[ p1' p2' norm(p1'-p2')]; % last parameter is sector length
    obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
    nID1=12; nID2=13; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
    secShape='L';
    secParam=[ p1' p2' norm(p1'-p2')]; % last parameter is sector length
    obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
    nID1=13; nID2=14; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
    secShape='L';
    secParam=[ p1' p2' norm(p1'-p2')]; % last parameter is sector length
    obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
    nID1=15; nID2=16; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
    secShape='L';
    secParam=[ p1' p2' norm(p1'-p2')]; % last parameter is sector length
    obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
    nID1=16; nID2=17; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
    secShape='L';
    secParam=[ p1' p2' norm(p1'-p2')]; % last parameter is sector length
    obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
    nID1=17; nID2=18; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
    secShape='L';
    secParam=[ p1' p2' norm(p1'-p2')]; % last parameter is sector length
    obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
    nID1=18; nID2=19; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
    secShape='L';
    secParam=[ p1' p2' norm(p1'-p2')]; % last parameter is sector length
    obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
    nID1=19; nID2=20; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
    secShape='L';
    secParam=[ p1' p2' norm(p1'-p2')]; % last parameter is sector length
    obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
    nID1=20; nID2=21; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
    secShape='L';
    secParam=[ p1' p2' norm(p1'-p2')]; % last parameter is sector length
    obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
    nID1=22; nID2=23; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
    secShape='L';
    secParam=[ p1' p2' norm(p1'-p2')]; % last parameter is sector length
    obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
    nID1=23; nID2=24; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
    secShape='L';
    secParam=[ p1' p2' norm(p1'-p2')]; % last parameter is sector length
    obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
    nID1=24; nID2=25; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
    secShape='L';
    secParam=[ p1' p2' norm(p1'-p2')]; % last parameter is sector length
    obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
    nID1=25; nID2=26; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
    secShape='L';
    secParam=[ p1' p2' norm(p1'-p2')]; % last parameter is sector length
    obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
    nID1=26; nID2=27; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
    secShape='L';
    secParam=[ p1' p2' norm(p1'-p2')]; % last parameter is sector length
    obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
    nID1=27; nID2=28; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
    secShape='L';
    secParam=[ p1' p2' norm(p1'-p2')]; % last parameter is sector length
    obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
    nID1=1; nID2=8; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
    secShape='L';
    secParam=[ p1' p2' norm(p1'-p2')]; % last parameter is sector length
    obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
    nID1=2; nID2=9; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
    secShape='L';
    secParam=[ p1' p2' norm(p1'-p2')]; % last parameter is sector length
    obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
    nID1=3; nID2=10; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
    secShape='L';
    secParam=[ p1' p2' norm(p1'-p2')]; % last parameter is sector length
    obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
    nID1=4; nID2=11; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
    secShape='L';
    secParam=[ p1' p2' norm(p1'-p2')]; % last parameter is sector length
    obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
    nID1=5; nID2=12; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
    secShape='L';
    secParam=[ p1' p2' norm(p1'-p2')]; % last parameter is sector length
    obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
    nID1=6; nID2=13; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
    secShape='L';
    secParam=[ p1' p2' norm(p1'-p2')]; % last parameter is sector length
    obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
    nID1=7; nID2=14; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
    secShape='L';
    secParam=[ p1' p2' norm(p1'-p2')]; % last parameter is sector length
    obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
    nID1=8; nID2=15; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
    secShape='L';
    secParam=[ p1' p2' norm(p1'-p2')]; % last parameter is sector length
    obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
    nID1=9; nID2=16; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
    secShape='L';
    secParam=[ p1' p2' norm(p1'-p2')]; % last parameter is sector length
    obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
    nID1=10; nID2=17; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
    secShape='L';
    secParam=[ p1' p2' norm(p1'-p2')]; % last parameter is sector length
    obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
    nID1=11; nID2=18; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
    secShape='L';
    secParam=[ p1' p2' norm(p1'-p2')]; % last parameter is sector length
    obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
    nID1=12; nID2=19; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
    secShape='L';
    secParam=[ p1' p2' norm(p1'-p2')]; % last parameter is sector length
    obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
    nID1=13; nID2=20; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
    secShape='L';
    secParam=[ p1' p2' norm(p1'-p2')]; % last parameter is sector length
    obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
    nID1=14; nID2=21; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
    secShape='L';
    secParam=[ p1' p2' norm(p1'-p2')]; % last parameter is sector length
    obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
    nID1=15; nID2=22; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
    secShape='L';
    secParam=[ p1' p2' norm(p1'-p2')]; % last parameter is sector length
    obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
    nID1=16; nID2=23; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
    secShape='L';
    secParam=[ p1' p2' norm(p1'-p2')]; % last parameter is sector length
    obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
    nID1=17; nID2=24; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
    secShape='L';
    secParam=[ p1' p2' norm(p1'-p2')]; % last parameter is sector length
    obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
    nID1=18; nID2=25; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
    secShape='L';
    secParam=[ p1' p2' norm(p1'-p2')]; % last parameter is sector length
    obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
    nID1=19; nID2=26; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
    secShape='L';
    secParam=[ p1' p2' norm(p1'-p2')]; % last parameter is sector length
    obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
    nID1=20; nID2=27; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
    secShape='L';
    secParam=[ p1' p2' norm(p1'-p2')]; % last parameter is sector length
    obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
    nID1=21; nID2=28; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
    secShape='L';
    secParam=[ p1' p2' norm(p1'-p2')]; % last parameter is sector length
    obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
    
   
    obj.precomputeMap();
    obj.drawMap();
end
function createSimpleMap22(obj)
    coor=nan(2,100);
    coor(:,1)=[0;2];
    coor(:,2)=[2;0];
    coor(:,3)=[2;2];
    coor(:,4)=[2;4];
    coor(:,5)=[5;2];
    coor(:,6)=[-5;2];
    coor(:,7)=[2;-5];
    coor(:,8)=[2;10];
    coor(:,9)=[10;2];
%     coor=[coor(2,:);coor(1,:)];


    nID1=1; nID2=3; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
    secShape='L';
    secParam=[ p1' p2' norm(p1'-p2')]; % last parameter is sector length
    obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
    nID1=2; nID2=3; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
    secShape='L';
    secParam=[ p1' p2' norm(p1'-p2')]; % last parameter is sector length
    obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
    nID1=3; nID2=4; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
    secShape='L';
    secParam=[ p1' p2' norm(p1'-p2')]; % last parameter is sector length
    obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
    nID1=3; nID2=5; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
    secShape='L';
    secParam=[ p1' p2' norm(p1'-p2')]; % last parameter is sector length
    obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
    nID1=6; nID2=1; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
    secShape='L';
    secParam=[ p1' p2' norm(p1'-p2')]; % last parameter is sector length
    obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
    nID1=7; nID2=2; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
    secShape='L';
    secParam=[ p1' p2' norm(p1'-p2')]; % last parameter is sector length
    obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
    nID1=8; nID2=4; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
    secShape='L';
    secParam=[ p1' p2' norm(p1'-p2')]; % last parameter is sector length
    obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
    nID1=9; nID2=5; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
    secShape='L';
    secParam=[ p1' p2' norm(p1'-p2')]; % last parameter is sector length
    obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
   
    obj.precomputeMap();
    obj.drawMap();
end
%===================================================================
   
end

end

