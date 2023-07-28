classdef fcnSIPPContainer
    methods (Static)
        %====================================================

        function setRoadOccupancy(obj,roadID,occupancy)
            st=obj.roadID(roadID);
            st.occupancy2=[st.occupancy2; occupancy];
            obj.roadID(roadID)=st;

            roadIDReverse = obj.roadID(roadID).twoWayID;
            if roadIDReverse
                st=obj.roadID(roadIDReverse);
                occupancyReverse=occupancy; occupancyReverse(4)=-1; % zasedeno v nasprotno smer
                st.occupancy2=[st.occupancy2; occupancyReverse];
                obj.roadID(roadIDReverse)=st;
            end
        end
        %====================================================
        function setNodeOccupancy(obj,nodeID,occupancy)
            st=obj.nodeID(nodeID);
            st.occupancy=[st.occupancy; occupancy];
            obj.nodeID(nodeID)=st;

            %nastavim se zasedenosti drugih vozlisc, ki so prevec blizu temu vozliscu
            % ali nearbyNode dodat še varnostni faktor za koliko povecat
            % zasedenost dvojnemu vozliscu
            if(~isempty(st.nearbyNode))
                for i=1:size(st.nearbyNode,2)
                    nearID=st.nearbyNode(i);
                    st=obj.nodeID(nearID);
                    st.occupancy=[st.occupancy; occupancy];
                    obj.nodeID(nearID)=st;
                end
            end
        end
        %====================================================
        function clearNodeOccupancy(obj,nodeID)
            st=obj.nodeID(nodeID);
            st.occupancy=[];
            obj.nodeID(nodeID)=st;

            %nastavim se zasedenosti drugih vozlisc, ki so prevec blizu temu vozliscu
            % ali nearbyNode dodat še varnostni faktor za koliko povecat
            % zasedenost dvojnemu vozliscu
            if(~isempty(st.nearbyNode))
                for i=1:size(st.nearbyNode,2)
                    nearID=st.nearbyNode(i);
                    st=obj.nodeID(nearID);
                    st.occupancy=[];
                    obj.nodeID(nearID)=st;
                end
            end
        end

        %===================================================
        function removeOccupanciesForUnfinishedPlan(agvSim,TP,agvID,casiPrekini)
            % ?? preverit ali je ok
            % TP ... plan
            if ~isempty(TP)
                for p=(find(TP(:,1)>casiPrekini))'  % grem cez vse postavke plana
                    % pobrisem zasedenosti za cesto  % [ts,te,waitTime,smer,agvID,priority,tVarn,pathSegment]
                    rid= TP(p,2);
                    road=agvSim.roadID(rid);
                    OC=road.occupancy2;
                    oc=find( OC(:,5)==agvID & OC(:,2)>casiPrekini);
                    if ~isempty(oc)
                        OC(oc,:)=[]; % pobrisem zasedenosti za cesto z indeksi oc
                        road.occupancy2=OC ;agvSim.roadID(rid)=road; % zapisem nazaj
                    end

                    % pobrisem zasednosti za vozlisca  [ts, te, priority, agvID, pathSegment]
                    sn=agvSim.roadID(rid).sNode;
                    en=agvSim.roadID(rid).eNode;

                    for n=[sn en]
                        node=agvSim.nodeID(n);
                        OC=node.occupancy;
                        if(~isempty(OC))
                            oc=find( OC(:,4)==agvID & OC(:,2)>casiPrekini);
                            if ~isempty(oc)
                                OC(oc,:)=[]; % pobrisem zasedenosti in nazaj zapisem
                                node.occupancy=OC; agvSim.nodeID(n)=node; % zapisem nazaj
                            end
                            %                   else
                            %                       g=1
                        end
                    end
                end
            end
        end
        %===================================================
        function   adaptOccupanciesForAgvThatStartOnRoad(agvSim,Orders,safetyMargin)
            if size(Orders,2)==9 % imamo strukturo Orders z moznostjo podajanja startne ceste in lokacije na njej
                for i=1:size(Orders,1)
                    if Orders(i,8)~=0 % ce podana cesta za zacetek
                        agvID=Orders(i,1);startTime=Orders(i,2); speed=Orders(i,3); Ns= Orders(i,4); Ne= Orders(i,5); len=Orders(i,6); wid=Orders(i,7);
                        safetyRadious=sqrt(len^2+wid^2)*safetyMargin; tVarn=safetyRadious/speed;
                        rID=Orders(i,8); loc=Orders(i,9);
                        t1= startTime - loc*agvSim.roadID(rID).length/speed;
                        t2= startTime + (1- loc)*agvSim.roadID(rID).length/speed;
                        fcnSIPPContainer.setRoadOccupancy(agvSim,rID,[t1,t2,0,1,agvID, 0, tVarn,1,speed]); % zasedenost ceste, tu gledam le centre vozil: [ts,te,waitTime,smer,agvID,priority,tVarn,pathSegment]
                        fcnSIPPContainer.setNodeOccupancy(agvSim,agvSim.roadID(rID).eNode,[t2-tVarn,t2+tVarn,0,agvID, 1]);  % zasedenost koncnega vozlisca [ts, te, priority, agvID, pathSegment]
                    end
                end
            end
        end
        %===================================================
        function   adaptOccupanciesForAgvTrapedDeadEndRoad(agvSim,Orders,safetyMargin,replantime)
            % najprej zasedem zacetne lege vsaj za cas voznje iz njih oz se za
            % zacetno cakanje startTime-replantime
            for i=1:size(Orders,1)  % Orders= [AGVid  tStart speed Ns Ne ....
                if (size(Orders,2)==7 || (size(Orders,2)==9 && Orders(i,8)==0) )% ce ni podana cesta za zacetek
                    agvID=Orders(i,1);startTime=Orders(i,2); speed=Orders(i,3); Ns= Orders(i,4); %Ne= Orders(i,5);
                    len=Orders(i,6); wid=Orders(i,7);
                    safetyRadious=sqrt(len^2+wid^2)*safetyMargin; tVarn=safetyRadious/speed;

                    fcnSIPPContainer.setNodeOccupancy(agvSim,Ns,[replantime-tVarn,startTime+tVarn,0,agvID, 1]);  % zasedenost koncnega vozlisca [ts, te, priority, agvID, pathSegment]
                end
            end

            % nato zasedem se slepo ulico in njeno koncno vozlisce (koncno vozlisce le ce kdo v njem ne zacne)
            for i=1:size(Orders,1)  % Orders= [AGVid  tStart speed Ns Ne ....
                if (size(Orders,2)==7 || (size(Orders,2)==9 && Orders(i,8)==0) )% ce ni podana cesta za zacetek
                    agvID=Orders(i,1);startTime=Orders(i,2); speed=Orders(i,3); Ns= Orders(i,4); %Ne= Orders(i,5);
                    len=Orders(i,6); wid=Orders(i,7);
                    safetyRadious=sqrt(len^2+wid^2)*safetyMargin; tVarn=safetyRadious/speed;
                    if( length(agvSim.nodeID(Ns).neighboarNodeIDs)==1 )  % je slepa ulica ?? preveri se, ce je vec povezav v slepi ulici ...
                        rID=agvSim.nodeID(Ns).neighboarRoadIDs(1);
                        if agvSim.roadID(rID).twoWayID~=0 % ni enosmerna
                            t1= startTime;
                            t2= startTime +agvSim.roadID(rID).length/speed;
                            fcnSIPPContainer.setRoadOccupancy(agvSim,rID,[t1,t2,0,1,agvID, 0, tVarn,1,speed]); % zasedenost ceste, tu gledam le centre vozil: [ts,te,waitTime,smer,agvID,priority,tVarn,pathSegment]
                            % vozlisce zasedi le ce je prosto (da v njem slucajno kdo ne zacne)
                            Nfin=agvSim.roadID(rID).eNode;
                            if(isempty(agvSim.nodeID(Nfin).occupancy))
                                fcnSIPPContainer.setNodeOccupancy(agvSim,Nfin,[t2-tVarn,t2+tVarn,0,agvID, 1]);  % zasedenost koncnega vozlisca [ts, te, priority, agvID, pathSegment]
                            else
                                OI=agvSim.nodeID(Nfin).occupancy(agvSim.nodeID(Nfin).occupancy(:,4)~=agvID,:); % predhodne zasedenosti vozlisca
                                OIme=[t2-tVarn,t2+tVarn];
                                gg=find( OI(:,2)>OIme(1) & OIme(2)>OI(:,1) );
                                if(isempty(gg))          % ce ni prekrivanja-> vozlisce ni zasedeno z zacetno zasedenestjo, potem ga lahko zasedem jaz
                                    fcnSIPPContainer.setNodeOccupancy(agvSim,Nfin,[t2-tVarn,t2+tVarn,0,agvID, 1]);  % zasedenost koncnega vozlisca [ts, te, priority, agvID, pathSegment]
                                end
                            end
                        end
                    end
                end
            end

        end
        
        function setOccupancySIPP(obj,Tpath,agvID,speed,safetyRadious,priority,replanTime)
            % deluje verjetno le za situacije kjer se caka le na zacetku ceste,
            % za cekanje na cestah je potrebno dopolnit.
            % In so lokacije v planu le 0 ali 1.

            if(isempty(Tpath)), error('setOccupancySIPP: path is empty'); end

            tVarn=safetyRadious/speed;
            t=replanTime;
            twaitOld=0;
            for i=1:size(Tpath,1)
                loc= Tpath(i,3);
                rid= Tpath(i,2);
                sn=obj.roadID(rid).sNode;

                m_time=obj.roadID(rid).length/speed;
                t2=Tpath(i,1); % cas ob koncu akcije (ceste)
                t1=t2 - loc*m_time ; % cas ko se zacnem premikat na cesti in zacetek voznje po cesti - do tu caka v prvem vozliscu
                twait=t1-t;

                if(twait<0), twait=0; end   % ce zacnemo na cesti je twait narobe dolocen. Tu se nic ne caka zato twait=0

                if twait<1e-9 % ce ne cakam zabelezim, drugace pa ob naslednji akciji dodam cakanje
                    % twait=t1-t;
                    twait=twait+twaitOld;
                    fcnSIPPContainer.setNodeOccupancy(obj,sn,[t-twaitOld-tVarn, t1+tVarn, priority, agvID,i-1]);  % zasedenost zacetnega vozlisca [ts, te, priority, agvID, pathSegment]
                    %   fcnSIPPContainer.setRoadOccupancy(obj,rid,[t1, t2, twait, 1, agvID, priority, tVarn,i]); % zasedenost ceste, tu gledam le centre vozil: [ts,te,waitTime,smer,agvID,priority,tVarn,pathSegment]
                    %??? zasedem kar celotno cesto tudi ce loc <1 -> ker se drugace ne uposteva prav zasedenost
                    fcnSIPPContainer.setRoadOccupancy(obj,rid,[t1, t1+m_time, twait, 1, agvID, priority, tVarn,i,speed]); % zasedenost ceste, tu gledam le centre vozil: [ts,te,waitTime,smer,agvID,priority,tVarn,pathSegment]

                    twaitOld=0;
                else
                    twaitOld=twait;
                end
                t=t2;    % povecam cas
            end
            % se za ciljno vozlišce
            en=obj.roadID(rid).eNode;

            % vozlisce zasedem le ce se ustavim na koncu ceste  -> trenutna verzija ustavljanja na cesti deluje ok le za polnilnico
            % ustavljanje na cesti ob normalnem delovanju ni priporoceno- ker detekcija trkov to ne zazna prav preko SI-jev vozlisc in OIjev cest
            if (loc==1)
                %    if( Tpath(end,2)==Tpath(end-1,2) && Tpath(end,3)==Tpath(end-1,3) ) % ce cakam na koncu plana potem ne zasedem do inf
                %      fcnSIPPContainer.setNodeOccupancy(obj,en,[Tpath(end-1,1)-tVarn, Tpath(end,1)+tVarn, priority, agvID,size(Tpath,1)]);  % zasedenost koncnega vozljisca
                %    else
                %      fcnSIPPContainer.setNodeOccupancy(obj,en,[t-tVarn, inf, priority, agvID,size(Tpath,1)]);  % zasedenost koncnega vozljisca
                %    end
                fcnSIPPContainer.setNodeOccupancy(obj,en,[t-tVarn, t+2+tVarn, priority, agvID,size(Tpath,1)]);  % zasedenost koncnega vozljisca

            end

            %   if (loc==1 || (m_time-t2+t1) < tVarn ) % ce se ustavim na koncu ceste ali blizu konca ceste
            %     fcnSIPPContainer.setNodeOccupancy(obj,en,[t-tVarn, inf, priority, agvID,size(Tpath,1)]);  % zasedenost koncnega vozljisca
            %   end
            %?? zadnje vozlisce zasedem do inf tudi ce koncam na cesti pred njim (polnilnica). Ostali lahko vstopajo na to cesto in se ustavijo pred vozliscem, noben pa ne more it skozi koncno vozlisce. Ga je potrebno prej sprostit...
            % fcnSIPPContainer.setNodeOccupancy(obj,en,[t-tVarn, inf, priority, agvID,size(Tpath,1)]);  % zasedenost koncnega vozljisca

        end
        %===================================================
        %===================================================
        function setOccupancySIPP2(obj,Tpath,agvID,speed,safetyRadious,priority,replanTime,goalWait)
            % deluje verjetno le za situacije kjer se caka le na zacetku ceste,
            % za cekanje na cestah je potrebno dopolnit.
            % In so lokacije v planu le 0 ali 1.

            if(isempty(Tpath)), error('setOccupancySIPP: path is empty'); end

            tVarn=safetyRadious/speed;
            t=replanTime;
            twaitOld=0;ridOld=0;locOld=0;
            for i=1:size(Tpath,1)
                loc= Tpath(i,3);
                rid= Tpath(i,2);
                sn=obj.roadID(rid).sNode;

                if(rid==ridOld)
                    m_time=(loc-locOld)*obj.roadID(rid).length/speed;
                else
                    m_time=loc*obj.roadID(rid).length/speed;
                end

                t2=Tpath(i,1); % cas ob koncu akcije (ceste)
                t1=t2 - m_time ; % cas ko se zacnem premikat na cesti in zacetek voznje po cesti - do tu caka v prvem vozliscu
                twait=t1-t;

                if(twait<0), twait=0; end   % ce zacnemo na cesti je twait narobe dolocen. Tu se nic ne caka zato twait=0

                if twait<1e-9 % ce ne cakam zabelezim, drugace pa ob naslednji akciji dodam cakanje
                    % twait=t1-t;
                    twait=twait+twaitOld;
                    fcnSIPPContainer.setNodeOccupancy(obj,sn,[t-twaitOld-tVarn, t1+tVarn, priority, agvID,i-1]);  % zasedenost zacetnega vozlisca [ts, te, priority, agvID, pathSegment]
                    %??? zasedem kar celotno cesto tudi ce loc <1 -> ker se drugace ne uposteva prav zasedenost
                    fcnSIPPContainer.setRoadOccupancy(obj,rid,[t1, t1+m_time, twait, 1, agvID, priority, tVarn,i,speed]); % zasedenost ceste, tu gledam le centre vozil: [ts,te,waitTime,smer,agvID,priority,tVarn,pathSegment]

                    twaitOld=0;
                else
                    twaitOld=twaitOld + twait;
                end
                t=t2;    % povecam cas
                ridOld=rid; locOld=loc;
            end
            % se za ciljno vozlišce
            en=obj.roadID(rid).eNode;

            % vozlisce zasedem le ce se ustavim na koncu ceste  -> trenutna verzija ustavljanja na cesti deluje ok le za polnilnico
            % ustavljanje na cesti ob normalnem delovanju ni priporoceno- ker detekcija trkov to ne zazna prav preko SI-jev vozlisc in OIjev cest
            if (loc==1)
                % fcnSIPPContainer.setNodeOccupancy(obj,en,[t-tVarn, t+goalWait+tVarn, priority, agvID,size(Tpath,1)]);  % zasedenost koncnega vozljisca
                if isinf(goalWait)
                    fcnSIPPContainer.setNodeOccupancy(obj,en,[t-tVarn, inf, priority agvID,size(Tpath,1)]);  % zasedenost koncnega vozljisca
                elseif twaitOld>0
                    fcnSIPPContainer.setNodeOccupancy(obj,en,[t-twaitOld-tVarn, t+tVarn, priority, agvID,size(Tpath,1)]);  % zasedenost koncnega vozljisca
                else
                    fcnSIPPContainer.setNodeOccupancy(obj,en,[t-tVarn, t+goalWait+tVarn, priority agvID,size(Tpath,1)]);  % zasedenost koncnega vozljisca
                end
            end
        end
        %===================================================
        %===================================================
        function setOccupancySIPProadWait2(obj,Tpath,agvID,speed,safetyRadious,priority,replanTime,goalWait,startTime,startLoc)
            % deluje verjetno le za situacije kjer se caka le na zacetku ceste,
            % za cekanje na cestah je potrebno dopolnit.
            % In so lokacije v planu le 0 ali 1.

            if(isempty(Tpath)), error('setOccupancySIPP: path is empty'); end


            %??? speed doloci glede na dejanske hitrosti vožnje po cestah - ? kako to ocenit

            %tVarn=safetyRadious/speed;
            t=replanTime;
            twaitOld=0; ridOld=0; locOld=0;  teOld=replanTime;
            Np=size(Tpath,1);

            tWaitS=0; tWaitE=0; tWaitR=0; % casi cakanja v zacetnem koncnem vozliscu in vmes


            locOld=startLoc;
            AGVspeed=speed;


            planAnaliza=[];  % ??? brisi


            for i=1:Np
                loc= Tpath(i,3);
                rid= Tpath(i,2);
                te=Tpath(i,1); % cas ob koncu akcije
                L=obj.roadID(rid).length;

                sn=obj.roadID(rid).sNode;

                % t ts te
                if i==1 % zacetna cesta
                    ts=startTime; % zacetni cas gibanja poznamo, do tu caka v zacetnem vozliscu
                    m_time=te-ts;
                    if(loc-startLoc)>0 && m_time>0
                        AGVspeed = (loc-startLoc)*L/m_time;
                        %           g=1
                    end

                    if loc==startLoc
                        if loc==0
                            tWaitS=te-t;%ts-t;  %?? te-t oz te-ts
                        elseif loc==1
                            tWaitE=te-t;%ts-t;
                        else
                            tWaitR=te-t;%ts-t;
                        end
                    end
                elseif rid~=ridOld  % vstop na novo cesto
                    if loc>0
                        tWaitS=0;
                        ts=t;
                        m_time=te-ts ;
                        if m_time>0
                            AGVspeed = loc*L/m_time;
                            %         g=1
                        end
                    else % loc==0
                        tWaitS=te-t;
                        ts=te;% t+tWaitS;  % ???
                    end
                else  % ista cesta
                    tInc=te - teOld;
                    if loc==locOld % cakanje
                        if loc==0
                            tWaitS=tWaitS+tInc;
                        elseif loc==1
                            tWaitE=tWaitE+tInc;
                        else
                            tWaitR=tWaitR+tInc;
                        end
                    elseif tInc>0
                        AGVspeed = (loc-locOld)*L/tInc;
                        %  g=1
                    end
                end


                posodobi=0;
                if i<Np
                    if rid~=Tpath(i+1,2) % menjava ceste bo v naslednjem koraku
                        posodobi=1;
                    end
                else % zadnja akcija
                    posodobi=1;
                end


                if posodobi
                    if loc==1
                        tend=Tpath(i,1); % cas ob koncu ceste
                    else
                        tend=Tpath(i,1)+(1-loc)*L/AGVspeed;
                    end

                    %twaitRoad=tWaitR;
                    twaitRoad=tWaitS+tWaitR+tWaitE*1;  % ????

                    %  ts=t+twaitRoad;
                    %  ts=t+tWaitS+tWaitR;
                    %  ts=t; % ???
                    %[rid ts te twsf tws twm twe ]
                    planAnaliza=[planAnaliza;rid,t,tend,twaitRoad,tWaitS,tWaitR,tWaitE];
                    if i==Np
                        Tpath
                        planAnaliza
                        g=1
                    end

                    %       if planManipulation
                    %
                    %       else
                    %
                    %       end



                    tVarn=safetyRadious/AGVspeed;

                    fcnSIPPContainer.setNodeOccupancy(obj,sn,[t-twaitOld-tVarn, ts+tVarn, priority, agvID,i-1]);  % zasedenost zacetnega vozlisca [ts, te, priority, agvID, pathSegment]
                    %??? zasedem kar celotno cesto tudi ce loc <1 -> ker se drugace ne uposteva prav zasedenost

                    if twaitRoad<0, twaitRoad=0; end
                    % zasedenost ceste predstavim kot cas cakanja na zacetku ceste twaitRoad in nato voznja po cesti [ts, tend]
                    % dejansko pa vozilo lahko caka na cesti in zato vstopi na cesto prej
                    fcnSIPPContainer.setRoadOccupancy(obj,rid,[ts, tend, twaitRoad, 1, agvID, priority, tVarn,i,AGVspeed]); % zasedenost ceste, tu gledam le centre vozil: [ts,te,waitTime,smer,agvID,priority,tVarn,pathSegment,speed]

                    twaitOld=tWaitE; % cakanje na koncu ceste dodam k zacetnemu vozliscu na naslednji cesti
                    tWaitS=0;tWaitE=0; tWaitR=0; % casi cakanja v zacetnem, koncnem vozliscu in vmes
                    t=tend;    % povecam cas

                end




                ridOld=rid; locOld=loc; teOld=te;
            end


            % se za ciljno vozlišce
            en=obj.roadID(rid).eNode;

            % vozlisce zasedem le ce se ustavim na koncu ceste  -> trenutna verzija ustavljanja na cesti deluje ok le za polnilnico
            % ustavljanje na cesti ob normalnem delovanju ni priporoceno- ker detekcija trkov to ne zazna prav preko SI-jev vozlisc in OIjev cest

            t=Tpath(Np,1);   % konec zadnje akcije

            if (loc==1)
                if isinf(goalWait)
                    fcnSIPPContainer.setNodeOccupancy(obj,en,[t-tVarn, inf, priority agvID,size(Tpath,1)]);  % zasedenost koncnega vozljisca
                elseif twaitOld>0
                    fcnSIPPContainer.setNodeOccupancy(obj,en,[t-twaitOld-tVarn, t+goalWait+tVarn, priority, agvID,size(Tpath,1)]);  % zasedenost koncnega vozljisca
                else
                    fcnSIPPContainer.setNodeOccupancy(obj,en,[t-tVarn, t+goalWait+tVarn, priority agvID,size(Tpath,1)]);  % zasedenost koncnega vozljisca
                end
            end
        end
        %===================================================

        function setOccupancySIPProadWait(obj,Tpath,agvID,speed,safetyRadious,priority,replanTime,goalWait)
            % deluje verjetno le za situacije kjer se caka le na zacetku ceste,
            % za cekanje na cestah je potrebno dopolnit.
            % In so lokacije v planu le 0 ali 1.

            if(isempty(Tpath)), error('setOccupancySIPP: path is empty'); end


            %??? speed dolo?i glede na dejanske hitrosti vožnje po cestah - ? kako to
            %ocenit

            tVarn=safetyRadious/speed;
            t=replanTime;
            twaitOld=0; ridOld=0; locOld=0;  teOld=replanTime;
            Np=size(Tpath,1);

            tWaitS=0;
            tWaitE=0;
            tWaitR=0; % casi cakanja v zacetnem koncnem vozliscu in vmes

            %stanje=0; % 0=zacetek, 1=zacetek



            for i=1:Np
                loc= Tpath(i,3);
                rid= Tpath(i,2);
                te=Tpath(i,1); % cas ob koncu akcije

                sn=obj.roadID(rid).sNode;

                % t ts te
                if rid~=ridOld  % vstop na novo cesto

                    m_time=loc*obj.roadID(rid).length/speed; % motion time
                    ts=te - m_time ; % cas ko se zacnem premikat na cesti  - do tu caka v  vozliscu ali na cesti (vse si predstavljas kot cakanje na zacetku in nato voznaj. ?eprav potem prestaviš ?akanje na cesto...)
                    tWaitS=ts-t;
                else  % ista cesta
                    m_time=(loc-locOld)*obj.roadID(rid).length/speed; % motion time
                    tWait=te - teOld;
                    if loc==locOld % cakanje
                        if loc==0
                            tWaitS=tWaitS+tWait;
                        elseif loc==1
                            tWaitE=tWaitE+tWait;
                        else
                            tWaitR=tWaitR+tWait;
                        end
                    end
                end


                posodobi=0;
                if i<Np
                    if rid~=Tpath(i+1,2) % menjava ceste bo v naslednjem koraku
                        posodobi=1;
                    end
                else % zadnja akcija
                    posodobi=1;
                end


                if posodobi
                    if loc==1
                        tend=Tpath(i,1); % cas ob koncu ceste
                    else
                        tend=Tpath(i,1)+(1-loc)*obj.roadID(rid).length/speed;
                    end

                    fcnSIPPContainer.setNodeOccupancy(obj,sn,[t-twaitOld-tVarn, ts+tVarn, priority, agvID,i-1]);  % zasedenost zacetnega vozlisca [ts, te, priority, agvID, pathSegment]
                    %??? zasedem kar celotno cesto tudi ce loc <1 -> ker se drugace ne uposteva prav zasedenost
                    %twaitRoad=tWaitR;
                    twaitRoad=tWaitS+tWaitR+tWaitE;

                    if twaitRoad<0, twaitRoad=0; end
                    % zasedenost ceste predstavim kot cas cakanja na zacetku ceste twaitRoad in nato voznja po cesti [ts, tend]
                    % dejansko pa vozilo lahko caka na cesti in zato vstopi na cesto prej
                    fcnSIPPContainer.setRoadOccupancy(obj,rid,[ts, tend, twaitRoad, 1, agvID, priority, tVarn,i,speed]); % zasedenost ceste, tu gledam le centre vozil: [ts,te,waitTime,smer,agvID,priority,tVarn,pathSegment]

                    twaitOld=tWaitE; % cakanje v koncnem vozliscu dodam k zacetnemu na naslednji cesti
                    tWaitS=0;tWaitE=0; tWaitR=0; % casi cakanja v zacetnem koncnem vozliscu in vmes
                    t=tend;    % povecam cas
                end




                ridOld=rid; locOld=loc; teOld=te;
            end


            % se za ciljno vozlišce
            en=obj.roadID(rid).eNode;

            % vozlisce zasedem le ce se ustavim na koncu ceste  -> trenutna verzija ustavljanja na cesti deluje ok le za polnilnico
            % ustavljanje na cesti ob normalnem delovanju ni priporoceno- ker detekcija trkov to ne zazna prav preko SI-jev vozlisc in OIjev cest

            t=Tpath(Np,1);   % konec zadnje akcije

            if (loc==1)
                if isinf(goalWait)
                    fcnSIPPContainer.setNodeOccupancy(obj,en,[t-tVarn, inf, priority agvID,size(Tpath,1)]);  % zasedenost koncnega vozljisca
                elseif twaitOld>0
                    fcnSIPPContainer.setNodeOccupancy(obj,en,[t-twaitOld-tVarn, t+goalWait+tVarn, priority, agvID,size(Tpath,1)]);  % zasedenost koncnega vozljisca
                else
                    fcnSIPPContainer.setNodeOccupancy(obj,en,[t-tVarn, t+goalWait+tVarn, priority agvID,size(Tpath,1)]);  % zasedenost koncnega vozljisca
                end
            end
        end

        %===================================================
        function TpathNew=manipulateTimePlanRoadWait(obj,Tplan,agvID,speed,safetyRadious,priority,replanTime,goalWait,startTime,maxSafetyRadious)
            % deluje verjetno le za situacije kjer se caka le na zacetku ceste,
            % za cekanje na cestah je potrebno dopolnit.
            % In so lokacije v planu le 0 ali 1.

            TpathNew=[];

            if(isempty(Tplan)), error('setOccupancySIPP: path is empty'); end

            AGVspeed=speed;

            tVarn=safetyRadious/AGVspeed;


            t=replanTime;
            Np=size(Tplan,1);


            for i=1:Np
                loc= Tplan(i,3);
                rid= Tplan(i,2);
                te=Tplan(i,1); % cas ob koncu akcije
                sn=obj.roadID(rid).sNode;


                % malo komplikacije, da se prav uposteva startTime mirovanje
                if i==1 && loc==0 && startTime>replanTime && te>startTime
                    TpathNew=[startTime, rid, loc; TpathNew ]; %vrinem cakanje do startTime
                    t=startTime;
                end


                %  if loc==0 %&& i~=1 % na zacetku ceste  - tu cakam ?? ali zacetno cakanje lahko prekinjam
                if loc==0 && ~(i==1 && te<=startTime) %&& i~=1 % na zacetku ceste  - tu cakam ?? ali zacetno cakanje lahko prekinjam
                    % Nwait=0;
                    sumOccupDist=maxSafetyRadious;  % za max dimenzijo pustim spredaj pred koncem ceste prostor, ce je cesta nezasedena
                    %AGVspeed=speed;
                    AGVspeedInFront = AGVspeed;

                    if(agvID==3)
                        g=1
                    end


                    if( ~isempty(obj.roadID(rid).occupancy2) ) % cesta vsebuje zasedenosti, preverim, ce so v mojem intervalu [t,te] in ce niso moje (npr zacetna zasedenost s prioriteto 0)
                        OIr=obj.roadID(rid).occupancy2(obj.roadID(rid).occupancy2(:,5)~=agvID,:);   % [ts,te,waitTime,smer,agvID,priority,tVarn,ind,speed]
                        if ~isempty(OIr)
                            % gg=find((t>OIr(:,2) | te<OIr(:,1))==0 & priority>OIr(:,6) & OIr(:,3)>0); % indexes of OIr (occupancy interval for road) that overlap with [t, te] and has higher priority and is eating on the road - ?? prioriteta ni pomembna, ker boj priritetni itak planirajo prej ?
                            %           gg=find((t>OIr(:,2) | te<OIr(:,1))==0 & OIr(:,3)>0); % indexes of OIr (occupancy interval for road) that overlap with [t, te] and has higher priority and is wating on the road - ?? prioriteta ni pomembna, ker boj priritetni itak planirajo prej ?
                            %   ?? ce so razlicne hitrosti moras gledat ce je na cesti nekdo tudi ce ne caka na cesti
                            gg=find((t>OIr(:,2) | te<OIr(:,1))==0 ); % indexes of OIr (occupancy interval for road) that overlap with [t, te] and has higher priority and is wating on the road - ?? prioriteta ni pomembna, ker boj priritetni itak planirajo prej ?


                            if ~isempty(gg)
                                % Nwait=length(gg); % toliko jih caka ?

                                allSafeR =OIr(gg,7).*OIr(gg,9);
                                sumOccupDist=sum(allSafeR*2)+maxSafetyRadious;

                                % AGVspeedInFront=OIr(gg,9)

                                % AGVspeed

                                % kako vem katero vozilo je pred mano - tisto, ki ima min razliko casa vstopa
                                % ?? mora bit pred mano

                                % pogoj=(t-OIr(gg,1))

                                [~,idxMin]=min(t-OIr(gg,1) );
                                AGVspeedInFront=OIr(gg(idxMin),9);
                                %                  AGVspeed
                                %                     g=1
                            end

                        end
                    end     %if( ~isempty(obj.roadID(rid).occupancy2) )

                    % ?? dodaj v strukturo ceste se parameter moznega intervala cakanja za cesto
                    % ce je intervalov vec pogledas njihovo normirano vsoto in
                    % primerjas za vsakega cca. (Nwait+1)*2*safetyRadious)/L

                    % ?? razlicne hitrosti voznje hitrejsi, pocasnejsi,...
                    % ce sem hitrejsi si lahko zmanjsam hitrost,...
                    % ce sem pocasnejsi potem je morda lahko problem, ker bo vozilo pred
                    % mano cakalo dlje in se mu bom ze prevec priblizal - lahko si dam
                    % isto hitrost in cakam dlje?

                    % lahko cakam, ce je cesta dovolj dolga
                    deltaLoc=(obj.roadID(rid).waitInterval(2)-obj.roadID(rid).waitInterval(1));
                    if deltaLoc>1,  error('WaitInterval v funkciji manipulateTimePlanRoadWait mora biti med 0 in 1.'); end

                    L=obj.roadID(rid).length;
                    Lpart= L * deltaLoc ; %dolzina ceste za cakanje

                    if (sumOccupDist+2*safetyRadious+maxSafetyRadious)<Lpart  % ali je dovolj prostora za obstojece + zame + se rezervo za zacezno vozlicsa
                        %if (Nwait+2)*2*safetyRadious<L  % ali je dovolj prostora za obstojece + zame (za n vozil je n+1 intervalov 2*safetyRadious)
                        %if locW>0  % cesta je dovolj dolga - lahko cakamo na njej. Prestavimo mesto cakanja na cesto




                        %locW= (L-(Nwait+1)*2*safetyRadious)/L;
                        locW= obj.roadID(rid).waitInterval(1)+(Lpart-sumOccupDist-safetyRadious)/L;
                        %           locW= obj.roadID(rid).waitInterval(1)+ (L-sumOccupDist-safetyRadious)/L*deltaLoc;

                        if locW>1, locW=1; disp('cudno loc>0 v funkciji manipulateTimePlanRoadWait'); end


                        if(AGVspeed>AGVspeedInFront) % pred mano je pocasnejse vozilo

                            t_slow=L*locW/AGVspeedInFront;      % Cas v katerem bi enako pot opravil pocasnejsi AGV
                            t_slow2=L*(1-locW)/AGVspeedInFront; % Cas v katerem bi enako pot opravil pocasnejsi AGV;

                            deltaT=L/AGVspeed;

                            if (t_slow+t_slow2) > (deltaT) % Ce je premalo casa, da bi vozili z manjso potem malo povecamo hitrost
                                fact=(t_slow+t_slow2)/(deltaT);
                                AGVspeedInFront=AGVspeedInFront*fact;
                                t_slow=L*locW/AGVspeedInFront;
                                t_slow2=L*(1-locW)/AGVspeedInFront;
                            end
                            tt=t+t_slow;

                            TpathNew=[TpathNew; tt, rid, locW ]; % Voznja do pozicije na cesti

                            tWait=te-t;
                            tprej=t+ L*locW/AGVspeed+tWait;

                            tWaitNew=tprej-tt;

                            %if (te-t) > (t_slow+t_slow2+ 1e-6*0) % Ce je ostalo se kaj casa potem caka na poziciji
                            if (tWaitNew>0)
                                % Pogoj ni izpolnjen ce ne pristejem tolerance ????
                                % timePlan=[timePlan; t2-t_slow2, roadID, pos_road]; % Cakanje na poziciji na cesti

                                %tWait=te-t;
                                % TpathNew=[TpathNew; tt+tWait, rid, locW ];
                                % TpathNew=[TpathNew; te+Lall*locW/AGVspeed-t_slow2, rid, locW ];
                                TpathNew=[TpathNew; tt+tWaitNew, rid, locW ];
                            end
                        else
                            tt=t+ L*locW/AGVspeed;
                            tWait=te-t;
                            TpathNew=[TpathNew; tt, rid, locW ;
                                tt+tWait, rid, locW ;
                                ];
                        end
                    else
                        TpathNew=[TpathNew;Tplan(i,:) ];
                    end


                else
                    TpathNew=[TpathNew;Tplan(i,:) ];
                end % if loc==0

                t=te;
            end

        end
        %===================================================



        function removeChargeEntranceRoad(agvSim,chargeEntranceNode,chargeEntranceRoad)
            % odstranim povezavo iz vstopnega vozlisca v polnilnico
            % lahko bi dodal se preverbo ali cesta in vozlisce obstajata
            if isempty(chargeEntranceNode), return; end

            if agvSim.nodeID.isKey(chargeEntranceNode)
                if ~agvSim.roadID.isKey(chargeEntranceRoad)
                    error('fcnSIPPContainer::removeChargeEntranceRoad: road chargeEntranceRoad does not exist?');
                end
            else
                error('fcnSIPPContainer::removeChargeEntranceRoad: node chargeEntranceNode does not exist?');
            end
            st=agvSim.nodeID(chargeEntranceNode); % lokalna kopija vozlisca
            vv=st.neighboarRoadIDs; nn=st.neighboarNodeIDs;
            vi=find(vv==chargeEntranceRoad);
            if ~isempty(vi)
                vv(vi)=[]; nn(vi)=[]; %Zacasno odstranim povezavo iz vozlisca
                st.neighboarRoadIDs=vv;
                st.neighboarNodeIDs=nn;
                agvSim.nodeID(chargeEntranceNode)=st;
            end
        end
        %===================================================
        function addChargeEntranceRoad(agvSim,chargeEntranceNode,chargeEntranceRoad)
            % dodam povezavo vstopnega vozlisca v polnilnico
            if isempty(chargeEntranceNode), return; end

            if agvSim.nodeID.isKey(chargeEntranceNode)
                if agvSim.roadID.isKey(chargeEntranceRoad)
                    vv=agvSim.nodeID(chargeEntranceNode).neighboarRoadIDs;
                    if( find(vv==chargeEntranceRoad) )  % ce povezava ze obstaja jo ne dodajaj
                        return;
                    end
                else
                    error('fcnSIPPContainer::addChargeEntranceRoad: road chargeEntranceRoad does not exist?');
                end
            else
                error('fcnSIPPContainer::addChargeEntranceRoad: node chargeEntranceNode does not exist?');
            end
            st=agvSim.nodeID(chargeEntranceNode); % lokalna kopija vozlisca
            vv=st.neighboarRoadIDs; nn=st.neighboarNodeIDs;
            vv=[vv,chargeEntranceRoad]; nn=[nn,agvSim.roadID(chargeEntranceRoad).eNode]; %dodam sosednjo povezav in sosednje vozlisce
            st.neighboarRoadIDs=vv;
            st.neighboarNodeIDs=nn;
            agvSim.nodeID(chargeEntranceNode)=st;
        end
        %===================================================
        %===================================================
        function   addOccupancyForAgvStartDeadEndRoad(agvSim,agvID,replantime,startTime,safetyRadious,speed, Ns)

            tVarn=safetyRadious/speed;

            % ?? namesto da dodajas omejitev bi lahko le podaljsal obstojeco ce obstaja
            % prekrivanje casa...
            %   fcnSIPPContainer.setNodeOccupancy(agvSim,Ns,[replantime-tVarn,startTime+tVarn,0,agvID, 1]);  % zasedenost koncnega vozlisca [ts, te, priority, agvID, pathSegment]

            % nato zasedem se slepo ulico in njeno koncno vozlisce (koncno vozlisce le ce kdo v njem ne zacne)



            if( length(agvSim.nodeID(Ns).neighboarNodeIDs)==1 )  % je slepa ulica ?? preveri se, ce je vec povezav v slepi ulici ...
                rID=agvSim.nodeID(Ns).neighboarRoadIDs(1);
                if agvSim.roadID(rID).twoWayID~=0 % je dvosmerna
                    t1= startTime;
                    t2= startTime +agvSim.roadID(rID).length/speed;
                    fcnSIPPContainer.setRoadOccupancy(agvSim,rID,[t1,t2,0,1,agvID, 0, tVarn,1,speed]); % zasedenost ceste, tu gledam le centre vozil: [ts,te,waitTime,smer,agvID,priority,tVarn,pathSegment]
                    % ??? vozlisce zasedi le ce je prosto (da v njem slucajno kdo ne zacne)
                    Nfin=agvSim.roadID(rID).eNode;
                    fcnSIPPContainer.setNodeOccupancy(agvSim,Nfin,[t2-tVarn,t2+tVarn,0,agvID, 1]);  % zasedenost koncnega vozlisca [ts, te, priority, agvID, pathSegment]
                end
            end

        end
        %===================================================

        % function   addOccupancyForAgvStartDeadEndRoad2(agvSim,agvID,replantime,startTime,safetyRadious,speed, Ns)
        %
        %    tVarn=safetyRadious/speed;
        %
        % % ?? namesto da dodajas omejitev bi lahko le podaljsal obstojeco ce obstaja
        % % prekrivanje casa...
        % %   fcnSIPPContainer.setNodeOccupancy(agvSim,Ns,[replantime-tVarn,startTime+tVarn,0,agvID, 1]);  % zasedenost koncnega vozlisca [ts, te, priority, agvID, pathSegment]
        %
        %     % nato zasedem se slepo ulico in njeno koncno vozlisce (koncno vozlisce le ce kdo v njem ne zacne)
        %          if( length(agvSim.nodeID(Ns).neighboarNodeIDs)==1 )  % je slepa ulica ?? preveri se, ce je vec povezav v slepi ulici ...
        %              rID=agvSim.nodeID(Ns).neighboarRoadIDs(1);
        %              t1= startTime;
        %              t2= startTime +agvSim.roadID(rID).length/speed;
        %              fcnSIPPContainer.setRoadOccupancy(agvSim,rID,[t1,t2,0,1,agvID, 0, tVarn,1,speed]); % zasedenost ceste, tu gledam le centre vozil: [ts,te,waitTime,smer,agvID,priority,tVarn,pathSegment]
        %              % vozlisce zasedi le ce je prosto (da v njem slucajno kdo ne zacne)
        %              Nfin=agvSim.roadID(rID).eNode;
        %              fcnSIPPContainer.setNodeOccupancy(agvSim,Nfin,[t2-tVarn,t2+tVarn,0,agvID, 1]);  % zasedenost koncnega vozlisca [ts, te, priority, agvID, pathSegment]
        %          end
        %
        % end

        %===================================================
        function timePlan=planPathPickDrop(agvSim,t,speed,frejAgv,Tasks,idxTasks,safetyRadious,priority)
            % planira sestavljeno pot od zacetne lokacije do pickup ter do dropoff
            timePlan=[];

            tw1=Tasks(idxTasks,4);
            tw2=Tasks(idxTasks,5);

            replanTime=t; startTime=t+tw1*0; % ne caka na zacetku, to je ze del predhodnega plana

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
            stop=Tasks(idxTasks,1);




            %extraTimeGoalDeadEnd=2; % ??? ali koncam v slepi ulici in rabim cakat
            if( length(agvSim.nodeID(stop).neighboarNodeIDs)==1 )  % je slepa ulica ?? preveri se, ce je vec povezav v slepi ulici ...
                rID=agvSim.nodeID(stop).neighboarRoadIDs(1);
                if agvSim.roadID(rID).twoWayID~=0 % je dvosmerna
                    extraTimeGoalDeadEnd=agvSim.roadID(rID).length/speed*2;
                else
                    extraTimeGoalDeadEnd=0;
                end
                % jaz koncam v cilju na koncu slepe ulice pred bolj prioritetnim, ki lahko vstopi v to ulico.
                % Preverim ali se lahko vrnem ven, tako da dam pogoj za veljaven plan,
                % da je ciljno vozlisce dodatno prosto se za extraTimeGoalDeadEnd. V
                % casu ko jaz zacnem voznjo ne sme noben vstopiti na to cesto oz ce
                % gledam kdaj lahko nekdo pride v koncno vozlisce moram upostevat cas,
                % da jaz pridem ven in nekdo drug ob tem casu vstopi ter pride do konca
                % poti torej mora biti ciljno vozlisce dodatno prosto dvojni cas voznje po cesti.
            else
                extraTimeGoalDeadEnd=0;
            end


            if( length(agvSim.nodeID(Tasks(idxTasks,2)).neighboarNodeIDs)==1 )  % je slepa ulica ?? preveri se, ce je vec povezav v slepi ulici ...
                rID=agvSim.nodeID(Tasks(idxTasks,2)).neighboarRoadIDs(1);
                if agvSim.roadID(rID).twoWayID~=0 % je dvosmerna
                    extraTimeGoalDeadEnd2=agvSim.roadID(rID).length/speed*2;
                else
                    extraTimeGoalDeadEnd2=0;
                end
            else
                extraTimeGoalDeadEnd2=0;
            end


            timePlan = findPathSIPP_multiGoal(agvSim, start, {[Tasks(idxTasks, 1), tw1, extraTimeGoalDeadEnd, 0], [Tasks(idxTasks, 2), tw2, extraTimeGoalDeadEnd2, 0] }, ...
                speed, startTime, safetyRadious, priority, replanTime, agvID, []);


        end


        %===================================================
        function [metrika]=metricsTasks(agvSim,Tasks,taskSol,notFoundPaths,idleSumTime)
            % le cca metrika uporabljena za analizo DEMO_SIPP_LifeTime_TasksPickupDropOff22.m
            Ntasks=size(Tasks,1);
            Nsol=size(taskSol,2);

            collisions=agvSim.KP; % posnete voznje simulatorja
            distance=agvSim.DP;

            procentPlanning=Nsol/Ntasks*100; % koliko taskov je splaniranih
            comulativePlanDelayCoordination=0; % grem cez vse plane in pogledam kje se caka na zacetku povezav...
            comulativeDrivingTime=0;
            timeToDeliverAllPlans=0;
            comulativeDrivingDistance=0;

            for i=1:Nsol
                TP=taskSol(i).timePlan;
                w=find(TP(:,3)==0);
                if ~isempty(w)
                    for j=w'
                        if(j>1)
                            comulativePlanDelayCoordination=comulativePlanDelayCoordination+ TP(j,1)-TP(j-1,1);
                        end
                    end
                end


                %dtStart=agvSim.roadID( TP(1,2)).length/speed*TP(1,3); % je ok, ce starta na zacetku ceste ni ok ce starta na cesti, ker zacetne lokacije v planu ni - bi moral gledat kje je zacel
                comulativeDrivingTime = comulativeDrivingTime + (TP(end,1)-taskSol(i).startT);


                %       if(TP(end,1)>timeToDeliverAllPlans)
                %          timeToDeliverAllPlans=TP(end,1);
                %       end


                preLoc=taskSol(i).startLOC; preRID=taskSol(i).startRID;preL=0;
                for j=1:size(TP,1)
                    cLOC=TP(j,3);cRID=TP(j,2);
                    L=agvSim.roadID(cRID).length;
                    if cRID~= preRID % grem na drugo cesto
                        comulativeDrivingDistance=comulativeDrivingDistance + (1-preLoc)*preL  + cLOC*L*1;
                    else % ostanem na isti cesti
                        comulativeDrivingDistance=comulativeDrivingDistance + (cLOC-preLoc)*L;
                    end
                    preLoc=cLOC; preRID=cRID;preL=L;
                end

            end

            %   comulativeDrivingDistance=0;
            %   numberSimulationConflicts=0;
            %   for i=1:size(collisions,2) % belezim le prvih 10 agv-jev  -> parameter AGVSim.maxAGVRec v AGVSim
            %       numberSimulationConflicts=numberSimulationConflicts+sum(diff(collisions(:,i))>0) ;

            %   [~,L]= findpeaks([distance(:,i);0]); %najde razdalje za posamezne plane
            %    comulativeDrivingDistance=comulativeDrivingDistance+sum(distance(L,i));
            %   end

            timeToDeliverAllPlans=agvSim.TP(end);  % kdaj se zadnji plan konca
            numberSimulationConflicts=0;
            for i=1:size(collisions,2) % belezim le prvih 10 agv-jev  -> parameter AGVSim.maxAGVRec v AGVSim
                numberSimulationConflicts=numberSimulationConflicts+sum(diff(collisions(:,i))>0) ;
            end




            comulativePlanDelayCoordination=comulativePlanDelayCoordination+idleSumTime;
            comulativeDrivingTime=comulativeDrivingTime+idleSumTime; %povecam se za cas ko ne uspejo najti plana



            % fprintf(1,' \n For %d plannigs path was not found. \n',notFoundPaths);
            fprintf(1,' \n=== planning metrics (%d AGVs, %d Task)==== \n',size(agvSim.AGV,2),Ntasks);

            %  notFoundPaths
            fprintf(1,'notFoundPaths = %d \n',notFoundPaths);  % % ta ni ok, ker gleda v vseh samplih in jih je veliko

            %procentPlanning
            fprintf(1,'procentPlanning = %6.2f \n',procentPlanning);  % % ta ni ok, ker gleda v vseh samplih in jih je veliko

            %comulativePlanDelayCoordination
            fprintf(1,'comulativePlanDelayCoordination = %6.2f \n',comulativePlanDelayCoordination);  % % ta ni ok, ker gleda v vseh samplih in jih je veliko

            %numberSimulationConflicts
            fprintf(1,'numberSimulationConflicts = %d \n',numberSimulationConflicts);  % % ta ni ok, ker gleda v vseh samplih in jih je veliko

            % timeToDeliveraAllPlans= taskSol(Nsol).timePlan(end,1)  % kdaj se zadnji plan konca
            %timeToDeliveraAllPlans=agvSim.TP(end)  % kdaj se zadnji plan konca

            %timeToDeliveraAllPlans
            fprintf(1,'timeToDeliverAllPlans = %6.2f \n',timeToDeliverAllPlans);  % % ta ni ok, ker gleda v vseh samplih in jih je veliko

            %comulativeDrivingTime
            fprintf(1,'comulativeDrivingTime = %6.2f \n',comulativeDrivingTime);  % % ta ni ok, ker gleda v vseh samplih in jih je veliko

            %comulativeDrivingDistance
            fprintf(1,'comulativeDrivingDistance = %6.2f \n',comulativeDrivingDistance);  % % ta ni ok, ker gleda v vseh samplih in jih je veliko
            fprintf(1,'idleSumTime = %6.2f \n',idleSumTime);  % %

            fprintf(1,'==================================== \n');

            metrika=[notFoundPaths;procentPlanning;comulativePlanDelayCoordination;
                numberSimulationConflicts; timeToDeliverAllPlans;comulativeDrivingTime;comulativeDrivingDistance;idleSumTime];

        end
        %===================================================

        function [metrika]=metricsTasksOnlyPlanning(agvSim,Tasks,taskSol,notFoundPaths,idleSumTime)
            % le cca metrika uporabljena za analizo DEMO_SIPP_LifeTime_TasksPickupDropOff22.m
            Ntasks=size(Tasks,1);
            Nsol=size(taskSol,2);

            collisions=agvSim.KP; % posnete voznje simulatorja
            distance=agvSim.DP;

            procentPlanning=Nsol/Ntasks*100; % koliko taskov je splaniranih
            comulativePlanDelayCoordination=0; % grem cez vse plane in pogledam kje se caka na zacetku povezav...
            comulativeDrivingTime=0;
            timeToDeliverAllPlans=0;
            comulativeDrivingDistance=0;
            for i=1:Nsol
                TP=taskSol(i).timePlan;
                w=find(TP(:,3)==0);
                if ~isempty(w)
                    for j=w'
                        if(j>1)
                            comulativePlanDelayCoordination=comulativePlanDelayCoordination+ TP(j,1)-TP(j-1,1);
                        end
                    end
                end

                comulativeDrivingTime = comulativeDrivingTime + (TP(end,1)-taskSol(i).startT);

                if(TP(end,1)>timeToDeliverAllPlans)
                    timeToDeliverAllPlans=TP(end,1);
                end

                preLoc=taskSol(i).startLOC; preRID=taskSol(i).startRID;preL=0;
                for j=1:size(TP,1)
                    cLOC=TP(j,3);cRID=TP(j,2);
                    L=agvSim.roadID(cRID).length;
                    if cRID~= preRID % grem na drugo cesto
                        comulativeDrivingDistance=comulativeDrivingDistance + (1-preLoc)*preL  + cLOC*L;
                    else % ostanem na isti cesti
                        comulativeDrivingDistance=comulativeDrivingDistance + (cLOC-preLoc)*L;
                    end
                    preLoc=cLOC; preRID=cRID;preL=L;
                end

            end



            % comulativeDrivingDistance=0;
            numberSimulationConflicts=0;
            %   for i=1:size(collisions,2) % belezim le prvih 10 agv-jev  -> parameter AGVSim.maxAGVRec v AGVSim
            %       numberSimulationConflicts=numberSimulationConflicts+sum(diff(collisions(:,i))>0) ;
            %
            %       [~,L]= findpeaks([distance(:,i);0]); %najde razdalje za posamezne plane
            %       comulativeDrivingDistance=comulativeDrivingDistance+sum(distance(L,i));
            %   end



            comulativePlanDelayCoordination=comulativePlanDelayCoordination+idleSumTime;
            comulativeDrivingTime=comulativeDrivingTime+idleSumTime; %povecam se za cas ko ne uspejo najti plana

            % notFoundPaths
            % procentRealisation=  % to bi se lahko potegnilo iz simulatorja ?

            % fprintf(1,' \n For %d plannigs path was not found. \n',notFoundPaths);
            fprintf(1,' \n=== planning metrics (%d AGVs, %d Task)==== \n',size(agvSim.AGV,2),Ntasks);

            %  notFoundPaths
            fprintf(1,'notFoundPaths = %d \n',notFoundPaths);  %

            %procentPlanning
            fprintf(1,'procentPlanning = %6.2f \n',procentPlanning);  %

            %comulativePlanDelayCoordination
            fprintf(1,'comulativePlanDelayCoordination = %6.2f \n',comulativePlanDelayCoordination);  %

            %numberSimulationConflicts
            fprintf(1,'numberSimulationConflicts = %d \n',numberSimulationConflicts);  %

            % timeToDeliveraAllPlans= taskSol(Nsol).timePlan(end,1)  % kdaj se zadnji plan konca
            %timeToDeliveraAllPlans=agvSim.TP(end)  % kdaj se zadnji plan konca

            %timeToDeliveraAllPlans
            fprintf(1,'timeToDeliverAllPlans = %6.2f \n',timeToDeliverAllPlans);  %

            %comulativeDrivingTime
            fprintf(1,'comulativeDrivingTime = %6.2f \n',comulativeDrivingTime);  % %

            %comulativeDrivingDistance
            fprintf(1,'comulativeDrivingDistance = %6.2f \n',comulativeDrivingDistance);  %

            fprintf(1,'idleSumTime = %6.2f \n ',idleSumTime);  % %
            fprintf(1,'========================================== \n');


            metrika=[notFoundPaths;procentPlanning;comulativePlanDelayCoordination;
                numberSimulationConflicts; timeToDeliverAllPlans;comulativeDrivingTime;comulativeDrivingDistance;idleSumTime];

        end
        %===================================================

        function [nid, nOC,rid, rOC]=getAllRoadsOccupancyHeatMap(agvSim,draw)
            % pogleda iz casov zasedenosti skupne case zasedenosti vozlisc in cest
            % ter to vrne
            % nid... seznam vozlisc
            % nOC... cas zasedenosti vozlisca
            % rid... seznam cest
            % rOC... cas zasedenosti cest

            % za ceste
            Nr=size(agvSim.roadID,1);
            rid=zeros(1,Nr);
            rOC=zeros(1,Nr); % skupna zasedenost posameznih cest
            for i=1:Nr
                rid(i)=str2num(agvSim.roadLabels{i});
                OC=agvSim.roadID(rid(i)).occupancy2;
                L=agvSim.roadID(rid(i)).length;
                for j=1:size(OC,1) % grem cez vse zasedenosti ceste
                    if( OC(j,4)==1) % upostevam le zasedenost v smeri vožnje
                        rOC(1,i)=rOC(1,i)+ (OC(j,2)-OC(j,1))/L;
                    end
                end
            end

            % za vozlisca
            Nn=size(agvSim.nodeID,1);
            nid=zeros(1,Nn);
            nOC=zeros(1,Nn); % skupna zasedenost posameznih vozlisc
            for i=1:Nn
                nid(i)=str2num(agvSim.nodeLabels{i});
                OC=agvSim.nodeID(nid(i)).occupancy;
                for j=1:size(OC,1) % grem cez vse zasedenosti vozlicsa
                    nOC(1,i)=nOC(1,i)+ OC(j,2)-OC(j,1);
                end
            end



            if draw
                % izpis na graf
                originalDraw=agvSim.DRAW;
                agvSim.mainFig=11; agvSim.DRAW=1;
                agvSim.drawRoadLabels=0;  agvSim.drawNodeLabels=1;

                figure(agvSim.mainFig)
                rr=rOC/max(rOC)/8*2;
                viscircles(agvSim.roadLabelPos(1:2,:)', rr,'Color','r');

                nn=nOC/max(nOC)/8*2;
                viscircles(agvSim.nodePoses(1:2,:)', nn,'Color','g');

                agvSim.drawMap();
                agvSim.mainFig=10;

                %  text(agvSim.roadLabelPos(1,:)-.5,agvSim.roadLabelPos(2,:)-.3, num2str(rOC','%3.1f'),'Color','r','fontsize',8);
                %  text(agvSim.nodePoses(1,:)+.1,agvSim.nodePoses(2,:)+.3, num2str(nOC','%3.1f'),'Color','k','fontsize',8);

                agvSim.DRAW=originalDraw;

                if 0
                    agvSim.drawMapHeatOccupancy(12);

                    T=agvSim.roadID(77).tableXYF;

                    plot(T(:,1),T(:,2),'linewidth',5)



                end

            end


        end
        %===================================================
    end
end