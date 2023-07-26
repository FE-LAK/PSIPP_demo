function [waiting_roadID, waiting_nodeID, entrance_roadID, entrance_nodeID] = addChargingStation_v2(obj, roadID, s_int, e_int, side, dist,type)
% Funkcija na obstojeco cesto doda polnilno postajo.
% Postajo doda samo na linerani del ceste (konec in zacetek intervala 
% morata biti na istem linearnem odseku ceste)
%
% Ustvari dva tipa polnilnih postaj: 
%       Serijskega kjer cakajo zaporedno vsi na eni cesti
%       Paralelnega kjer caka vsak na svoji cesti in ima direkten dostop na
%       glavno cesto
%
% Po klicu/klicih funkcije za dodajanje polnilnih postaj je potrebno klicati se naslednje funkcije:
% figure(10); clf; axis equal; agvSim.precomputeMap(); agvSim.drawMap();
%
% -------------------------------------------------------------------
% Inputs
% obj:      Graph struktura iz simulatorja AGVSim
% roadID:   Cesta na katero zelimo dodat polnilno postajo
% side:     Stran ceste na kateri zelimo postajo ('L'=levo, 'R'=desno)
% s_int:    Zacetek zeljenega intervala (0-1)
% e_int:    Konec zeljenega intervala (0-1)
% dist:     Razdalja med obstojeco cesto in cakalno cesto polnile postaje
% type:     Tip polnilne postaje ('S'=series=cakajo zaporedno, 'P'=parallel=cakajo vzporedno)
%
% Outputs
%       waiting_roadID:   ID cest na katerih agv-ji cakajo
%       waiting_nodeID:   ID vhodnih nodov cest na katerih agv-ji cakajo
%       entrance_roadID:  ID vhodna cesta
%       entrance_nodeID:  ID node-a vhodne ceste

    % Preveri tip postaje
    if type ~= 'S' && type ~= 'P'
        error('Unknown charging station type!');
    end

    % Preveri ali ta cesta obstaja
    if obj.roadID.isKey(roadID)==0 
        error('RoadID doesnt exist!');
    end
    if side ~= 'L' && side ~= 'R'
        error('Invalid road side, use L (left) or R (right)!');
    end

    len_road=obj.roadID(roadID).length; % Dolzina ceste
    rid_shape=obj.roadID(roadID).secShape;
    rid_param=obj.roadID(roadID).param;

    sec_len=[];
    for s=1:size(rid_shape,2) % Pojdi cez odseke originalne ceste (L, A,...)
        sec_len(end+1)=rid_param(s, 5); % Dobi dolzino odseka originalne ceste (v enotah dolzine)
    end
    
    % Izracunaj kje je konec vsakega odseka originalne ceste (v procentih)
    % to rabimo, ker postajo dodamo samo na linearni odsek ceste
    sec_len=sec_len./len_road; 
    i=0;
    for s=1:size(sec_len,2)
        if s>1
            sec_len(s)=sec_len(s)+sec_len(s-1);
            if s_int > sec_len(s-1) && e_int <= sec_len(s)
                i=i+1;
                break;
            end
        else
            if s_int >= 0 && e_int <= sec_len(s)
                i=i+1;
                break;
            end
        end
    end
    if i<1 && size(sec_len,2)>1
        error('Start and end of interval are not on the same road section!');
    elseif rid_shape(s) ~= 'L'
        error('Interval is not on Linear section of the road!');
    end

    sNode=obj.roadID(roadID).sNode; % Zacetni nodeID original ceste
    eNode=obj.roadID(roadID).eNode; % Koncni nodeID original ceste
    sNode_pos=obj.getPoseOnRoad(roadID,0); % Koordinate zacetnega original node-a
    eNode_pos=obj.getPoseOnRoad(roadID,1); % Koordinate koncnega original node-a  

    % Izberi ustrezen nacin preoblikvanja obstojece ceste
    if s_int>0 && e_int<1 % Zacetek in konec intervala nista v obstojecih node-ih ceste
        new_nodes=addNodesToRoad_v2(obj, roadID, [s_int, e_int], 'M', dist);
        idx1=1; idx2=1; % Indexi za pravilno branje iz new_nodes
    elseif s_int==0 && e_int<1 % Konec intervala ni v obstojecem node-u ceste
        new_nodes=addNodesToRoad_v2(obj, roadID, [e_int], 'M', dist);
        idx1=0; idx2=1; % Indexi za pravilno branje iz new_nodes
    elseif e_int==1 && s_int>0 % Zacetek intervala ni v obstojecem node-u ceste
        new_nodes=addNodesToRoad_v2(obj, roadID, [s_int], 'M', dist);
        idx1=1; idx2=0; % Indexi za pravilno branje iz new_nodes
    elseif s_int==0 && e_int==1 % Zacetek in konec intervala sta v obstojecih node-ih ceste
        new_nodes=[[sNode_pos; 0; sNode], [eNode_pos; 1; eNode]];
        idx1=0; idx2=0; % Indexi za pravilno branje iz new_nodes
    end

    max_nodeID = double(max(cell2mat(keys(obj.nodeID)))); % Najvisji nodeID, ki ze obstaja v zemljevidu

    % Doloci pravokotni kot glede na stran ceste
    if side == 'L' 
        perp_angle = pi/2;
    elseif side == 'R'
        perp_angle = -pi/2;
    end

    % Za d_min premaknjen node pravokotno glede na cesto
    new_point1(1) = new_nodes(1,1+idx1) + (dist*cos(new_nodes(3,1+idx1)+perp_angle));
    new_point1(2) = new_nodes(2,1+idx1) + (dist*sin(new_nodes(3,1+idx1)+perp_angle));
    % Za d_min premaknjen node pravokotno glede na cesto
    new_point2(1) =  new_nodes(1,end-idx2) + (dist*cos(new_nodes(3,end-idx2)+perp_angle));
    new_point2(2) = new_nodes(2,end-idx2) + (dist*sin(new_nodes(3,end-idx2)+perp_angle));

    % Ustvari nove node in povezave za serijski tip postaje (cakajo
    % en za drugim)
    nID1=new_nodes(5,1+idx1); nID2=max_nodeID+1; p1=new_nodes([1,2],1+idx1); p2=new_point1'; rID12=nID1*100+nID2; twoway=0; rID21=nID2*100+nID1;
    entrance_nodeID = nID1;
    entrance_roadID = rID12;
    waiting_nodeID = nID2;
    secShape=['L'];
    secParam=[ p1' p2' norm(p1'-p2)]; 
    obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21)
    max_nodeID=max_nodeID+1;
    nID1=max_nodeID; nID2=max_nodeID+1; p1=new_point1'; p2=new_point2'; rID12=nID1*100+nID2; twoway=0; rID21=nID2*100+nID1;
    parallel_roadID=rID12;
    waiting_roadID=parallel_roadID;
    secShape=['L'];
    secParam=[ p1' p2' norm(p1'-p2')]; 
    obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21)
    max_nodeID=max_nodeID+1;
    nID1=max_nodeID; nID2=new_nodes(5,end-idx2); p1=new_point2'; p2=new_nodes([1,2],end-idx2); rID12=nID1*100+nID2; twoway=0; rID21=nID2*100+nID1;
    secShape=['L'];
    secParam=[ p1' p2' norm(p1'-p2')]; 
    obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21)

    % Za paralelni tip postaje dodamo se dodatne node in povezave
    if type=='P'
        waiting_roadID=[]; waiting_nodeID=[];
        main_roadID=new_nodes(5,1+idx1)*100+new_nodes(5,end-idx2);
        main_new_nodes=addNodesToRoad_v2(obj, main_roadID, [0,1], 'A', dist);
        parallel_new_nodes=addNodesToRoad_v2(obj, parallel_roadID, [0,1], 'A', dist);
    
        if size(main_new_nodes,2) ~= size(parallel_new_nodes,2)
            error('Not the same number of nodes on both roads??');
        end
        for i=1:size(main_new_nodes,2)-2
            nID1=parallel_new_nodes(5,1+i); nID2=main_new_nodes(5,1+i); p1=parallel_new_nodes([1,2],1+i); p2=main_new_nodes([1,2],1+i); rID12=nID1*100+nID2; twoway=0; rID21=nID2*100+nID1;
            waiting_roadID=[waiting_roadID; rID12];
            waiting_nodeID=[waiting_nodeID; nID1];
            secShape=['L'];
            secParam=[ p1' p2' norm(p1'-p2')]; 
            obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21)
        end
    end
    end