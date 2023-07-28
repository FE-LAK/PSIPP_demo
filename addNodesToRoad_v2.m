function [new_nodes] = addNodesToRoad_v2(obj, roadID, interval, mode, d_min)
% Funkcija v nacinu "Auto" (mode='A') glede na podano minimalno razdaljo 
% (v enotah dolzine oz. d_min) ter podan zacetek in konec intervala 
% (v procentih celotne dolzine ceste oz med 0-1) ki jih podamo v interval=[s_int, e_int]
% ustvari nove node in povezave na lokaciji obstojece ceste.
%
% Ce je zacetek/konec intervala blizje node-u originalne ceste, kot je minimalna
% razdalja (d_min), potem funkcija node-a na mestu zacetka/konca intervala ne ustvari.
% Uporabno, da robot ne bi cakal v node-u, ki je preblizu node-u originalne ceste, 
% kar lahko povzroci trk.
%
% Funkcija v nacinu "Manual" (mode='M') ustvari nove node glede na podane
% lokacije na cesti (v procentih celotne dolzine ceste oz med 0-1) ki jih podamo 
% v interval=[node1, node2, node3, ...]
% Uporabno kadar zelimo na tocno dolocenih mestih ceste ustvariti en ali vec node-ov.
%
% Po klicu/klicih funkcije za dodajanje vmesnih vozlisc je potrebno klicati se naslednje funkcije:
% figure(10); clf; axis equal; agvSim.precomputeMap(); agvSim.drawMap();
%
% -------------------------------------------------------------------
% Inputs
% obj:  Graph struktura iz simulatorja AGVSim
% roadID:  Cesta na katero zelimo dodat vmesne node
% interval: [s_int, e_int] kjer je:
%       s_int:   Zacetek zeljenega intervala (0-1)
%       e_int:   Konec zeljenega intervala (0-1)
% mode:    Nacin 'A' = auto, 'M' = manual
% dmin:    Minimalna razdalja med novimi node-i
%
% Outputs
% new_nodes: podatki node-ov na cesti [x, y, phi, location_on_road, nodeID; 
%                                      x, y, phi, location_on_road, nodeID;
%                                       ...]

    % Za Auto mode rabimo samo zacetek in konec intervala
    if mode == 'A' && size(interval,2)>2
        error('Too much elements in interval in Auto mode!');
    elseif mode == 'A' && size(interval,2)<2 
       error('Too few elements in interval in Auto mode!');
    end

    s_int=interval(1);
    e_int=interval(end);

    if obj.roadID.isKey(roadID)==0 % Preveri ali ta cesta obstaja
        error('RoadID doesnt exist!');
    elseif s_int > e_int % Zacetek ne sme bit vecji od konca
        error('The beginning value of the interval is greater than the value of the end!');
    elseif e_int < s_int % Konec ne sme bit manjsi od zacetka
        error('The end of the interval value is less than the value of the beginning!');
    end

    opp_roadID=obj.roadID(roadID).twoWayID; % Ime nasprotne ceste (ce je 2 smerna >0, ce je enosmerna =0)
    rid_shape=obj.roadID(roadID).secShape; % Zapomni si odseke original ceste preden jo zbrises
    rid_param=obj.roadID(roadID).param; % Zapomni si parametre original ceste preden jo zbrises

    len_road=obj.roadID(roadID).length; % Dolzina ceste
    len_int=len_road-(len_road*s_int+len_road*(1-e_int)); % Dolzina intervala
    num_conn=floor(len_int/d_min); % Stevilo novih povezav
    len_conn=len_int/num_conn; % Dejanska dolzina novih povezav je >= d_min

    if num_conn<=1 && s_int~=e_int % Stevilo novih povezav mora biti > 1 drugace je prekratka cesta
        error('Road or selected interval is too short according to the minimum distance!');
    end
    if s_int==e_int && (s_int*len_road<d_min || (1-e_int)*len_road<d_min) % Node mora biti oddaljn od 
        % node-ov originalne ceste vsaj za razdaljo d_min tudi kadar zelimo ustavriti samo enega
        error('Desired node position is too close to existing node according to the minimum distance!');
    end
    if num_conn<=1 && s_int==e_int % Ce je zacetek in konec intervala enaka vrednost naredimo node na tej vrednosti (0-1)
        num_conn=2;
    end

    sNode=obj.roadID(roadID).sNode; % Zacetni nodeID original ceste
    eNode=obj.roadID(roadID).eNode; % Koncni nodeID original ceste
    sNode_pos=obj.getPoseOnRoad(roadID,0); % Zacetni original node lokacija
    eNode_pos=obj.getPoseOnRoad(roadID,1); % Koncni original node lokacija

    new_nodes=[];
    ln=[];
    if mode == 'M' % Manual mode
        % V tem primeru nove node dobimo kar iz seznama interval
        for k = 1:size(interval,2) % Dobimo kje hocemo nove node
            ln(end+1)=interval(k); % Kje na intervalu je node (0-1)
            new_nodes(:,end+1)=obj.getPoseOnRoad(roadID,ln(k)); % x, y in fi node-a
        end
        if s_int*len_road<d_min % Razdalja med sNode in s_int mora bit >= d_min
            error('Desired node position is too close to existing node according to the minimum distance!');
        end
        if (1-e_int)*len_road<d_min % Razdalja med eNode in e_int mora bit >= d_min
            error('Desired node position is too close to existing node according to the minimum distance!');
        end
    elseif mode == 'A' % Auto mode
        % V tem primeru nove node izracunamo
        % Dobi lokacije (x, y) za vmesne node
        for k = 1:num_conn-1
            ln(end+1)=s_int+(e_int-s_int)/num_conn*k; % Kje na intervalu je node (0-1)
            new_nodes(:,end+1)=obj.getPoseOnRoad(roadID,ln(k)); % x, y in fi node-a
        end
        if s_int*len_road>=d_min && s_int~=e_int % Ce razdalja med sNode (zacetni node originalne ceste) 
            % in s_int >= d_min dodamo tudi s_int (zacetek zeljenega intervala) na seznam node-ov, drugace pa ne
            st_point=obj.getPoseOnRoad(roadID, s_int);
            ln=[s_int ln];
            new_nodes=[st_point new_nodes];
        end
        if (1-e_int)*len_road>=d_min && e_int~=s_int % Ce razdalja med eNode (koncni node originalne ceste)
            % in e_int >= d_min dodamo tudi e_int (konec zeljenega intervala) na seznam node-ov, drugace pa ne
            end_point=obj.getPoseOnRoad(roadID, e_int);
            ln=[ln e_int];
            new_nodes=[new_nodes end_point];
        end
    end

    % Dodamo tudi sNode (zacetni node originalne ceste) na seznam node-ov
    ln=[0 ln];
    new_nodes=[sNode_pos new_nodes];
    % Dodamo tudi eNode (koncni node originalne ceste) na seznam node-ov
    ln=[ln 1];
    new_nodes=[new_nodes eNode_pos];

    max_nodeID = double(max(cell2mat(keys(obj.nodeID)))); % Najvisji nodeID, ki ze obstaja v zemljevidu. 
    % Mora biti pretvorjen v double drugace celo strukturo pretvori v tip int64...
    % Dodamo nove ID-je node-ov
    ID=[];
    for i=2:size(new_nodes,2)-1 
        ID(end+1)=max_nodeID+1; max_nodeID=max_nodeID+1;
    end
    ID=[sNode ID]; % ID obstojecega node-a originalne ceste
    ID=[ID eNode]; % ID obstojecega node-a originalne ceste

    new_nodes(end+1,:)=ln; % Dodaj lokacije na cesti (0-1)
    new_nodes(end+1,:)=ID; % Dodaj nodeID

    remove(obj.roadID, roadID);  % Odstrani cesto iz seznama cest

    % Pobrisemo cesto iz seznama sosednjih cest od sNode
    nRids=obj.nodeID(sNode).neighboarRoadIDs;
    idx=find(nRids==roadID); % Index ceste
    st=obj.nodeID(sNode); % Preberi
    st.neighboarRoadIDs(idx)=[]; % Pobrisi
    obj.nodeID(sNode)=st; % Zapisi nazaj

    % Pobrisemo koncni node ceste iz seznama sosednjih od zacetnega node-a 
%     nNodes=obj.nodeID(sNode).neighboarNodeIDs;
%     idx=find(nNodes==eNode); % Index node-a
    % To je zakomentirano, ker drugace v primeru, ko imamo med dvema
    % vozliscema dve vzporedni cesti potem iz seznama sosednjih vozlisc lahko izbrise
    % tistega ki pripada cesti, ki je ne spreminjamo... -> index node-a iz neighboarNodeIDs
    % mora bit enak indexu ceste do tega node-a iz neighboarRoadIDs...
    st=obj.nodeID(sNode); % Preberi
    st.neighboarNodeIDs(idx)=[]; % Pobrisi
    obj.nodeID(sNode)=st; % Zapisi nazaj

    if opp_roadID > 0 % Ce obstaja nasprotna cesta oz. je izbrana cesta dvosmerna
        remove(obj.roadID, opp_roadID);  % Odstrani tudi nasprotno cesto iz seznama cest (ce je 2 smerna)
        % Pobrisemo cesto iz seznama sosednjih cest od eNode
        nRids=obj.nodeID(eNode).neighboarRoadIDs;
        idx=find(nRids==opp_roadID); % Index ceste
        st=obj.nodeID(eNode); % Preberi
        st.neighboarRoadIDs(idx)=[]; % Pobrisi
        obj.nodeID(eNode)=st; % Zapisi nazaj

        % Pobrisemo zacetni node ceste iz seznama sosednjih od koncnega node-a  
%         nNodes=obj.nodeID(eNode).neighboarNodeIDs;
%         idx=find(nNodes==sNode); % Index node-a
        % To je zakomentirano, ker drugace v primeru, ko imamo med dvema
        % vozliscema dve vzporedni cesti potem iz seznama sosednjih vozlisc lahko izbrise
        % tistega ki pripada cesti, ki je ne spreminjamo... -> index node-a iz neighboarNodeIDs
        % mora bit enak indexu ceste do tega node-a iz neighboarRoadIDs...
        st=obj.nodeID(eNode); % Preberi
        st.neighboarNodeIDs(idx)=[]; % Pobrisi
        obj.nodeID(eNode)=st; % Zapisi nazaj

        twoway=1;
    else
        % Ce je cesta enosmerna v koncnem node-u ni potrebno brisat zacetnega node-a 
        % ker ga ni v seznamu, prav tako tudi ni obratne ceste
        twoway=0;
    end 

    sec_len=[];
    for s=1:size(rid_shape,2) % Pojdi cez odseke originalne ceste (L, A,...)
        sec_len(end+1)=rid_param(s, 5); % Dobi dolzino odseka originalne ceste (v enotah dolzine)
    end
    
    % Izracunaj kje je konec vsakega odseka originalne ceste (v procentih)
    sec_len=sec_len./len_road; 
    for s=1:size(sec_len,2)
        if s>1
            sec_len(s)=sec_len(s)+sec_len(s-1);
        end
    end

    % Dodajanje novih node-ov in povezav med njimi
    offst=0;
    secShape=[]; secParam=[];
    for s=1:size(rid_shape,2) % Pojdi cez odseke originalne ceste
        for i=offst+1:size(new_nodes,2)-1 % Pojdi cez vse vmesne node
            if new_nodes(4,i+1) <= sec_len(s) % Trenutni in naslednji node sta na istem odseku originalne ceste
                if rid_shape(s)=='L'
                    %disp('L')
                    nID1=new_nodes(5,i); nID2=new_nodes(5,i+1); 
                    p1= new_nodes([1,2],i); p2=new_nodes([1,2],i+1); rID12=nID1*100+nID2; rID21=nID2*100+nID1;
                    secShape(end+1)=['L'];
                    secParam(end+1,:)=[p1' p2' norm(p1-p2)];
                    obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
                    secShape=[]; secParam=[];
                    offst=i;
                    %break
                elseif rid_shape(s)=='A'
                    %disp('A')
                    nID1=new_nodes(5,i); nID2=new_nodes(5,i+1); 
                    p1= new_nodes([1,2],i); p2=new_nodes([1,2],i+1); rID12=nID1*100+nID2; rID21=nID2*100+nID1;
                    secShape(end+1)=['A'];
                    % A=[x0,y0,fi_start,delta_fi,razdalja_po_kroznem_loku]
                    d_fi=new_nodes(3,i+1)-new_nodes(3,i);
                    arclen=norm(p1-p2)/(2*sin(d_fi/2))*d_fi;
                    secParam(end+1,:)=[p1' new_nodes(3,i) d_fi arclen];
                    %(end+1,:)=[p1' mid_nodes(3,i) mid_nodes(3,i+1)-mid_nodes(3,i) len_sec]
                    obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
                    secShape=[]; secParam=[];
                    offst=i;
                    %break
                end
            elseif new_nodes(4,i) > sec_len(s) % Trenutni node se nahaja na naslednem odseku originalne ceste. Da ne vlece povezave 
                % od konca prejsnega odseka do prvega node-a na zdejsnjem odseku
                break;
            else
                % Tu je treba sestavlat skupaj nove vmesne ceste, ki so delno v enem in
                % drugem odseku originalne ceste (naprimer del v 'L' in del v 'A'...)
                if isempty(secShape) % Ce je prazen secShape je to nov odsek med node-om in mejo odseka originalne ceste
                    if rid_shape(s)=='L'
                        %disp('LL')
                        p1= new_nodes([1,2],i); p2=rid_param(s,[3,4])'; 
                        secShape(end+1)=['L'];
                        secParam(end+1,:)=[p1' p2' norm(p1-p2)];
                    elseif rid_shape(s)=='A'
                        %disp('AA')
                        p1= new_nodes([1,2],i); p2=rid_param(s+1,[1,2])'; 
                        secShape(end+1)=['A'];
                        % A=[x0,y0,fi_start,delta_fi,razdalja_po_kroznem_loku]
                        d_fi=rid_param(s,3)+rid_param(s,4)-new_nodes(3,i);
                        arclen=norm(p1-p2)/(2*sin(d_fi/2))*d_fi;
                        secParam(end+1,:)=[p1' new_nodes(3,i) d_fi arclen];
                    end
                end
                if new_nodes(4,i+1) > sec_len(s+1) % Naslednji node ni na naslednjem odseku ampak je vmes se en odsek original ceste brez node-a
                    if rid_shape(s+1)=='L'
                        %disp('LL NEW')
                        p1= rid_param(s+1,[1,2])'; p2=rid_param(s+1,[3,4])'; 
                        secShape(end+1)=['L'];
                        secParam(end+1,:)=[p1' p2' norm(p1-p2)];
                    elseif rid_shape(s+1)=='A'
                        %disp('AA NEW')
                        p1= rid_param(s+1,[1,2])'; p2=rid_param(s+2,[1,2])'; 
                        secShape(end+1)=['A'];
                        % A=[x0,y0,fi_start,delta_fi,razdalja_po_kroznem_loku]
                        d_fi=rid_param(s+1,4);
                        arclen=norm(p1-p2)/(2*sin(d_fi/2))*d_fi;
                        secParam(end+1,:)=[p1' rid_param(s+1,3) d_fi arclen];
                    end
                    break; % Ne vem tocno zakaj mora biti tukaj break... ampak deluje
                end
                if ~isempty(secShape) % Ce ni prazen secShape je to nov odsek med mejo odseka originalne ceste in do naslednjega node-a
                    if rid_shape(s+1)=='L'
                        %disp('LLL')
                        nID1=new_nodes(5,i); nID2=new_nodes(5,i+1); 
                        p1= rid_param(s+1,[1,2])'; p2=new_nodes([1,2],i+1); rID12=nID1*100+nID2; rID21=nID2*100+nID1;
                        secShape(end+1)=['L'];
                        secParam(end+1,:)=[p1' p2' norm(p2-p1)];
                        p1=secParam(1,[1,2])';
                        obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21)
                        secShape=[]; secParam=[];
                        
                    elseif rid_shape(s+1)=='A'
                        %disp('AAA')
                        nID1=new_nodes(5,i); nID2=new_nodes(5,i+1); 
                        p1= rid_param(s+1,[1,2])'; p2=new_nodes([1,2],i+1); rID12=nID1*100+nID2; rID21=nID2*100+nID1;
                        secShape(end+1)=['A'];
                        % A=[x0,y0,fi_start,delta_fi,razdalja_po_kroznem_loku]
                        d_fi=new_nodes(3,i+1)-rid_param(s+1,3);
                        arclen=norm(p1-p2)/(2*sin(d_fi/2))*d_fi;
                        secParam(end+1,:)=[p1' rid_param(s+1,3) d_fi arclen];
                        %secParam(end+1,:)=[p1' rid_param(s+1,3) mid_nodes(3,i+1)-rid_param(s+1,3) mid_nodes(4,i+1)*len_road-rid_param(s,5)];
                        p1=secParam(1,[1,2])';
                        obj.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
                        secShape=[]; secParam=[];
                    end
                end
                offst=i;
                break
            end
        end
    end
end

