classdef MapClass    
    methods(Static)

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

        function drawTimelines(agvSim, AGVlist, showStart, DLloc)
            
            figure;
            hold on;

             % Sample data
            timestamps = [1, 2, 3, 4, 5, 6, 7]; % Timestamps for each point
            transitions = [2, 4, 6]; % Timestamps for transitions (vertical lines)
            stops = [3, 5]; % Timestamps for stops (rectangles)
            stopTimes = [0.5, 1.5]; % Time spent in stopped state for each stop (additional parameter)
            vehicleColor = 'red'; % Color of the vehicle line
            
            lineHeight = 0.2; % Height of the vertical lines and rectangles
            

            AGVoffset = 0;

            maxTime = 0;

            for AGV = AGVlist
                plan = AGV.planRoadTimeSinc;
                if ~isempty(AGV.lifeLongPlanHistory)
                    plan = AGV.lifeLongPlanHistory;
                end

                iddc = AGV.id;
                COLORS=[[0 0.4470 0.7410];[0.8500 0.3250 0.0980];[0.9290 0.6940 0.1250]	;[0.4940 0.1840 0.5560]	;[0.4660 0.6740 0.1880]	;[0.3010 0.7450 0.9330]	;[0.6350 0.0780 0.1840]	];
                
                N=size(plan,1);

                if N < 1
                    continue;
                end

                PP=[];
                roadOld=-1;

                if plan(end,1) > maxTime
                    maxTime = plan(end,1);
                end

                vehicleColor = COLORS(iddc,:);
    
                % Plotting the main line
                plot([0, plan(end, 1)], AGVoffset + zeros(2,1), 'LineWidth', 10, 'Color', vehicleColor);
                
                w = plan(1,1) - plan(1,4);

                % Start node
                nID = agvSim.roadID(plan(1,2)).sNode;                
                
                if showStart
                    if w > 0.1
                        % Initial stop
                        rectangle('Position', [0, (AGVoffset-lineHeight/2), w, lineHeight], 'FaceColor', 'black');
                        text(w / 2, AGVoffset -lineHeight/2 - 0.1, num2str(nID), 'HorizontalAlignment', 'center');                    
                    else
                        % Just regular start
                        line([0, 0], AGVoffset + [-lineHeight/2, lineHeight/2], 'Color', 'black');
                        %text(0, AGVoffset - lineHeight/2 - 0.1, num2str(nID), 'HorizontalAlignment', 'center');
                    end
                end
                % Plotting vertical lines for transitions
                w = 0;
                wasStop = 0;
                
                if plan(1,3) < 0.1
                    wasStop = 1;
                end

                for i = 1:N
                    road = plan(i,2);
                    if plan(i,3) < 0.1
                        nID = agvSim.roadID(road).sNode;
                    else
                        nID = agvSim.roadID(road).eNode;
                    end        

                    if i < N && plan(i,4) == plan(i+1,4)
                        w = plan(i+1,1) - plan(i,1);

                        isStart = plan(i, 5) == 1;
                        isGoal = plan(i, 5) == 2;

                        color = 'black';
                        if isStart
                            color = 'green';
                        elseif isGoal
                            color = 'red';
                        end

                        rectangle('Position', [plan(i,1), (AGVoffset-lineHeight/2), w, lineHeight], 'FaceColor', color);
                        if wasStop == 0
                            text(plan(i,1) + w / 2, AGVoffset -lineHeight/2 - 0.1, num2str(nID), 'HorizontalAlignment', 'center');
                        end
                        wasStop = 1;

                        if any(nID == DLloc)
                            text(plan(i,1) + w / 2, AGVoffset, '$DL$', 'HorizontalAlignment', 'center', 'Interpreter','latex', 'Color', 'white');
                        end
                    else
                        if wasStop
                            wasStop = 0;
                            continue;
                        end
                        line([plan(i,1), plan(i,1)], AGVoffset + [-lineHeight/2, lineHeight/2], 'Color', 'black');
                        %text(plan(i,1), AGVoffset - lineHeight/2 - 0.1, num2str(nID), 'HorizontalAlignment', 'center');
                    end
                end                

                AGVoffset = AGVoffset - 0.5;
            end                  
            
            % Adjusting plot limits and appearance
            xlim([-1, maxTime + 1]);
            ylim([AGVoffset + 0.2, 0.2]);
            %axis off;
            xlabel('$t$ [s]', 'Interpreter','latex', 'FontSize', 12);
            a = gca();
            %set(gca, 'ytick', []);
            a.YAxis.Visible = 'off';
            hold off;
        end

        %=====================================================================================
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
            MapClass.drawMap(obj);
        end



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
            MapClass.drawMap(obj);

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
            MapClass.drawMap(obj);
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
            MapClass.drawMap(obj);
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
            MapClass.drawMap(obj);


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
            MapClass.drawMap(obj);


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
            MapClass.drawMap(obj);
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
            MapClass.drawMap(obj);
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
            MapClass.drawMap(obj);
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
            MapClass.drawMap(obj);
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
            MapClass.drawMap(obj);
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
            MapClass.drawMap(obj);
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
            MapClass.drawMap(obj);
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
            MapClass.drawMap(obj);
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
            MapClass.drawMap(obj);
        end

        function createSimpleMap8a3moved(obj)
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
            MapClass.drawMap(obj);
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
            MapClass.drawMap(obj);
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
            MapClass.drawMap(obj);
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
            MapClass.drawMap(obj);
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
            MapClass.drawMap(obj);
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
            MapClass.drawMap(obj);
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
            MapClass.drawMap(obj);
        end
        %===================================================================
    end
end