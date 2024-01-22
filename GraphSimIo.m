classdef GraphSimIo < handle
    properties (Access = public)
        pick = [];
        drop = [];

        settings = [];

        forceTwoway = false;
    end

    properties (Access = protected)
        agvSim = []; %AgvSim object

        node = []; %ROS node
        pubPlans = []; %ROS publisher of plans
        subScenario = []; %ROS subscriber to scenarios
    end
    
    methods (Access = public)
        function obj = GraphSimIo(agvSim)
            %GraphSim input/output interface
            %
            %in agvSim AgvSim object.
            %
            %out obj This object.
            obj = obj@handle();
            obj.agvSim = agvSim;
        end

        function obj = loadMap(obj, file)
            map = flipud(imread(file));

            cm = uint8(zeros(256, 3));
            cm(256,:) = [255, 255, 255];
            cm(101,:) = [255, 0, 0];
            cm(151,:) = [255, 255, 0];
            cm(201,:) = [0, 1, 0];
            cm(1,:) = [220, 220, 220];
            
            [M, N] = size(map);
            mx = linspace(1, N, N)-0.5;
            my = linspace(1, M, M)-0.5;
            
            image(mx, my, map);
   %         colormap(cm);

            axis tight equal;
        end

        function obj = loadScenario(obj, file)
            %Load scenario from file
            %
            %in file File name.
            %
            %out obj This object.
            data = fileread(file);
            parcel = jsondecode(data);
            obj.getWorld(parcel.worlds(1));

            obj.getAgvs(parcel.agvs);
            obj.getStations(parcel.stations);

            obj.settings = parcel.settings;
        end

        function obj = loadWorld(obj, file)
            %Load world from file
            %
            %in file File name.
            %
            %out obj This object.
            data = fileread(file);
            parcel = jsondecode(data);
            obj.getWorld(parcel);
        end

        function obj = getWorld(obj, parcel)
            %Get world from parcel
            %
            %in parcel Parcel.
            %
            %out obj This object.
            world = GraphSimIo.parcel2world(parcel, obj.forceTwoway);
            
            for i = 1:length(world.connections)
                c = world.connections{i};
                obj.agvSim.addTwoNodesAndConnection(c.from, c.to, c.loc_from, c.loc_to, c.id, c.modes, c.params, c.twoway, c.di);
            end
            obj.agvSim.precomputeMap();
            
            n = length(parcel.nodes);
            for i = 1:n
                node = parcel.nodes(i);
                if isfield(node, 'nearbyNode')
                    a = obj.agvSim.nodeID(node.id);
                    a.nearbyNode = node.nearbyNode;
                    obj.agvSim.nodeID(node.id) = a;
                end
            end
            
            obj.agvSim.drawMap();
        end

        function obj = getAgvs(obj, parcel)
            %Get AGVs from parcel
            %
            %in parcel Parcel.
            %
            %out obj This object.
            for i = 1:length(parcel)
                obj.getAgv(parcel(i));
            end
        end

        function obj = getAgv(obj, parcel)
            %Get AGV from parcel
            %
            %in parcel Parcel.
            %
            %out obj This object.
            agv = GraphSimIo.parcel2agv(parcel);
            obj.agvSim.createAGVonRoadLW(agv.id, agv.road, agv.at, max(abs(agv.shape(1,:))), max(abs(agv.shape(2,:))), 0);
        end

        function obj = getStations(obj, parcel)
            %Get stations from parcel
            %
            %in parcel Parcel.
            %
            %out obj This object.
            for i = 1:length(parcel)
                obj.getStation(parcel(i));
            end
        end

        function obj = getStation(obj, parcel)
            %Get station from parcel
            %
            %in parcel Parcel.
            %
            %out obj This object.
            station = GraphSimIo.parcel2station(parcel);

            if strcmp(station.type, 'pick_drop')
                obj.pick = [obj.pick; station.node];
                obj.drop = [obj.drop; station.node];
            elseif strcmp(station.type, 'pick_up')
                obj.pick = [obj.pick; station.node];
            elseif strcmp(station.type, 'drop_off')
                obj.drop = [obj.drop; station.node];
            end
        end

        function obj = saveAgvs(obj, file, orders, plans)
            %Save agvs to file
            %
            %in file File name.
            %in agvs A matrix of orders.
            %in plans A cell array of plans.
            %
            %out obj This object.
            agvs = {};
            colors = {'cyan', 'orange', 'yellow', 'violet', 'lime'};
            for i = 1:size(orders, 1)
                road = 0;
                at = 0;
                if i <= length(plans) && ~isempty(plans{i})
                    road = plans{i}(1,2);
                    at = plans{i}(1,3);
                end
                L = orders(i,6);
                W = orders(i,7);
                agv = struct('id', orders(i,1), 'world', 1, 'road', road, 'at', at, 'max_speed', orders(i,3), 'shape', [-L, -W, L*0.8, -W, L, 0, L*0.8, W, -L, W, -L, 0], 'colour', colors{1+mod(i-1, length(colors))});
                agvs = [agvs(:); agv];
            end
            data = jsonencode(agvs);
            f = fopen(file, 'w'); fprintf(f, data); fclose(f);
        end

        function obj = savePlans(obj, file, agvs, plans)
            %Save plans to file
            %
            %in file File name.
            %in agvs An array of AGV IDs.
            %in plans A cell array of plans.
            %
            %out obj This object.
            parcel = GraphSimIo.plans2parcel(agvs, plans);
            data = jsonencode(parcel);
            f = fopen(file, 'w'); fprintf(f, data); fclose(f);

%             for i = 1:length(agvs)
%                 if ~isempty(plans{i}) 
%                     obj.agvSim.setPlanRoadTimeSinc (agvs(i), plans{i});
%                 end
%             end
        end

        function obj = initNode(obj, name)
            if nargin < 2 || isempty(name)
                name = 'graph_sim_io';
            end
            obj.node = ros2node(name);

            obj.pubPlans = ros2publisher(obj.node, 'plans', 'std_msgs/String');

            obj.subScenario = ros2subscriber(obj.node, 'scenario', 'std_msgs/String', @(msg)obj.handleScenario(msg)); %'Durability', 'transientlocal', 'History', 'keeplast', 'Depth', 1
        end

        function obj = publishPlans(obj, agvs, plans)
            msgPlans = ros2message(obj.pubPlans);
            parcel = GraphSimIo.plans2parcel(agvs, plans);
            msgPlans.data = jsonencode(parcel);
        
            send(obj.pubPlans, msgPlans);
        end

        function handleScenario(obj, msg)
            parcel = jsondecode(msg);
            obj.setWorld(parcel);
        end
    end

    methods (Static)
        function world = parcel2world(data, forceTwoway)
            if isfield(data, 'id')
                world_id = data.id;
            else
                world_id = 0;
            end
            if nargin<2 || isempty(forceTwoway)
                forceTwoway = false;
            end
    
            connections = {};
            n = length(data.roads);
            fix = false;
            if n == 0
                n = length(data.edges);
                fix = true;
            end
            for i = 1:n
                if fix
                    road = data.edges(i);    
                else
                    road = data.roads(i);
                end
                if road.id == 0
                    road.id = 1000000; %FIX
                end
    
                loc_from = [];
                loc_to = [];
                modes = '';
                params = [];
    
                s = length(road.shape);
                for j = 1:s
                    shape = road.shape{j};
                    if shape{1} == 'X'
                        shape{1} = 'L';
                    end
                    %param = [];
                    if shape{1} == 'L'
                        p = [shape{2:end}];
                        d = sum(sqrt(sum(diff(reshape(p, 2, []), [], 2).^2)));
                        param = [p, d];
                        if isempty(loc_from)
                            loc_from = p(1:2);
                        end
                        loc_to = p(end-1:end);
                    elseif shape{1} == 'A'
                        Sx = shape{2} + shape{4} * cos(shape{5} / 180.0 * pi);
                        Sy = shape{3} + shape{4} * sin(shape{5} / 180.0 * pi);
                        Ex = shape{2} + shape{4} * cos((shape{5} + shape{6}) / 180.0 * pi);
                        Ey = shape{3} + shape{4} * sin((shape{5} + shape{6}) / 180.0 * pi);
                        fi0 = (shape{5} + shape{6}) / 180.0 * pi;
                        dD = abs(shape{4}) * abs(shape{6} / 180.0 * pi); % abs je brez veze ce je RR je sicer vedno pozitiven?
                        param = [Sx, Sy, fi0, shape{6} / 180.0 * pi, dD]; % parametri kroznega loka
                        if isempty(loc_from)
                            loc_from = [Sx, Sy];
                        end
                        loc_to = [Ex, Ey];
                    else
                        continue;
                    end
                    modes = [modes, shape{1}];
                    params = [params; param];
                end
    
                if ~isempty(modes)
                    if forceTwoway
                        skip = false;
                        for i = 1:length(connections)
                            c = connections{i};
                            if c.from == road.to && c.to == road.from
                                connections{i}.twoway = true;
                                skip = true;
                                break;
                            end
                        end
                        if skip, continue; end
                    end
                    connections = [connections(:); struct('id', road.id, 'from', road.from, 'to', road.to, 'loc_from', loc_from, 'loc_to', loc_to, 'modes', modes, 'params', params, 'twoway', boolean(road.twoway), 'di', -road.id)];                    
                end
            end

            world = struct('id', world_id, 'connections', {connections});
        end

        function worlds = parcel2worlds(parcel, forceTwoway)
            if nargin<2 || isempty(forceTwoway)
                forceTwoway = false;
            end
            worlds = {};
            for w = 1:length(parcel)
                worlds = [worlds(:); parcel2world(parcel{w}, forceTwoway)];
            end
        end

        function agv = parcel2agv(data)
            if isfield(data, 'id')
                agv_id = data.id;
            else
                agv_id = 0;
            end

            agv = struct('id', agv_id, 'world', data.world, 'road', data.road,  'at', data.at, 'shape', reshape(data.shape, 2, []), 'max_speed', data.max_speed, 'reverse', data.reverse, 'pose', data.pose);
        end

        function station = parcel2station(data)
            if isfield(data, 'id')
                station_id = data.id;
            else
                station_id = 0;
            end

            station = struct('id', station_id, 'node', data.node, 'type', data.type);
        end

        function parcel = plans2parcel(agvs, plans)
            n = length(agvs);
            parcel = {};
            for i = 1:n
                agv = agvs(i);
                plan = plans{i};
                if i > length(plans) || isempty(plan)
                    continue;
                end
                actions = {};
                for j = 1:size(plan, 1)
                    time = plan(j,1);
                    road_id = int64(plan(j,2));
                    road_at = plan(j,3);
                    action = struct('time', time, 'at', road_at, 'road', road_id);
                    actions = [actions(:); action];
                end
                parcel = [parcel(:); struct('id', i, 'agv', agv, 'actions', {actions})];
            end
        end
    end
end