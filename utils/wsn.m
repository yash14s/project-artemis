clear all
close all
clc

total_nodes = 50;
base_station = 1;
triggered_node = 3;
nodePosition(1,:) = [1 1 1]; %(Base station fixed position)
no_of_neighbors = 5;
communication_range = 60;
iterationcounter = 1;

% Node position allocation
for a = 2 : total_nodes
	nodeId = a;
	pos.x = randi([1 200]); %Xpos allocation
	pos.y = randi([1 200]); %Ypos allocation
	nodePosition(a,:) = [nodeId pos.x pos.y]; %NodeID, X and Y position into nodePosition table
end

% Euclidean Distance calc from one node to all others
for i = 1 : total_nodes
	for j = 1: total_nodes
		pos.x1 = nodePosition(i,2);
		pos.x2 = nodePosition(j,2);
		pos.y1 = nodePosition(i,3);
		pos.y2 = nodePosition(j,3);
		euclidean_distance(i,j) = sqrt((pos.x1 - pos.x2)^2 + (pos.y1 - pos.y2)^2);
	end
end

% edge weight: 1 if within communication range and 0 otherwise
weights = lt(euclidean_distance,communication_range);

% Graph construction
WSN = graph(weights,'omitselfloops'); %Graph creation based on adjacency matrix (Edges matrix) built above

% Euclidean distance extraction for all existent end-to-end formed by "distance tolerance" (communication_range variable value)
for a = 1 : height(WSN.Edges)
	start_node = WSN.Edges.EndNodes(a,1);
	terminal_node = WSN.Edges.EndNodes(a,2);
	edge_distance(a,:) = euclidean_distance(start_node,terminal_node);
 end
WSN.Edges.euclidean_distance = edge_distance(:,1);

%End points allocation to WSN
for a = 1 : height(WSN.Edges)
	start_node = WSN.Edges.EndNodes(a,1);
	terminal_node = WSN.Edges.EndNodes(a,2);
	WSN.Edges.start_node_pos_x(a) = nodePosition(start_node,2);
	WSN.Edges.start_node_pos_y(a) = nodePosition(start_node,3);
	WSN.Edges.terminal_node_pos_x(a) = nodePosition(terminal_node,2);
	WSN.Edges.terminal_node_pos_y(a) = nodePosition(terminal_node,3);
	WSN.Edges.ActiveEdge(a) = 1;
end

% Original WSN plot
figure('units','normalized','innerposition',[0 0 1 1],'MenuBar','none')
subplot(1,1,1) %1,3,1 (Line number,collumn number, graph id) - if you want to show more than 1 graph in same windows
Xmax = 210;
Xmin = 0;
Ymax = 210;
Ymin = 0;
p = plot(WSN,'XData',(nodePosition(:,2)),'YData',(nodePosition(:,3)));
line(nodePosition(base_station,2),nodePosition(base_station,3),'color','green','marker','o','linestyle','none','markersize',50)
line(nodePosition(triggered_node,2),nodePosition(triggered_node,3),'color','green','marker','o','linestyle','none','markersize',50)
ax = gca;
ax.XAxis.TickValues = 0:20:200;
ax.YAxis.TickValues = 0:20:200;
grid on
hold on
title('Original WSN')
pause(2)

dead_node_list = [];

% Finding shortest path route
data_route = shortestpathtree(WSN,base_station,triggered_node);
total_hops = 0;

%% Initialize path existence test for loops
while ~isempty(data_route.Edges)
	data_route = shortestpathtree(WSN,base_station,triggered_node);
	iterationcounter = iterationcounter + 1;
	if isempty(data_route.Edges)
		break
	end

	%Find edges found in WSN
	for a = 1 : height(data_route.Edges)
		source = data_route.Edges.EndNodes(a,1);
		target = data_route.Edges.EndNodes(a,2);
		edges_row(a,:) = findedge(WSN,source,target);
	end

	%Find nodes involved in routing event
	routing_nodes = unique(data_route.Edges.EndNodes);
	route_path = shortestpath(WSN,base_station,triggered_node);

	for a = 1 : length(routing_nodes)
		routing_nodeID = routing_nodes(a,1);
		routing_nodePosition(a,:)=nodePosition(routing_nodeID,:);
	end

	%Randomly kill nodes
	all_available_nodes = uint32(2):uint32(50);
	shuffled_nodes = randperm(numel(all_available_nodes));
	nodes_to_kill = transpose(shuffled_nodes(1:5));

	[dead_node_row] = nodes_to_kill;
	for a = 1 : length(dead_node_row)
		dead_node = dead_node_row(a,1);
		for b = 1 : height(WSN.Edges)
			if ismember(WSN.Edges.EndNodes(b,1),dead_node) == 1 || ismember(WSN.Edges.EndNodes(b,2),dead_node) == 1
				WSN.Edges.ActiveEdge(b)=0;
				pause(2)
			end
		end
	end
	dead_node_list(length(dead_node_list)+1,1) = dead_node;
	[dead_egde_row]=find(WSN.Edges.ActiveEdge==0);

	%Get number of hops between source and destination for this routing session
	total_hops = length(routing_nodes);

	%plot for every dead edges iteration's result
	figure('units','normalized','innerposition',[0 0 1 1],'MenuBar','none')
	p = plot(WSN,'XData',(nodePosition(:,2)),'YData',(nodePosition(:,3)));
	line(nodePosition(base_station,2),nodePosition(base_station,3),'color','green','marker','o','linestyle','none','markersize',50)
	line(nodePosition(triggered_node,2),nodePosition(triggered_node,3),'color','green','marker','o','linestyle','none','markersize',50)
	random_name.ax = gca;
	random_name.ax.XAxis.TickValues = 0:20:200;
	random_name.ax.YAxis.TickValues = 0:20:200;
	hold on

	%Plot all dead nodes in red
	for a = 1 : length(dead_node_list)
		random_name.b=dead_node_list(a,1);
		scatter(nodePosition(random_name.b,2),nodePosition(random_name.b,3),'MarkerFaceColor','red');
	end

	%Title for every iteration
	title(['WSN shortest path in interation : ',num2str(iterationcounter),' |Hops: ',num2str(total_hops),' |Dead node: ',num2str(dead_node),' |Router nodes: ', num2str(route_path)])
	grid on
	pause(2)

	%Remove dead edges from graph
	WSN = rmedge(WSN,dead_egde_row(:,1));

	%Mark nodes involved in route with green color
	scatter(routing_nodePosition(:,2),routing_nodePosition(:,3),'MarkerFaceColor','green');
	pause(0.2)

	%Clear router nodes position to avoid erroneous plot
	clear routing_nodePosition
end

%Plot dead nodes
	for a = 1 : length(dead_node_list)
		random_name.b=dead_node_list(a,1);
		scatter(nodePosition(random_name.b,2),nodePosition(random_name.b,3),'MarkerFaceColor','red');
	end

%Title for the last plot
title(['WSN shortest path in iteration: ',num2str(iterationcounter),' |Hops: ',num2str(total_hops),' |Dead node: ',num2str(dead_node),' |Router nodes: ', num2str(route_path)])