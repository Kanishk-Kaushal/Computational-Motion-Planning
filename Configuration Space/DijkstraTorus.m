function route = DijkstraTorus (input_map, start_coords, dest_coords)

% Run Dijkstra's algorithm on a grid.

% Inputs : 
%   input_map : a logical array where the freespace cells are false or 0 and
%   the obstacles are true or 1
%   start_coords and dest_coords : Coordinates of the start and end cell
%   respectively, the first entry is the row and the second the column.

% Output :
%    route : An array containing the linear indices of the cells along the
%    shortest route from start to dest or an empty array if there is no
%    route. This is a single dimensional vector
%    numExpanded: Remember to also return the total number of nodes
%    expanded during your search. Do not count the goal node as an expanded node.

% set up color map for display

% 1 - white - clear cell
% 2 - black - obstacle
% 3 - red = visited
% 4 - blue  - on list
% 5 - green - start
% 6 - yellow - destination

cmap = [1 1 1; ...
        0 0 0; ...
        1 0 0; ...
        0 0 1; ...
        0 1 0; ...
        1 1 0];

colormap(cmap);


[nrows, ncols] = size(input_map);

% map - a table that keeps track of the state of each grid cell
map = zeros(nrows,ncols);

map(~input_map) = 1;  % Mark free cells
map(input_map)  = 2;  % Mark obstacle cells

% Generate linear indices of start and dest nodes
start_node = sub2ind(size(map), start_coords(1), start_coords(2));
dest_node  = sub2ind(size(map), dest_coords(1),  dest_coords(2));

map(start_node) = 5;
map(dest_node)  = 6;

% Initialize distance array
distances = Inf(nrows,ncols);

% For each grid cell this array holds the index of its parent
parent = zeros(nrows,ncols);

distances(start_node) = 0;

% Main Loop
while true
    
    % Draw current map
    map(start_node) = 5;
    map(dest_node) = 6;
    
    image(1.5, 1.5, map);
    grid on;
    axis image;
    drawnow;
    
    % Find the node with the minimum distance
    [min_dist, current] = min(distances(:));
    
    if ((current == dest_node) || isinf(min_dist))
        break;
    end;
    
    % Update map
%    map(current) = 3;         % mark current node as visited
%    distances(current) = Inf; % remove this node from further consideration
    
    % Compute row, column coordinates of current node
    [i, j] = ind2sub(size(distances), current);
    
    % Visit each neighbor of the current node and update the map, distances
    % and parent tables appropriately.
   
    %%% All of your code should be between the two lines of stars. 
    % *******************************************************************
    
    % Accessible Neighbors -> Exlude Red and Black, Visit White and Blue
    n = Dijkstra_Grid_Neighbors (map, i, j);   

    % Dijkstra Algorithm:

    % Length of edge = 1 as it is a Square Grid
    k = distances(n) > (distances(current) + 1);        % Which cell has higher distance value 
    Index_Cell = find (k ==1);                          % Index_Cell is the index in n of those cells
    distances(n(Index_Cell)) = distances(current) + 1;  % Update distance
    distances(current) = Inf;                           % Remove this node from further consideration
    map(current) = 3;                                   % Mark current node as "expanded" (red)
    parent(n(Index_Cell)) = current; 
    map(n(Index_Cell)) = 4;                             % Blue = (4) = On List
    
    % *******************************************************************
    
end

if (isinf(distances(dest_node)))
    route = [];
else
    route = [dest_node];
    
    while (parent(route(1)) ~= 0)
        route = [parent(route(1)), route];
    end
end

    function update (i,j,d,p)
        if ( (map(i,j) ~= 2) && (map(i,j) ~= 3) && (map(i,j) ~= 5) && (distances(i,j) > d) )
            distances(i,j) = d;
            map(i,j) = 4;
            parent(i,j) = p;
        end
    end

end

% Find the neighbors of cell (I, J) on a 2D grid for Dijkstra's Algorithm
% Input 1 -> Grid - the 2D grid presented as a matrix
% Input 2 & 3 -> (I, J) - the Coordinate of the Current Node or cell on the grid, measured from the top-left corner as (1, 1), I = row, J = column             
% Output -> List of linear positions of the neighbors on the grid

function n = Dijkstra_Grid_Neighbors (G, I, J)

    nrows = size(G, 1);
    ncols = size(G, 2);
    
    n = [];
    i = 1;
    
    % sub2ind(sz, row, col) -> gives linear position of the cell with
    %                          respect to the grid ie. in a grid of 3X3 (1,1) = 1 , (1,2) = 2 ,
    %                          (1,3) = 3 , (2,1) = 4 and so on.
    
    if (I == 1) && (G(nrows, J) ~= 2 && G(nrows, J) ~= 3 && G(nrows, J) ~= 5)   % If the cell is on the TOP (1st ROW) 
        n(i) = sub2ind(size(G), nrows, J);                                      % Its Neighbor is the cell at the BOTTOM (LAST ROW)
        i = i+1;
    end
    
    if I > 1 && (G(I-1, J) ~= 2 && G(I-1, J) ~= 3 && G(I-1, J) ~= 5)            % If the UPPER cell is inside of the grid and it is not an obstacle and not the visited cell and not the start node 
        n(i) = sub2ind(size(G), I-1, J);
        i = i+1;
    end
    
    if I == nrows && (G(1, J) ~= 2 && G(1, J) ~= 3 && G(1, J) ~= 5)             % If the cell is at the BOTTOM (LAST ROW)
        n(i) = sub2ind(size(G), 1, J);                                          % Its neighbor is on the TOP (1st ROW)
        i = i+1;
    end
    
    if I + 1 <= nrows && (G(I+1, J) ~= 2 && G(I+1, J) ~= 3 && G(I+1, J) ~= 5)   % If the LOWER cell is inside of the grid and it is not an obstacle and not the visited cell and not the start node
        n(i) = sub2ind(size(G), I+1, J);
        i = i+1;
    end
    
    if J == 1 && (G(I, ncols) ~= 2 && G(I, ncols) ~= 3 && G(I, ncols) ~= 5)     % If the cell is on the LEFT MOST
        n(i) = sub2ind(size(G), I, ncols);                                      % its neighbor is on the RIGHT MOST
        i = i+1;
    end
    
    if J - 1 > 0 && (G(I, J-1) ~= 2 && G(I, J-1) ~= 3 && G(I, J-1) ~= 5)        % Cell on the LEFT
        n(i) = sub2ind(size(G), I, J-1);
        i = i+1;
    end
    
    if J == ncols && (G(I, 1) ~= 2 && G(I, 1) ~= 3 && G(I, 1) ~= 5)             % If the cell is on the RIGHT MOST
        n(i) = sub2ind(size(G), I, 1);                                          % Its neighbor is on the LEFT MOST
        i = i+1;
    end
    
    if J + 1 <= ncols && (G(I, J+1) ~= 2 && G(I, J+1) ~= 3 && G(I, J+1) ~= 5)   % Cell on the RIGHT
        n(i) = sub2ind(size(G), I, J+1);
    end

end