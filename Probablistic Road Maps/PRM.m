% PRM -> Probablistic Road Maps
function roadmap = PRM (RandomSample, Dist, LocalPlanner, nsamples, k)

% PRM - ProbablisticRoadMap : This procedure computes a probabilistic road
% map of configuration space. It relies on 3 functions
% RandomSample which generates the coordinate vector for a random sample in
% free space. Dist which measures the distance between two
% coordinate vectors and LocalPlanner which decides whether two samples are
% connected.
%
% Inputs :
%
%   RandomSample : A function that returns a random sample in freespace
%
%   Dist : A function that computes the distance between a given point in
%        configuration space and all of the entries in an array of samples
%
%   LocalPlanner :  A function that determines whether there is a collision
%        free straight line path between two points in configuration space
%
%   nsamples : The number of random samples to generate
%
%   k : The number of neighbors that should be considered in
%        forming the roadmap graph.
%
% Output :
%   roadmap - a structure the samples, the edges and edge lengths in the
%        roadmap graph

x = RandomSample();

% Array of random samples, each column corresponds to the coordinates
% of a point in configuration space.
samples = repmat(x(:), 1, nsamples);

% edges - an array with 2 rows each column has two integer entries
% (i, j) which encodes the fact that sample i and sample j are connected
% by an edge. For each 
edges = zeros(nsamples*k, 2);
edge_lengths = zeros(nsamples*k, 1);

% nedges - this integer keeps track of the number of edges we
% have in the graph so far
nedges = 0;

for i = 2:nsamples

    % Note that we are assuming that RandomSample returns a sample in freespace
    x = RandomSample();
    
    % samples(:, m) => m^th Column of Matrix "samples"
    % x(:) => Reshapes all elements of "samples" into a single column vector

    samples(:, i) = x(:);
    
    % Find the nearest neighbors
    
    % Here we assume that the Dist function can compute the
    % distance to multiple samples corresponding to the columns of
    % the second argument
    % at the end of this call the array distances will indicate the
    % distance between the new sample and each of the samples that has been
    % generated so far in the program.

    distances = Dist(x, samples(:,1:(i-1)));
    
    %%% YOUR CODE HERE
    %
    % Find the closest k samples, use the LocalPlanner function to see if
    % you can forge an edge to any of these samples and update the edges,
    % edge_lengths and nedges variables accordingly.
    %
    
    [closest, idx] = sort(distances, 'ascend');

    for j = 1 : min(k, i-1)

        % LocalPlanner => Function sketches a straight line between x and its closest neighbor and checks wether the said line intersects with any obstacles

        % If the line between x and its closest neighbor is in free space
        if LocalPlanner(x, samples(:,idx(j)))    

            % 1. Increment the nedges variable to reflect the fact that a new node has been added
            % 2. Add an entry to edges array indicating the indices of the two samples that have just been joined.
            % 3. Add a corresponding entry to the edge lengths array indicating the length of this new edge.            
            % 4. These edges and edge lengths constitute a graph which will be used later by the "ShortestPathDijkstra" routine to plan paths through the roadmap.

            nedges = nedges + 1;
            edges(nedges, :) = [i, idx(j)];
            edge_lengths(nedges) = closest(j);

        end

    end  

    fprintf (1, 'nsamples = %d, nedges = %d\n', i, nedges);
   
end

roadmap.samples = samples;
roadmap.edges = edges(1:nedges, :);
roadmap.edge_lengths = edge_lengths(1:nedges);