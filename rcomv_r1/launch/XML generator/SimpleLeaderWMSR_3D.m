function [F, norx, leaders, malicious] = SimpleLeaderWMSR_3D(n, k, graph) %#codegen
%-------------------------------------------------------------------------%
% Simple Leader W-MSR Consensus
% (LeBlanc et al., 2013: Resilient Asymptotic Consensus in Robust Networks)
% 
% This algorithm implements the Weighted-Mean-Subsequent-Reduced algorithm
% in LeBlanc's 2013 paper listed above. Agents achieve positional consensus
% on a 2D plane. Agents have point kinematics and do not avoid collision.
%
% Required functions: Dmatrix.m, kCirculant.m
%
% Outputs: Graph of difference between maximum and minimum normal nodes
% versus step number, graph of all node states vs. timestep.
%-------------------------------------------------------------------------%



% default number of agents
if nargin < 2
    n = 30; % Allows up to a (5,5)-robust graph if the graph is complete
    k = 15; % Used for kCirculant graphs
end

% Nodes are indexed by state as following: [1 2 3 4....n]'

% Specify the type of graph
if nargin < 3
    graph = 'kCirculantDir';
end

% Assumed upper bound to number of adversaries.
if strcmp(graph, 'kCirculantDir')
    % Circulant digraphs:
    if mod(k,2) == 0
        F = floor((k+2)/4)-1;
    else
        F = floor((k+3)/4)-1;
    end
elseif strcmp(graph, 'kCirculantUndir')
    if mod(k,2) == 0
        F = floor((k+1)/2)-1;
    else
        F = floor((k+2)/2)-1;
    end
end


% define k Circulant type
DIR = 1;
UNDIR = 2;

if strcmp(graph,'arbcomplete') || strcmp(graph,'path') || strcmp(graph,'spantree')
    D = Dmatrix(n,[],graph);
    L = D*D';
elseif strcmp(graph,'kCirculantDir')
    L = kCirculant(n,k, DIR);
elseif strcmp(graph,'kCirculantUndir')
    L = kCirculant(n,k, UNDIR);
end

% Generate random positions
% seed the random number generator
if coder.target('MATLAB')
  % Executing in MATLAB, call function absval
  rng('shuffle')
else
    coder.cinclude('"time.h"');
    sd=0;
    sd=coder.ceval('time',[]);
    rng(sd,'twister');
end


x = 50*rand(1,n)-25;
y = 20*rand(1,n)-10; % USE FOR 2D
%y = zeros(1,n); % ONLY USE FOR 1D
z = 10*rand(1,n)-5;

% Set of malicious nodes, indexed by their order in the state list.
% randomly malicious agents
rand_list = randperm(n , F+1);
malicious = rand_list(1: F);
malicious = sort(malicious);

% List of normal nodes
norx = zeros(1, n - length(malicious));
idx = 1;
for i = 1:n
   if (sum(malicious == i) == 0)
       norx(idx) = i;
       idx = idx+1;
   end
end

% List of Leaders
% set 2*F+1 consecutive agents to be leaders

st_leader = randi(n);
leaders = (1: (2*F+1)) + st_leader;
leaders = mod(leaders, n); 
leaders(leaders==0) = n;

x(leaders) = 30;
y(leaders) = -30;
z(leaders) = 15;

% Number of steps algorithm goes through
steps = 100; 

xFinal = zeros(steps+2, n);
yFinal = zeros(steps+2, n);
zFinal = zeros(steps+2, n);
xFinal(1,:) = x;
yFinal(1,:) = y;
zFinal(1,:) = z;


% Keep track of the maximum and minimum x state values of the good vectors.
xMax = zeros(steps+2, 1);
xMin = zeros(steps+2, 1);
xMax(1) = max(xFinal(1, norx));
xMin(1) = min(xFinal(1, norx));

yMax = zeros(steps+2, 1);
yMin = zeros(steps+2, 1);
yMax(1) = max(yFinal(1, norx));
yMin(1) = min(yFinal(1, norx));

zMax = zeros(steps+2, 1);
zMin = zeros(steps+2, 1);
zMax(1) = max(zFinal(1, norx));
zMin(1) = min(zFinal(1, norx));



% W-MSR Algorithm

% Number of steps algorithm goes through
steps = 100; 


% Initialize states
x1 = zeros(1,n);
y1 = zeros(1,n);
z1 = zeros(1,n);

for t=0:1:steps %timespan and steps
    for i=1:1:n % Iterate through all nodes for each time step
        if ismember(i,malicious) % Check to see if the node is malicious
            % Update state information arbitrarily
            
            % ORIGINAL
            x1(i) = x(i) + (1/20)*cos(t/20)+1/15*sin(t/(i+5))+10*cos(i*t+4);
            y1(i) = y(i) + (1/20)*cos(t/20)+1/15*sin(t/(i+5))+10*cos(i*t+4);
            z1(i) = max(0, z(i) - 1);
            
            
            
        elseif ismember(i,leaders) && ~ismember(i,malicious)
            
            % Stay put
            x1(i) = x(i);
            y1(i) = y(i);
            z1(i) = z(i);           
            
        else % If the node isn't malicious...
            
           % Reset the lists
           listx = zeros(1,0); % List of x values actually used for update protocol
           listy = zeros(1,0); % List of y values actually used for update protocol
           listz = zeros(1,0); % List of z values actually used for update protocol
           upperx = zeros(1,0); % X values higher than node i's x value
           uppery = zeros(1,0); % Y values higher than node i's y value
           upperz = zeros(1,0); % Z values higher than node i's z value
           lowerx = zeros(1,0); % X values lower than node i's x value
           lowery = zeros(1,0); % Y values lower than node i's y value
           lowerz = zeros(1,0); % Z values lower than node i's z value
                
            for j=1:1:n
                if i~=j % Prevents algorithm from looking at the diagonal of the Laplacian
                    if L(i,j) ~= 0 % Identifies whether node j is in node i's neighboring set
                        % Sort x values by whether they're greater than, equal to, or less
                        % than node i's value
                        if x(j)==x(i)
                            listx = [listx x(j)];
                        elseif x(j) > x(i)
                            upperx = [upperx x(j)];
                        else
                            lowerx = [lowerx x(j)];
                        end
                        
                        % Sort y values
                        if y(j)==y(i)
                            listy = [listy y(j)];
                        elseif y(j) > y(i)
                            uppery = [uppery y(j)];
                        else
                            lowery = [lowery y(j)];
                        end
                        
                         % Sort z values
                        if z(j)==z(i)
                            listz = [listz z(j)];
                        elseif z(j) > z(i)
                            upperz = [upperz z(j)];
                        else
                            lowerz = [lowerz z(j)];
                        end
                    end
                end
            end  
            % Sort upper and lower lists.
            upperx = sort(upperx, 'descend'); % Descending order so the highest entries are first
            uppery = sort(uppery, 'descend');
            upperz = sort(upperz, 'descend');
            lowerx = sort(lowerx); % Ascending order, so lowest entries are first.
            lowery = sort(lowery);
            lowerz = sort(lowerz);
            
            % If more than F entries present in each list,
            % remove the first F entries from each list. Otherwise,
            % remove all entries.
            if length(upperx) > F
                upperx = upperx(F+1:end);
            else
                upperx = [];
            end
                       
            if length(uppery) > F
                uppery = uppery(F+1:end);
            else
                uppery = [];
            end
            
            if length(upperz) > F
                upperz = upperz(F+1:end);
            else
                upperz = [];
            end
                        
            if length(lowerx) > F
                lowerx = lowerx(F+1:end);
            else
                lowerx = [];
            end
                        
            if length(lowery) > F
                lowery = lowery(F+1:end);
            else
                lowery = [];
            end
            
             if length(lowerz) > F
                lowerz = lowerz(F+1:end);
            else
                lowerz = [];
            end
            
                        
            % Add remaining values to listx and listy, along
            % with the value of node i itself
            listx_total = [listx upperx lowerx x(i)];
            listy_total = [listy uppery lowery y(i)];
            listz_total = [listz upperz lowerz z(i)];
                        
            % Calculate weights according to footnote 3 of
            % LeBlanc 2013
            wx = 1/length(listx_total);
            wy = 1/length(listy_total);
            wz = 1/length(listz_total);
            
            % Create x[t+1] and y[t+1] states for agents. Note
            % that once these have been calculated for all
            % agents, THEN these values will be put into x[t]
            % and y[t]. Otherwise, changes to the states of
            % early nodes will propagate too soon into the
            % states of the other nodes.
            x1(i) = sum(wx*listx_total); % THIS ASSUMES WEIGHTS ARE EQUAL
            y1(i) = sum(wy*listy_total); % USE FOR 2D
            %y1(i) = 0; % ONLY USE FOR 1D CASE
            z1(i) = sum(wz*listz_total);
                        
        end 
    end

% Update state values
x = x1;
y = y1;
z = z1;

xFinal(t+2,:) = x1;
yFinal(t+2,:) = y1;
zFinal(t+2,:) = z1;

xMax(t+2,:) = max(x1(norx));
xMin(t+2,:) = min(x1(norx));

yMax(t+2,:) = max(y1(norx));
yMin(t+2,:) = min(y1(norx));

zMax(t+2,:) = max(z1(norx));
zMin(t+2,:) = min(z1(norx));
  
end

% output data
centers.x = xFinal;
centers.y = yFinal;
centers.z = zFinal;

% % store data to  txt files
% fid=fopen('SimpleLeaderWMSR_xMaxMin_3D.txt','w');
% fprintf(fid, '%8s %8s \n', 'xMin', 'xMax', 'yMin', 'yMax', 'zMin', 'zMax');
% for i = 1:steps+2
%     fprintf(fid, '%6.4f %6.4f \r\n', xMin(i,1), xMax(i,1), yMin(i,1), yMax(i,1), zMin(i,1), zMax(i,1));
% end
% fclose(fid);

% fid=fopen('SimpleLeaderWMSR_Final_3D.txt','w');
% for i = 1:steps+2
%     for j = 1:n
%         fprintf(fid, '%10.4f', xFinal(i,j));
%     end
%     for j = 1:n
%         fprintf(fid, '%10.4f', yFinal(i,j));
%     end
%     for j = 1:n
%         fprintf(fid, '%10.4f', zFinal(i,j));
%     end
%     fprintf(fid, '\r\n');
% end
% fclose(fid);


% % Plot the difference between xMax and xMin
 stepsVec = [0:steps+1];
% figure(1)
% subplot(3,1,1);
% diff = xMax - xMin;
% plot(stepsVec, diff);
% title('Difference between xMax and xMin')
% xlabel('Time Step t')
% ylabel('xMax - xMin')
% 
% subplot(3,1,2);
% diff = yMax - yMin;
% plot(stepsVec, diff);
% title('Difference between yMax and yMin')
% xlabel('Time Step t')
% ylabel('yMax - yMin')
% 
% subplot(3,1,3);
% diff = zMax - zMin;
% plot(stepsVec, diff);
% title('Difference between zMax and zMin')
% xlabel('Time Step t')
% ylabel('zMax - zMin')
% 
figure(2)
subplot(3,1,1);
plot(stepsVec,xFinal(:,norx), stepsVec, xFinal(:,malicious), '--r')
xlim([0 steps])
ylim([-40 40])
ylabel('x')
xlabel('Time Step t')

subplot(3,1,2);
plot(stepsVec,yFinal(:,norx), stepsVec, yFinal(:,malicious), '--r')
xlim([0 steps])
ylim([-40 40])
ylabel('y')
xlabel('Time Step t')

subplot(3,1,3);
plot(stepsVec,zFinal(:,norx), stepsVec, zFinal(:,malicious), '--r')
xlim([0 steps])
ylim([-30 30])
ylabel('z')
xlabel('Time Step t')

end