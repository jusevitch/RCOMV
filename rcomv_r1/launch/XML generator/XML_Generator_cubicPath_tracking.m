%
clc
clear all

% seed the random generator
rng('shuffle')

%% Variables

% name of the saved launch filef
file_list = {'test_cubic', 'test_world_cubic','test_ruin_world_cubic'};
% -------------------------- modify here ----------------------------
file_name = file_list{1};

file_name = 'eightTraj_circleFormation_8agents_smallWorld';
% -------------------------------------------------------------------


% world description
world_list = {'basic', 'terrain_copy', 'small_terrain', 'small_building',... 
                'small_collapsed_building','circular_track', 'eight_track'};
% -------------------------- modify here ----------------------------
world_idx = 7;
world_name = world_list{world_idx}';
paused = 'true';
% -------------------------------------------------------------------

% size of the formation
% k must be greater or equal to 5
% -------------------------- modify here ----------------------------
n = 8;    % number of agents
k = 7;    % number of neighbours for each agent 
% -------------------------------------------------------------------


% type of demo
%demo = 3;

% randomly generate agents
[F, norx, lead, mali] = SimpleLeaderWMSR_3D(n, k);

% reference center trajectory
lead_path_type = 'cubic';
% -------------------------- modify here ----------------------------
% intial/final locations
r = 10;
lead_qi_x = r;   lead_qi_y = -5; lead_qi_theta = pi/2;
lead_qf_x = -r; lead_qf_y = -5; lead_qf_theta = pi/2;
% total travel time (in secs)
T = 30;
% polynomial paramter 
poly_k = 4*r;
% loop flag
% if set true, switch the final/initial locations every T secs.
endless = true;
% -------------------------------------------------------------------
center_path = struct('qi_x',lead_qi_x, 'qi_y',lead_qi_y, 'qi_theta', lead_qi_theta,...
                     'qf_x',lead_qf_x, 'qf_y',lead_qf_y, 'qf_theta', lead_qf_theta,...
                     'T', T, 'poly_k', poly_k);


% Parameters of Input/Output Linearization Feedback Control
% -------------------------- modify here ----------------------------
% a fixed distance ahead of the vehicle 
% the feedback controller is based on the dynamics of this point
b = 0.05;
% control gains
k1 = 0.5;
k2 = 0.5;
% velocity limits
vmax = 2;
wmax = 4;
% -------------------------------------------------------------------

% choose the formation shape
% -------------------------- modify here ----------------------------
% list of formations:
% line_formation: a straight line with the middle agent as the center
% star_formation: a star shape with 5 branches of equal agents
% circle_formation: a circle with unliformly distributed agents
% square_formation: a square with unliformly distributed agents
% diamond_formation: a diamond with unliformly distributed agents
% V_formation: a V shape with unliformly distributed agents

initPose = @(idx,n,center) circle_formation(idx, n, center);
% -------------------------------------------------------------------


% model specifications of the ugv
ugv_name = 'r1_sim';
model_base_color = {'Red', 'Black', 'Blue'};  % order: malicious, normal, leaders

% gazebo logging enable
enable_logging = 'true';



%% ------------------------- Generating XML -------------------------------
docNode = com.mathworks.xml.XMLUtils.createDocument('launch');
launch = docNode.getDocumentElement;

% add arguments for gazebo
arg = argument(docNode, 0, 'ugv_name', ugv_name); launch.appendChild(arg);
arg = argument(docNode, 0, 'world_name', world_name); launch.appendChild(arg);
arg = argument(docNode, 0, 'enable_logging', enable_logging); launch.appendChild(arg);
arg = argument(docNode, 0, 'enable_ground_truth', 'true'); launch.appendChild(arg);
arg = argument(docNode, 0, 'log_file', '$(arg ugv_name)'); launch.appendChild(arg);

env = environment(docNode, 1, 'GAZEBO_MODEL_PATH', '${GAZEBO_MODEL_PATH}:$(find rotors_gazebo)/models'); launch.appendChild(env);
env = environment(docNode, 1, 'GAZEBO_RESOURCE_PATH', '${GAZEBO_RESOURCE_PATH}:$(find rotors_gazebo)/models'); launch.appendChild(env);

% include world launch file
world = docNode.createElement('include');
world.setAttribute('file', '$(find gazebo_ros)/launch/empty_world.launch');
arg1 = argument(docNode, 1, 'world_name', '$(find rcomv_uav)/worlds/$(arg world_name).world');
arg2 = argument(docNode, 1, 'paused', paused);
world.appendChild(arg1);
world.appendChild(arg2);
launch.appendChild(world);

% add wmsr node flag
arg = argument(docNode, 0, 'wmsr_node', 'true'); launch.appendChild(arg);
% add arguments for ugvs (read from variables defined in the above section)
arg = argument(docNode, 0, 'n', num2str(n)); launch.appendChild(arg);
arg = argument(docNode, 0, 'k', num2str(k)); launch.appendChild(arg);
arg = argument(docNode, 0, 'F', num2str(F)); launch.appendChild(arg);

% add reference path 
arg = argument(docNode, 0, 'path_type', lead_path_type); launch.appendChild(arg); 
arg = argument(docNode, 0, 'qi_x', num2str(lead_qi_x)); launch.appendChild(arg); 
launch.appendChild(docNode.createComment('reference cubic polynomial path'));
arg = argument(docNode, 0, 'qi_y', num2str(lead_qi_y)); launch.appendChild(arg);
arg = argument(docNode, 0, 'qi_theta', num2str(lead_qi_theta)); launch.appendChild(arg);
arg = argument(docNode, 0, 'qf_x', num2str(lead_qf_x)); launch.appendChild(arg);
arg = argument(docNode, 0, 'qf_y', num2str(lead_qf_y)); launch.appendChild(arg);
arg = argument(docNode, 0, 'qf_theta', num2str(lead_qf_theta)); launch.appendChild(arg);
arg = argument(docNode, 0, 'poly_k', num2str(poly_k)); launch.appendChild(arg);
arg = argument(docNode, 0, 'T', num2str(T)); launch.appendChild(arg);
if (endless)
    arg = argument(docNode, 0, 'endless', "true"); launch.appendChild(arg);
else
    arg = argument(docNode, 0, 'endless', "false"); launch.appendChild(arg);
end


% add pid gains
arg = argument(docNode, 0, 'b', num2str(b)); launch.appendChild(arg);
launch.appendChild(docNode.createComment('controller paramters'));
arg = argument(docNode, 0, 'k1', num2str(k1)); launch.appendChild(arg);
arg = argument(docNode, 0, 'k2', num2str(k2)); launch.appendChild(arg);
arg = argument(docNode, 0, 'vmax', num2str(vmax)); launch.appendChild(arg);
arg = argument(docNode, 0, 'wmax', num2str(wmax)); launch.appendChild(arg);


% add the upper level switch node that turns on WMSR nodes, when all robots
% are successfully spawned in Gazebo
switchNode = docNode.createElement('node');
switchNode.setAttribute('name', 'switch_node');
switchNode.setAttribute('pkg', 'rcomv_r1');
switchNode.setAttribute('type','switch');
switchNode.setAttribute('output','screen');
switchNode.setAttribute('launch-prefix','xterm -e');
launch.appendChild(switchNode);
launch.appendChild(docNode.createComment('upper level switch node'));

% include ugvs
for i = 1:n
   
   % element handle for ugvs
   ugv = docNode.createElement('include');
   ugv.setAttribute('file', '$(find rcomv_r1)/launch/ugv_with_InOutLin_control.launch');
   
   % add comment
   ugv.appendChild(docNode.createComment(['start of ugv',num2str(i)]));
   
   %  wmsr node flag
   arg = argument(docNode, 1, 'wmsr_node', '$(arg wmsr_node)'); ugv.appendChild(arg);

   % arg: formation size
   arg = argument(docNode, 1, 'n', '$(arg n)'); ugv.appendChild(arg);
   ugv.appendChild(docNode.createComment('size of the formation'));
   arg = argument(docNode, 1, 'k', '$(arg k)'); ugv.appendChild(arg);
   arg = argument(docNode, 1, 'F', '$(arg F)'); ugv.appendChild(arg);
  
   % arg: ugv model name
   arg = argument(docNode, 1, 'ugv_name', '$(arg ugv_name)'); ugv.appendChild(arg);
   % arg: enable logging in gazebo
   arg = argument(docNode, 1, 'enable_logging', '$(arg enable_logging)'); ugv.appendChild(arg);
   % arg: namespace
   arg = argument(docNode, 1, 'name_space', ['ugv',num2str(i)]); ugv.appendChild(arg);
   ugv.appendChild(docNode.createComment('group namespace'));
   
   % arg: idx, role
   arg = argument(docNode, 1, 'idx', num2str(i)); ugv.appendChild(arg);
   if (sum(mali == i))
       role = 1;
   elseif (sum(lead == i))
       role = 3;
   else
       role = 2;
   end
   ugv.appendChild(docNode.createComment('idex and type of the agent'));
   arg = argument(docNode, 1, 'role', num2str(role)); ugv.appendChild(arg);
   
   % arg: model base color
   % arg = argument(docNode, 1, 'color', model_base_color{role}); ugv.appendChild(arg);
   
   % arg: initial pose
   [x,y,z,Ri,alphai] = initPose(i, n, center_path); 
   arg = argument(docNode, 1, 'x', num2str(x)); ugv.appendChild(arg);
   ugv.appendChild(docNode.createComment('initial_pose'));
   arg = argument(docNode, 1, 'y', num2str(y)); ugv.appendChild(arg);
   arg = argument(docNode, 1, 'z', num2str(z)); ugv.appendChild(arg);
   arg = argument(docNode, 1, 'yaw', num2str(lead_qi_theta)); ugv.appendChild(arg);
   arg = argument(docNode, 1, 'Ri', num2str(Ri)); ugv.appendChild(arg);
   arg = argument(docNode, 1, 'alphai', num2str(alphai)); ugv.appendChild(arg);
   
   % arg: leader's inform state: reference path parameters
   % arg: other agents' inform state: reference path + random noise
    if (role == 3)
        arg = argument(docNode, 1, 'path_type', '$(arg path_type)'); ugv.appendChild(arg); 
        arg = argument(docNode, 1, 'qi_x', '$(arg qi_x)'); ugv.appendChild(arg);
        ugv.appendChild(docNode.createComment('reference cubic polynomial path'));
        arg = argument(docNode, 1, 'qi_y', '$(arg qi_y)'); ugv.appendChild(arg);
        arg = argument(docNode, 1, 'qi_theta', '$(arg qi_theta)'); ugv.appendChild(arg);
        arg = argument(docNode, 1, 'qf_x', '$(arg qf_x)'); ugv.appendChild(arg);
        arg = argument(docNode, 1, 'qf_y', '$(arg qf_y)'); ugv.appendChild(arg);
        arg = argument(docNode, 1, 'qf_theta', '$(arg qf_theta)'); ugv.appendChild(arg);
        arg = argument(docNode, 1, 'poly_k', '$(arg poly_k)'); ugv.appendChild(arg);
        arg = argument(docNode, 1, 'T', '$(arg T)'); ugv.appendChild(arg);
        arg = argument(docNode, 1, 'endless', '$(arg endless)'); ugv.appendChild(arg);
    else
        arg = argument(docNode, 1, 'path_type', '$(arg path_type)'); ugv.appendChild(arg); 
        arg = argument(docNode, 1, 'qi_x', num2str(lead_qi_x+0.1*randn)); ugv.appendChild(arg);
        ugv.appendChild(docNode.createComment('reference cubic polynomial path'));
        arg = argument(docNode, 1, 'qi_y', num2str(lead_qi_y+0.1*randn)); ugv.appendChild(arg);
        arg = argument(docNode, 1, 'qi_theta', num2str(lead_qi_theta+randn*0.05)); ugv.appendChild(arg);
        arg = argument(docNode, 1, 'qf_x', num2str(lead_qf_x+0.1*randn)); ugv.appendChild(arg);
        arg = argument(docNode, 1, 'qf_y', num2str(lead_qf_y+0.1*randn)); ugv.appendChild(arg);
        arg = argument(docNode, 1, 'qf_theta', num2str(lead_qf_theta+randn*0.05)); ugv.appendChild(arg);
        arg = argument(docNode, 1, 'poly_k', num2str(poly_k+0*randn)); ugv.appendChild(arg);
        arg = argument(docNode, 1, 'T', num2str(T+0*randn)); ugv.appendChild(arg);
        arg = argument(docNode, 1, 'endless', '$(arg endless)'); ugv.appendChild(arg);
    end
   
    % add controller parameters
    arg = argument(docNode, 1, 'b', '$(arg b)'); ugv.appendChild(arg);
    ugv.appendChild(docNode.createComment('controller paramters'));
    arg = argument(docNode, 1, 'k1', '$(arg k1)'); ugv.appendChild(arg);
    arg = argument(docNode, 1, 'k2', '$(arg k2)'); ugv.appendChild(arg);
    arg = argument(docNode, 1, 'vmax', '$(arg vmax)'); ugv.appendChild(arg);
    arg = argument(docNode, 1, 'wmax', '$(arg wmax)'); ugv.appendChild(arg);
   
   % append the ugv  include element the the launch file
   launch.appendChild(ugv);
   
   % add comment
   launch.appendChild(docNode.createComment(['end of ugv',num2str(i)]));
end

% store the XML file
xmlFileName = [file_name,'.launch'];
xmlwrite(xmlFileName,docNode);
type(xmlFileName);



%% some helper functions
% create attribute
function attr = Attribute(dn, name, value)
    attr = dn.createAttribute(name);
    attr.setNodeValue(value);
end

% create argugment
function arg = argument(dn, type, name, default)
    % type=0: default attribute
    % type=1: value attribute
    arg = dn.createElement('arg');
    name_attr = Attribute(dn, 'name', name);
    if type == 0
        default_attr = Attribute(dn, 'default', default);
    end
    if type == 1
        default_attr = Attribute(dn, 'value', default);
    end
    
    arg.setAttributeNode(default_attr);
    arg.setAttributeNode(name_attr); 
end


% create environment  argument
function env = environment(dn, type, name, default)
    % type=0: default attribute
    % type=1: value attribute
    env = dn.createElement('env');
    name_attr = Attribute(dn, 'name', name);
    if type == 0
        default_attr = Attribute(dn, 'default', default);
    end
    if type == 1
        default_attr = Attribute(dn, 'value', default);
    end
    
    env.setAttributeNode(default_attr);
    env.setAttributeNode(name_attr); 
end


%
%%
% helper functions: create formations
% initialize the pose for straight line formation
function [x,y,z,R,theta] =line_formation(idx, n, center) 
    c_idx = ceil(n/2);
    
    % center pose
    cx = center.qi_x;
    cy = center.qi_y;
    ctheta = center.qi_theta;
    cz = 0.1;
    % offset of the agent (in the body-fixed frame)
    R = c_idx - idx;
    theta = 0;
    
    % intial agent pose in the global frame
    x = cx + R*cos(ctheta + theta);
    y = cy + R*sin(ctheta + theta);
    z = cz;
end

% initialize the pose for circle formation
function [x,y,z,R,theta] =circle_formation(idx, n, center) 
    % center pose
    cx = center.qi_x;
    cy = center.qi_y;
    ctheta = center.qi_theta;
    cz = 0.1;
    
    % offset of the agent (in the body-fixed frame)
    R = 2;
    theta = (idx-1)/n * (2*pi);
    
    % intitial postion in the global frame
    x = cx + R*cos(ctheta + theta);
    y = cy + R*sin(ctheta + theta);
    z = cz; 
end

% initialize the pose for square formation
function [x,y,z,R,theta] = square_formation(idx, n, center) 
    % center pose
    cx = center.qi_x;
    cy = center.qi_y;
    ctheta = center.qi_theta;
    cz = 0.1;
    
    % square width
    l = n/4 * 1;
    
    % offset of the agent (in the body-fixed frame)
    theta = (idx-1)/n * (2*pi) + pi/4;
    if (idx <= n/4)
       R = l/2 / sin(theta); 
    elseif (idx <= n/2)
       R = -l/2 / cos(theta);
    elseif (idx <= n*3/4)
       R = -l/2 / sin(theta);
    else
       R =l/2 / cos(theta);
    end
       
    % intitial postion in the global frame
    x = cx + R*cos(ctheta + theta);
    y = cy + R*sin(ctheta + theta);
    z = cz; 
end

% initialize the pose for diamond formation
function [x,y,z,R,theta] = diamond_formation(idx, n, center) 
    % center pose
    cx = center.qi_x;
    cy = center.qi_y;
    ctheta = center.qi_theta;
    cz = 0.1;
    
    % distance
    d = 1;
    num = n/4;
    % skew angle
    s = pi/10;
    
    % offset of the agent (in the body-fixed frame)
    %theta = (idx-1)/n * (2*pi) + pi/4
    if (idx <= n/4)
       x = (num-1)*(d*cos(s)) - (idx-1)*(d*cos(s)); 
       y = (idx-1)*(d*sin(s));
       R = sqrt(x^2+y^2);
       theta = atan2(y,x);
    elseif (idx <= n/2)
       x =  - (idx-1-n/4)*(d*cos(s)); 
       y = (num-1)*(d*sin(s)) - (idx-1-n/4)*(d*sin(s));
       R = sqrt(x^2+y^2);
       theta = atan2(y,x);
    elseif (idx <= n*3/4)
       x = -(num-1)*(d*cos(s)) + (idx-1-n/2)*(d*cos(s)); 
       y =  - (idx-1-n/2)*(d*sin(s));
       R = sqrt(x^2+y^2); 
       theta = atan2(y,x);
    else
       x = (idx-1-n*3/4)*(d*cos(s)); 
       y = -(num-1)*(d*sin(s)) + (idx-1-n*3/4)*(d*sin(s));
       R = sqrt(x^2+y^2); 
       theta = atan2(y,x);
    end
       
    % intitial postion in the global frame
    x = cx + R*cos(ctheta + theta);
    y = cy + R*sin(ctheta + theta);
    z = cz; 
end

% initialize the pose for v shape formation
function [x,y,z,R,theta] = v_formation(idx, n, center) 
    % center pose
    cx = center.qi_x;
    cy = center.qi_y;
    ctheta = center.qi_theta;
    cz = 0.1;
    
    % distance
    d = 1;
    dk = ceil(n/4);
    % skew angle
    s = pi/10;
    
    % center idx
    c_idx = ceil(n/2);
    
    % offset of the agent (in the body-fixed frame)
    x = cx + dk - d*abs(idx-c_idx) * cos(s);
    y = cy + d*(idx-c_idx) * sin(s);
    theta = atan2(y,x);
    R = sqrt(x^2+y^2);
    
    % intitial postion in the global frame
    x = cx + R*cos(ctheta + theta);
    y = cy + R*sin(ctheta + theta);
    z = cz; 
    
end

% initialize the pose for star shape formation
function [x,y,z,R,theta] = star_formation(idx, n, center)
    g_num = 5;
    g_size = n / g_num;
    sigma = 0.5;
    
    cx = center.qi_x;
    cy = center.qi_y;
    ctheta = center.qi_theta;
    cz = 0.1;
    
    % the group of the node idx, and the index inside that group
    g = floor((idx-1) / g_size);
    idx_g = mod(idx-1, g_size);
    
    %
    R = idx_g + 1;
    
    switch g
        case 0
            theta = 0;
            x = cx + R * cos(theta+ctheta);
            y = cy + R * sin(theta+ctheta);
            z = cz;
        case 1
            theta = 2/5*pi;
            x = cx + R * cos(theta+ctheta);
            y = cy + R * sin(theta+ctheta);
            z = cz;
        case 2
            theta = 4/5*pi;
            x = cx + R * cos(theta+ctheta);
            y = cy + R * sin(theta+ctheta);
            z = cz;
        case 3
            theta = 6/5*pi;
            x = cx + R * cos(theta+ctheta);
            y = cy + R * sin(theta+ctheta);
            z = cz;
        case 4
            theta = 8/5*pi;
            x = cx + R * cos(theta+ctheta);
            y = cy + R * sin(theta+ctheta);
            z = cz;
    end
            
end