%
clc
clear all

% seed the random generator
rng('shuffle')

%% ------------------------ Set Parameters -------------------------------
% name of the saved launch file
file_list = {'test', 'test_world','test_ruin_world'};
% -------------------------- modify here ----------------------------
file_name = file_list{1};
file_name='four test/test3';
% -------------------------------------------------------------------


% world description
world_list = {'basic', 'terrain_copy', 'small_terrain', 'small_building',... 
                'small_collapsed_building'};
% -------------------------- modify here ----------------------------
world_idx = 5;
world_name = world_list{world_idx}';
paused = 'true';
% -------------------------------------------------------------------


% size of the formation
% k must be greater or equal to 5
% -------------------------- modify here ----------------------------
n = 15;    % number of agents
k = 7;    % number of neighbours for each agent 
% -------------------------------------------------------------------

% type of demo
% demo = 1: 1D motion in y direction
% demo = 2: 1D motion in z direction
% demo = 3: 3D motion
% -------------------------- modify here ----------------------------
demo = 3;
% -------------------------------------------------------------------

% randomly generate agents
[F, norx, lead, mali] = SimpleLeaderWMSR_3D(n, k);


% leaders' static inform states (formation center location)
% -------------------------- modify here ----------------------------
lead_x = 15;
lead_y = -15;
lead_z = 15;
center = struct('x',lead_x, 'y',lead_y, 'z', lead_z);
% -------------------------------------------------------------------



% model specification of the uav 
% -------------------------- modify here ----------------------------
mav_list = {'hummingbird', 'pelican', 'firefly'};
mav_name= mav_list{3};
model_base_color = {'Red', 'Black', 'Blue'};  % order: malicious, normal, leaders
% -------------------------------------------------------------------

% gazebo logging enable
enable_logging = 'true';



%% Generating XML
docNode = com.mathworks.xml.XMLUtils.createDocument('launch');
launch = docNode.getDocumentElement;

% add arguments for gazebo
arg = argument(docNode, 0, 'mav_name', mav_name); launch.appendChild(arg);
arg = argument(docNode, 0, 'world_name', world_name); launch.appendChild(arg);
arg = argument(docNode, 0, 'enable_logging', enable_logging); launch.appendChild(arg);
arg = argument(docNode, 0, 'enable_ground_truth', 'true'); launch.appendChild(arg);
arg = argument(docNode, 0, 'log_file', '$(arg mav_name)'); launch.appendChild(arg);

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

% add arguments for uavs (read from variables defined in the above section)
arg = argument(docNode, 0, 'n', num2str(n)); launch.appendChild(arg);
arg = argument(docNode, 0, 'k', num2str(k)); launch.appendChild(arg);
arg = argument(docNode, 0, 'F', num2str(F)); launch.appendChild(arg);

arg = argument(docNode, 0, 'demo', num2str(demo)); launch.appendChild(arg);

arg = argument(docNode, 0, 'lead_x', num2str(lead_x)); launch.appendChild(arg);
arg = argument(docNode, 0, 'lead_y', num2str(lead_y)); launch.appendChild(arg);
arg = argument(docNode, 0, 'lead_z', num2str(lead_z)); launch.appendChild(arg);

% add the upper level switch node that turns on WMSR nodes, when all robots
% are successfully spawned in Gazebo
switchNode = docNode.createElement('node');
switchNode.setAttribute('name', 'switch_node');
switchNode.setAttribute('pkg', 'rcomv_uav');
switchNode.setAttribute('type','switch');
switchNode.setAttribute('output','screen');
switchNode.setAttribute('launch-prefix','xterm -e');
%arg = argument(docNode, 1, 'n', '$(arg n)');
%switchNode.appendChild(arg);
launch.appendChild(switchNode);
launch.appendChild(docNode.createComment('upper level switch node'));

% include uavs
for i = 1:n
   
   % element handle for uavs
   uav = docNode.createElement('include');
   uav.setAttribute('file', '$(find rcomv_uav)/launch/uav_with_control.launch');
   
   % add comment
   uav.appendChild(docNode.createComment(['start of uav',num2str(i)]));
   
   % arg: formation size
   arg = argument(docNode, 1, 'n', '$(arg n)'); uav.appendChild(arg);
   arg = argument(docNode, 1, 'k', '$(arg k)'); uav.appendChild(arg);
   arg = argument(docNode, 1, 'F', '$(arg F)'); uav.appendChild(arg);
  
   % arg: uav model name
   arg = argument(docNode, 1, 'mav_name', '$(arg mav_name)'); uav.appendChild(arg);
   % arg: enable logging in gazebo
   arg = argument(docNode, 1, 'enable_logging', '$(arg enable_logging)'); uav.appendChild(arg);
   % arg: namespace
   arg = argument(docNode, 1, 'name_space', ['uav',num2str(i)]); uav.appendChild(arg);
   
   % arg: idx, role
   arg = argument(docNode, 1, 'idx', num2str(i)); uav.appendChild(arg);
   if (sum(mali == i))
       role = 1;
   elseif (sum(lead == i))
       role = 3;
   else
       role = 2;
   end
   arg = argument(docNode, 1, 'role', num2str(role)); uav.appendChild(arg);
   
   % arg: model base color
   arg = argument(docNode, 1, 'color', model_base_color{role}); uav.appendChild(arg);
   
   % arg: initial pose
   [x,y,z] = initPose(i, n, center); 
   arg = argument(docNode, 1, 'x', num2str(x)); uav.appendChild(arg);
   arg = argument(docNode, 1, 'y', num2str(y)); uav.appendChild(arg);
   arg = argument(docNode, 1, 'z', num2str(z)); uav.appendChild(arg);
   
   % arg: leader's inform state (the consensus state)
   arg = argument(docNode, 1, 'lead_x', '$(arg lead_x)'); uav.appendChild(arg);
   arg = argument(docNode, 1, 'lead_y', '$(arg lead_y)'); uav.appendChild(arg);
   arg = argument(docNode, 1, 'lead_z', '$(arg lead_z)'); uav.appendChild(arg);
   
  
   % arg: demo
   arg = argument(docNode, 1, 'demo', '$(arg demo)'); uav.appendChild(arg);
   
   % append the uav  include element the the launch file
   launch.appendChild(uav);
   
   % add comment
   launch.appendChild(docNode.createComment(['end of uav',num2str(i)]));
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

% initialize the pose
function [x,y,z] = initPose(idx, n, center)
    g_num = 5;
    g_size = n / g_num;
    sigma = 0.5;
    
    cx = center.x;
    cy = center.y;
    cz = center.z;
    
    % the group of the node idx, and the index inside that group
    g = floor((idx-1) / g_size);
    idx_g = mod(idx-1, g_size);
    
    %
    switch g
        case 0
            x = cx + 5 + idx_g;
            y = cy;
            z = cz + rand * sigma;
        case 1
            x = cx + 5;
            y = cy + 3 + idx_g;
            z = cz + rand * sigma;
        case 2
            x = cx - 5;
            y = cy + 3 + idx_g;
            z = cz + rand * sigma;
        case 3
            x = cx - 5;
            y = cy - 3 - idx_g;
            z = cz + rand * sigma;
        case 4
            x = cx + 5;
            y = cy - 3 - idx_g;
            z = cz + rand * sigma;
    end
            
end

