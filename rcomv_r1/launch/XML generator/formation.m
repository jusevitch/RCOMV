clear all
clc
close all

%%
n = 15;
k = 7;

[c,norx, leaders, malicious] = SimpleLeaderWMSR_3D(n, k);

%%
radius = [5 10 15];
levels = [floor(n/3) floor(n/3) n-2*floor(n/3)];
cumsum_levels = cumsum([0 levels]);

theta = zeros(1,n);
vector_x = zeros(1,n);
vector_y = zeros(1,n);
for i=1:length(radius)
    theta(1, cumsum_levels(i)+1:cumsum_levels(i+1)) = (1:levels(i)) * (2*pi/levels(i));
    vector_x(1, cumsum_levels(i)+1:cumsum_levels(i+1)) = radius(i) * cos((1:levels(i)) * (2*pi/levels(i)));
    vector_y(1, cumsum_levels(i)+1:cumsum_levels(i+1)) = radius(i) * sin((1:levels(i)) * (2*pi/levels(i)));

end


displacement.x = c.x + vector_x;
displacement.y = c.y + vector_y;
displacement.z = c.z;

%% plot final formation
figure(3)
plot3(displacement.x(end,norx), displacement.y(end,norx), displacement.z(end,norx), 'x'); hold on;
plot3(displacement.x(end,leaders), displacement.y(end,leaders),  displacement.z(end,leaders), 'k+');
plot3(displacement.x(end,malicious), displacement.y(end,malicious), displacement.z(end,malicious), 'ro'); hold off;
xlim([-50, 50]);
ylim([-50, 50]);
legend('normal agents', 'lead agents', 'malicious agents');

%% Make the movie object

mov = struct('cdata', [], 'colormap', []);
%What is the Name of your AVI File?
Title='W-MSR_formation';                  
vidObj = VideoWriter(Title);
%What is the Frame Rate?
FR = 10;
vidObj.FrameRate=FR;
open(vidObj)

% Draw the initial positions to the movie
figure(4)
plot3(displacement.x(1,norx), displacement.y(1,norx), displacement.z(1,norx), 'bx',...
        displacement.x(1,malicious), displacement.y(1,malicious), displacement.z(1,malicious), 'ro',...
        displacement.x(1,leaders), displacement.y(1,leaders), displacement.z(1,leaders), 'k+');
xlim([-50, 50]);
ylim([-50, 50]);
legend('normal agents', 'malicious agents', 'lead agents');
drawnow update
mov= getframe(gcf);

% Make the initial positions play in the movie for 2 seconds
for k=1:1:2*FR
    writeVideo(vidObj,mov)
end

steps = 100;
for j=1:1:steps % Make sure this matches the timesteps for the main W-MSR loop
%Plot, set the axis limits you want to maintain and then drawnow update.    
    plot3(displacement.x(j,norx), displacement.y(j,norx), displacement.z(j,norx), 'bx',...
        displacement.x(j,malicious), displacement.y(j,malicious), displacement.z(j,malicious), 'ro',...
        displacement.x(j,leaders), displacement.y(j,leaders), displacement.z(j,leaders), 'k+');
    xlim([-50, 50]);
    ylim([-50, 50]);
    legend('normal agents', 'malicious agents', 'lead agents');
    drawnow update
    mov= getframe(gcf);
	writeVideo(vidObj,mov)
	clear mov
	mov = struct('cdata', [], 'colormap', []);
end
%Close the file when you're done
close(vidObj)
