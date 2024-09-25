clc
close all
clear all
global q1 q2 q3 p1 p2 p3 ep k1 k2 k3 q1d q2d q3d g Kv q p P11 P12 P21 P22 MdInv
syms q1 q2 q3 p1 p2 p3 ep k1 k2 k3 q1d q2d q3d g Kv q p P11 P12 P21 P22 MdInv

%Definition of model parameters
q = [q1; q2; q3];
p = [p1; p2; p3];

%% Simulation parameters
ep = 1;

scenario = input("Enter a number between 0 and 4 to decide the setting scenario: ");

switch scenario
    % Takeoff simulation: ground -> sky
    case 0
        q1d = 4
        q2d = 22
        x0 = [0; 0; 0; 0.1; 0.1; 0.1];
        Kv = [ 50      0; 
              -50      50]  
    
   % Landing simulation: sky -> ground
    case 1 %only for q3=0
        q1d = 0
        q2d = 0
        x0 = [4; 22; 0; 1; 0.1; 0.1];
        Kv = [200   0; 
              0    50]
    % Challenging simulation Vertical
    case 2 %also for q3=pi and q3=pi/2
        q1d = 32
        q2d = 6
        x0 = [0; 0; 0; 0.1; 0.1; 0.1];
        Kv = [ 90      10; 
              -90      100]

    % Challenging simulation Horizontal
    case 3 %also for q3=pi/2
        q1d = 0
        q2d = 6
        x0 = [32; 0; 0; 0.1; 0.1; 0.1];
        Kv = [ 50      50; 
              -50      90]
    
    case 4
        q1d = 0
        q2d = 6
        x0 = [32; 0; pi; 0.1; 0.1; 0.1];
        Kv = [ 50      100; 
              -50      90]
    otherwise
        error("The number of scenario is invalid!")
end


q3_0 = input("Enter the initial roll angle: ")
x0(3) = q3_0;
x0

% Damping Injection Matrix 
% Kv = [ 50      50; 
%        -50      0];



% Parameters for Md virtual Coupling Matrix
k1 = 1;
k2 = 0.75;
k3 = 5.1;

% Gravity acceleration
g = 9.8;

% Parameters for Potential Energy shaping
P11 = 0.1;
P12 = 0.1;
P21 = 0.1;
P22 = 2;

% Initial conditions
% x0 = [32; 0; 0; 0.1; 0.1; 0.1];
% 
% % References
% q1d = 0;
% q2d = 6;

% Simulation time span
tspan=0:0.1:40;

%% System Simulation
[t, x]=ode45(@VTOL_dynamics,tspan,x0);

% Control Effort Vectors' extraction (v1, v2)
[~,U1,~] = cellfun(@(t,x) VTOL_dynamics(t,x.'), num2cell(t), num2cell(x,2),'uni',0);
[~,~,U2] = cellfun(@(t,x) VTOL_dynamics(t,x.'), num2cell(t), num2cell(x,2),'uni',0);


%% Video animation

% Create VTOL drawing
model = createpde;

% VTOL Vertices position
%xr1 = [1 1  7   5   10  15  13  20  20  13  10  7]
%yr1 = [7 10 10  12  11  12  10  10  7    7   2  7]
R1 = [2,12, 1,1,7,5,10,15,13,20,20,13,10,7, 7,10,10,12,11,12,10,10,7,7,2,7]';
R2 = [2, 5, 7, 8, 12,13,10, 8,9.5,9.5,8,7]';
E1 = [4,0,8,1.5,3,0]';
E2 = [4,21,8,1.5,3,0]';

E1 = [E1;zeros(length(R1) - length(E1),1)];
E2 = [E2;zeros(length(R1) - length(E2),1)];
R2 = [R2;zeros(length(R1) - length(R2),1)];

sf = '(R1-R2+E1+E2)';
ns = char('R1','R2','E1','E2');
ns = ns';

% Create VTOL Geometry
geom = decsg([R1,R2,E1,E2],sf,ns);
geometryFromEdges(model,geom);

% Compute center of mass like this or directly define (below)
%{
xes = model.Geometry.Vertices(:,1);
yes = model.Geometry.Vertices(:,2);
c = [(max(xes)+min(xes))/2;(max(yes)+min(yes))/2];
%}

% Original Center of mass position for VTOL
c = [10;7];

% Scale VTOL to fit animation sizes
model.Geometry = scale(model.Geometry, 0.2,c);

% Prepare figure for Animation
figure()

set(gcf, 'Units', 'Normalized', 'OuterPosition', [0, 0, 1, 1]);

% Extract q single components
X = x(:,1);
Y = x(:,2);
Theta = x(:,3);

% Define Graph margins
xmin = min(X)-4;
xmax = max(X)+4;
ymin = min(Y)-4;
ymax = max(Y)+4;

xlim([xmin, xmax]);
ylim([ymin, ymax]);

hold on
axis equal

% Place Initial position of VTOL to match initial conditions
model.Geometry = translate(model.Geometry, [X(1);Y(1)]-c);
c = [X(1);Y(1)];
model.Geometry = rotate(model.Geometry, rad2deg(Theta(1)),c);
generateMesh(model);

% Start Animation
for k = 2:length(X)
    % Update Center of mass Position
    c = [X(k);Y(k)];
    
    % Relative Translation from time K-1 to K 
    model.Geometry = translate(model.Geometry, [X(k)-X(k-1); Y(k)-Y(k-1)]);
    % Relative Rotation from time K-1 to K 
    model.Geometry = rotate(model.Geometry, rad2deg(Theta(k)-Theta(k-1)),c);
    mesh = generateMesh(model);
    
    % Plot desired position as a yellow circle 
    plot(q1d,q2d,'o','MarkerSize',15, 'MarkerFaceColor',[0.9290 0.6940 0.1250])
    hold on
    % Plot VTOL with right position and roll angle theta
    pdemesh(model)
    hold off

    xlim([xmin, xmax]);
    ylim([ymin, ymax]);
    grid on
    drawnow
end


%% Static Rapresentation

% Plot Trajectory in XY reference frame
figure(1)
%Plot Trajectory
plot(X, Y, 'b-o');
hold on

% Plot Desired Position qd
plot(q1d,q2d, 'x','Linewidth',2)

title('Trajectory of VTOL', 'FontName','courier','FontSize',14)
legend('VTOL Trajectory' ,'FontName','courier','FontSize',10,...
    'Location','northeast','NumColumns',3)
xlim([xmin,xmax])
ylim([ymin,ymax])
xlabel('x(t)');
ylabel('y(t)');
grid on;

% Transform extracted v1,v2 to vectors for plotting
v1 = cell2mat(U1);
v2 = cell2mat(U2);

v1max = 20*ones(size(v1));
v2max = 40*ones(size(v2));
% Plot control efforts
figure(2)
plot(t,v1, 'b',t, v2,'r','LineWidth',2)
hold on
plot(t,v1max, 'k--',t, -v1max, 'k--',t, v2max,'k--',t,-v2max,'k--','LineWidth',1)
hold off
xlabel('time','FontName','courier','FontSize',14)
ylabel('v1=[m/s^2], v2=[rad/s^2]','FontName','courier','FontSize',14)
title('Control Effort v1 and v2', 'FontName','courier','FontSize',14)
legend('v1','v2' ,'FontName','courier','FontSize',10,...
    'Location','northeast','NumColumns',3)
grid on;

%% Vtol Dynamics Simulation

function [dx, U1, U2] =  VTOL_dynamics(t,x)
global  q1 q2 q3 p1 p2 p3 ep k1 k2 k3 q1d q2d q3d g Kv q p P11 P12 P21 P22 MdInv
q1 = x(1); 
q2 = x(2); 
q3 = x(3);
p1= x(4); 
p2 = x(5);
p3 = x(6);

p = [p1; p2; p3];

G = [1 0; 0 1; 1/ep*cos(q3) 1/ep*sin(q3)];

Md = [k1*ep*cos(q3)^2+k3      k1*ep*cos(q3)*sin(q3)      k1*cos(q3);
      k1*ep*cos(q3)*sin(q3)  -k1*ep*cos(q3)^2+k3         k1*sin(q3);
      k1*cos(q3)              k1*sin(q3)                 k2];
MdInv = (inv(Md));

% Calculate gradient of Hd (storage)
diff1 = double((P12*(q2 - q2d + ((cos(q3) - 1)*(k3 - ep*k1))/(k1 - ep*k2)))/2 - (P11*(q1d - q1 + (k3*sin(q3))/(k1 - ep*k2)))/2 + P21*(q2/2 - q2d/2 + ((cos(q3) - 1)*(k3 - ep*k1))/(2*(k1 - ep*k2))) - P11*(q1d/2 - q1/2 + (k3*sin(q3))/(2*(k1 - ep*k2))));
diff2 = double((P22*(q2 - q2d + ((cos(q3) - 1)*(k3 - ep*k1))/(k1 - ep*k2)))/2 - (P21*(q1d - q1 + (k3*sin(q3))/(k1 - ep*k2)))/2 + P22*(q2/2 - q2d/2 + ((cos(q3) - 1)*(k3 - ep*k1))/(2*(k1 - ep*k2))) - P12*(q1d/2 - q1/2 + (k3*sin(q3))/(2*(k1 - ep*k2))));
diff3 = double(((P11*k3*cos(q3))/(2*(k1 - ep*k2)) + (P21*sin(q3)*(k3 - ep*k1))/(2*(k1 - ep*k2)))*(q1d - q1 + (k3*sin(q3))/(k1 - ep*k2)) - ((P12*k3*cos(q3))/(2*(k1 - ep*k2)) + (P22*sin(q3)*(k3 - ep*k1))/(2*(k1 - ep*k2)))*(q2 - q2d + ((cos(q3) - 1)*(k3 - ep*k1))/(k1 - ep*k2)) + (g*sin(q3))/(k1 - ep*k2) - (sin(q3)*(P22*(q2/2 - q2d/2 + ((cos(q3) - 1)*(k3 - ep*k1))/(2*(k1 - ep*k2))) - P12*(q1d/2 - q1/2 + (k3*sin(q3))/(2*(k1 - ep*k2))))*(k3 - ep*k1))/(k1 - ep*k2) - (k3*cos(q3)*(P21*(q2/2 - q2d/2 + ((cos(q3) - 1)*(k3 - ep*k1))/(2*(k1 - ep*k2))) - P11*(q1d/2 - q1/2 + (k3*sin(q3))/(2*(k1 - ep*k2)))))/(k1 - ep*k2));

gradHd = [(k1^2*p1 - k2*k3*p1 - k1^2*p1*cos(q3)^2 - (k1^2*p2*sin(2*q3))/2 + k1*k3*p3*cos(q3) - ep*k1^2*p3*cos(q3) + ep*k1*k2*p1*cos(q3)^2 + (ep*k1*k2*p2*sin(2*q3))/2)/(k2*ep^2*k1^2*cos(q3)^2 - ep*k1^3*cos(q3)^2 + k1^2*k3 - k2*k3^2);
           -(2*k2*k3*p2 - 2*k1^2*p2*cos(q3)^2 + k1^2*p1*sin(2*q3) - 2*k1*k3*p3*sin(q3) + 2*ep*k1*k2*p2*cos(q3)^2 - ep*k1*k2*p1*sin(2*q3))/(2*k1^2*k3 - 2*k2*k3^2 - 2*ep*k1^3*cos(q3)^2 + 2*ep^2*k1^2*k2*cos(q3)^2);
           (k1*k3*p1*cos(q3) - k3^2*p3 + k1*k3*p2*sin(q3) + ep^2*k1^2*p3*cos(q3)^2 - ep*k1^2*p1*cos(q3))/(k2*ep^2*k1^2*cos(q3)^2 - ep*k1^3*cos(q3)^2 + k1^2*k3 - k2*k3^2)];
 
% Passivating Output 
y = G'*gradHd;

% Calculate input in modified coordinates
uT = double(inv(transpose(G)*G)*transpose(G)*([0; 0; -g/ep*sin(q3)]-Md*[diff1; diff2; diff3]) -Kv*y);

% Recalculate input in original coordinates
v1 = g*cos(q3) - sin(q3)*uT(1) + cos(q3)*uT(2);
v2 = g/ep*sin(q3) + 1/ep*cos(q3)*uT(1) + 1/ep*sin(q3)*uT(2);

% Saturation (based on real values for DJI Phantom 4)
v1max = 10;
v2max = 20;
% Max vertical acceleration
if abs(v1) > v1max
    v1 = sign(v1)*v1max;
end
% Max rotational (yaw) acceleration
if abs(v2) > v2max
    v2 = sign(v2)*v2max;
end

t;

% Simulate system in original coordinates
dqT = p;
dpT = [-sin(q3) ep*cos(q3); cos(q3) ep*sin(q3); 0 1]*[v1;v2]+[0;-g;0];

% Extract input values for plotting control effort
U1 = v1;
U2 = v2;
dx = [dqT; dpT];
end


