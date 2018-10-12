close all
clear all
%%                   %%%                         %%
%%%%%% KINEMATIC ANALYSIS OF FOUR BAR MECHANISM%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Structure for defining the parameters

Topology             = struct ();
%% Defining the Time Step (h) and Simulation Time (ST)

Topology.h           = 0.001;
EndTime              = 1;
Topology.ST          = 0:Topology.h:EndTime;
%% Length of bar

L                    = 1; 
omega                = 10;
%% Defining the model bodies

%Body 1
Type                 = 'bar';
Length               =    L;
GlobalPosition       =  [0,L/2,pi/2]';
Body1                = struct('type',Type, 'L',Length,'R', GlobalPosition);

%Body 2
Type                 = 'bar';
Length               =    L;
GlobalPosition       =  [L/2,L,0]';
Body2                = struct('type',Type, 'L',Length,'R', GlobalPosition);


% %Body 3
Type                 = 'bar';
Length               =    L;
GlobalPosition       =  [L,L/2,-pi/2]';
Body3                = struct('type',Type, 'L',Length,'R', GlobalPosition);


Topology.Bodies      = [Body1,Body2,Body3];

%% Defining the joints

% Joint 1
JointType            = 'revolute';
JointBodies          =  [0 1];
JointLocation        =  [[0;0],[-L/2;0]];
Joint1               = struct ('Type',JointType, 'JointID',JointBodies  ,'BodiesCoordinates', JointLocation);
 
% Joint 2
JointType            = 'revolute';
JointBodies          =  [1 2];
JointLocation        =  [[L/2;0],[-L/2;0]];
Joint2               = struct ('Type',JointType, 'JointID',JointBodies  ,'BodiesCoordinates', JointLocation);


% Joint 3
JointType           = 'revolute';
JointBodies         =  [2 3];
JointLocation       =  [[L/2;0],[-L/2;0]];
Joint3               = struct ('Type',JointType, 'JointID',JointBodies  ,'BodiesCoordinates', JointLocation);

% Joint 4  
JointType            = 'revolute';
JointBodies          =  [3 0];
JointLocation        =  [[L/2;0],[L;0]];
Joint4               = struct ('Type',JointType, 'JointID',JointBodies  ,'BodiesCoordinates', JointLocation);

Topology.Joints      = [Joint1,Joint2,Joint3,Joint4];
%%  Defining the time dependent constraints

ConstraintBody = 1;
ConstraintDOF  = 3;
ConstraintFun  =@(t) cos(omega*t+ pi/2);
ConstraintdFun = @(t) omega;
ConstraintddFun = @(t) 0;
TimeConstraint  = struct('body',ConstraintBody,'DOF',ConstraintDOF,'Fun', ConstraintFun,'dFun',ConstraintdFun,'ddFun',ConstraintddFun);

Topology.TimeConstraint = [TimeConstraint];



%% Simulation settings
[t, q, qp,qpp]       = KinematicAnalysis(Topology);

%% Reading the ADAMS, commercial software, results for the kinematic analysis

x = xlsread('Position.xlsx','Position','A1:B1002');
v = xlsread('Velocity.xlsx','Velocity','A1:C1002');
a = xlsread('Acceleration.xlsx','Acceleration','A1:C1002');

%% Plotting the software results with ADAMS

%Position of body 1
figure(1)
plot(t,q(2,:),'k',x(:,1),x(:,2),'--k','Linewidth',1.5)
grid on
legend('General software on MATLAB','ADAMS result')
xlabel('Time(s)')
ylabel('Position(m)')
Plotting(gcf,'Times New Roman',12);
print('Position of body 1 along x-axis','-depsc')

% Velocity of body 1
figure(2)
plot(t,qp(1,:),'k',v(:,1),v(:,2),'--k','Linewidth',1.5)
grid on
legend('General software on MATLAB','ADAMS result')
xlabel('Time(s)')
ylabel('Velocity(m/s)')
Plotting(gcf,'Times New Roman',12);
print('Velocity of body 1 along x-axis','-depsc')

% Acceleration of body 1
figure(3)
plot(t,qpp(1,:),'k',a(:,1),a(:,2),'--k','Linewidth',1.5)
grid on
legend('General software on MATLAB','ADAMS result')
xlabel('Time(s)')
ylabel('Acceleration(ms^-2)')
Plotting(gcf,'Times New Roman',12);
print('Acceleration of body 1 along x-axis','-depsc')








