close all
clear all
%%                   %%%                         %%
%%%%%% KINEMATIC ANALYSIS OF FOUR BAR MECHANISM%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Structure for defining the parameters

Topology             = struct ();
%% Defining the Time Step (h) and Simulation Time (ST)

Topology.h           = 0.01;
EndTime              = 1;
Topology.ST          = 0:Topology.h:EndTime;
%% Length of bar

L                    = 1; 
omega                = -1;
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
GlobalPosition       =  [L/2,L,-pi/2]';
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
ConstraintFun  =@(t) omega*t+ pi/2;
ConstraintdFun = @(t) omega;
ConstraintddFun = @(t) 0;
TimeConstraint  = struct('body',ConstraintBody,'DOF',ConstraintDOF,'Fun', ConstraintFun,'dFun',ConstraintdFun,'ddFun',ConstraintddFun);

Topology.TimeConstraint = [TimeConstraint];



%% Simulation settings
[t, q, qp,qpp]       = KinematicAnalysis(Topology);
% 
% 
% 
