close all
clear all

%% KINEMATIC ANALYSIS OF FOUR BAR MECHANISM

L = 1; 

%% Defining the model bodies

Body1.Type              = 'bar';
Body1.ID                = [1 2];
Body1.Coordinates       = [0 L/2 pi/2];
Topology.Body1 = struct('Type','BodyType','ID','Location','Coordinates','BodyDef');


Body2.Type              = 'bar';
Body2.ID                = [2 3];
Body2.Coordinates       = [L/2 L 0];
Topology.Body2 = struct('Type','BodyType','ID','Location','Coordinates','BodyDef');

Body3.Type              = 'bar';
Body3.ID                = [3 4];
Body3.Coordinates       = [L L/2 -pi/2];
Topology.Body3 = struct('Type','BodyType','ID','Location','Coordinates','BodyDef');

Topology.Bodies = [Body1,Body2, Body3]  