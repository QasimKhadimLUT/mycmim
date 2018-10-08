function [time, q, qp, qpp] = KinematicAnalysis(Topology)

%% Defining the TSpan
time                        = Topology.ST;
dt                          = Topology.h;
%% Definng the initial conditions of position, velocity and acceleration

q0                          = zeros(3*numel(Topology.Bodies),1);

for n  = 1: numel(Topology.Bodies)
    q0(3*(n-1)+1:3*n)       = Topology.Bodies(n).R;
end
    q                       = zeros (length (q0), length(time));
    qp                      = zeros (length (q0), length(time));
    qpp                     = zeros (length (q0), length(time));
   
%% Solving the system for each time step
for t = 1: length(time)
    tol                     = 1e-6;
    maxIterations           = 1000;
    h                       = 1e-4;
    
    if t ==1
        q(:,t)              = q0;
        qp0                 = zeros (3*numel(Topology.Bodies),1);
        qpp0                = zeros (3*numel(Topology.Bodies),1);
    else
        x0                  = q(:,t-1)+ qp(:,t-1)*dt+ 0.5*qpp(:,t-1)*dt^2;
%% Position analysis
     ConstEq                = @(x) ConstraintEqs(Topology,x,time (t));
     JacDiff                = @(x) JacobianDiff(ConstEq ,x,h);
     q(:,t)                 = NR   (ConstEq,JacDiff,x0,tol,maxIterations);      
    end
%% Velocity analysis
     ConstEqD                = @(dx) JacobianMatrix(Topology,q(:,t))*dx+ Ct(Topology,q(:,t),dx,time (t));
     JacDiffD                = @(dx) JacobianDiff(ConstEqD ,dx,h);
     qp(:,t)                 = NR   (ConstEqD,JacDiffD,qp0,tol,maxIterations);
%% Acceleration analysis
     ConstEqDD               = @(ddx) JacobianMatrix(Topology,q(:,t))*ddx-Cgamma(Topology,q(:,t),qp(:,t),time (t));
     JacDiffDD               = @(ddx) JacobianDiff(ConstEqDD ,ddx,h);
     qpp(:,t)                = NR   (ConstEqDD,JacDiffDD,qpp0,tol,maxIterations);
end


