function Cq = JacobianMatrix(Topology,x)

Cq = [];
%% Defining the Jacobian for Joints (Revolute joints)
for n = 1: numel(Topology.Joints)
    JointType          = Topology.Joints(n).Type;
    JointID            = Topology.Joints(n).JointID;
       u               = Topology.Joints(n).BodiesCoordinates;  
       
        if JointID(1) == 0
           R           = [[0;0;0], x(rangeCal(JointID(2)))];
       elseif JointID(2) == 0
           R           = [x(rangeCal(JointID(1))), [0;0;0]];
       else
           R           = [x(rangeCal(JointID(1))),  x(rangeCal(JointID(2)))];
        end
       
        if strcmp(JointType,'revolute')
            Cqn        = [1, 0, -u(1,1)*sin(R(3,1))-u(2,1)*cos(R(3,1)), -1, 0, u(1,2)*sin(R(3,2))+u(2,2)*cos(R(3,2))
                          0, 1,  u(1,1)*cos(R(3,1))-u(2,1)*sin(R(3,1)),  0, -1,-u(1,2)*cos(R(3,2))+u(2,2)*sin(R(3,2))];
    
            Cqt        = zeros(2, length(x));
            
       if JointID(1) == 0
         Cqt(:,rangeCal(JointID(2))) =Cqt(:,rangeCal(JointID(2)))+ Cqn(:,4:6);
       elseif JointID(2) == 0
         Cqt(:,rangeCal(JointID(1))) =Cqt(:,rangeCal(JointID(1)))+ Cqn(:,1:3);
       else
         Cqt(:,[rangeCal(JointID(1)),rangeCal(JointID(2))])= Cqt(:,[rangeCal(JointID(1)),rangeCal(JointID(2))])+Cqn;         
       end
       
       Cq = [Cq; Cqt];
       end         
end

%% Jacobian of time dependent constraints
for i = 1: numel(Topology.TimeConstraint)
        BodyDOF              = rangeCal(Topology.TimeConstraint(i).body);
        ConstraintDOF        = BodyDOF(Topology.TimeConstraint(i).DOF);
        
        Cqb                  = zeros(1,length(x));
        Cqb(ConstraintDOF )  = -1;
        
        Cq                   = [Cq; Cqb];
end
end

