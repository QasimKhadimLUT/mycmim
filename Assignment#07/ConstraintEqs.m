function ConstraintEqs = ConstraintEqs(Topology,x,time)

ConstraintEqs          = [];
%% Defining the constraint equations for joints

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
 %% Using revolute joint
       if strcmp (JointType,'revolute')
           ConstraintEqs = [ConstraintEqs
                        R(1,1)+u(1,1)*cos(R(3,1))-u(2,1)*sin(R(3,1))-R(1,2)-u(1,2)*cos(R(3,2))+u(2,2)*sin(R(3,2))
                        R(2,1)+u(1,1)*sin(R(3,1))+u(2,1)*sin(R(3,1))-R(2,2)-u(1,2)*sin(R(3,2))-u(2,2)*cos(R(3,2))];
       end

end

%% Time dependent constraints

 for i = 1: numel(Topology.TimeConstraint)
        BodyDOF              = rangeCal(Topology.TimeConstraint(i).body);
        ConstraintDOF        = BodyDOF(Topology.TimeConstraint(i).DOF);
        ConstraintExpression = Topology.TimeConstraint(i).Fun;
        
        ConstraintEqs  = [ConstraintEqs;ConstraintExpression(time)-x(ConstraintDOF)];
end

end
