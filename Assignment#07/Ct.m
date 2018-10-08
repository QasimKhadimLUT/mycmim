function Ct =Ct(Topology,x,v,time)

Ct = [];

for n = 1: numel(Topology.Joints)
    JointType          = Topology.Joints(n).Type;
    
    if strcmp(JointType,'revolute')
        Ct = [Ct; zeros(2,1)];
    end
end
for  i = 1: numel(Topology.TimeConstraint)
        BodyDOF              = rangeCal(Topology.TimeConstraint(i).body);
        ConstraintDOF        = BodyDOF(Topology.TimeConstraint(i).DOF);
        
        CDiff                = Topology.TimeConstraint(i).dFun;
        Ct                   = [Ct;CDiff(time)];
end
end
