function Cgamma = Cgamma(Topology,x,v,time)

Cqq = [];
for n = 1: numel(Topology.Joints)
    JointType          = Topology.Joints(n).Type;
    JointID            = Topology.Joints(n).JointID;
       u               = Topology.Joints(n).BodiesCoordinates; 
       
      if JointID(1)      == 0
           R           = [[0 0 0]' x(3*(numel(Topology.Bodies(2))-1)+1:3*numel(Topology.Bodies(2)))];
           V           = [[0 0 0]' v(3*(numel(Topology.Bodies(2))-1)+1:3*numel(Topology.Bodies(2)))];
       elseif JointID(2) == 0
           R           = [x(3*(numel(Topology.Bodies(1))-1)+1:3*numel(Topology.Bodies(1))) [0 0 0]'];
           V           = [v(3*(numel(Topology.Bodies(1))-1)+1:3*numel(Topology.Bodies(1))) [0 0 0]'];
       else
           R           = [x(3*(numel(Topology.Bodies(1))-1)+1:3*numel(Topology.Bodies(1))) x(3*(numel(Topology.Bodies(2))-1)+1:3*numel(Topology.Bodies(2)))];
           V           = [v(3*(numel(Topology.Bodies(1))-1)+1:3*numel(Topology.Bodies(1))) v(3*(numel(Topology.Bodies(2))-1)+1:3*numel(Topology.Bodies(2)))];
      end  
      
      if strcmp(JointType,'revolute')
         Cqq           =[Cqq
                         ( u(2,1)*sin(R(3,1))-u(1,1)*cos(R(3,1)))*V(3,1)^2+(u(1,2)*cos(R(3,2))-u(2,2)*sin(R(3,2)))*V(3,2)^2
                         (-u(2,1)*cos(R(3,1))-u(1,1)*sin(R(3,1)))*V(3,1)^2+(u(2,2)*cos(R(3,2))+u(1,2)*sin(R(3,2)))*V(3,2)^2];
end

end
for i = 1: numel(Topology.TimeConstraint)
    Cqq                = [Cqq; 0];
end
    Ctt                = [];

for j = 1: numel(Topology.Joints)
    if strcmp(JointType,'revolute')
    Ctt = [Ctt; zeros(2,1)];
    end
end

for s = 1: numel(Topology.TimeConstraint)
    CDDiff                = Topology.TimeConstraint(s).ddFun;
    Ctt                   = [Ctt;CDDiff(time)];
end
    Cqt                   = zeros(size(Ctt));
    Cgamma                = -Cqq-Cqt-Ctt;
end
