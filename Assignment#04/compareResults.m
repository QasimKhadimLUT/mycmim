function compareResults( t1, y1, t2, y2 )
%compareResults Compare two sets of results

nt1 = length(t1);
nt2 = length(t2);
if nt1 > nt2
    error('Wrong sizes')
end
divFactor = round(nt2/nt1);
% verify
diffT = norm(t2(1:divFactor:end)-t1); % if zero indexes match

% so test solution
rmsY1 = rms(y2(1:divFactor:end, 1)- y1);
rmsY2 = rms(y2(1:divFactor:end, 2)- y1);
rmsY3 = rms(y2(1:divFactor:end, 3)- y1);


disp(['Diff in time: ',num2str(diffT), ...
    ' rmse in Y1: ', num2str(rmsY1), ...
    ' rmse in Y2: ', num2str(rmsY2), ...
    ' rmse in Y3: ', num2str(rmsY3)])


end
