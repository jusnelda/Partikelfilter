function points = data_simu(start, fin)

%start = [207,138];
%fin = [236,203];
alpha = atan2((fin(2) - start(2)), (fin(1) - start(1)));
dist = sqrt((fin(1)-start(1))^2 + (fin(2)-start(2))^2);

motion = dist/20;
for i = 1 : 20
    if i == 1
        points(1,1) =  start(1) + motion * cos(alpha);
        points(1,2) = start(2) + motion * sin(alpha);
    else
        points(i,1) = points(i-1,1) + motion * cos(alpha);
        points(i,2) = points(i-1,2) + motion * sin(alpha);
    end
    
end

% plot(start(1), start(2), '*r')
% hold on
% plot(fin(1), fin(2),'*r')
% hold on
% plot(points(:,1), points(:,2), '.b')

end