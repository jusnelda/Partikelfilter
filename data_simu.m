function points = data_simu(start, fin)

%start = [207,138];
%fin = [236,203];
alpha = atan2((fin(2) - start(2)), (fin(1) - start(1)));
dist = sqrt((fin(1)-start(1))^2 + (fin(2)-start(2))^2);

motion = dist/20;
for i = 1 : 20
    if i == 1
        points(1,1) =  start(1);
        points(1,2) = start(2);
    else
        points(i,1) = points(i-1,1) + motion * cos(alpha);
        points(i,2) = points(i-1,2) + motion * sin(alpha);
    end
    
end
points(:,3) = alpha;


end