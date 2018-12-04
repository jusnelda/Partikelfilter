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
        points(1,3) = alpha;
    else
        points(i,1) = points(i-1,1) + motion * cos(alpha);
        points(i,2) = points(i-1,2) + motion * sin(alpha);
        points(i,3) = alpha;
    end
%     alpha = alpha + 0.1;
end
% points(:,3) = alpha;


end