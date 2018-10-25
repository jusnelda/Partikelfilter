function [ inMap ] = isInMap( walls, point )

in = inpolygon(point(1), point(2), walls(:,1), walls(:,2));
   
inMap = in;
end

