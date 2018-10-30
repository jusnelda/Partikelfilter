function partikelfilter(particles, ROI)

N = length(particles);
particles_resampled = zeros(N,4);
particles_resampled(:,4) = 1/N;

% Calculate mean of roi segments
num_segments = 10;
min_angle = min(ROI.angle);
max_angle = max(ROI.angle);
step = (max_angle - min_angle) / num_segments;
point_index = 1;
segment_points = [ROI.x(1), ROI.y(1), ROI.z(1), ROI.dist(1), ROI.angle(1)];

for s = 1 : num_segments
    
    for i = 1 : length(ROI.x)
        if ROI.angle(i) < min_angle + step
            segment_points(point_index,:) = [ROI.x(i), ROI.y(i), ROI.z(i), ROI.dist(i), ROI.angle(i)];
            point_index = point_index + 1;
        end
    end

    % Median
%     segments(s,1) = median(segment_points(:,1));
%     segments(s,2) = median(segment_points(:,2));
%     segments(s,3) = median(segment_points(:,3));
    % Mitteln
    segments(s,1) = mean(segment_points(:,1));
    segments(s,2) = mean(segment_points(:,2));
    segments(s,3) = mean(segment_points(:,3));
    
    % Strecke von (0,0,0) 3D
    segments(s,4) = sqrt( (segments(s,1))^2 + (segments(s,2))^2 + (segments(s,3))^2 );
    % Winkel zu (0,0) 2D
    segments(s,5) = atan2( segments(s,3),segments(s,1) );
    point_index = 1;
    min_angle = min_angle + step;
    s =  s + 1;
end


% Partikelfilter
%%%%%%%%%%%%% TODO Distanzen, die Null bzw. isnan sind von den
%%%%%%%%%%%%% intersectionPts nochmal ueberlegen, wie man das macht...
L = 0.2;

%     % Bewertung
for i=1:N
    for s = 1 : num_segments
        v(s,1) = particles.(distances{s})(i) - segments(s,4);
        
        weights(s,1) = exp(-0.5*v(s,1)'*inv(L)*v(s,1));
    end
    particles.weights(i) = sum(weights) + 0.01/N;
    
end

sum_weight = sum(particles.weights);

for i=1:N
    particles.weights(i) = particles.weights(i) / sum_weight;
end
end