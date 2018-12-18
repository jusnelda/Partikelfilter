function [resampled_struct] = partikelfilter(particles, ROI)

N = length(particles.x);

% Solange wir simulierte Daten verwenden brauche wir das nicht
% Calculate mean of roi segments
num_segments = 10;
% min_angle = min(ROI.angle);
% max_angle = max(ROI.angle);
% step = (max_angle - min_angle) / num_segments;
% point_index = 1;
% segment_points = [ROI.x(1), ROI.y(1), ROI.z(1), ROI.dist(1), ROI.angle(1)];

% Bei simulierten Daten das verwenden
segments(:,1) = ROI(:,1);
segments(:,2) = zeros;
segments(:,3) = ROI(:,2);
segments(:,4) = ROI(:,3);

% for s = 1 : num_segments
%     
%     for i = 1 : length(ROI.x)
%         if ROI.angle(i) < min_angle + step
%             segment_points(point_index,:) = [ROI.x(i), ROI.y(i), ROI.z(i), ROI.dist(i), ROI.angle(i)];
%             point_index = point_index + 1;
%         end
%     end
%     
%     % Mitteln
%     segments(s,1) = mean(segment_points(:,1));
%     segments(s,2) = mean(segment_points(:,2));
%     segments(s,3) = mean(segment_points(:,3));
%     
%     % Strecke von (0,0,0) 3D
%     segments(s,4) = sqrt( (segments(s,1))^2 + (segments(s,2))^2 + (segments(s,3))^2 );
%     % Winkel zu (0,0) 2D
%     segments(s,5) = atan2( segments(s,3),segments(s,1) );
%     point_index = 1;
%     min_angle = min_angle + step;
%     s =  s + 1;
% end


% Partikelfilter

% Ab 1 Meter schlechte Gewichtung
L = 0.05;
distance_names = {'d1'; 'd2'; 'd3'; 'd4'; 'd5'; 'd6'; 'd7'; 'd8'; 'd9'; 'd10'};

for i=1:N
    particles.weights(i) = 0;
    for s = 1 : num_segments
        % Differenzen zwischen Particel Abstand und Kinect Mittelwert
        % berechnen
        v(s,1) = abs(segments(s,4) - particles.distances(i).(distance_names{s}));
%         v(s,1) = (particles.distances(i).(distance_names{s})) - (segments(s,4));
%         particles.weights(i) = particles.weights(i) + exp(-0.5*v(s,1)'*inv(L)*v(s,1)) + 0.01/N;
        
    end
    particles.weights(i) = particles.weights(i) / num_segments;
    % Medianfilter (per Hand) filtert die schlechtesten (gr��ten) Werte
    % heraus
    v_sort = sort(v);
    v_med = v_sort(1:8,1);
    v_mean = mean(v_med);
    v_all(i) = v_mean;
    particles.weights(i) = exp(-0.5*v_mean'*inv(L)*v_mean);% / N = w0;
%     particles.weights(i) = exp(-0.5*v_mean'*inv(L)*v_mean) / 0.01;
%     particles.weights(i) = exp(-0.5*v_mean) / 0.01;
    
end

sum_weight = sum(particles.weights);

particles.weights = (particles.weights / sum_weight);%*100;

random_numbers = rand(N,1);%*100;

for i = 1 : N
    
    weight_threshold = 0;
    
    for u = 1 : N
        
        weight_threshold = weight_threshold + particles.weights(u);
        if(random_numbers(i) < weight_threshold)
            resampled_struct.x(i) = particles.x(u);
            resampled_struct.y(i) = particles.y(u);
            resampled_struct.z(i) = particles.z(u);
            resampled_struct.orientation(i) = particles.orientation(u);
            resampled_struct.weights(i) = 1/N;
            break;
        end
        
    end
    
end

% estimate(t,:) = [mean(particles(:,1)) mean(particles(:,2))];
% figure(2)
% plot(v_all,particles.weights(:), '.')
% ylabel('Gewichtung');
% xlabel('Differenzen [meter]');
% title({'Verteilung der Gewichtung abhaengig von der Differenz der einzelnen Distanzen',...
%     ['L (Covariance): ', num2str(L)]})
% legend('Gewichtung ueber Differenz')

end
