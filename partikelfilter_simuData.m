function [resampled_struct] = partikelfilter_simuData(particles, ROI)

N = length(particles.x);

num_segments = 10;

segments(:,1) = ROI(:,1);
segments(:,2) = zeros;
segments(:,3) = ROI(:,2);
segments(:,4) = ROI(:,3);


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
    end
    
    particles.weights(i) = particles.weights(i) / num_segments;
    % Medianfilter (per Hand) filtert die schlechtesten (groessten) Werte
    % heraus
    v_sort = sort(v);
    v_med = v_sort(1:8,1);
    v_mean = mean(v_med);
    v_all(i) = v_mean;
    
    particles.weights(i) = exp(-0.5*v_mean'*inv(L)*v_mean);
end

sum_weight = sum(particles.weights);

particles.weights = (particles.weights / sum_weight);

random_numbers = rand(N,1);

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

% figure(2)
% plot(v_all,particles.weights(:), '.')
% ylabel('Gewichtung');
% xlabel('Differenzen [meter]');
% title({'Verteilung der Gewichtung abhaengig von der Differenz der einzelnen Distanzen',...
%     ['L (Covariance): ', num2str(L)]})
% legend('Gewichtung ueber Differenz')

end
