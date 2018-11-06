function [moved_particle] = motionModel(particle)

% Gefahrene Strecke wird einfach auf 0.25 m gesetzt
traveled_dist = 0.25;
moved_particle.x = particle.x + traveled_dist * cos(particle.orientation);
moved_particle.z = particle.z + traveled_dist * sin(particle.orientation);
moved_particle.y = particle.y;
moved_particle.orientation = particle.orientation;
moved_particle.weights = particle.weights;
end

