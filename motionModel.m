function [moved_particle_x, moved_particle_z] = motionModel(particle)
% particle ( X | Z | ORIENTATION )
% Gefahrene Strecke wird einfach auf 0.25 m gesetzt
q_angle = randn(1) * 0.2;
particle(3) = 1.15 + q_angle;
q_range = randn(1) * 0.4;
traveled_dist = 0.25 + q_range;
moved_particle_x = particle(1) + traveled_dist * cos(particle(3));
moved_particle_z = particle(2) + traveled_dist * sin(particle(3));
% moved_particle.y = particle.y;
% moved_particle.orientation = particle.orientation;
% moved_particle.weights = particle.weights;
% moved_particle.distances.d1 = 0;
% moved_particle.distances.d2 = 0;
% moved_particle.distances.d3 = 0;
% moved_particle.distances.d4 = 0;
% moved_particle.distances.d5 = 0;
% moved_particle.distances.d6 = 0;
% moved_particle.distances.d7 = 0;
% moved_particle.distances.d8 = 0;
% moved_particle.distances.d9 = 0;
% moved_particle.distances.d10 = 0;
end

