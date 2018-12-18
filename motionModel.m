function [moved_particle_x, moved_particle_z] = motionModel(particle)
% particle ( X | Z | ORIENTATION )
% Gefahrene Strecke wird einfach auf 0.25 m gesetzt
q_angle = -0.2 + (0.2+0.2)*rand(1);
% particle(3) = 1.15 + q_angle;
q_range = randn(1) * 0.2;
traveled_dist = 0.35 + q_range;
moved_particle_x = particle(1) + traveled_dist * cos(particle(3) + q_angle);
moved_particle_z = particle(2) + traveled_dist * sin(particle(3) + q_angle);

end

