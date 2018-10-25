function [particle] = gen_random_particle(range_x,range_y)
    particle(1,1) = range_x(1) + rand(1) * range_x(2);
    particle(1,2) = range_y(1) + rand(1) * range_y(2);
end

