function route = GradientBasedPlanner (f, start_coords, end_coords, max_its)
% GradientBasedPlanner : This function plans a path through a 2D
% environment from a start to a destination based on the gradient of the
% function f which is passed in as a 2D array. The two arguments
% start_coords and end_coords denote the coordinates of the start and end
% positions respectively in the array while max_its indicates an upper
% bound on the number of iterations that the system can use before giving
% up.
% The output, route, is an array with 2 columns and n rows where the rows
% correspond to the coordinates of the robot as it moves along the route.
% The first column corresponds to the x coordinate and the second to the y coordinate

[gx, gy] = gradient (-f);

route = zeros(max_its,2);
iteration =0;
interval = 1;
position = start_coords;
while iteration < max_its
    iteration = iteration + 1;
    coords = round(position);
    route(iteration,:) = position';
    v_x = interval*gx(coords(2),coords(1));
    v_y = interval*gy(coords(2),coords(1));
    v  = sqrt(v_x^2 + v_y^2);
    v_x = v_x/v;
    v_y = v_y/v;
	position = position + [v_x, v_y];
    if all(coords == end_coords)
        break
    end
end
route(min(max_its,iteration+1):end,:) = [];
end
