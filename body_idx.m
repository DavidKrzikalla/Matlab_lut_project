%BODY_IDX Returns indices of the coordinates for body ii 

function indices = body_idx(ii)
    indices = 3 * ii + (-2:0);
end
