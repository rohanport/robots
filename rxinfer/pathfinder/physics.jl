using LinearAlgebra

food_radius = 3.0

function change_in_hunger(food_p::Vector{Float64}, p::Vector{Float64})
    return 0.5 * (tanh(3 * (norm(food_p - p) - food_radius)) - 0.6)
end

function p_to_food_smell_dir(food_p::Vector{Float64}, p::Vector{Float64})
    dist_from_food_vec = food_p - p
    return dist_from_food_vec / (abs(dist_from_food_vec[1]) + abs(dist_from_food_vec[2])) 
end

function p_to_food_smell_int(food_p::Vector{Float64}, p::Vector{Float64})
    dist_from_food = norm(food_p - p)
    return 1 / (dist_from_food^3) 
end