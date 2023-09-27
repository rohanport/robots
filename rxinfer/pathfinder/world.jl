include("physics.jl")

using Random, LinearAlgebra

#=

    variables:
        p = position (x, y coord)
        a = action move [up,right,down,left]
=#

function create_pathfinder_world(
    initial_p = [9.0, 4.0],
    initial_hunger = 10.0,
    initial_action = [0.0, 0.0]
)    
    p = initial_p
    hunger = initial_hunger
    action = initial_action

    food_p = 20.0 * [rand(), rand()]
    food_countdown = 60
    
    food_smell_dir = p_to_food_smell_dir(food_p, p)
    food_smell_int = p_to_food_smell_int(food_p, p)

    randomness_scale = 3.0
    randomness = 0.0
    velocity = norm(action)

    user_inputs = []

    step = () -> begin
        food_countdown = food_countdown - 1
        if (food_countdown < 0)
            food_p = 20.0 * [rand(), rand()]
            food_countdown = 60
        end

        random_variance = randomness_scale * rand()^10
        random_noise = random_variance * [rand() - 0.5, rand() - 0.5] 
        p = p + action + random_noise
        hunger = hunger + change_in_hunger(food_p, p)
        food_smell_dir = p_to_food_smell_dir(food_p, p)
        food_smell_int = p_to_food_smell_int(food_p, p)
        randomness = norm(random_noise)
        velocity = norm(action)
    end
    
    execute = (new_action::Vector{Float64}) -> begin
        action_limit  = max(1.0, norm(action))
        action = new_action / action_limit
    end
    
    observe = () -> begin 
        return (
            p = p, 
            hunger = hunger,
            food_smell_dir = food_smell_dir,
            food_smell_int = food_smell_int,
        )
    end

    observe_state = () -> begin
        return (
            p = p, 
            hunger = hunger,
            food_p = food_p,
            randomness = randomness,
            velocity = velocity,
        )
    end

    input = (direction) -> begin
        push!(user_inputs)
    end
        
    return (step, execute, observe, observe_state)
end