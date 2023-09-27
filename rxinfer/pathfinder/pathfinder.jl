include("hunger_change_node.jl")
include("food_position_node.jl")

using RxInfer, StableRNGs

#=
    Variables:
        position: the latest position of the agent
        pred_action: the predicted actions of the agent
=#
@model function pathfinder(T)
    # Beliefs about the world

    food_smell_dir = datavar(Vector{Float64})
    food_smell_int = datavar(Float64)

    # Sensory input state
    p = datavar(Vector{Float64})
    p_k = randomvar(T - 1)
    hunger = datavar(Float64) 
    hunger_k = randomvar(T) 
    
    # Future actions state
    action_k = randomvar(T) # predicted distribution of x/y change of actions 
 
    # Internal states    
    curr_p ~ MvNormalMeanCovariance(p, 0.00001 * diageye(2))
    prev_p = curr_p
    
    curr_hunger ~ NormalMeanVariance(hunger, 0.00001)
    prev_hunger = curr_hunger

    curr_food_smell_dir ~ MvNormalMeanCovariance(food_smell_dir, 0.00001 * diageye(2))
    curr_food_smell_int ~ NormalMeanVariance(food_smell_int, 0.00001)
    food_p ~ FoodPositionNode(curr_p, curr_food_smell_dir, curr_food_smell_int)

    hunger_change_k = randomvar(T)
    
    println("making look ahead states")
    for k in 1:T
        println(k)
        
        action_k[k] ~ MvNormalMeanCovariance(zeros(2), diageye(2))
        
        hunger_change_k[k] ~ HungerChangeNode(food_p, prev_p, action_k[k])
        hunger_k[k] ~ prev_hunger +  hunger_change_k[k]
        hunger_k[k] ~ NormalMeanVariance(10.0, ((T - k) / 2) + 1.0) 
        prev_hunger = hunger_k[k]
        
        if (k < T)
            p_k[k] ~ prev_p + action_k[k]
            prev_p = p_k[k]
        end
    end

    println("model done")
end

@meta function pathfinder_meta()
    HungerChangeNode() -> HungerChangeMeta(StableRNG(123), 100)
end

function create_pathfinder_agent(sensory_datastream)
    lookahead = 10
     
    engine = rxinference(
        model         = pathfinder(lookahead),
        datastream    = sensory_datastream,
        meta          = pathfinder_meta(),
        initmarginals = (
            food_p = vague(MvNormalMeanCovariance, 2),
            action_k = vague(MvNormalMeanCovariance, 2),
            curr_p = vague(MvNormalMeanCovariance, 2),
            p_k = vague(MvNormalMeanCovariance, 2),
            curr_hunger = vague(NormalMeanVariance),
            hunger_change_k = vague(NormalMeanVariance),
            hunger_k = vague(NormalMeanVariance),
        ),
        initmessages = (
            food_p = vague(MvNormalMeanCovariance, 2),
            action_k = vague(MvNormalMeanCovariance, 2),
            curr_p = vague(MvNormalMeanCovariance, 2),
            p_k = vague(MvNormalMeanCovariance, 2),
            curr_hunger = vague(NormalMeanVariance),
            hunger_change_k = vague(NormalMeanVariance),
            hunger_k = vague(NormalMeanVariance),
        ),
        returnvars = (
            :action_k,
            :p_k,
            :hunger_k,
            :food_p
         ),
        iterations    = 10,
        autostart     = false,
        free_energy   = true,
    )
    
    RxInfer.start(engine)
    
    return engine
end
