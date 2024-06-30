using RxInfer, StableRNGs

@model function wheely(T)
    # Sensory input state
    p = datavar(Vector{Float64})
    p_k = randomvar(T)
    
    # Future actions state
    action_k = randomvar(T) # predicted distribution of x/y change of actions 
 
    # Internal states    
    curr_p ~ MvNormalMeanCovariance(p, 0.00001 * diageye(2))
    prev_p = curr_p
    
    println("making look ahead states")
    for k in 1:T
        println(k)
        
        action_k[k] ~ MvNormalMeanCovariance(zeros(2), diageye(2))
        p_k[k] ~ prev_p + action_k[k]
        p_k[k] ~ MvNormalMeanCovariance([5.0, 10.0], diageye(2))
        prev_p = p_k[k]
    end

    println("model done")
end

@meta function wheely_meta()
end

function create_wheely_agent(sensory_datastream)
    lookahead = 10
     
    engine = rxinference(
        model         = wheely(lookahead),
        datastream    = sensory_datastream,
        meta          = wheely_meta(),
        initmarginals = (
            action_k = vague(MvNormalMeanCovariance, 2),
            curr_p = vague(MvNormalMeanCovariance, 2),
            p_k = vague(MvNormalMeanCovariance, 2),
        ),
        initmessages = (
            action_k = vague(MvNormalMeanCovariance, 2),
            curr_p = vague(MvNormalMeanCovariance, 2),
            p_k = vague(MvNormalMeanCovariance, 2),
        ),
        returnvars = (
            :action_k,
            :p_k,
         ),
        iterations    = 10,
        autostart     = false,
        free_energy   = true,
    )
    
    RxInfer.start(engine)
    
    return engine
end
