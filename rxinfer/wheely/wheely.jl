include("next_position_node.jl")

using RxInfer, StableRNGs

@model function wheely(T)
    # Sensory input state
    p = datavar(Vector{Float64}) # position
    p_k = randomvar(T) # predicted future positions
    o = datavar(Float64) # orientation
    o_k = randomvar(T) # predicted future orientations

    trans_vel_k = randomvar(T) # predicted distribution of translation velocity actions 
    ang_vel_k = randomvar(T) # predicted distribution of angular velocity actions 
 
    # Internal states    
    curr_p ~ MvNormalMeanCovariance(p, 0.00001 * diageye(2))
    prev_p = curr_p

    curr_o ~ NormalMeanVariance(o, 0.00001)
    prev_o = curr_o
    
    println("making look ahead states")
    t = 0.0
    for k in 1:T
        t_delta = 0.2 * (1.5 ^ (k - 1))
        t = t + t_delta
        println("t=$(t)")

        ang_vel_k[k] ~ NormalMeanVariance(0.0, 0.6)
        o_k[k] ~ prev_o + ang_vel_k[k]
        
        trans_vel_k[k] ~ NormalMeanVariance(0.0, 0.3)

        p_k[k] ~ NextPositionNode(prev_p, o_k[k], trans_vel_k[k]) where { meta = ( t_delta = t_delta,), }
        
        if (k > T / 2)
            p_k[k] ~ MvNormalMeanCovariance([5.0, 10.0], diageye(2)) # Goal state for second half of states
        end
        
        prev_o = o_k[k]
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
            o_k = vague(NormalMeanVariance),
            curr_o = vague(NormalMeanVariance),
            ang_vel_k = vague(NormalMeanVariance),
            trans_vel_k = vague(NormalMeanVariance),
            curr_p = vague(MvNormalMeanCovariance, 2),
            p_k = vague(MvNormalMeanCovariance, 2),
        ),
        initmessages = (
            o_k = vague(NormalMeanVariance),
            curr_o = vague(NormalMeanVariance),
            ang_vel_k = vague(NormalMeanVariance),
            trans_vel_k = vague(NormalMeanVariance),
            curr_p = vague(MvNormalMeanCovariance, 2),
            p_k = vague(MvNormalMeanCovariance, 2),
        ),
        returnvars = (
            :ang_vel_k,
            :trans_vel_k,
            :o_k,
            :p_k,
         ),
        iterations    = 10,
        autostart     = false,
        free_energy   = true,
    )
    
    RxInfer.start(engine)
    
    return engine
end
