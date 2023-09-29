using RxInfer

function create_streams() 
    state_stream = Subject(
        NamedTuple{(
            :p,
            :hunger,
            :food_p,
            :randomness,
            :velocity,), 
        Tuple{
            Vector{Float64},
            Float64,
            Vector{Float64},
            Float64,
            Float64,}
        }
    )

    observations_stream = Subject(
        NamedTuple{(
            :p,
            :hunger,
            :food_smell_dir,
            :food_smell_int,), 
        Tuple{
            Vector{Float64},
            Float64,
            Vector{Float64},
            Float64,}
        }
    )

    actions_stream = Subject(
        NamedTuple{(
            :action,
            :predicted_ps,
            :predicted_hungers,
            :predicted_food_p,
            :free_energy), 
        Tuple{
            Vector{Float64},
            Vector{Vector{Float64}},
            Vector{Float64},
            Vector{Float64},
            Float64}
        }
    ) 

    return (state_stream, observations_stream, actions_stream)
end