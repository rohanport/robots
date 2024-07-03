using RxInfer

function create_streams() 
    observations_stream = Subject(
        NamedTuple{(
            :p,:o,), 
        Tuple{Vector{Float64},Float64,}}
    )

    actions_stream = Subject(
        NamedTuple{(
            :ang_vel,:trans_vel), 
        Tuple{Float64,Float64,}}
    )

    predictions_stream = Subject(
        NamedTuple{(
            :ps,), 
        Tuple{Vector{Vector{Float64}},}}
    )

    free_energy_stream = Subject(
        NamedTuple{(
            :free_energy,), 
        Tuple{Float64,}}
    )

    return (observations_stream, actions_stream, predictions_stream, free_energy_stream)
end