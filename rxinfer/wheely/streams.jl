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

    return (observations_stream, actions_stream)
end