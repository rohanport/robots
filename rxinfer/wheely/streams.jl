using RxInfer

function create_streams() 
    observations_stream = Subject(
        NamedTuple{(
            :p,), 
        Tuple{Vector{Float64},}}
    )

    actions_stream = Subject(
        NamedTuple{(
            :action,), 
        Tuple{Vector{Float64},}}
    ) 

    return (observations_stream, actions_stream)
end