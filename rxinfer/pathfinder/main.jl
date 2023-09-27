include("./world.jl")
include("./pathfinder.jl")

using HTTP.WebSockets, JSON

pathfinder_server = WebSockets.listen("0.0.0.0", 8081) do ws    
    (step, execute, observe, observe_state) = create_pathfinder_world()

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

    is_paused = false
    iterate_state_actor = (count) -> begin
        if (!is_paused)
            step()
            next!(observations_stream, observe())
            next!(state_stream, observe_state())
        end
    end

    calculate_action_actor = ((actions, ps, hungers, food_p, free_energy),) -> begin
        action = mean(actions[1])
        p = mean(ps[1])
        predicted_ps = map(x -> mean(x), ps)
        predicted_hungers = map(x -> mean(x), hungers)
        hunger = mean(hungers[1])
        predicted_food_p = mean(food_p)

        payload = (
            action = action,
            predicted_ps = predicted_ps,
            predicted_hungers = predicted_hungers,
            predicted_food_p = predicted_food_p,
            free_energy = free_energy,
        )
        next!(actions_stream, payload)
    end

    perform_action_actor = (event) -> execute(event.action)

    send_message_actor = (event) -> begin
        send(ws, JSON.json(Dict(pairs(event))))
    end

    subscribe!(actions_stream, perform_action_actor)
    subscribe!(observations_stream, logger())
    subscribe!(actions_stream, logger())
    subscribe!(state_stream, send_message_actor)
    subscribe!(actions_stream, send_message_actor)

    engine = create_pathfinder_agent(observations_stream)
    next!(observations_stream, observe()) # Kick things off

    subscribe!(
            engine.posteriors[:action_k]
            |> with_latest(
                engine.posteriors[:p_k], 
                engine.posteriors[:hunger_k],
                engine.posteriors[:food_p],
                engine.free_energy),
            calculate_action_actor
    )

    subscribe!(interval(300), iterate_state_actor)

    for msg in ws
        is_paused = !is_paused
    end
end

println("Press enter to iterate")
while(true)
    readline()
    iterate_state_actor(1)
end