include("../common/Common.jl")
include("./world.jl")
include("./pathfinder.jl")
include("./streams.jl")

using HTTP.WebSockets, JSON, DotEnv, .Common
DotEnv.config()

pathfinder_server = WebSockets.open(ENV["PUB_SUB_URL"]) do ws
    
    ##########################################################
    #          Set up rxinfer engine and world               #
    ##########################################################

    (step, execute, observe, observe_state) = create_pathfinder_world()
    (state_stream, observations_stream, actions_stream) = create_streams()
    
    iterate_state_actor = (count) -> begin
        step()
        next!(observations_stream, observe())
        next!(state_stream, observe_state())
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

    subscribe!(actions_stream, perform_action_actor)

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

    ########################################################
    #          Set up connections to pub-sub               #
    ########################################################

    sender = (msg) -> send(ws, msg)

    # Subscribe to relevant topics
    pubsub_sub("rxinfer_pathfinder_state", sender)
    pubsub_sub("rxinfer_pathfinder_action", sender)
    
    # Start publishing new events from datastreams
    subscribe!(state_stream, pubsub_pub_actor("rxinfer_pathfinder_state", sender))
    subscribe!(actions_stream, pubsub_pub_actor("rxinfer_pathfinder_action", sender))

    for msg in ws
        println("got ", msg)
    end
end