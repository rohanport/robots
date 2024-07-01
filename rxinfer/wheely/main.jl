include("../common/Common.jl")
include("./wheely.jl")
include("./streams.jl")

using HTTP.WebSockets, JSON, DotEnv, .Common
DotEnv.config()

pathfinder_server = WebSockets.open(ENV["PUB_SUB_URL"]) do ws
    
    ##########################################################
    #          Set up rxinfer engine and world               #
    ##########################################################

    (observations_stream, actions_stream) = create_streams()

    ready_for_next_observation = false
     
    calculate_action_actor = ((ang_vels, trans_vels, ps, free_energy),) -> begin
        ang_vel = mean(ang_vels[1])
        trans_vel = mean(trans_vels[1])
        p = mean(ps[1])
        predicted_ps = map(x -> mean(x), ps)

        println("free_energy: $free_energy")
        
        payload = (
            ang_vel = ang_vel,
            trans_vel = trans_vel,
        )
        next!(actions_stream, payload)
        ready_for_next_observation = true
    end
 
    engine = create_wheely_agent(observations_stream)

    subscribe!(
            engine.posteriors[:ang_vel_k]
            |> with_latest(
                engine.posteriors[:trans_vel_k],
                engine.posteriors[:p_k],
                engine.free_energy),
            calculate_action_actor
    )

    ########################################################
    #          Set up connections to pub-sub               #
    ########################################################

    sender = (msg) -> send(ws, msg)

    # Subscribe to relevant topics
    pubsub_sub("/sensory_states/position", sender)
    
    # Start publishing new events from datastreams
    subscribe!(actions_stream, pubsub_pub_actor("/rxinfer/wheely/action/diff_drive", sender))

    ready_for_next_observation = true
    for msg in ws
        json_msg = JSON.parse(msg)
        if (json_msg["type"] == "event")
            payload = json_msg["payload"]
            if (payload["topic"] == "/sensory_states/position")
                data = payload["data"]
                if (ready_for_next_observation)
                    ready_for_next_observation = false
                    next!(observations_stream, (p = [data["x"], data["y"]], o = data["yaw"],))
                end
            end
        end
    end
end