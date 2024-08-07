include("../common/Common.jl")
include("./wheely.jl")
include("./streams.jl")

using HTTP.WebSockets, JSON, DotEnv, .Common
DotEnv.config()

pathfinder_server = WebSockets.open(ENV["PUB_SUB_URL"]) do ws
    
    ##########################################################
    #          Set up rxinfer engine and world               #
    ##########################################################

    (
        observations_stream, 
        actions_stream, 
        predictions_stream, 
        free_energy_stream
    ) = create_streams()

    ready_for_next_observation = false
     
    actions_actor = ((ang_vels, trans_vels),) -> begin
        ang_vel = mean(ang_vels[1])
        trans_vel = mean(trans_vels[1])

        payload = (
            ang_vel = ang_vel,
            trans_vel = trans_vel,
        )
        next!(actions_stream, payload)
        ready_for_next_observation = true
    end

    predictions_actor = (p_dists) -> begin
        ps = map(x -> mean(x), p_dists)
        
        payload = (
            ps = ps,
        )
        next!(predictions_stream, payload)
    end
 
    engine = create_wheely_agent(observations_stream)

    subscribe!(
            engine.posteriors[:ang_vel_k] |> with_latest(engine.posteriors[:trans_vel_k]),
            actions_actor
    )

    subscribe!(engine.posteriors[:p_k], predictions_actor)
    subscribe!(engine.free_energy, free_energy -> next!(free_energy_stream, (free_energy = free_energy, )))

    ########################################################
    #          Set up connections to pub-sub               #
    ########################################################

    sender = (msg) -> send(ws, msg)

    # Subscribe to relevant topics
    pubsub_sub("/ros/wheely/sensory_states/position", sender)
    pubsub_sub("/ros/wheely/food/position", sender)
    
    # Start publishing new events from datastreams
    subscribe!(actions_stream, pubsub_pub_actor("/rxinfer/wheely/action/diff_drive", sender))
    subscribe!(predictions_stream, pubsub_pub_actor("/rxinfer/wheely/predictions", sender))
    subscribe!(free_energy_stream, pubsub_pub_actor("/rxinfer/wheely/free_energy", sender))


    ########################################################
    #          Handle messages from pub-sub                #
    ########################################################

    ready_for_next_observation = true
    latest_food_p = [0.0, 0.0]
    latest_p = [0.0, 0.0]
    latest_o = 0.0
    for msg in ws
        json_msg = JSON.parse(msg)
        if (json_msg["type"] == "event")
            payload = json_msg["payload"]
            if (payload["topic"] == "/ros/wheely/sensory_states/position")
                data = payload["data"]
                latest_p = [data["x"], data["y"]]
                latest_o = data["yaw"]
            end

            if (payload["topic"] == "/ros/wheely/food/position")
                data = payload["data"]
                latest_food_p = [data["x"], data["y"]]
            end

            if (ready_for_next_observation)
                ready_for_next_observation = false
                observation = (food_p = latest_food_p, p = latest_p, o = latest_o,)
                println("observation: $(observation)")
                next!(observations_stream, observation)
            end
        end
    end
end