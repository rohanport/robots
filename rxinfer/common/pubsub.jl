using JSON

# Subscribe to a topic
function pubsub_sub(topic::String, sender::Function)
    sender(JSON.json(Dict([
        ("type", "subscribe"),
        ("payload", Dict("topic"=>topic))
    ])))
end

# Buids a lambda function that sends events to pub-subscribe
function pubsub_pub_actor(topic::String, sender::Function)
    return (event_data) -> begin
        data = Dict(pairs(event_data))
        payload = Dict(
            "topic"=>topic,
            "data"=>data
        )
        event = JSON.json(Dict(
            "type"=>"event",
            "payload"=>payload
        ))
        sender(event)
    end
end