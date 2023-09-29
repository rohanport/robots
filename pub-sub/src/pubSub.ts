import { RosbridgeServer } from "./RosbridgeServer";

type SubscribeMessage = {
  type: "subscribe";
  payload: {
    topic: string;
  };
};

type UnsubscribeMessage = {
  type: "unsubscribe";
  payload: {
    topic: string;
  };
};

type EventMessage = {
  type: "event";
  payload: {
    topic: string;
    data: Record<string, unknown>;
  };
};

type Message = SubscribeMessage | UnsubscribeMessage | EventMessage;

export const startPubSubServer = (rosBridgeServer: RosbridgeServer) => {
  console.log("Starting pub-sub web-server");

  return Bun.serve<Message>({
    fetch(req, server) {
      // upgrade the request to a WebSocket
      if (server.upgrade(req)) {
        return; // do not return a Response
      }
      return new Response("Upgrade failed :(", { status: 500 });
    },
    websocket: {
      publishToSelf: true,
      message: (ws, messageJsonString) => {
        const message = JSON.parse(messageJsonString.toString());
        console.log("new message:", message);
        const { type } = message;
        const { topic } = message.payload;

        if (type === "subscribe") ws.subscribe(topic);
        if (type === "unsubscribe") ws.unsubscribe(topic);
        if (type === "event") {
          ws.publish(topic, JSON.stringify(message.payload.data));
          rosBridgeServer.publish(topic, message.payload.data); // Publish all events through to ROS
        }
      },
    },
  });
};
