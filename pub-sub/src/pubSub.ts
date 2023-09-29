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

const allTopics = new Set<string>();

export const startPubSubServer = () => {
  console.log("Starting pub-sub web-server");

  Bun.serve<Message>({
    fetch(req, server) {
      // upgrade the request to a WebSocket
      if (server.upgrade(req)) {
        return; // do not return a Response
      }
      return new Response("Upgrade failed :(", { status: 500 });
    },
    websocket: {
      message: (ws) => {
        const { type } = ws.data;
        const { topic } = ws.data.payload;
        allTopics.add(topic);

        if (type === "subscribe") ws.subscribe(topic);
        if (type === "unsubscribe") ws.unsubscribe(topic);
        if (type === "event")
          ws.publish(topic, JSON.stringify(ws.data.payload.data));
      },
      close: (ws) => {
        allTopics.forEach((topic) => ws.unsubscribe(topic));
      },
    },
  });
};
