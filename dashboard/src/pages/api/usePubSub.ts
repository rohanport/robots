import useWebSocket from "react-use-websocket";

export const usePubSub = <Messages>() => {
  const pubSubUrl = process.env.NEXT_PUBLIC_PUB_SUB_URL;
  if (!pubSubUrl) {
    throw new Error("Missing .env var NEXT_PUBLIC_PUB_SUB_URL");
  }

  return useWebSocket<Messages>(pubSubUrl);
};

export const pubSubSubscribe = (
  topic: string,
  sendMessage: (msg: string) => void
) => {
  const subscriptionMessage = {
    type: "subscribe",
    payload: {
      topic,
    },
  };
  sendMessage(JSON.stringify(subscriptionMessage));
};

export const pubSubUnsubscribe = (
  topic: string,
  sendMessage: (msg: string) => void
) => {
  const subscriptionMessage = {
    type: "unsubscribe",
    payload: {
      topic,
    },
  };
  sendMessage(JSON.stringify(subscriptionMessage));
};
