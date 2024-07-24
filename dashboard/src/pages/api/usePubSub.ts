import { useEffect, useState } from "react";
import useWebSocket, { ReadyState } from "react-use-websocket";

export const usePubSub = <Messages>(
  topics: string[] = [],
  messageHandler: (m: Messages) => void = (m: unknown) => {}
) => {
  const pubSubUrl = process.env.NEXT_PUBLIC_PUB_SUB_URL;
  if (!pubSubUrl) {
    throw new Error("Missing .env var NEXT_PUBLIC_PUB_SUB_URL");
  }

  const [connected, setConnected] = useState(false);
  const { lastJsonMessage, readyState, sendMessage } =
    useWebSocket<Messages>(pubSubUrl);

  useEffect(() => {
    if (!connected && readyState === ReadyState.OPEN) {
      topics.forEach((topic) => pubSubSubscribe(topic, sendMessage));
      setConnected(true);
    }
  }, [connected, readyState, topics, setConnected, sendMessage]);

  useEffect(() => {
    if (lastJsonMessage === null) return;

    messageHandler(lastJsonMessage);
  }, [lastJsonMessage, messageHandler]);

  return { sendMessage, readyState };
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

export const pubSubPublishEvent = (
  topic: string,
  data: Record<string, unknown>,
  sendMessage: (msg: string) => void
) => {
  const eventMessage = {
    type: "event",
    payload: {
      topic,
      data,
    },
  };

  sendMessage(JSON.stringify(eventMessage));
};
