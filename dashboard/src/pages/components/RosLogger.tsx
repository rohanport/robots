import { useEffect } from "react";
import { pubSubSubscribe, usePubSub } from "../api";
import { ReadyState } from "react-use-websocket";

export const RosLogger = () => {
  const { lastJsonMessage, readyState, sendMessage } = usePubSub<{
    food_p?: number[];
    p?: number[];
    predicted_ps?: number[][];
    action?: number[];
    hunger?: number;
    predicted_hungers?: number[];
    predicted_food_p?: number[];
    free_energy?: number;
    velocity?: number;
    randomness?: number;
  }>();

  useEffect(() => console.log(lastJsonMessage), [lastJsonMessage]);

  useEffect(() => {
    if (readyState === ReadyState.OPEN) {
      pubSubSubscribe("/chatter", sendMessage);
    }
  }, [readyState, sendMessage]);

  return <></>;
};
