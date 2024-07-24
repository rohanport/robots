import { usePubSub } from "../api";

export const RosLogger = () => {
  usePubSub<{
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
  }>(["/chatter"], (msg: unknown) => console.log(msg));

  return <></>;
};
