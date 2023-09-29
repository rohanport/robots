import React, { useCallback, useEffect, useState } from "react";
import { ReadyState } from "react-use-websocket";
import { PathfinderChart } from "./PathfinderChart";
import { LineChart } from "./LineChart";
import { HungerChart } from "./HungerChart";
import { pubSubSubscribe, pubSubUnsubscribe, usePubSub } from "@/pages/api";

const freeEnergyHistoryLength = 50;

const topics = ["rxinfer_pathfinder_state", "rxinfer_pathfinder_action"];

export const Pathfinder = () => {
  const [foodP, setFoodP] = useState([0.0, 0.0]);
  const [p, setP] = useState([0.0, 0.0]);
  const [hunger, setHunger] = useState(0.0);
  const [predictedPs, setPredictedPs] = useState<number[][]>([]);
  const [predictedHungers, setPredictedHungers] = useState<number[]>([]);
  const [predictedFoodP, setPredictedFoodP] = useState<number[]>([0.0, 0.0]);
  const [freeEnergyHistory, setFreeEnergyHistory] = useState(
    new Array(freeEnergyHistoryLength).fill(0)
  );
  const [randomnessHistory, setRandomnessHistory] = useState(
    new Array(freeEnergyHistoryLength).fill(0)
  );
  const [velocityHistory, setVelocityHistory] = useState(
    new Array(freeEnergyHistoryLength).fill(0)
  );
  const [connected, setConnected] = useState(false);
  const [paused, setPaused] = useState(false);

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

  useEffect(() => {
    if (lastJsonMessage === null) return;
    const {
      food_p: newFoodP,
      p: newP,
      predicted_ps: newPredictedPs,
      hunger: newHunger,
      predicted_hungers: newPredictedHungers,
      predicted_food_p: newPredictedFoodP,
      free_energy: freeEnergy,
      velocity,
      randomness,
    } = lastJsonMessage;
    if (newFoodP !== undefined) setFoodP(newFoodP);
    if (newP !== undefined) setP(newP);
    if (newPredictedPs !== undefined) setPredictedPs(newPredictedPs);
    if (newHunger !== undefined) setHunger(newHunger);
    if (newPredictedHungers !== undefined)
      setPredictedHungers(newPredictedHungers);
    if (newPredictedFoodP !== undefined) setPredictedFoodP(newPredictedFoodP);
    if (freeEnergy !== undefined)
      setFreeEnergyHistory(freeEnergyHistory.slice(1).concat([freeEnergy]));
    if (velocity !== undefined)
      setVelocityHistory(velocityHistory.slice(1).concat([velocity]));
    if (randomness !== undefined)
      setRandomnessHistory(randomnessHistory.slice(1).concat([randomness]));
  }, [lastJsonMessage]);

  const subscribeToTopics = () =>
    topics.forEach((topic) => pubSubSubscribe(topic, sendMessage));
  const unsubscribeFromTopics = () =>
    topics.forEach((topic) => pubSubUnsubscribe(topic, sendMessage));

  const pauseFn = useCallback(
    (event: KeyboardEvent) => {
      if (event.code !== "KeyP") return;

      if (paused) {
        subscribeToTopics();
      } else {
        unsubscribeFromTopics();
      }
      setPaused(!paused);
    },
    [sendMessage, paused, setPaused]
  );

  useEffect(() => {
    document.addEventListener("keydown", pauseFn, false);

    return () => {
      document.removeEventListener("keydown", pauseFn, false);
    };
  }, [pauseFn]);

  useEffect(() => {
    if (!connected && readyState === ReadyState.OPEN) {
      subscribeToTopics();
      setConnected(true);
    }
  }, [connected, readyState, sendMessage, setConnected]);

  if (readyState !== ReadyState.OPEN) {
    return null;
  }

  return (
    <>
      <div
        style={{
          display: "flex",
          flexDirection: "row",
          flexGrow: 2,
          width: "100%",
          height: "100%",
        }}
      >
        <PathfinderChart
          foodP={foodP}
          p={p}
          predictedPs={predictedPs}
          predictedFoodP={predictedFoodP}
        />
        <div
          style={{
            display: "flex",
            flexDirection: "column",
            width: "100%",
            flexGrow: 1,
          }}
        >
          <div style={{ display: "flex", flexDirection: "row", width: "100%" }}>
            <HungerChart hunger={hunger} predictedHungers={predictedHungers} />
            <LineChart title="Velocity" dataPoints={velocityHistory} />
          </div>
          <div style={{ display: "flex", flexDirection: "row", width: "100%" }}>
            <LineChart title="Free Energy" dataPoints={freeEnergyHistory} />
            <LineChart title="Randomness" dataPoints={randomnessHistory} />
          </div>
        </div>
      </div>
    </>
  );
};
