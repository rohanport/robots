import React, { useEffect, useState } from "react";
import styled from "styled-components";
import ReactECharts from "echarts-for-react";
import { usePubSub, pubSubSubscribe } from "@/pages/api";
import { ReadyState } from "react-use-websocket";

const Container = styled.div`
  flex-grow: 1;
  display: flex;
  justify-content: center;
  height: 100%;
  width: 100%;
  aspect-ratio: 1 / 1;
`;

const xMin = -15;
const xMax = 15;
const yMin = -15;
const yMax = 15;

const topics = [
  "/ros/wheely/sensory_states/position",
  "/rxinfer/wheely/predictions",
];

type PositionMessage = {
  type: "event";
  payload: {
    topic: "/ros/wheely/sensory_states/position";
    data: { x: number; y: number; yaw: number };
  };
};

type PredictionsMessage = {
  type: "event";
  payload: {
    topic: "/rxinfer/wheely/predictions";
    data: { ps: [number, number][] };
  };
};

export const WheelyChart = () => {
  const [p, setP] = useState([0.0, 0.0]);
  const [predictedPs, setPredictedPs] = useState<[number, number][]>([]);

  const [connected, setConnected] = useState(false);
  const { lastJsonMessage, readyState, sendMessage } = usePubSub<
    PositionMessage | PredictionsMessage
  >();

  const subscribeToTopics = () =>
    topics.forEach((topic) => pubSubSubscribe(topic, sendMessage));

  useEffect(() => {
    if (!connected && readyState === ReadyState.OPEN) {
      subscribeToTopics();
      setConnected(true);
    }
  }, [connected, readyState, sendMessage, setConnected]);

  useEffect(() => {
    if (lastJsonMessage === null) return;

    const { payload } = lastJsonMessage;
    const { topic, data } = payload;
    if (topic === "/ros/wheely/sensory_states/position") {
      setP([data.x, data.y]);
    }
    if (topic === "/rxinfer/wheely/predictions") {
      setPredictedPs(data.ps);
    }
  }, [lastJsonMessage]);

  const option = {
    title: { text: "Agent", textStyle: { color: "white" } },
    xAxis: {
      min: xMin,
      max: xMax,
    },
    yAxis: {
      min: yMin,
      max: yMax,
    },
    series: [
      {
        symbolSize: 30,
        data: [p],
        type: "scatter",
        color: "#5e71c0",
      },
      ...predictedPs.map((loc, index) => ({
        type: "scatter",
        symbolSize: 15,
        data: [loc],
        color: `rgb(124, 252, 0, ${0.9 - index / predictedPs.length})`,
      })),
    ],
  };

  return (
    <Container>
      <ReactECharts option={option} style={{ height: 600, width: 600 }} />
    </Container>
  );
};
