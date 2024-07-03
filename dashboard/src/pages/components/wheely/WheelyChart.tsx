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

const xMin = -30;
const xMax = 30;
const yMin = -30;
const yMax = 30;

const topics = ["/ros/wheely/sensory_states/position"];

type PositionMessage = {
  type: "event";
  payload: {
    topic: "/ros/wheely/sensory_states/position";
    data: { x: number; y: number; yaw: number };
  };
};

export const WheelyChart = () => {
  const [p, setP] = useState([0.0, 0.0]);
  const [connected, setConnected] = useState(false);
  const { lastJsonMessage, readyState, sendMessage } =
    usePubSub<PositionMessage>();

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
    ],
  };

  return (
    <Container>
      <ReactECharts option={option} style={{ height: 600, width: 600 }} />
    </Container>
  );
};
