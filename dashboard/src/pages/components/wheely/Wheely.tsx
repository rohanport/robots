import React, { useState } from "react";
import { WheelyChart } from "./WheelyChart";
import styled from "styled-components";
import { Button } from "@mui/material";
import { usePubSub } from "@/pages/api";
import { ReadyState } from "react-use-websocket";
import { pubSubPublishEvent } from "@/pages/api/usePubSub";

const Container = styled.div`
  display: flex;
  flex-direction: row;
  flex-grow: 2;
  width: 100%;
  height: 100%;
`;

export const Wheely = () => {
  const { readyState, sendMessage } = usePubSub();

  const [paused, setPaused] = useState(true);

  const togglePause = () => {
    setPaused(!paused);
    pubSubPublishEvent(
      "/dashboard/wheely/pause",
      { pause: !paused },
      sendMessage
    );
  };

  if (readyState !== ReadyState.OPEN) {
    return null;
  }

  return (
    <Container>
      <Button onClick={togglePause}>{paused ? "Start" : "Stop"}</Button>
      <WheelyChart />
    </Container>
  );
};
