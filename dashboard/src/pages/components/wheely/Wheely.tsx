import React from "react";
import { WheelyChart } from "./WheelyChart";
import styled from "styled-components";

const Container = styled.div`
  display: flex;
  flex-direction: row;
  flex-grow: 2;
  width: 100%;
  height: 100%;
`;

export const Wheely = () => {
  return (
    <Container>
      <WheelyChart />
    </Container>
  );
};
