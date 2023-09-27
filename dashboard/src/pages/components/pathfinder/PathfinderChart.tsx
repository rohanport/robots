import React from "react";
import styled from "styled-components";
import ReactECharts from "echarts-for-react";

const Container = styled.div`
  flex-grow: 1;
  display: flex;
  justify-content: center;
  height: 100%;
  width: 100%;
  aspect-ratio: 1 / 1;
`;

type Props = {
  foodP: number[];
  p: number[];
  predictedPs: number[][];
  predictedFoodP: number[];
};
const xMin = -5;
const xMax = 25;
const yMin = -5;
const yMax = 25;

const foodPRadiusPoints = (foodLoc: number[], k = 50) => {
  return new Array(k + 1)
    .fill(0)
    .map((_, i) => [
      foodLoc[0] + 3.0 * Math.cos((2 * i * Math.PI) / k),
      foodLoc[1] + 3.0 * Math.sin((2 * i * Math.PI) / k),
    ]);
};

export const PathfinderChart = ({
  p,
  foodP,
  predictedPs = [],
  predictedFoodP,
}: Props) => {
  const option = {
    title: { text: "Agent", textStyle: { color: "white" } },
    xAxis: {
      min: Math.min(xMin, p[0], foodP[0], ...predictedPs.map((pp) => pp[0])),
      max: Math.max(xMax, p[0], foodP[0], ...predictedPs.map((pp) => pp[0])),
    },
    yAxis: {
      min: Math.min(yMin, p[1], foodP[1], ...predictedPs.map((pp) => pp[1])),
      max: Math.max(yMax, p[1], foodP[1], ...predictedPs.map((pp) => pp[1])),
    },
    series: [
      {
        symbolSize: 0,
        data: foodPRadiusPoints(foodP),
        type: "line",
        color: "red",
      },
      {
        symbolSize: 20,
        data: [foodP],
        type: "scatter",
        color: "red",
      },
      {
        symbolSize: 20,
        data: [predictedFoodP],
        type: "scatter",
        color: "yellow",
      },
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
