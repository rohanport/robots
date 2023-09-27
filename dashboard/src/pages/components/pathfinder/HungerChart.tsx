import React from "react";
import styled from "styled-components";
import ReactECharts from "echarts-for-react";

const StyledReactECharts = styled(ReactECharts)`
  flex-grow: 1;
  display: flex;
  height: 100%;
  width: 100%;
`;

type Props = {
  hunger: number;
  predictedHungers: number[];
};

export const HungerChart = ({ hunger, predictedHungers }: Props) => {
  const option = {
    title: { text: "Hunger", textStyle: { color: "white" } },
    xAxis: {
      type: "category",
      data: [0].concat(predictedHungers.map((_, i) => i + 1)),
    },
    yAxis: {
      min: Math.min(0, ...predictedHungers),
      max: Math.max(20, ...predictedHungers),
    },
    series: [
      { data: [hunger, ...predictedHungers], type: "bar", smooth: true },
    ],
  };

  return <StyledReactECharts option={option} />;
};
