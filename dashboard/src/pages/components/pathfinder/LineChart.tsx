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
  title: string;
  dataPoints: number[];
};

export const LineChart = ({ title, dataPoints }: Props) => {
  const data = dataPoints.map((val, index) => [
    index + 1 - dataPoints.length,
    val,
  ]);
  const option = {
    title: { text: title, textStyle: { color: "white" } },
    xAxis: {},
    yAxis: { min: Math.min(0, ...dataPoints) },
    series: [{ data, symbolSize: 0, type: "line", smooth: true }],
  };

  return <StyledReactECharts option={option} />;
};
