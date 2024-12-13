import React, { useEffect, useState } from "react";
import styled from "styled-components";
import ReactECharts from "echarts-for-react";
import { usePubSub } from "@/pages/api";

const Container = styled.div`
  flex-grow: 1;
  display: flex;
  justify-content: center;
  height: 100%;
  width: 100%;
  aspect-ratio: 1 / 1;
`;


const freeEnergyHistoryLength = 50;

const topics = [
    "/rxinfer/wheely/free_energy",
  ];

type FreeEnergyMessage = {
    type: "event";
    payload: {
      topic: "/rxinfer/wheely/free_energy";
      data: { free_energy: number };
    };
  };

export const FreeEnergyChart = () => {
    const [latestFreeEnergy, setLatestFreeEnergy] = useState<number>(0);
    const [freeEnergyHistory, setFreeEnergyHistory] = useState<number[]>(new Array(freeEnergyHistoryLength).map(() => 0));

    usePubSub(
        topics,
        (msg:  FreeEnergyMessage) => {
          const { payload } = msg;
          const { topic, data } = payload;
          if (topic === "/rxinfer/wheely/free_energy") {
            setLatestFreeEnergy(data.free_energy);
          }
        }
      );

      useEffect(() => {
        //Implementing the setInterval method
        const interval = setInterval(() => {
            setFreeEnergyHistory(freeEnergyHistory.concat(latestFreeEnergy).slice(1));
        }, 500);

        //Clearing the interval
        return () => clearInterval(interval);
    }, [freeEnergyHistory]);


  const data = freeEnergyHistory.map((val, index) => [
    index + 1 - freeEnergyHistory.length,
    val,
  ]);
  const option = {
    title: { text: 'Free Energy', textStyle: { color: "white" } },
    xAxis: { },
    yAxis: { min: Math.min(0, ...freeEnergyHistory) },
    series: [{ data, symbolSize: 0, type: "line", smooth: true }],
  };

  return <Container><ReactECharts option={option} style={{ height: 600, width: 600 }}/></Container> ;
};
