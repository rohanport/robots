import { ReactNode, useState } from "react";
import Button from "@mui/material/Button";
import { Pathfinder, RosLogger } from "./components";
import styled from "styled-components";

const VIEWS = {
  viewPicker: "viewPicker",
  pathFinder: "pathFinder",
  rosLogger: "rosLogger",
};

const ViewPickerContainer = styled.div`
  display: flex;
`;

const ViewContainer = styled.div`
  display: flex;
  flex-direction: column;
`;

const ViewWrapper = ({
  children,
  onBack,
}: {
  children: ReactNode | ReactNode[];
  onBack: () => void;
}) => {
  return (
    <ViewContainer>
      <Button onClick={onBack}>Back</Button>
      <>{children}</>
    </ViewContainer>
  );
};

export const Main = () => {
  const [view, setView] = useState(VIEWS.viewPicker);
  const onBack = () => setView(VIEWS.viewPicker);

  if (view === VIEWS.pathFinder) {
    return (
      <ViewWrapper onBack={onBack}>
        <Pathfinder />
      </ViewWrapper>
    );
  } else if (view === VIEWS.rosLogger) {
    return (
      <ViewWrapper onBack={onBack}>
        <RosLogger />
      </ViewWrapper>
    );
  } else {
    return (
      <ViewPickerContainer>
        <Button onClick={() => setView(VIEWS.pathFinder)}>Pathfinder</Button>
        <Button onClick={() => setView(VIEWS.rosLogger)}>RosLogger</Button>
      </ViewPickerContainer>
    );
  }
};
