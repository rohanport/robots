import { RosbridgeServer } from "./RosbridgeServer";
import { startPubSubServer } from "./pubSub";

const rosBridgeServer = new RosbridgeServer();
const pubSubServer = startPubSubServer(rosBridgeServer);
rosBridgeServer.linkToPubSub(pubSubServer);
