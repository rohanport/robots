import { Server } from "bun";
import { Ros, Topic } from "roslib";

export class RosbridgeServer extends Ros {
  private pubSubServer: Server | null;
  private readonly topics: Record<string, Topic>;
  private topicsPoll: Timer | null;

  public constructor() {
    super({ url: "ws://localhost:9090", transportLibrary: "websocket" });
    this.pubSubServer = null;
    this.topics = {};
    this.topicsPoll = null;

    this.on("connection", () => {
      console.log("Connected to rosbridge_server.");
      this.topicsPoll = setInterval(() => this.updateTopics(), 1000);
    });
    this.on("error", () => {
      console.log("Error connecting to rosbridge_server.");
      if (this.topicsPoll) clearInterval(this.topicsPoll);
    });
    this.on("close", () => {
      console.log("Closed connected to rosbridge_server.");
      if (this.topicsPoll) clearInterval(this.topicsPoll);
    });
  }

  /**
   * Forward all events that come through RosBridgeServer to pub-sub
   */
  public linkToPubSub(pubSub: Server) {
    this.pubSubServer = pubSub;
  }

  private forwardToPubSub(topic: string, message: Record<string, unknown>) {
    const pubSubServer = this.pubSubServer;
    if (!pubSubServer) return;

    pubSubServer.publish(
      topic,
      JSON.stringify({ type: "event", payload: { topic, data: message } })
    );
  }

  private subscribeToTopic(name: string, messageType: string) {
    if (this.topics[name]) return;

    console.log("subscribing to", name);

    const topic = new Topic<Record<string, unknown>>({
      ros: this,
      name,
      messageType,
    });
    this.topics[name] = topic;
    topic.subscribe((message) => this.forwardToPubSub(name, message));
  }

  /**
   * Fetch all topics in ROS and subscribe to them
   */
  private updateTopics() {
    this.getTopics(({ topics, types }) => {
      topics.forEach((topicName, i) => {
        this.subscribeToTopic(topicName, types[i]);
      });
    });
  }

  /**
   * Publish message to ROS
   */
  public publish(topicName: string, data: Record<string, unknown>) {
    const topic = this.topics[topicName];
    if (!topic) return;

    topic.publish(data);
  }
}
