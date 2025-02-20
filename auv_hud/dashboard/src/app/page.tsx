"use client";
import { useState, useEffect } from "react";
import ROSLIB from "roslib";

// Define the type for the MAVROS state message
interface MavrosState {
  mode: string;
  armed: boolean;
  connected: boolean;
}

export default function Home() {
  const [rosData, setRosData] = useState("Waiting...");

  useEffect(() => {
    const ros = new ROSLIB.Ros({
      url: "ws://localhost:9090",
    });

    ros.on("connection", () => console.log("Connected to ROS"));

    const listener = new ROSLIB.Topic<MavrosState>({
      ros: ros,
      name: "/mavros/state",
      messageType: "mavros_msgs/msg/State",
    });
    
    listener.subscribe((message) => {
      setRosData(`Armed: ${message.armed}`);
    });    

    return () => listener.unsubscribe();
  }, []);

  return (
    <div className="flex flex-col items-center justify-center min-h-screen bg-gray-900 text-white">
      <h1 className="text-3xl font-bold mb-4">ROS Dashboard</h1>
      <div className="bg-gray-800 p-4 rounded-lg shadow-lg">
        <p className="text-lg">ROS Data: {rosData}</p>
      </div>
    </div>
  );
}
