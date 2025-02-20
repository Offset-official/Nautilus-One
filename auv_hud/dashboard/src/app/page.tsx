"use client";
import { useState, useEffect } from "react";
import ROSLIB from "roslib";

export default function Home() {
  const [velocity, setVelocity] = useState({ x: "0.00", y: "0.00", z: "0.00" });

  useEffect(() => {
    const ros = new ROSLIB.Ros({ url: "ws://localhost:9090" });

    ros.on("connection", () => console.log("Connected to ROS"));

    const velocityListener = new ROSLIB.Topic({
      ros,
      name: "/mavros/local_position/velocity_body",
      messageType: "geometry_msgs/msg/TwistStamped",
    });

    velocityListener.subscribe((message) => {
      const twistStampedMsg = message as {
        twist: {
          linear: { x: number; y: number; z: number };
          angular: { x: number; y: number; z: number };
        };
      };

      setVelocity({
        x: twistStampedMsg.twist.linear.x.toFixed(2),
        y: twistStampedMsg.twist.linear.y.toFixed(2),
        z: twistStampedMsg.twist.linear.z.toFixed(2),
      });
    });

    return () => {
      velocityListener.unsubscribe();
    };
  }, []);

  return (
    <div className="h-screen overflow-hidden flex flex-col bg-gray-900 text-white">
      <div className="fixed bottom-3 left-1/2 -translate-x-1/2 bg-gray-800 text-white p-3 rounded-lg shadow-lg w-full max-w-sm mx-auto">
        <h2 className="text-lg font-semibold text-center">Velocity</h2>
        <div className="flex justify-around">
          <p>X: {velocity.x} m/s</p>
          <p>Y: {velocity.y} m/s</p>
          <p>Z: {velocity.z} m/s</p>
        </div>
      </div>
    </div>
  );
}
