"use client";
import { useState, useEffect } from "react";
import ROSLIB from "roslib";
import Link from "next/link";


// Type Definitions
interface MavrosState {
    armed: boolean;
}

export default function Navbar() {
  const [armed, setArmed] = useState<boolean | null>(null);
  const ros = new ROSLIB.Ros({ url: "ws://localhost:9090" });

  useEffect(() => {
    ros.on("connection", () => console.log("Connected to ROS"));

    // Subscribe to /mavros/state
    const listener = new ROSLIB.Topic<MavrosState>({
      ros: ros,
      name: "/mavros/state",
      messageType: "mavros_msgs/msg/State",
    });

    listener.subscribe((message) => {
      setArmed(message.armed); // Update toggle state
    });

    return () => listener.unsubscribe();
  }, []);

  // Function to send arm/disarm command
  const toggleArmStatus = () => {
    if (armed === null) return; // Prevent sending command before connected

    const armCommand = new ROSLIB.Service({
      ros: ros,
      name: "/mavros/cmd/arming",
      serviceType: "mavros_msgs/srv/CommandBool",
    });

    const request = new ROSLIB.ServiceRequest({
      value: !armed, // Toggle between arm and disarm
    });

    armCommand.callService(request, (result) => {
      if (result.success) {
        console.log(`Successfully ${!armed ? "Armed" : "Disarmed"}`);
        setArmed(!armed); // Update UI instantly
      } else {
        console.error("Failed to change arm status");
      }
    });
  };

  return (
    <nav className="w-full h-16 bg-gray-800 flex items-center justify-between px-6 shadow-md fixed top-0 left-0">
      <h1 className="text-white text-xl font-bold">Nautilus One</h1>

      {/* Arm/Disarm Toggle Button */}
      <div className="flex items-center space-x-4">
        <span className="text-white">Status:</span>
        <button
          onClick={toggleArmStatus}
          className={`w-16 h-8 rounded-full flex items-center p-1 transition duration-300 ${
            armed ? "bg-green-500 justify-end" : "bg-red-500 justify-start"
          }`}
        >
          <div className="w-6 h-6 bg-white rounded-full shadow-md"></div>
        </button>
        <div className="flex space-x-4">
            <Link href="/" className="text-white">Home</Link>
            <Link href="/streams" className="text-white">Streams</Link>
            <Link href="/params" className="text-white">Params</Link>
        </div>
      </div>
    </nav>
  );
}
