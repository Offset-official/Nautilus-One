"use client";
import { useState, useEffect } from "react";
import Image from "next/image";
import ROSLIB from "roslib";

// Define CompressedImage message structure
interface CompressedImageMsg {
  format: string;
  data: string;
}

export default function Home() {
  const [velocity, setVelocity] = useState({ x: "0.00", y: "0.00", z: "0.00" });
  const [selectedCamera, setSelectedCamera] = useState("/usb_cam_0/image_raw/compressed");
  const [images, setImages] = useState<{ [key: string]: string }>({});

  // Define camera topics with custom aliases
  const cameraOptions = [
    { topic: "/usb_cam_0/image_raw/compressed", alias: "Front Camera" },
    { topic: "/usb_cam_1/image_raw/compressed", alias: "Rear Camera" },
    { topic: "/usb_cam_2/image_raw/compressed", alias: "Side Camera" },
  ];

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

    const subscribeToImage = (topic: string) => {
      const listener = new ROSLIB.Topic({
        ros,
        name: topic,
        messageType: "sensor_msgs/msg/CompressedImage",
      });

      listener.subscribe((message) => {
        const compressedImage = message as CompressedImageMsg; 
      
        if (!compressedImage.data) {
          console.error("Invalid image message:", compressedImage);
          return;
        }

        try {
          // Convert base64 string to binary data
          const byteCharacters = atob(compressedImage.data);
          const byteNumbers = Array.from(byteCharacters, (char) => char.charCodeAt(0));
          const byteArray = new Uint8Array(byteNumbers);
          const blob = new Blob([byteArray], { type: "image/jpeg" });

          const imageUrl = URL.createObjectURL(blob);
          setImages((prev) => ({ ...prev, [topic]: imageUrl }));
        } catch (err) {
          console.error("Failed to decode image data", err);
        }
      });

      return listener;
    };

    // Subscribe to the selected camera
    const imageListener = subscribeToImage(selectedCamera);

    return () => {
      velocityListener.unsubscribe();
      imageListener.unsubscribe();
    };
  }, [selectedCamera]);

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

      <div className="fixed w-96 bottom-3 left-3 bg-gray-800 text-white p-3 rounded-lg shadow-lg">
        <select
          className="w-full bg-gray-900 text-white p-2 rounded-lg mt-2"
          value={selectedCamera}
          onChange={(e) => setSelectedCamera(e.target.value)}
        >
          {cameraOptions.map(({ topic, alias }) => (
            <option key={topic} value={topic}>
              {alias}
            </option>
          ))}
        </select>
      </div>

      <div className="fixed bottom-16 left-3 w-96 h-60 bg-black flex items-center justify-center text-gray-500 rounded-lg shadow-lg">
        {images[selectedCamera] ? (
          <Image
          src={images[selectedCamera]}
          alt="Camera Feed"
          className="w-full h-full object-cover rounded-lg"
          width={640}  
          height={480}
          priority 
        />
        ) : (
          `Stream: ${cameraOptions.find((c) => c.topic === selectedCamera)?.alias || "Unknown"}`
        )}
      </div>
    </div>
  );
}
