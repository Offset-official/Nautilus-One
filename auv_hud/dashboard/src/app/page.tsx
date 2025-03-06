"use client";
import { useState, useEffect } from "react";
import Image from "next/image";
import ROSLIB from "roslib";

interface CompressedImageMsg {
  format: string;
  data: string;
}

export default function Home() {
  const [ros, setRos] = useState<ROSLIB.Ros | null>(null);
  const [velocity, setVelocity] = useState({ x: "0.00", y: "0.00", z: "0.00" });
  const [selectedCamera, setSelectedCamera] = useState("/auv_camera_front/image_raw/compressed");
  const [images, setImages] = useState<{ [key: string]: string }>({});
  const [recordingStatus, setRecordingStatus] = useState("Not recording");

  // Camera options (for image streaming)
  const cameraOptions = [
    { topic: "/auv_camera_front/image_raw/compressed", alias: "Front Jetson Camera" },
    { topic: "/auv_camera_down/image_raw/compressed", alias: "Down Jetson Camera" },
    { topic: "/usb_cam_2/image_raw/compressed", alias: "Side Camera" },
  ];

  // Initialize ROS connection once.
  useEffect(() => {
    const rosInstance = new ROSLIB.Ros({ url: "ws://localhost:9090" });
    setRos(rosInstance);
    rosInstance.on("connection", () => console.log("Connected to ROS"));
    rosInstance.on("error", (error: any) => console.error("Error connecting to ROS:", error));
    return () => {
      rosInstance.close();
    };
  }, []);

  // Subscribe to velocity and selected camera image topics.
  useEffect(() => {
    if (!ros) return;

    const velocityListener = new ROSLIB.Topic({
      ros,
      name: "/mavros/local_position/velocity_body",
      messageType: "geometry_msgs/msg/TwistStamped",
    });
    velocityListener.subscribe((message) => {
      const twistStampedMsg = message as {
        twist: { linear: { x: number; y: number; z: number } };
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

    const imageListener = subscribeToImage(selectedCamera);

    return () => {
      velocityListener.unsubscribe();
      imageListener.unsubscribe();
    };
  }, [selectedCamera, ros]);

  // Start recording: simply call the start_recording service.
  const startRecording = () => {
    if (!ros) {
      console.error("ROS not connected");
      return;
    }
    const startService = new ROSLIB.Service({
      ros,
      name: "start_recording",
      serviceType: "std_srvs/srv/Trigger",
    });
    const request = new ROSLIB.ServiceRequest({});
    startService.callService(request, (result: any) => {
      if (result.success) {
        setRecordingStatus("Recording started (all topics)");
      } else {
        setRecordingStatus("Failed to start recording: " + result.message);
      }
    });
  };

  // Stop recording: call the stop_recording service.
  const stopRecording = () => {
    if (!ros) {
      console.error("ROS not connected");
      return;
    }
    const stopService = new ROSLIB.Service({
      ros,
      name: "stop_recording",
      serviceType: "std_srvs/srv/Trigger",
    });
    const request = new ROSLIB.ServiceRequest({});
    stopService.callService(request, (result: any) => {
      if (result.success) {
        setRecordingStatus("Recording stopped");
      } else {
        setRecordingStatus("Failed to stop recording: " + result.message);
      }
    });
  };

  return (
    <div className="h-screen overflow-hidden flex flex-col bg-gray-900 text-white">
      {/* Velocity Display (Bottom Center) */}
      <div className="fixed bottom-3 left-1/2 -translate-x-1/2 bg-gray-800 p-3 rounded-lg shadow-lg w-full max-w-sm mx-auto">
        <h2 className="text-lg font-semibold text-center">Velocity</h2>
        <div className="flex justify-around">
          <p>X: {velocity.x} m/s</p>
          <p>Y: {velocity.y} m/s</p>
          <p>Z: {velocity.z} m/s</p>
        </div>
      </div>

      {/* Camera Selection (Bottom Left) */}
      <div className="fixed w-96 bottom-3 left-3 bg-gray-800 p-3 rounded-lg shadow-lg">
        <select
          className="w-full bg-gray-900 p-2 rounded-lg mt-2"
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

      {/* Camera Feed Display */}
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

      {/* Recording Control Panel (Bottom Right) */}
      <div className="fixed bottom-3 right-3 bg-gray-800 p-3 rounded-lg shadow-lg w-72">
        <div className="flex justify-between items-center">
          <h3 className="font-bold">Recording Control</h3>
          <div className="space-x-2">
            <button onClick={startRecording} className="bg-green-600 px-2 py-1 rounded">
              Start
            </button>
            <button onClick={stopRecording} className="bg-red-600 px-2 py-1 rounded">
              Stop
            </button>
          </div>
        </div>
        <p className="mt-2">Status: {recordingStatus}</p>
        <p className="text-xs mt-2">Recording all topics.</p>
      </div>
    </div>
  );
}

