"use client";
import { useEffect, useState } from "react";
import Image from "next/image";
import ROSLIB from "roslib";

// Define CompressedImage message structure
interface CompressedImageMsg {
  format: string;
  data: string;
}

export default function VideoStream() {
  const [imageUrls, setImageUrls] = useState<{ [key: string]: string }>({
    "/auv_camera_down/image_raw/compressed": "",
    "/auv_camera_front/image_raw/compressed": "",
    "/usb_cam_2/image_raw/compressed": "",
  });

  useEffect(() => {
    const ros = new ROSLIB.Ros({ url: "ws://localhost:9090" });

    ros.on("connection", () => console.log("Connected to ROS"));
    ros.on("error", (error) => console.error("ROS connection error:", error));

    const createSubscriber = (topicName: string) => {
      const listener = new ROSLIB.Topic({
        ros,
        name: topicName,
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
          const byteNumbers = new Array(byteCharacters.length)
            .fill(null)
            .map((_, i) => byteCharacters.charCodeAt(i));
          const byteArray = new Uint8Array(byteNumbers);
          const blob = new Blob([byteArray], { type: "image/jpeg" });
      
          const imageUrl = URL.createObjectURL(blob);
          setImageUrls((prev) => ({ ...prev, [topicName]: imageUrl }));
        } catch (error) {
          console.error(`Failed to process image from ${topicName}:`, error);
        }
      });
      

      return listener;
    };

    // Create subscribers for all three cameras
    const subscribers = [
      createSubscriber("/auv_camera_down/image_raw/compressed"),
      createSubscriber("/auv_camera_front/image_raw/compressed"),
      createSubscriber("/usb_cam_2/image_raw/compressed"),
    ];

    return () => {
      subscribers.forEach((sub) => sub.unsubscribe());
      console.log("Unsubscribed from ROS topics");
    };
  }, []);

  return (
    <div className="flex flex-col items-center justify-center min-h-screen bg-gray-900 text-white">
      <h1 className="text-3xl font-bold mb-4">Video Streams</h1>
      <div className="grid grid-cols-1 md:grid-cols-3 gap-4">
        {Object.entries(imageUrls).map(([, imgSrc], index) => (
          <div key={index} className="p-4 bg-gray-800 rounded-lg shadow-lg">
            <h2 className="text-lg font-bold mb-2">Camera {index}</h2>
            {imgSrc ? (
              <Image
                src={imgSrc}
                alt={`Stream ${index}`}
                className="rounded-lg w-full"
                width={640}
                height={480}
                unoptimized
              />
            ) : (
              <p>Waiting for image...</p>
            )}
          </div>
        ))}
      </div>
    </div>
  );
}
