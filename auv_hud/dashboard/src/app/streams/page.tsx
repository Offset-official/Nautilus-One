"use client";
import { useEffect, useState, useRef } from "react";
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
  
  // State for the selected camera to view in full resolution
  const [selectedCamera, setSelectedCamera] = useState<string | null>(null);
  
  // Reference to the ROS connection to be used across functions
  const rosRef = useRef<ROSLIB.Ros | null>(null);

  useEffect(() => {
    // Connect to ROS bridge websocket server
    const ros = new ROSLIB.Ros({ url: "ws://localhost:9090" });
    rosRef.current = ros;

    ros.on("connection", () => console.log("Connected to ROS"));
    ros.on("error", (error) => console.error("ROS connection error:", error));
    ros.on("close", () => console.log("Connection to ROS bridge closed"));

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
      
      // Close the ROS connection
      if (rosRef.current && rosRef.current.isConnected) {
        rosRef.current.close();
      }
    };
  }, []);

  return (
    <div className="flex flex-col items-center justify-center min-h-screen bg-gray-900 text-white">
      <h1 className="text-3xl font-bold mb-4">Video Streams</h1>
      <div className="grid grid-cols-1 md:grid-cols-3 gap-4">
        {Object.entries(imageUrls).map(([topicName, imgSrc], index) => {
          // Extract the camera name from the topic path
          const cameraName = topicName.split('/')[1] || `Camera ${index + 1}`;
          
          return (
            <div key={index} className="p-4 bg-gray-800 rounded-lg shadow-lg">
              <h2 className="text-lg font-bold mb-2">{cameraName}</h2>
              <div className="relative">
                {imgSrc ? (
                  <Image
                    src={imgSrc}
                    alt={`Stream ${cameraName}`}
                    className="rounded-lg w-full"
                    width={640}
                    height={480}
                    unoptimized
                  />
                ) : (
                  <div className="w-full h-48 flex items-center justify-center bg-gray-700 rounded-lg">
                    <p>Waiting for image...</p>
                  </div>
                )}
                
                <div className="mt-2 flex flex-wrap gap-2">
                  <button
                    onClick={() => setSelectedCamera(topicName)}
                    className="px-4 py-2 bg-blue-600 text-white rounded hover:bg-blue-700 font-medium"
                  >
                    View Full Resolution
                  </button>
                </div>
              </div>
            </div>
          );
        })}
      </div>
      
      {/* Modal for full resolution view */}
      {selectedCamera && (
        <div 
          className="fixed inset-0 flex items-center justify-center bg-black bg-opacity-80 z-50"
          onClick={() => setSelectedCamera(null)}
        >
          <div 
            className="relative p-4 bg-gray-800 rounded-lg"
            onClick={(e) => e.stopPropagation()}
          >
            <button 
              onClick={() => setSelectedCamera(null)}
              className="absolute top-2 right-2 text-white text-xl"
            >
              &times;
            </button>
            {imageUrls[selectedCamera] ? (
              <img 
                src={imageUrls[selectedCamera]} 
                alt="Full Resolution" 
                className="max-w-full max-h-screen"
              />
            ) : (
              <p>Loading image...</p>
            )}
          </div>
        </div>
      )}
    </div>
  );
}