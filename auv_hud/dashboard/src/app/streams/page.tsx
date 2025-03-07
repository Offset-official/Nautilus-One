"use client";
import { useEffect, useState, useRef } from "react";
import Image from "next/image";
import ROSLIB from "roslib";

// Define CompressedImage message structure
interface CompressedImageMsg {
  format: string;
  data: string;
}

// Define the structure for each recorded frame including timestamp
interface FrameData {
  data: Uint8Array;
  timestamp: number;
}

export default function VideoStream() {
  const [imageUrls, setImageUrls] = useState<{ [key: string]: string }>({
    "/auv_camera_down/image_raw/compressed": "",
    "/auv_camera_front/image_raw/compressed": "",
    "/usb_cam_2/image_raw/compressed": "",
  });
  
  // State to track recording status for each camera
  const [recording, setRecording] = useState<{ [key: string]: boolean }>({
    "/auv_camera_down/image_raw/compressed": false,
    "/auv_camera_front/image_raw/compressed": false,
    "/usb_cam_2/image_raw/compressed": false,
  });
  
  // Refs to store frame data (with timestamps) for recording
  const frameBuffers = useRef<{ [key: string]: FrameData[] }>({
    "/auv_camera_down/image_raw/compressed": [],
    "/auv_camera_front/image_raw/compressed": [],
    "/usb_cam_2/image_raw/compressed": [],
  });
  
  // Refs for MediaRecorder objects
  const mediaRecorders = useRef<{ [key: string]: MediaRecorder | null }>({
    "/auv_camera_down/image_raw/compressed": null,
    "/auv_camera_front/image_raw/compressed": null,
    "/usb_cam_2/image_raw/compressed": null,
  });
  
  // Ref to store the most recent raw image data (not used for saving now)
  const rawImageData = useRef<{ [key: string]: Uint8Array | null }>({
    "/auv_camera_down/image_raw/compressed": null,
    "/auv_camera_front/image_raw/compressed": null,
    "/usb_cam_2/image_raw/compressed": null,
  });

  // State for the selected camera to view in full resolution
  const [selectedCamera, setSelectedCamera] = useState<string | null>(null);

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
          
          // Update the latest frame data
          rawImageData.current[topicName] = byteArray;
          
          // If recording, add the frame along with the timestamp
          if (recording[topicName]) {
            frameBuffers.current[topicName].push({ data: byteArray, timestamp: Date.now() });
          }
          
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
      
      // Clean up any ongoing recordings
      Object.entries(mediaRecorders.current).forEach(([topicName, recorder]) => {
        if (recorder && recorder.state !== "inactive") {
          recorder.stop();
        }
      });
    };
  }, [recording]);

  // Start recording for a specific camera
  const startRecording = (topicName: string) => {
    // Clear previous frames
    frameBuffers.current[topicName] = [];
    
    setRecording((prev) => ({
      ...prev,
      [topicName]: true
    }));
    
    console.log(`Started recording for ${topicName}`);
  };

  // Stop recording and save video
  const stopRecording = (topicName: string) => {
    setRecording((prev) => ({
      ...prev,
      [topicName]: false
    }));
    
    const frames = frameBuffers.current[topicName];
    if (frames.length === 0) {
      console.warn("No frames captured for recording");
      return;
    }
    
    // Create a video from captured frames
    const chunks: Blob[] = [];
    const canvas = document.createElement('canvas');
    const ctx = canvas.getContext('2d');
    
    if (!ctx) {
      console.error("Could not get canvas context");
      return;
    }
    
    // Set canvas size based on first frame (assuming all frames are the same size)
    const firstFrameBlob = new Blob([frames[0].data], { type: 'image/jpeg' });
    const imgForSize = new window.Image();
    
    imgForSize.onload = () => {
      canvas.width = imgForSize.width;
      canvas.height = imgForSize.height;
      
      // Start recording with MediaRecorder using the canvas stream at 30fps
      const stream = canvas.captureStream(30);
      const mediaRecorder = new MediaRecorder(stream, { mimeType: 'video/webm; codecs=vp9' });
      
      mediaRecorder.ondataavailable = (e) => {
        if (e.data.size > 0) {
          chunks.push(e.data);
        }
      };
      
      // Optimized file saving logic for long recordings
      mediaRecorder.onstop = () => {
        (async () => {
          const blob = new Blob(chunks, { type: 'video/webm' });
          const topicParts = topicName.split('/');
          const fileName = `${topicParts[topicParts.length - 3]}_recording_${new Date().toISOString()}.webm`;
          
          // If the File System Access API is available, use it to prompt a save dialog.
          if (window.showSaveFilePicker) {
            try {
              const options = {
                suggestedName: fileName,
                types: [{
                  description: 'WebM Video',
                  accept: { 'video/webm': ['.webm'] },
                }],
              };
              const handle = await window.showSaveFilePicker(options);
              const writable = await handle.createWritable();
              await writable.write(blob);
              await writable.close();
              console.log(`Recording saved as ${fileName}`);
            } catch (e) {
              console.error("File saving cancelled or failed", e);
            }
          } else {
            // Fallback to anchor-based download
            const url = URL.createObjectURL(blob);
            const a = document.createElement('a');
            a.href = url;
            a.download = fileName;
            a.click();
            URL.revokeObjectURL(url);
            console.log(`Recording saved as ${fileName}`);
          }
        })();
      };
      
      mediaRecorders.current[topicName] = mediaRecorder;
      mediaRecorder.start();
      
      let frameIndex = 0;
      
      // Process each frame with dynamic delay based on capture timestamps
      function processFrame() {
        if (frameIndex < frames.length) {
          const currentFrame = frames[frameIndex];
          const blob = new Blob([currentFrame.data], { type: 'image/jpeg' });
          const url = URL.createObjectURL(blob);
          const tempImg = new window.Image();
          
          tempImg.onload = () => {
            ctx.drawImage(tempImg, 0, 0);
            URL.revokeObjectURL(url);
            
            // Determine the delay until the next frame using timestamps
            let delay = 33; // Fallback delay (approx. 30fps)
            if (frameIndex < frames.length - 1) {
              const nextFrame = frames[frameIndex + 1];
              delay = nextFrame.timestamp - currentFrame.timestamp;
              if (delay <= 0) {
                delay = 33;
              }
            }
            
            frameIndex++;
            setTimeout(processFrame, delay);
          };
          tempImg.src = url;
        } else {
          // All frames processed, stop the recorder
          mediaRecorder.stop();
        }
      }
      
      // Start processing frames
      processFrame();
    };
    
    imgForSize.src = URL.createObjectURL(firstFrameBlob);
  };

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
                  {recording[topicName] ? (
                    <button
                      onClick={() => stopRecording(topicName)}
                      className="px-4 py-2 bg-red-600 text-white rounded hover:bg-red-700 font-medium"
                    >
                      Stop Recording
                    </button>
                  ) : (
                    <button
                      onClick={() => startRecording(topicName)}
                      className="px-4 py-2 bg-green-600 text-white rounded hover:bg-green-700 font-medium"
                    >
                      Start Recording
                    </button>
                  )}
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