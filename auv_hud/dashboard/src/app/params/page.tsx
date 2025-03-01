"use client";

import { useEffect, useState, useRef } from "react";
import { Input } from "@/components/ui/input";
import { Button } from "@/components/ui/button";
import {
  Select,
  SelectContent,
  SelectItem,
  SelectTrigger,
  SelectValue,
} from "@/components/ui/select";

// List of available nodes - this could be fetched dynamically
const AVAILABLE_NODES = [
  "dumb_controller",
  // Add more nodes as needed
];

export default function Params() {
  const [params, setParams] = useState<{ [key: string]: any }>({});
  const [editableParams, setEditableParams] = useState<{ [key: string]: any }>({});
  const [paramNames, setParamNames] = useState<string[]>([]);
  const [selectedNode, setSelectedNode] = useState<string>("dumb_controller");
  const rosRef = useRef<WebSocket | null>(null);
  const namesRef = useRef<string[]>([]);

  useEffect(() => {
    rosRef.current = new WebSocket("ws://localhost:9090");
    const ros = rosRef.current;

    ros.onopen = () => {
      console.log("Connected to ROS Bridge");
      fetchParameters();
    };

    ros.onmessage = (event) => {
      try {
        const data = JSON.parse(event.data);
        console.log("Received message:", data);

        if (data.op === "service_response") {
          if (data.service === `/${selectedNode}/list_parameters`) {
            if (data.values?.result?.names) {
              const names = data.values.result.names;
              console.log("Got parameter names:", names);
              namesRef.current = names;
              setParamNames(names);
              fetchParameterValues(names);
            }
          } else if (data.service === `/${selectedNode}/get_parameters`) {
            if (data.values?.values) {
              console.log("Got parameter values:", data.values.values);
              const updatedParams: { [key: string]: any } = {};
              
              data.values.values.forEach((value: any, index: number) => {
                const name = namesRef.current[index];
                if (name) {
                  let paramValue;
                  switch (value.type) {
                    case 1:
                      paramValue = value.bool_value;
                      break;
                    case 2:
                      paramValue = value.integer_value;
                      break;
                    case 3:
                      paramValue = value.double_value;
                      break;
                    case 4:
                      paramValue = value.string_value;
                      break;
                    default:
                      paramValue = null;
                  }
                  
                  if (paramValue !== null) {
                    updatedParams[name] = paramValue;
                  }
                }
              });

              console.log("Setting params to:", updatedParams);
              setParams(updatedParams);
              setEditableParams(updatedParams);
            }
          }
        }
      } catch (error) {
        console.error("Error processing message:", error);
      }
    };

    return () => {
      if (ros.readyState === WebSocket.OPEN) {
        ros.close();
      }
    };
  }, [selectedNode]); // Reconnect when node changes

  function fetchParameters() {
    if (!rosRef.current || rosRef.current.readyState !== WebSocket.OPEN) return;

    console.log("Fetching parameters list...");
    rosRef.current.send(
      JSON.stringify({
        op: "call_service",
        service: `/${selectedNode}/list_parameters`,
        args: {},
        id: "list_params_" + Date.now()
      })
    );
  }

  function fetchParameterValues(names: string[]) {
    if (!rosRef.current || rosRef.current.readyState !== WebSocket.OPEN) return;
    if (!names || names.length === 0) return;
    
    console.log("Fetching values for:", names);
    rosRef.current.send(
      JSON.stringify({
        op: "call_service",
        service: `/${selectedNode}/get_parameters`,
        args: { names },
        id: "get_params_" + Date.now()
      })
    );
  }

  function updateParameter(name: string) {
    if (!rosRef.current || rosRef.current.readyState !== WebSocket.OPEN) return;

    const newValue = editableParams[name];
    let paramValue: any = {};

    const value = params[name];
    if (typeof value === "boolean") {
      paramValue = { type: 1, bool_value: newValue === "true" };
    } else if (Number.isInteger(value)) {
      paramValue = { type: 2, integer_value: parseInt(newValue) };
    } else if (typeof value === "number") {
      paramValue = { type: 3, double_value: parseFloat(newValue) };
    } else {
      paramValue = { type: 4, string_value: newValue };
    }

    console.log("Updating parameter:", name, "to:", paramValue);
    rosRef.current.send(
      JSON.stringify({
        op: "call_service",
        service: `/${selectedNode}/set_parameters`,
        args: {
          parameters: [{ name, value: paramValue }]
        },
        id: "set_param_" + Date.now()
      })
    );
  }

  return (
    <div className="min-h-screen bg-gray-900 flex items-center justify-center p-6">
      <div className="w-full max-w-4xl bg-gray-800 rounded-lg shadow-xl">
        <div className="p-6 border-b border-gray-700">
          <div className="flex items-center justify-between mb-6">
            <h1 className="text-2xl font-semibold text-gray-100">Parameter Manager</h1>
            <Select 
              value={selectedNode} 
              onValueChange={setSelectedNode}
            >
              <SelectTrigger className="w-[200px] bg-gray-700 border-gray-600 text-gray-100">
                <SelectValue placeholder="Select node" />
              </SelectTrigger>
              <SelectContent className="bg-gray-700 border-gray-600">
                {AVAILABLE_NODES.map(node => (
                  <SelectItem 
                    key={node} 
                    value={node}
                    className="text-gray-100 hover:bg-gray-600 focus:bg-gray-600"
                  >
                    {node}
                  </SelectItem>
                ))}
              </SelectContent>
            </Select>
          </div>

          {Object.keys(params).length > 0 ? (
            <div className="overflow-auto max-h-[70vh]">
              <table className="w-full border-collapse">
                <thead className="bg-gray-700">
                  <tr>
                    <th className="p-3 text-left text-gray-300 border-b border-gray-600">Parameter</th>
                    <th className="p-3 text-left text-gray-300 border-b border-gray-600">Value</th>
                    <th className="p-3 text-left text-gray-300 border-b border-gray-600">Actions</th>
                  </tr>
                </thead>
                <tbody>
                  {Object.entries(params).map(([key, value]) => (
                    <tr key={key} className="border-b border-gray-700 hover:bg-gray-700/50">
                      <td className="p-3 text-gray-300">{key}</td>
                      <td className="p-3">
                        <Input
                          className="w-full bg-gray-700 border-gray-600 text-gray-200 focus:border-blue-500"
                          value={editableParams[key] ?? ""}
                          onChange={(e) =>
                            setEditableParams({ ...editableParams, [key]: e.target.value })
                          }
                        />
                      </td>
                      <td className="p-3">
                        <Button 
                          className="bg-blue-600 hover:bg-blue-700 text-white" 
                          size="sm" 
                          onClick={() => updateParameter(key)}
                        >
                          Update
                        </Button>
                      </td>
                    </tr>
                  ))}
                </tbody>
              </table>
            </div>
          ) : (
            <p className="text-gray-400 text-center py-8">Loading parameters...</p>
          )}
        </div>
      </div>
    </div>
  );
}