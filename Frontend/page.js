'use client'; // Make this file a Client Component

import React, { useEffect, useState, useRef } from 'react';
import ROSLIB from 'roslib';
import '@fontsource/montserrat'; // 默认权重400
import '@fontsource/montserrat/600.css'; // 加粗
import '@fontsource/montserrat/700.css'; // 更加粗


const ProgressBarPopup = ({ onClose }) => {
  const [progress, setProgress] = useState(0);

  useEffect(() => {
    const interval = setInterval(() => {
      setProgress((prev) => {
        if (prev >= 100) {
          clearInterval(interval);
          onClose(); // Close the popup when progress reaches 100%
          return 100;
        }
        return prev + 1;
      });
    }, 20);

    return () => clearInterval(interval);
  }, [onClose]);

  return (
    <div className="fixed inset-0 flex items-center justify-center bg-gray-800 bg-opacity-50">
      <div className="bg-white p-5 rounded-lg shadow-lg w-96 text-center">
        <h2 className="text-lg font-bold mb-4">Processing...</h2>
        <div className="relative w-full bg-gray-200 rounded-full h-6 overflow-hidden">
          <div
            className="h-full bg-blue-500 transition-all duration-100"
            style={{ width: `${progress}%` }}
          ></div>
        </div>
        <p className="mt-2">{progress}%</p>
      </div>
    </div>
  );
};

export default function Page() {
  // State to store the ROS connection
  const [ros, setRos] = useState(null);
  const [wasteTypes, setWasteTypes] = useState(["Vomit", "Liquid", "Poop"]);
  const [selectedWasteTypes, setSelectedWasteTypes] = useState(["Vomit", "Liquid", "Poop"]);
  const [statusOptions, setStatusOptions] = useState(["Not cleaned", "Cleaned", "Cleaning"]);
  const [selectedStatus, setSelectedStatus] = useState(["Not cleaned", "Cleaned", "Cleaning"]);
  const [showPopup, setShowPopup] = useState("");
  const [showProgress, setShowProgress] = useState("");
  const intervalRef = useRef(null);

  // Automatically connect to rosbridge on mount
  useEffect(() => {
    console.log('Connecting to ROS...');
    const rosInstance = new ROSLIB.Ros({
      url: 'ws://10.19.159.63:9090', // Adjust if your rosbridge_server is elsewhere
    });

    rosInstance.on('connection', () => {
      console.log('Connected to ROSBridge server');
      setRos(rosInstance);
    });

    rosInstance.on('error', (error) => {
      console.error('Error connecting to ROSBridge server:', error);
    });

    rosInstance.on('close', () => {
      console.log('Connection to ROSBridge server closed');
      setRos(null);
    });

    // Clean up when unmounting
    return () => {
      rosInstance.close();
    };
  }, []);

  // Function to publish a Twist command to /cmd_vel
  const sendCommand = (linear, angular) => {
    if (!ros) {
      console.warn('ROS is not connected yet!');
      return;
    }

    const cmdVel = new ROSLIB.Topic({
      ros: ros,
      name: '/cmd_vel',
      messageType: 'geometry_msgs/Twist',
    });

    const twist = new ROSLIB.Message({
      linear: { x: linear, y: 0, z: 0 },
      angular: { x: 0, y: 0, z: angular },
    });
    
    cmdVel.publish(twist);
  };
  
  const handleMouseDown = (linear, angular) => {
    sendCommand(linear, angular);
    intervalRef.current = setInterval(() => sendCommand(linear, angular), 100);
  };
  
  const handleMouseUp = () => {
    sendCommand(0, 0);
    clearInterval(intervalRef.current);
  };

  const handleWasteTypeChange = (index, event) => {
    const newSelectedWasteTypes = [...selectedWasteTypes];
    newSelectedWasteTypes[index] = event.target.value;
    setSelectedWasteTypes(newSelectedWasteTypes);
  };

  const handleStatusChange = (index, event) => {
    const newSelectedStatus = [...selectedStatus];
    newSelectedStatus[index] = event.target.value;
    setSelectedStatus(newSelectedStatus);
  };

  const handleSuctionClick = () => {
    setShowPopup("Suction");
  };

  const handleScrubberClick = () => {
    setShowPopup("Scrubber");
  };

  const closePopup = () => {
    setShowPopup(false);
  };

  const handleConfirm = () => {
    console.log("in handleConfirm");
    var typeWork = showPopup;
    setShowPopup(false);
    setShowProgress(typeWork);
    const interval = setInterval(() => {
      setProgress((prev) => {
        if (prev >= 100) {
          clearInterval(interval);
          return 100;
        }
        return prev + 1;
      });
    }, 100); // 100ms interval, so 100 steps in 10 seconds

    return () => clearInterval(interval);
  };

  // URL of your web_video_server stream (change port/topic if needed)
  const IMAGE_URL = "http://10.19.159.63:8080/stream?topic=/output_image";
  // const IMAGE_URL = "/images/example.png";
  const FIXED_IMAGE_URL = "/images/st_logo.png";

  return (
    <div style={{ display: 'flex', flexDirection: 'column', height: '100vh' }}>
      {/* Live feed at the top, centered, occupying 60% of the screen height */}
      <div style={{ display: 'flex', justifyContent: 'center', alignItems: 'center', height: '60%', backgroundColor: '#f0f0f0', position: 'relative' }}>
        {/* 固定图片 */}
        <img
          src={FIXED_IMAGE_URL} 
          alt="Fixed Image"
          style={{ position: 'absolute', top: 0, left: 0, maxWidth: "300px" }}
        />
        
        {/* 直播摄像头画面 */}
        <img
          src={IMAGE_URL}
          alt="Live Feed from Camera"
          style={{ maxWidth: '100%', maxHeight: '100%', border: '1px solid black' }}
        />
      </div>

      {/* Container for control buttons, table, and right-side buttons */}
      <div style={{ display: 'flex', alignItems: 'center', justifyContent: 'space-between', height: '40%' }}>
        {/* Control buttons positioned to the left */}
        <div style={{ width: '25%', display: 'flex', justifyContent: 'center', alignItems: 'center' }}>
          <div style={{ display: 'grid', gridTemplateColumns: 'repeat(3, auto)', gap: '10px', alignItems: 'center' }}>
            <div></div>
            
            
            
            {/* <button onClick={() => sendCommand(1, 0)} style={{ ...buttonStyle, clipPath: 'polygon(50% 0%, 0% 100%, 100% 100%)' }}></button>
            <div></div>
            <button onClick={() => sendCommand(0, 0.4)} style={{ ...buttonStyle, clipPath: 'polygon(0% 50%, 100% 0%, 100% 100%)' }}></button>
            <button onClick={() => sendCommand(0, 0)} style={stopButtonStyle}>Stop</button>
            <button onClick={() => sendCommand(0, -0.4)} style={{ ...buttonStyle, clipPath: 'polygon(100% 50%, 0% 0%, 0% 100%)' }}></button>
            <div></div>
            <button onClick={() => sendCommand(-1, 0)} style={{ ...buttonStyle, clipPath: 'polygon(50% 100%, 0% 0%, 100% 0%)' }}></button>  */}
            
            
            
            
            <button onMouseDown={() => handleMouseDown(1, 0)} onMouseUp={handleMouseUp}  style={{ ...buttonStyle, clipPath: 'polygon(50% 0%, 0% 100%, 100% 100%)' }}></button>
            <div></div>
            <button onMouseDown={() => handleMouseDown(0, 0.8)} onMouseUp={handleMouseUp}  style={{ ...buttonStyle, clipPath: 'polygon(0% 50%, 100% 0%, 100% 100%)' }}></button>
            <button onClick={() => sendCommand(0, 0)} style={stopButtonStyle}>Stop</button>
            <button onMouseDown={() => handleMouseDown(0, -0.8)} onMouseUp={handleMouseUp} style={{ ...buttonStyle, clipPath: 'polygon(100% 50%, 0% 0%, 0% 100%)' }}></button>
            <div></div>
            <button onMouseDown={() => handleMouseDown(-1, 0)} onMouseUp={handleMouseUp}  style={{ ...buttonStyle, clipPath: 'polygon(50% 100%, 0% 0%, 100% 0%)' }}></button>
            
            
            
            
            <div></div>
          </div>
        </div>

        {/* Table positioned in the center */}
        <div style={{ width: '50%', padding: '30px', textAlign: 'center' }}>

        <table style={{ width: '100%', borderCollapse: 'collapse', border: '1px solid black' }}>
            <thead>
              <tr>
                <th style={tableHeaderStyle}>Waste Type</th>
                <th style={tableHeaderStyle}>Status</th>
                <th style={tableHeaderStyle}>Location</th>
                <th style={tableHeaderStyle}>Time</th>
              </tr>
            </thead>
            <tbody>
              {selectedWasteTypes.map((wasteType, index) => (
                <tr key={index}>
                  <td style={tableCellStyle}>
                    <select value={wasteType} onChange={(e) => handleWasteTypeChange(index, e)}>
                      {wasteTypes.map((type, i) => (
                        <option key={i} value={type}>{type}</option>
                      ))}
                    </select>
                  </td>
                  <td style={tableCellStyle}>
                    <select value={selectedStatus[index]} onChange={(e) => handleStatusChange(index, e)}>
                      {statusOptions.map((status, i) => (
                        <option key={i} value={status}>{status}</option>
                      ))}
                    </select>
                  </td>
                  <td style={tableCellStyle}>{["Westlake Elevator 1", "U District Elevator 2", "Chinatown Elevator 1"][index]}</td>
                  <td style={tableCellStyle}>{["Nov. 1, 5:11 PM", "Nov. 1, 2:21 PM", "Nov. 1, 1:51 PM"][index]}</td>
                </tr>
              ))}
            </tbody>
          </table>

        </div>

        {/* Right-side buttons */}
        <div style={{ display: 'flex', flexDirection: 'column', alignItems: 'center', gap: '10px', width: '25%' }}>
          <button style={controlButtonStyle} onClick={handleSuctionClick}>Suction</button>
          <button style={controlButtonStyle} onClick={handleScrubberClick}>Scrubber</button>
          <button style={controlButtonStyle} >To Dock</button>
        </div>
      </div>

      {showPopup && (
        <div style={popupStyle}>
          <p>The {showPopup} will start in 3 seconds, please confirm</p>
          <div style={{ display: 'flex', justifyContent: 'center', gap: '10px' }}>
            <button onClick={handleConfirm} style={popupButtonStyle}>Confirm</button>
            <button onClick={closePopup} style={popupButtonStyle}>Cancel</button>
          </div>
        </div>
      )}
      {showProgress && <ProgressBarPopup onClose={() => setShowProgress(false)} />}
      
    </div>
  );
}

const popupStyle = {
  position: 'fixed',
  top: '50%',
  left: '50%',
  transform: 'translate(-50%, -50%)',
  padding: '20px',
  backgroundColor: 'white',
  border: '1px solid grey',
  boxShadow: '2px 2px 10px rgba(0,0,0,0.3)',
  zIndex: 1000,
  fontFamily: 'Montserrat, sans-serif'
};

const popupButtonStyle = {
  padding: '10px',
  cursor: 'pointer',
  border: '1px solid grey',
  borderRadius: "5px",
  backgroundColor: "#2a376e",
  color: "white",
  fontFamily: 'Montserrat, sans-serif'
};

const controlButtonStyle = {
  padding: "10px 10px",
  fontSize: "18px",
  cursor: "pointer",
  border: "none",
  borderRadius: "5px",
  backgroundColor: "#2a376e",
  color: "white",
  minWidth: "100px",
  fontFamily: "Montserrat, sans-serif"
};

const buttonStyle = {
  padding: "10px 20px",
  fontSize: "24px",
  cursor: "pointer",
  border: "none",
  borderRadius: "5px",
  backgroundColor: "#2a376e",
  color: "white",
  minWidth: "60px",
  height: "60px",
  textAlign: 'center',
  fontFamily: "Montserrat, sans-serif"
};

const stopButtonStyle = {
  fontSize: "20px",
  cursor: "pointer",
  border: "none",
  borderRadius: "5px",
  backgroundColor: "#bb0000",
  color: "white",
  minWidth: "60px",
  maxWidth: "60px",
  height: "60px",
  textAlign: 'center',
  fontFamily: "Montserrat, sans-serif"
};

const tableHeaderStyle = {
  border: '1px solid grey',
  padding: '8px',
  textAlign: 'center',
  backgroundColor: '#ddd',
  fontFamily: "Montserrat, sans-serif"
};

const tableCellStyle = {
  border: '1px solid grey',
  padding: '8px',
  fontFamily: "Montserrat, sans-serif"
};
