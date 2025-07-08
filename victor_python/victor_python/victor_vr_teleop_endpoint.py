#!/usr/bin/env python3

import rclpy
import threading
import time
from ros_tcp_endpoint import TcpServer
from sensor_msgs.msg import Image

class VictorVRTeleopEndpoint:
    """
    A singleton TCP server endpoint for Victor VR teleop integration.
    This node runs a persistent ROS-TCP endpoint that Isaac Sim and other 
    components can register publishers/subscribers with.
    """
    
    _instance = None
    _lock = threading.Lock()
    
    def __new__(cls):
        with cls._lock:
            if cls._instance is None:
                cls._instance = super(VictorVRTeleopEndpoint, cls).__new__(cls)
            return cls._instance
    
    def __init__(self):
        # Only initialize once
        if hasattr(self, '_initialized'):
            return
        self._initialized = True
        
        self.tcp_server = None
        self.server_thread = None
        self.is_running = False
        self._shutdown_event = threading.Event()
        
        # Camera subscription setup
        self.camera_subscriber = None
        self.camera_thread = None
        self.camera_topic = "/victor_sim_bridge/camera_viewport"

        # VR controller info setup
        self.vr_controller_topic = "vr_controller_info"
        
    def start_server(self, tcp_ip="0.0.0.0", tcp_port=10000, node_name="VictorVRTeleopEndpoint"):
        """Start the TCP server if it's not already running"""
        if self.is_running:
            print(f"✅ Victor VR Teleop Endpoint already running on {tcp_ip}:{tcp_port}")
            return True
            
        try:
            print(f"🚀 Starting Victor VR Teleop Endpoint on {tcp_ip}:{tcp_port}")
            
            # Create the TCP server
            self.tcp_server = TcpServer(
                node_name=node_name,
                buffer_size=65536,      # Large buffer for image data
                connections=50,         # Support multiple connections
                tcp_ip=tcp_ip,
                tcp_port=tcp_port
            )
            
            # Start the server
            self.tcp_server.start()
            self.is_running = True
            
            # Set up camera subscription and Unity publisher
            self._setup_camera_bridge()
            
            print("✅ Victor VR Teleop Endpoint started successfully!")
            print(f"🔗 Clients can connect to: {tcp_ip}:{tcp_port}")
            return True
            
        except Exception as e:
            print(f"❌ Failed to start Victor VR Teleop Endpoint: {e}")
            self.tcp_server = None
            self.is_running = False
            return False
    
    def is_alive(self):
        """Check if the TCP server is running and healthy"""
        if not self.is_running or self.tcp_server is None:
            return False
            
        try:
            # Check if the server node is still valid
            return hasattr(self.tcp_server, 'publishers_table')
        except Exception:
            return False
    
    def get_server_info(self):
        """Get information about the running server"""
        if not self.is_alive():
            return None
            
        return {
            'node_name': self.tcp_server.node_name,
            'tcp_ip': self.tcp_server.tcp_ip,
            'tcp_port': self.tcp_server.tcp_port,
            'publishers': list(self.tcp_server.publishers_table.keys()),
            'subscribers': list(self.tcp_server.subscribers_table.keys())
        }
    
    def shutdown(self):
        """Shutdown the TCP server gracefully"""
        if self.is_running and self.tcp_server:
            print("🛑 Shutting down Victor VR Teleop Endpoint...")
            try:
                self.is_running = False  # Stop camera thread
                
                # Wait for camera thread to finish
                if hasattr(self, 'camera_thread') and self.camera_thread and self.camera_thread.is_alive():
                    self.camera_thread.join(timeout=1.0)
                
                self.tcp_server.destroy_nodes()
                self.tcp_server = None
                print("✅ Victor VR Teleop Endpoint shutdown complete")
            except Exception as e:
                print(f"⚠️  Warning during shutdown: {e}")
    
    def _setup_camera_bridge(self):
        """Set up camera subscription from Isaac Sim and publisher to Unity"""
        try:
            # Register Unity publisher for camera feed
            self.tcp_server.syscommands.subscribe(
                topic=self.camera_topic,
                message_name="sensor_msgs/Image",
            )
            
            # Register subscriber for VR controller info from Unity
            self.tcp_server.syscommands.publish(
                topic=self.vr_controller_topic,
                message_name="vr_ros2_bridge_msgs/ControllersInfo",
                queue_size=10,
            )

            # Start camera forwarding thread
            # self._start_camera_forwarding()
            print(self.tcp_server.publishers_table)
            print(self.tcp_server.subscribers_table)
            self.tcp_server.setup_executor()
            
            print(f"📷 Camera bridge configured: {self.camera_topic} -> {self.camera_topic}")
            print(f"🎮 VR controller subscriber configured: {self.vr_controller_topic}")
            
        except Exception as e:
            print(f"❌ Failed to setup camera bridge: {e}")
    
# Global instance for easy access
_global_endpoint = None

def get_victor_vr_endpoint():
    """Get the global Victor VR Teleop Endpoint instance"""
    global _global_endpoint
    if _global_endpoint is None:
        _global_endpoint = VictorVRTeleopEndpoint()
    return _global_endpoint


def main(args=None):
    """Main function to run as a standalone ROS node"""
    rclpy.init(args=args)
    
    try:
        # Get the endpoint instance
        endpoint = get_victor_vr_endpoint()
        
        # Start the server
        if endpoint.start_server():
            print("🎯 Victor VR Teleop Endpoint is running...")
            print("📡 Register publishers/subscribers as needed")
            print("🔄 Press Ctrl+C to shutdown")
            
            # Keep the node alive
            try:
                while rclpy.ok():
                    time.sleep(0.03)
            except KeyboardInterrupt:
                print("\n🛑 Shutdown requested...")
            
        else:
            print("❌ Failed to start Victor VR Teleop Endpoint")
            
    except Exception as e:
        print(f"❌ Error in main: {e}")
        
    finally:
        # Cleanup
        endpoint = get_victor_vr_endpoint()
        endpoint.shutdown()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
