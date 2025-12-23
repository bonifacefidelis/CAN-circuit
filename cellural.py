"""
CAN Bus to 4G LTE Cellular Transmission Module
Supports multiple 4G modems: SIM7600, SIM800, Quectel, etc.
Transmits CAN data to cloud server via HTTP/MQTT
"""

import serial
import threading
import time
import json
import requests
from queue import Queue
from datetime import datetime
from typing import Optional, Dict, Any, List
import base64


# === 4G LTE Modem Interface ===
class LTEModem:
    """
    4G LTE Modem controller for cellular data transmission
    Supports: SIM7600, SIM800, EC25, BG96, etc.
    """
    
    def __init__(self, port: str, baudrate: int = 115200, apn: str = "internet"):
        self.port = port
        self.baudrate = baudrate
        self.apn = apn
        
        self.serial_conn: Optional[serial.Serial] = None
        self.is_connected = False
        self.has_network = False
        self.signal_strength = 0
        
    def connect(self) -> bool:
        """Connect to 4G modem"""
        try:
            self.serial_conn = serial.Serial(
                self.port,
                self.baudrate,
                timeout=1
            )
            time.sleep(2)  # Wait for modem to initialize
            
            print(f"✅ Connected to 4G modem on {self.port}")
            self.is_connected = True
            
            # Initialize modem
            if self._initialize_modem():
                return True
            else:
                print("❌ Modem initialization failed")
                return False
                
        except Exception as e:
            print(f"❌ Failed to connect to 4G modem: {e}")
            return False
    
    def _initialize_modem(self) -> bool:
        """Initialize 4G modem with AT commands"""
        print("📡 Initializing 4G modem...")
        
        # Test modem
        if not self._send_at_command("AT"):
            print("❌ Modem not responding")
            return False
        print("✓ Modem responding")
        
        # Disable echo
        self._send_at_command("ATE0")
        
        # Check SIM card
        response = self._send_at_command("AT+CPIN?")
        if "READY" not in response:
            print("❌ SIM card not ready")
            return False
        print("✓ SIM card ready")
        
        # Check network registration
        for i in range(30):  # Wait up to 30 seconds
            response = self._send_at_command("AT+CREG?")
            if ",1" in response or ",5" in response:  # Registered
                print("✓ Network registered")
                self.has_network = True
                break
            time.sleep(1)
            print(f"  Waiting for network... {i+1}/30")
        
        if not self.has_network:
            print("❌ Network registration failed")
            return False
        
        # Check signal strength
        self._update_signal_strength()
        
        # Configure APN
        self._send_at_command(f'AT+CGDCONT=1,"IP","{self.apn}"')
        print(f"✓ APN configured: {self.apn}")
        
        # Activate PDP context
        self._send_at_command("AT+CGACT=1,1")
        time.sleep(2)
        
        print("✅ 4G modem initialized successfully")
        return True
    
    def _send_at_command(self, command: str, timeout: int = 1) -> str:
        """Send AT command to modem"""
        if not self.serial_conn:
            return ""
        
        try:
            self.serial_conn.write((command + "\r\n").encode())
            time.sleep(0.1)
            
            response = ""
            start_time = time.time()
            
            while (time.time() - start_time) < timeout:
                if self.serial_conn.in_waiting:
                    response += self.serial_conn.read(self.serial_conn.in_waiting).decode(errors='ignore')
                    if "OK" in response or "ERROR" in response:
                        break
                time.sleep(0.1)
            
            return response
            
        except Exception as e:
            print(f"AT command error: {e}")
            return ""
    
    def _update_signal_strength(self):
        """Get signal strength (0-31)"""
        response = self._send_at_command("AT+CSQ")
        try:
            # Parse response: +CSQ: <rssi>,<ber>
            if "+CSQ:" in response:
                parts = response.split(":")[1].split(",")
                self.signal_strength = int(parts[0].strip())
                bars = min(self.signal_strength // 6, 5)
                print(f"📶 Signal strength: {self.signal_strength}/31 {'█' * bars}{'░' * (5-bars)}")
        except:
            pass
    
    def get_status(self) -> Dict[str, Any]:
        """Get modem status"""
        self._update_signal_strength()
        return {
            'connected': self.is_connected,
            'network': self.has_network,
            'signal_strength': self.signal_strength,
            'apn': self.apn
        }
    
    def disconnect(self):
        """Disconnect from modem"""
        if self.serial_conn:
            self.serial_conn.close()
        self.is_connected = False
        print("🔌 Disconnected from 4G modem")


# === HTTP Data Transmitter ===
class HTTPTransmitter:
    """
    Transmit CAN data via HTTP POST to cloud server
    """
    
    def __init__(self, server_url: str, api_key: Optional[str] = None):
        self.server_url = server_url
        self.api_key = api_key
        self.send_queue = Queue()
        self.is_running = False
        self.worker_thread: Optional[threading.Thread] = None
        self.stats = {
            'sent': 0,
            'failed': 0,
            'bytes_sent': 0
        }
        
    def start(self):
        """Start transmission worker"""
        self.is_running = True
        self.worker_thread = threading.Thread(target=self._worker, daemon=True)
        self.worker_thread.start()
        print("📤 HTTP transmitter started")
    
    def stop(self):
        """Stop transmission worker"""
        self.is_running = False
        if self.worker_thread:
            self.worker_thread.join(timeout=2)
        print("📤 HTTP transmitter stopped")
    
    def queue_data(self, can_messages: List[Dict[str, Any]]):
        """Queue CAN messages for transmission"""
        self.send_queue.put(can_messages)
    
    def _worker(self):
        """Worker thread to send data"""
        while self.is_running:
            try:
                if not self.send_queue.empty():
                    messages = self.send_queue.get(timeout=1)
                    self._send_batch(messages)
                else:
                    time.sleep(0.5)
            except:
                continue
    
    def _send_batch(self, messages: List[Dict[str, Any]]):
        """Send batch of CAN messages to server"""
        try:
            # Prepare payload
            payload = {
                'timestamp': datetime.now().isoformat(),
                'device_id': 'can_analyzer_001',  # Configure this
                'message_count': len(messages),
                'messages': messages
            }
            
            # Prepare headers
            headers = {
                'Content-Type': 'application/json'
            }
            
            if self.api_key:
                headers['Authorization'] = f'Bearer {self.api_key}'
            
            # Send POST request
            response = requests.post(
                self.server_url,
                json=payload,
                headers=headers,
                timeout=10
            )
            
            if response.status_code == 200:
                self.stats['sent'] += len(messages)
                self.stats['bytes_sent'] += len(json.dumps(payload))
                print(f"✅ Sent {len(messages)} messages to cloud ({response.status_code})")
            else:
                self.stats['failed'] += len(messages)
                print(f"⚠️ Server returned status {response.status_code}")
                
        except requests.exceptions.Timeout:
            self.stats['failed'] += len(messages)
            print("❌ HTTP request timeout")
        except requests.exceptions.ConnectionError:
            self.stats['failed'] += len(messages)
            print("❌ Connection error - check network")
        except Exception as e:
            self.stats['failed'] += len(messages)
            print(f"❌ HTTP send error: {e}")
    
    def get_stats(self) -> Dict[str, int]:
        """Get transmission statistics"""
        return self.stats.copy()


# === MQTT Data Transmitter ===
class MQTTTransmitter:
    """
    Transmit CAN data via MQTT (requires paho-mqtt)
    Install: pip install paho-mqtt
    """
    
    def __init__(self, broker: str, port: int = 1883, topic: str = "can/data",
                 username: Optional[str] = None, password: Optional[str] = None):
        self.broker = broker
        self.port = port
        self.topic = topic
        self.username = username
        self.password = password
        
        self.client = None
        self.is_connected = False
        self.send_queue = Queue()
        self.is_running = False
        
        try:
            import paho.mqtt.client as mqtt
            self.client = mqtt.Client()
            
            if username and password:
                self.client.username_pw_set(username, password)
            
            self.client.on_connect = self._on_connect
            self.client.on_disconnect = self._on_disconnect
            
        except ImportError:
            print("⚠️ paho-mqtt not installed. Install with: pip install paho-mqtt")
    
    def connect(self) -> bool:
        """Connect to MQTT broker"""
        if not self.client:
            return False
        
        try:
            print(f"📡 Connecting to MQTT broker {self.broker}:{self.port}...")
            self.client.connect(self.broker, self.port, 60)
            self.client.loop_start()
            
            # Wait for connection
            for i in range(10):
                if self.is_connected:
                    return True
                time.sleep(0.5)
            
            print("❌ MQTT connection timeout")
            return False
            
        except Exception as e:
            print(f"❌ MQTT connection failed: {e}")
            return False
    
    def _on_connect(self, client, userdata, flags, rc):
        """MQTT connection callback"""
        if rc == 0:
            print("✅ Connected to MQTT broker")
            self.is_connected = True
        else:
            print(f"❌ MQTT connection failed with code {rc}")
    
    def _on_disconnect(self, client, userdata, rc):
        """MQTT disconnection callback"""
        print("🔌 Disconnected from MQTT broker")
        self.is_connected = False
    
    def start(self):
        """Start transmission worker"""
        self.is_running = True
        threading.Thread(target=self._worker, daemon=True).start()
        print("📤 MQTT transmitter started")
    
    def stop(self):
        """Stop transmission"""
        self.is_running = False
        if self.client:
            self.client.loop_stop()
            self.client.disconnect()
    
    def queue_data(self, can_messages: List[Dict[str, Any]]):
        """Queue CAN messages for transmission"""
        self.send_queue.put(can_messages)
    
    def _worker(self):
        """Worker thread to send data"""
        while self.is_running:
            try:
                if not self.send_queue.empty() and self.is_connected:
                    messages = self.send_queue.get(timeout=1)
                    self._send_batch(messages)
                else:
                    time.sleep(0.5)
            except:
                continue
    
    def _send_batch(self, messages: List[Dict[str, Any]]):
        """Send batch to MQTT"""
        try:
            payload = {
                'timestamp': datetime.now().isoformat(),
                'messages': messages
            }
            
            result = self.client.publish(
                self.topic,
                json.dumps(payload),
                qos=1
            )
            
            if result.rc == 0:
                print(f"✅ Published {len(messages)} messages to MQTT")
            else:
                print(f"⚠️ MQTT publish failed with code {result.rc}")
                
        except Exception as e:
            print(f"❌ MQTT send error: {e}")


# === CAN to 4G Bridge ===
class CANto4GBridge:
    """
    Main bridge class that connects CAN interface to 4G transmission
    """
    
    def __init__(self, can_interface, modem: LTEModem, 
                 transmitter, batch_size: int = 10, batch_interval: float = 5.0):
        self.can_interface = can_interface
        self.modem = modem
        self.transmitter = transmitter
        
        self.batch_size = batch_size
        self.batch_interval = batch_interval
        
        self.message_buffer: List[Dict[str, Any]] = []
        self.last_send_time = time.time()
        
        self.is_running = False
        self.buffer_lock = threading.Lock()
        
    def start(self):
        """Start the bridge"""
        print("=" * 60)
        print("🌉 Starting CAN to 4G Bridge")
        print("=" * 60)
        
        # Connect modem
        if not self.modem.connect():
            print("❌ Cannot start bridge - modem connection failed")
            return False
        
        # Start transmitter
        self.transmitter.start()
        
        # Set up CAN callback
        self.can_interface.set_message_callback(self._on_can_message)
        
        # Start monitoring thread
        self.is_running = True
        threading.Thread(target=self._monitor_buffer, daemon=True).start()
        
        print("✅ CAN to 4G Bridge active")
        return True
    
    def stop(self):
        """Stop the bridge"""
        self.is_running = False
        self.transmitter.stop()
        self.modem.disconnect()
        print("🛑 CAN to 4G Bridge stopped")
    
    def _on_can_message(self, message):
        """Handle incoming CAN message"""
        # Convert CAN message to dict
        msg_dict = {
            'timestamp': datetime.now().isoformat(),
            'can_id': f"0x{message.arbitration_id:X}",
            'data': message.data.hex(),
            'dlc': len(message.data)
        }
        
        with self.buffer_lock:
            self.message_buffer.append(msg_dict)
            
            # Send if batch is full
            if len(self.message_buffer) >= self.batch_size:
                self._send_buffer()
    
    def _monitor_buffer(self):
        """Monitor buffer and send periodically"""
        while self.is_running:
            time.sleep(1)
            
            # Send if interval exceeded and buffer not empty
            if (time.time() - self.last_send_time) >= self.batch_interval:
                with self.buffer_lock:
                    if self.message_buffer:
                        self._send_buffer()
    
    def _send_buffer(self):
        """Send buffered messages"""
        if not self.message_buffer:
            return
        
        messages_to_send = self.message_buffer.copy()
        self.message_buffer.clear()
        self.last_send_time = time.time()
        
        print(f"📦 Queuing {len(messages_to_send)} messages for transmission")
        self.transmitter.queue_data(messages_to_send)
    
    def get_status(self) -> Dict[str, Any]:
        """Get bridge status"""
        return {
            'running': self.is_running,
            'modem': self.modem.get_status(),
            'buffer_size': len(self.message_buffer),
            'transmitter_stats': self.transmitter.get_stats() if hasattr(self.transmitter, 'get_stats') else {}
        }


# === Example Usage ===
if __name__ == "__main__":
    print("CAN to 4G LTE Cellular Transmission Example")
    print("=" * 60)
    
    # Configuration
    MODEM_PORT = 'COM4'           # 4G modem serial port
    MODEM_BAUD = 115200
    APN = "internet"              # Your carrier's APN
    
    # Option 1: HTTP transmission
    SERVER_URL = "https://your-server.com/api/can-data"
    API_KEY = "your-api-key-here"
    
    # Option 2: MQTT transmission
    MQTT_BROKER = "mqtt.example.com"
    MQTT_PORT = 1883
    MQTT_TOPIC = "vehicle/can/data"
    MQTT_USER = "your-username"
    MQTT_PASS = "your-password"
    
    # Create 4G modem
    modem = LTEModem(MODEM_PORT, MODEM_BAUD, APN)
    
    # Choose transmitter (HTTP or MQTT)
    USE_HTTP = True  # Set to False for MQTT
    
    if USE_HTTP:
        transmitter = HTTPTransmitter(SERVER_URL, API_KEY)
    else:
        transmitter = MQTTTransmitter(MQTT_BROKER, MQTT_PORT, MQTT_TOPIC, MQTT_USER, MQTT_PASS)
        transmitter.connect()
    
    # Note: You need to provide your CAN interface
    # from your existing can_interface.py
    # can_interface = CANInterface(...)
    # can_interface.connect()
    
    # Create and start bridge
    # bridge = CANto4GBridge(
    #     can_interface=can_interface,
    #     modem=modem,
    #     transmitter=transmitter,
    #     batch_size=10,
    #     batch_interval=5.0
    # )
    
    # bridge.start()
    # can_interface.start_listening()
    
    print("\n💡 Bridge will batch CAN messages and send via 4G")
    print("   - Batch size: 10 messages")
    print("   - Batch interval: 5 seconds")
    print("   - Whichever comes first triggers transmission")