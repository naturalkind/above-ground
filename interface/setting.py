import serial
import struct
import time
from datetime import datetime
import json
import binascii
import argparse
from dataclasses import dataclass
from typing import Optional, Dict, List
import threading
import queue
import sys

# CRSF Protocol Constants
CRSF_SYNC_BYTE = 0xC8
CRSF_INVERTED_SYNC_BYTE = 0xEA
CRSF_FRAME_SIZE_MAX = 64

# Packet types
CRSF_FRAMETYPE_RC_CHANNELS_PACKED = 0x16
CRSF_FRAMETYPE_LINK_STATISTICS = 0x14
CRSF_FRAMETYPE_BATTERY_SENSOR = 0x08
CRSF_FRAMETYPE_GPS = 0x02
CRSF_FRAMETYPE_ATTITUDE = 0x1E
CRSF_FRAMETYPE_FLIGHT_MODE = 0x21
CRSF_FRAMETYPE_VARIO = 0x07
CRSF_FRAMETYPE_BARO_ALTITUDE = 0x09
CRSF_FRAMETYPE_DEVICE_PING = 0x28
CRSF_FRAMETYPE_DEVICE_INFO = 0x29

@dataclass
class PacketInfo:
    timestamp: float
    relative_time: float
    packet_type: str
    payload_hex: str
    payload_length: int
    raw_packet: str
    direction: str = "RX"
    sync_byte: str = "0xC8"
    frame_size: int = 0
    crc: str = ""
    parsed_data: dict = None

def crc8_dvb_s2(data: bytes) -> int:
    """CRC8 calculation for CRSF protocol"""
    crc = 0
    for byte in data:
        crc ^= byte
        for _ in range(8):
            if crc & 0x80:
                crc = ((crc << 1) ^ 0xD5) & 0xFF
            else:
                crc = (crc << 1) & 0xFF
    return crc

def create_crsf_rc_packet(channels=None):
    """Create CRSF RC channels packet - ESSENTIAL FOR ELRS HANDHAKE"""
    if channels is None:
        # Default channels: [AIL, ELE, THR, RUD, ARM, MODE, ...]
        channels = [1500, 1500, 988, 1500, 988, 1500] + [988] * 10
        channels = channels[:16]  # Ensure exactly 16 channels
    
    crsf_channels = []
    for ch in channels:
        ch = max(988, min(2012, ch))
        crsf_val = int((ch - 988) * 2047 / (2012 - 988))
        crsf_val = max(0, min(2047, crsf_val))
        crsf_channels.append(crsf_val)
    
    # Pack 16 channels of 11 bits each into 22 bytes
    packet = bytearray(22)
    bits_written = 0
    
    for ch_val in crsf_channels:
        bits_remaining = 11
        while bits_remaining > 0:
            byte_index = bits_written // 8
            bit_index = bits_written % 8
            bits_to_write = min(bits_remaining, 8 - bit_index)
            mask = (1 << bits_to_write) - 1
            bits = (ch_val >> (bits_remaining - bits_to_write)) & mask
            if bit_index == 0:
                packet[byte_index] = bits
            else:
                packet[byte_index] |= bits << bit_index
            bits_written += bits_to_write
            bits_remaining -= bits_to_write
    
    # Build complete packet
    frame_size = 24  # 1(type) + 22(data) + 1(crc)
    packet_with_header = bytearray([CRSF_SYNC_BYTE, frame_size, CRSF_FRAMETYPE_RC_CHANNELS_PACKED])
    packet_with_header.extend(packet)
    crc = crc8_dvb_s2(packet_with_header[2:])
    packet_with_header.append(crc)
    
    return packet_with_header

class CRSFPacketAnalyzer:
    def __init__(self, port: str = "/dev/ttyUSB0", baudrate: int = 115200):
        self.port = port
        self.baudrate = baudrate
        self.serial = None
        self.running = False
        self.connected = False
        self.send_rc = True  # Enable RC packet sending by default
        
        # Statistics
        self.stats = {
            'total_packets': 0,
            'valid_packets': 0,
            'crc_errors': 0,
            'bytes_received': 0,
            'bytes_sent': 0,
            'rc_packets_sent': 0,
            'start_time': time.time(),
            'packets_by_type': {}
        }
        
        # Packet queues
        self.packet_queue = queue.Queue()
        self.raw_data = []
        self.max_raw_data = 10000
        
        # Data buffer for packet assembly
        self.rx_buffer = bytearray()
        
        # Logging
        self.log_file = None
        self.logging_active = False
        
        # Threading
        self.read_thread = None
        self.rc_thread = None
        self.rc_channels = [1500, 1500, 988, 1500, 988, 1500] + [988] * 10
        
        # Verbose output
        self.verbose = True
        
    def connect(self) -> bool:
        """Connect to serial port"""
        try:
            print(f"Connecting to {self.port} at {self.baudrate} baud...")
            self.serial = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                timeout=0.1,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                rtscts=False,
                dsrdtr=False
            )
            
            time.sleep(0.5)
            self.serial.reset_input_buffer()
            self.serial.reset_output_buffer()
            time.sleep(0.1)
            
            self.connected = True
            self.running = True
            
            # Start reading thread
            self.read_thread = threading.Thread(target=self._read_serial_thread, daemon=True)
            self.read_thread.start()
            
            # Start RC sending thread if enabled
            if self.send_rc:
                self.rc_thread = threading.Thread(target=self._rc_sender_thread, daemon=True)
                self.rc_thread.start()
                print("RC packet transmitter: ENABLED (50Hz)")
            
            print(f"Connected successfully")
            return True
            
        except Exception as e:
            print(f"Connection error: {e}")
            return False
    
    def _rc_sender_thread(self):
        """Thread for sending RC packets at 50Hz - REQUIRED FOR ELRS"""
        while self.running and self.connected:
            try:
                if self.serial and self.serial.is_open:
                    rc_packet = create_crsf_rc_packet(self.rc_channels)
                    self.serial.write(rc_packet)
                    self.stats['bytes_sent'] += len(rc_packet)
                    self.stats['rc_packets_sent'] += 1
                    
                    # Log sent packet if verbose
                    if self.verbose and self.stats['rc_packets_sent'] % 100 == 0:
                        print(f"RC packets sent: {self.stats['rc_packets_sent']}")
                
                # 50Hz = 20ms interval
                time.sleep(0.02)
                
            except Exception as e:
                print(f"RC sender error: {e}")
                time.sleep(0.1)
    
    def _read_serial_thread(self):
        """Thread for reading serial data"""
        while self.running and self.connected:
            try:
                if self.serial and self.serial.is_open:
                    available = self.serial.in_waiting
                    if available > 0:
                        data = self.serial.read(available)
                        if data:
                            self.stats['bytes_received'] += len(data)
                            self._process_raw_data(data)
                    else:
                        time.sleep(0.001)
                
            except Exception as e:
                print(f"Serial read error: {e}")
                time.sleep(0.1)
    
    def _process_raw_data(self, data: bytes):
        """Process incoming raw data and extract packets"""
        self.raw_data.extend(data)
        if len(self.raw_data) > self.max_raw_data:
            self.raw_data = self.raw_data[-self.max_raw_data:]
        
        self.rx_buffer.extend(data)
        
        pos = 0
        while pos < len(self.rx_buffer):
            # Look for sync byte
            if self.rx_buffer[pos] in [CRSF_SYNC_BYTE, CRSF_INVERTED_SYNC_BYTE]:
                if len(self.rx_buffer) - pos < 2:
                    break
                
                frame_size = self.rx_buffer[pos + 1]
                total_packet_size = frame_size + 2
                
                if frame_size < 3 or frame_size > CRSF_FRAME_SIZE_MAX:
                    pos += 1
                    continue
                
                if len(self.rx_buffer) - pos < total_packet_size:
                    break
                
                packet = bytes(self.rx_buffer[pos:pos + total_packet_size])
                
                # Verify CRC
                if len(packet) >= 4:
                    crc_calculated = crc8_dvb_s2(packet[2:-1])
                    crc_received = packet[-1]
                    
                    if crc_calculated == crc_received:
                        self._process_packet(packet)
                    else:
                        self.stats['crc_errors'] += 1
                        if self.verbose:
                            print(f"CRC Error: calc=0x{crc_calculated:02X}, recv=0x{crc_received:02X}")
                
                pos += total_packet_size
            else:
                pos += 1
        
        # Remove processed data from buffer
        if pos > 0:
            del self.rx_buffer[:pos]
    
    def _process_packet(self, packet: bytes):
        """Process a complete packet"""
        if len(packet) < 4:
            return
        
        timestamp = time.time()
        relative_time = timestamp - self.stats['start_time']
        
        sync_byte = packet[0]
        frame_size = packet[1]
        packet_type = packet[2]
        payload = packet[3:-1] if len(packet) > 4 else b''
        crc = packet[-1]
        
        # Update statistics
        self.stats['total_packets'] += 1
        self.stats['valid_packets'] += 1
        
        # Update packet type statistics
        type_key = f"0x{packet_type:02X}"
        if type_key not in self.stats['packets_by_type']:
            self.stats['packets_by_type'][type_key] = 0
        self.stats['packets_by_type'][type_key] += 1
        
        # Create packet info
        packet_info = PacketInfo(
            timestamp=timestamp,
            relative_time=relative_time,
            packet_type=type_key,
            payload_hex=payload.hex(),
            payload_length=len(payload),
            raw_packet=packet.hex(),
            sync_byte=f"0x{sync_byte:02X}",
            frame_size=frame_size,
            crc=f"0x{crc:02X}",
            parsed_data={}
        )
        
        # Parse payload based on packet type
        parsed_data = self._parse_payload(packet_type, payload)
        if parsed_data:
            packet_info.parsed_data = parsed_data
        
        # Add to queue
        self.packet_queue.put(packet_info)
        
        # Log to file if active
        if self.logging_active and self.log_file:
            self._log_packet(packet_info)
        
        # Print packet info
        self._print_packet_info(packet_info)
    
    def _parse_payload(self, packet_type: int, payload: bytes) -> dict:
        """Parse payload based on packet type"""
        parsed = {}
        
        try:
            if packet_type == CRSF_FRAMETYPE_LINK_STATISTICS and len(payload) >= 10:
                # Link statistics
                uplink_rssi_1 = struct.unpack('b', payload[0:1])[0]
                uplink_rssi_2 = struct.unpack('b', payload[1:2])[0]
                uplink_lq = payload[2]
                downlink_lq = payload[3]
                downlink_rssi = struct.unpack('b', payload[8:9])[0]
                snr = struct.unpack('b', payload[9:10])[0]
                
                parsed = {
                    'type': 'LINK_STATS',
                    'uplink_rssi_avg': (uplink_rssi_1 + uplink_rssi_2) / 2,
                    'uplink_lq': uplink_lq,
                    'downlink_lq': downlink_lq,
                    'downlink_rssi': downlink_rssi,
                    'snr': snr
                }
                
            elif packet_type == CRSF_FRAMETYPE_BATTERY_SENSOR and len(payload) >= 2:
                # Battery sensor
                try:
                    voltage = struct.unpack('>H', payload[0:2])[0] * 0.1
                    if 3.0 <= voltage <= 26.0:
                        parsed = {
                            'type': 'BATTERY',
                            'voltage': voltage
                        }
                except:
                    pass
                    
            elif packet_type == CRSF_FRAMETYPE_ATTITUDE and len(payload) >= 6:
                # Attitude
                pitch = struct.unpack('<h', payload[0:2])[0] / 100.0
                roll = struct.unpack('<h', payload[2:4])[0] / 100.0
                yaw = struct.unpack('<h', payload[4:6])[0] / 100.0
                
                parsed = {
                    'type': 'ATTITUDE',
                    'pitch': pitch,
                    'roll': roll,
                    'yaw': yaw
                }
                
            elif packet_type == CRSF_FRAMETYPE_GPS and len(payload) >= 15:
                # GPS
                lat = struct.unpack('<i', payload[0:4])[0] / 10000000.0
                lon = struct.unpack('<i', payload[4:8])[0] / 10000000.0
                speed = struct.unpack('<h', payload[8:10])[0] * 0.1
                heading = struct.unpack('<h', payload[10:12])[0]
                altitude = struct.unpack('<h', payload[12:14])[0]
                satellites = payload[14]
                
                parsed = {
                    'type': 'GPS',
                    'lat': lat,
                    'lon': lon,
                    'speed': speed,
                    'heading': heading,
                    'altitude': altitude,
                    'satellites': satellites
                }
                
            elif packet_type == CRSF_FRAMETYPE_VARIO and len(payload) >= 2:
                # Vario
                vertical_speed = struct.unpack('<h', payload[0:2])[0] / 100.0
                parsed = {'type': 'VARIO', 'v_speed': vertical_speed}
                
            elif packet_type == CRSF_FRAMETYPE_BARO_ALTITUDE and len(payload) >= 4:
                # Baro altitude
                altitude = struct.unpack('<f', payload[0:4])[0]
                parsed = {'type': 'BARO', 'altitude': altitude}
                
            elif packet_type == CRSF_FRAMETYPE_FLIGHT_MODE and len(payload) > 0:
                # Flight mode
                try:
                    flight_mode = payload.decode('ascii', errors='ignore').strip()
                    parsed = {'type': 'FLIGHT_MODE', 'mode': flight_mode}
                except:
                    pass
                    
            elif packet_type == CRSF_FRAMETYPE_DEVICE_INFO and len(payload) > 0:
                # Device info
                try:
                    device_info = payload.decode('ascii', errors='ignore').strip()
                    parsed = {'type': 'DEVICE_INFO', 'info': device_info}
                except:
                    pass
                    
        except Exception as e:
            parsed = {'type': 'ERROR', 'error': str(e)}
        
        return parsed
    
    def _print_packet_info(self, packet_info: PacketInfo):
        """Print packet information to console"""
        timestamp_str = datetime.fromtimestamp(packet_info.timestamp).strftime('%H:%M:%S.%f')[:-3]
        
        # Get packet type name
        type_names = {
            '0x14': 'LINK_STATS',
            '0x08': 'BATTERY',
            '0x1E': 'ATTITUDE',
            '0x02': 'GPS',
            '0x07': 'VARIO',
            '0x09': 'BARO',
            '0x21': 'FLIGHT_MODE',
            '0x29': 'DEVICE_INFO',
            '0x16': 'RC_CHANNELS'
        }
        
        type_name = type_names.get(packet_info.packet_type, packet_info.packet_type)
        
        print(f"[{timestamp_str}] {type_name:12} Size:{packet_info.frame_size:3}B "
              f"Payload:{packet_info.payload_length:3}B "
              f"CRC:{packet_info.crc}")
        
        if packet_info.parsed_data:
            data_str = []
            for k, v in packet_info.parsed_data.items():
                if k != 'type':
                    data_str.append(f"{k}:{v}")
            if data_str:
                print(f"  Parsed: {', '.join(data_str)}")
    
    def start_logging(self, filename: Optional[str] = None) -> str:
        """Start logging packets to file"""
        if filename is None:
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            filename = f"crsf_telemetry_{timestamp}.json"
        
        self.log_file = open(filename, 'w')
        self.log_file.write("[\n")
        self.logging_active = True
        
        print(f"Started logging to {filename}")
        return filename
    
    def stop_logging(self):
        """Stop logging and close file"""
        if self.log_file:
            self.log_file.write("\n]")
            self.log_file.close()
            self.log_file = None
            self.logging_active = False
            print("Logging stopped")
    
    def _log_packet(self, packet_info: PacketInfo):
        """Log packet to JSON file"""
        if self.log_file:
            log_entry = {
                'timestamp': packet_info.timestamp,
                'relative_time': packet_info.relative_time,
                'packet_type': packet_info.packet_type,
                'payload_hex': packet_info.payload_hex,
                'payload_length': packet_info.payload_length,
                'raw_packet': packet_info.raw_packet,
                'sync_byte': packet_info.sync_byte,
                'frame_size': packet_info.frame_size,
                'crc': packet_info.crc,
                'direction': packet_info.direction,
                'parsed_data': packet_info.parsed_data
            }
            
            if self.stats['valid_packets'] > 1:
                self.log_file.write(",\n")
            
            json.dump(log_entry, self.log_file, indent=2)
    
    def disconnect(self):
        """Disconnect from serial port"""
        self.running = False
        
        # Wait for threads to finish
        if self.read_thread:
            self.read_thread.join(timeout=1.0)
        if self.rc_thread:
            self.rc_thread.join(timeout=1.0)
        
        if self.serial and self.serial.is_open:
            self.serial.close()
        
        self.connected = False
        
        if self.logging_active:
            self.stop_logging()
        
        print("\nDisconnected")
    
    def print_statistics(self):
        """Print statistics"""
        elapsed = time.time() - self.stats['start_time']
        
        print("\n" + "="*80)
        print("CRSF PACKET ANALYZER STATISTICS")
        print("="*80)
        print(f"Elapsed time: {elapsed:.1f} seconds")
        print(f"Total packets: {self.stats['total_packets']}")
        print(f"Valid packets: {self.stats['valid_packets']}")
        print(f"CRC errors: {self.stats['crc_errors']}")
        print(f"Bytes received: {self.stats['bytes_received']}")
        print(f"Bytes sent: {self.stats['bytes_sent']}")
        print(f"RC packets sent: {self.stats['rc_packets_sent']}")
        print(f"Data rate: {self.stats['bytes_received'] / max(1, elapsed):.1f} B/s")
        print(f"Packet rate: {self.stats['valid_packets'] / max(1, elapsed):.1f} packets/s")
        
        print("\nPackets by type:")
        for ptype, count in sorted(self.stats['packets_by_type'].items()):
            percentage = (count / self.stats['valid_packets'] * 100) if self.stats['valid_packets'] > 0 else 0
            type_names = {
                '0x14': 'Link Stats',
                '0x08': 'Battery',
                '0x1E': 'Attitude',
                '0x02': 'GPS',
                '0x07': 'Vario',
                '0x09': 'Baro',
                '0x21': 'Flight Mode',
                '0x29': 'Device Info',
                '0x16': 'RC Channels'
            }
            name = type_names.get(ptype, ptype)
            print(f"  {name:15} {ptype:6}: {count:5} ({percentage:5.1f}%)")
        print("="*80)
    
    def save_raw_data(self, filename: Optional[str] = None):
        """Save raw binary data to file"""
        if filename is None:
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            filename = f"crsf_raw_data_{timestamp}.bin"
        
        with open(filename, 'wb') as f:
            f.write(bytes(self.raw_data))
        
        print(f"Raw data saved to {filename} ({len(self.raw_data)} bytes)")
        return filename
    
    def run(self, duration: Optional[float] = None):
        """Run packet analyzer"""
        print(f"\nCRSF Packet Analyzer v1.0")
        print(f"Port: {self.port} @ {self.baudrate} baud")
        print(f"RC Transmission: {'ENABLED' if self.send_rc else 'DISABLED'}")
        print("Press Ctrl+C to stop\n")
        
        last_stats_time = time.time()
        
        try:
            while self.running:
                # Process packets from queue
                processed = 0
                while not self.packet_queue.empty():
                    try:
                        packet_info = self.packet_queue.get_nowait()
                        processed += 1
                    except queue.Empty:
                        break
                
                # Print statistics every 5 seconds
                current_time = time.time()
                if current_time - last_stats_time >= 5:
                    print(f"\n[STATUS] Packets: {self.stats['valid_packets']}, "
                          f"Rate: {self.stats['valid_packets']/max(1, current_time-self.stats['start_time']):.1f}/s, "
                          f"RC sent: {self.stats['rc_packets_sent']}")
                    last_stats_time = current_time
                
                # Check duration limit
                if duration and (current_time - self.stats['start_time']) > duration:
                    print(f"\nDuration limit ({duration}s) reached.")
                    break
                
                time.sleep(0.01)
                
        except KeyboardInterrupt:
            print("\nStopped by user")
        finally:
            self.disconnect()
            self.print_statistics()

def main():
    parser = argparse.ArgumentParser(description='Professional CRSF Packet Analyzer for ELRS')
    parser.add_argument('--port', default='/dev/ttyUSB0', help='Serial port (default: /dev/ttyUSB0)')
    parser.add_argument('--baud', type=int, default=115200, 
                       choices=[115200, 57600, 400000], 
                       help='Baud rate (default: 115200)')
    parser.add_argument('--log', action='store_true', help='Enable packet logging to JSON file')
    parser.add_argument('--no-rc', action='store_true', help='Disable RC packet transmission')
    parser.add_argument('--duration', type=float, help='Analysis duration in seconds')
    parser.add_argument('--verbose', action='store_true', help='Verbose output')
    
    args = parser.parse_args()
    
    print("="*80)
    print("ELRS CRSF PACKET ANALYZER")
    print("="*80)
    print("IMPORTANT: This tool sends RC packets to establish ELRS handshake")
    print("The micro RF module requires constant RC packets to stay connected")
    print("="*80)
    
    analyzer = CRSFPacketAnalyzer(port=args.port, baudrate=args.baud)
    analyzer.send_rc = not args.no_rc
    analyzer.verbose = args.verbose
    
    if not analyzer.connect():
        sys.exit(1)
    
    if args.log:
        analyzer.start_logging()
    
    analyzer.run(duration=args.duration)

if __name__ == "__main__":
    main()
