#!/usr/bin/env python3
"""
Simple GDL90 receiver for testing
Listens on UDP port 4000 and decodes basic GDL90 messages
"""

import socket
import struct
import sys
from datetime import datetime

def decode_gdl90(data):
    """Decode GDL90 message"""
    if len(data) < 5:
        return None
    
    # Remove frame delimiters (0x7E)
    if data[0] == 0x7E and data[-1] == 0x7E:
        data = data[1:-1]
    
    # Unescape control characters
    unescaped = bytearray()
    i = 0
    while i < len(data):
        if data[i] == 0x7D:
            if i + 1 < len(data):
                unescaped.append(data[i+1] ^ 0x20)
                i += 2
            else:
                i += 1
        else:
            unescaped.append(data[i])
            i += 1
    
    if len(unescaped) < 3:
        return None
    
    msg_id = unescaped[0]
    
    if msg_id == 0x00:
        return f"HEARTBEAT (GPS: {'Yes' if unescaped[1] & 0x80 else 'No'})"
    elif msg_id == 0xCC:
        return "STRATUX HEARTBEAT"
    elif msg_id == 0x0A:
        return "OWNSHIP REPORT"
    elif msg_id == 0x0B:
        return "OWNSHIP GEO ALTITUDE"
    elif msg_id == 0x14:
        if len(unescaped) >= 28:
            # Decode ICAO address
            icao = (unescaped[2] << 16) | (unescaped[3] << 8) | unescaped[4]
            
            # Decode latitude
            lat_raw = (unescaped[5] << 16) | (unescaped[6] << 8) | unescaped[7]
            if lat_raw & 0x800000:  # Sign extend
                lat_raw = lat_raw - 0x1000000
            lat = lat_raw * 180.0 / 8388608.0
            
            # Decode longitude
            lon_raw = (unescaped[8] << 16) | (unescaped[9] << 8) | unescaped[10]
            if lon_raw & 0x800000:  # Sign extend
                lon_raw = lon_raw - 0x1000000
            lon = lon_raw * 180.0 / 8388608.0
            
            # Decode altitude
            alt_raw = ((unescaped[11] << 4) | (unescaped[12] >> 4)) & 0xFFF
            if alt_raw == 0xFFF:
                alt = "Invalid"
            else:
                alt = (alt_raw * 25) - 1000
            
            return f"TRAFFIC: ICAO={icao:06X} Lat={lat:.4f} Lon={lon:.4f} Alt={alt}"
    
    return f"MSG ID {msg_id:02X}"

def main():
    # Create UDP socket
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind(('', 4000))
    
    print("Listening for GDL90 messages on UDP port 4000...")
    print("Press Ctrl+C to exit")
    print()
    
    msg_count = 0
    last_heartbeat = None
    
    try:
        while True:
            data, addr = sock.recvfrom(1024)
            msg_count += 1
            
            decoded = decode_gdl90(data)
            if decoded:
                timestamp = datetime.now().strftime("%H:%M:%S")
                print(f"[{timestamp}] {decoded}")
                
                if "HEARTBEAT" in decoded:
                    if last_heartbeat is None:
                        print(f"  ✓ Receiving heartbeats from {addr[0]}")
                    last_heartbeat = datetime.now()
            
            # Show statistics every 10 messages
            if msg_count % 10 == 0:
                print(f"  ({msg_count} messages received)")
                
    except KeyboardInterrupt:
        print(f"\nReceived {msg_count} total messages")
        if last_heartbeat:
            print(f"Last heartbeat: {(datetime.now() - last_heartbeat).seconds}s ago")
        else:
            print("No heartbeats received - check if server is running")

if __name__ == "__main__":
    main()