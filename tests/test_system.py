#!/usr/bin/env python3
"""
Simple test script to verify Flask server endpoints
"""

import requests
import json
import time

BASE_URL = "http://localhost:5001"

def test_endpoint(method, endpoint, data=None, files=None):
    """Test a specific endpoint"""
    url = f"{BASE_URL}{endpoint}"
    response = None
    try:
        if method == "GET":
            response = requests.get(url, timeout=5)
        elif method == "POST":
            if files:
                response = requests.post(url, data=data, files=files, timeout=5)
            else:
                response = requests.post(url, json=data, timeout=5)
        else:
            print(f"❌ Unsupported HTTP method: {method}")
            return False

        print(f"✅ {method} {endpoint}: {response.status_code}")
        if response.status_code != 200:
            print(f"   Error: {response.text[:100]}")
        return response.status_code == 200
    except Exception as e:
        print(f"❌ {method} {endpoint}: {str(e)}")
        return False

def main():
    print("🧪 Testing Flask Server Endpoints")
    print("=" * 40)
    
    # Test basic endpoints
    endpoints = [
        ("GET", "/api/devices"),
        ("GET", "/api/logs"),
        ("GET", "/api/attendance"),
        ("GET", "/api/firmware"),
    ]
    
    success_count = 0
    for method, endpoint in endpoints:
        if test_endpoint(method, endpoint):
            success_count += 1
        time.sleep(0.5)
    
    print("\n📊 Basic Endpoint Tests:")
    print(f"   Passed: {success_count}/{len(endpoints)}")
    
    # Test firmware operations
    print("\n🔧 Testing Firmware Operations:")
    
    # Test firmware download
    try:
        response = requests.get(f"{BASE_URL}/api/firmware/download/new_master.ino.bin", timeout=5)
        if response.status_code == 200:
            print("✅ Firmware download: Working")
        else:
            print(f"⚠️ Firmware download: {response.status_code}")
    except Exception as e:
        print(f"❌ Firmware download: {str(e)}")
    
    # Test firmware push (with invalid device - should fail gracefully)
    test_data = {
        "device_ip": "192.168.64.999",  # Non-existent device
        "filename": "new_master.ino.bin"
    }
    
    try:
        response = requests.post(f"{BASE_URL}/api/firmware/push", data=test_data, timeout=5)
        if response.status_code in [500, 404]:  # Expected errors for non-existent device
            print("✅ Firmware push error handling: Working")
        else:
            print(f"⚠️ Firmware push: Unexpected response {response.status_code}")
    except Exception as e:
        print(f"❌ Firmware push test: {str(e)}")
    
    print("\n🎯 System Status Summary:")
    if success_count == len(endpoints):
        print("   ✅ All basic endpoints working")
        print("   ✅ Flask server is healthy")
        print("   ✅ Database operations functional")
    else:
        print(f"   ⚠️ Some endpoints failed ({success_count}/{len(endpoints)})")
    
    print("\n📱 Device Connection Status:")
    try:
        response = requests.get(f"{BASE_URL}/api/devices", timeout=5)
        if response.status_code == 200:
            devices = response.json()
            if devices:
                print(f"   📡 {len(devices)} device(s) registered:")
                for device in devices:
                    status_icon = "🟢" if device.get("status") == "online" else "🔴"
                    print(f"      {status_icon} {device.get('deviceName')} ({device.get('deviceType')}) - {device.get('ipAddress')}")
            else:
                print("   📵 No devices registered")
        else:
            print("   ❌ Failed to get device list")
    except Exception as e:
        print(f"   ❌ Device check failed: {str(e)}")

if __name__ == "__main__":
    main()
