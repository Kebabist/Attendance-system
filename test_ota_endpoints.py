#!/usr/bin/env python3
"""
Test script to verify OTA endpoints work correctly
"""
import requests
import json

BASE_URL = "http://localhost:5001"

def test_endpoint(method, endpoint, data=None):
    """Test an API endpoint"""
    try:
        url = f"{BASE_URL}{endpoint}"
        print(f"\n🧪 Testing {method} {endpoint}")
        
        if method == "GET":
            response = requests.get(url, timeout=10)
        elif method == "POST":
            response = requests.post(url, json=data, timeout=10)
        else:
            print(f"❌ Unsupported method: {method}")
            return
        
        print(f"📊 Status: {response.status_code}")
        
        if response.headers.get('content-type', '').startswith('application/json'):
            result = response.json()
            print(f"📋 Response: {json.dumps(result, indent=2)}")
        else:
            print(f"📋 Response: {response.text[:200]}...")
        
        return response.status_code == 200
        
    except Exception as e:
        print(f"❌ Error: {str(e)}")
        return False

def main():
    print("="*60)
    print("🚀 Testing OTA Endpoints")
    print("="*60)
    
    # Test OTA devices endpoint
    test_endpoint("GET", "/api/ota/devices")
    
    # Test firmware list endpoint  
    test_endpoint("GET", "/api/firmware")
    
    # Test WiFi OTA push (will fail without actual device, but we can see if endpoint exists)
    test_endpoint("POST", "/api/ota/wifi_push", {
        "device_ip": "192.168.1.100",
        "firmware_file": "test.bin"
    })
    
    # Test LoRa OTA push (will fail without actual firmware, but we can see if endpoint exists)
    test_endpoint("POST", "/api/ota/lora_push", {
        "slave_id": "SLAVE-TEST",
        "master_ip": "192.168.1.100", 
        "firmware_file": "test.bin"
    })
    
    # Test OTA status endpoint
    test_endpoint("GET", "/api/ota/status/192.168.1.100")
    
    print("\n" + "="*60)
    print("✅ OTA Endpoint Testing Complete")
    print("="*60)

if __name__ == "__main__":
    main()
