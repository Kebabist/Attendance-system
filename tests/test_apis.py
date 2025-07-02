import requests
import json

base_url = "http://127.0.0.1:5001"

def test_api(endpoint):
    try:
        response = requests.get(f"{base_url}{endpoint}", timeout=5)
        print(f"\n=== {endpoint} ===")
        print(f"Status: {response.status_code}")
        if response.status_code == 200:
            data = response.json()
            print(f"Data type: {type(data)}")
            if isinstance(data, list):
                print(f"Length: {len(data)}")
                if data:
                    print(f"Sample: {data[0]}")
            else:
                print(f"Response: {data}")
        else:
            print(f"Error: {response.text}")
    except Exception as e:
        print(f"\n=== {endpoint} ===")
        print(f"Error: {e}")

if __name__ == "__main__":
    endpoints = [
        "/api/students",
        "/api/attendance", 
        "/api/devices",
        "/api/logs",
        "/api/firmware"
    ]
    
    for endpoint in endpoints:
        test_api(endpoint)
