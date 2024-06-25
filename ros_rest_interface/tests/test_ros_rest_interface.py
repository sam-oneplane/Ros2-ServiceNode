import requests
import json

def test_get_topic(topic):
    response = requests.get(f'http://localhost:8080/{topic}')
    if response.status_code == 200:
        print(f"GET {topic} Success: {response.json()}")
    else:
        print(f"GET {topic} Failed: {response.status_code} {response.text}")

def test_post_topic_json(topic, data):
    headers = {'Content-Type': 'application/json'}
    payload = json.dumps(data)
    response = requests.post(f'http://localhost:8080/{topic}', headers=headers, data=payload)
    if response.status_code == 200:
        print(f"POST {topic} JSON Success")
    else:
        print(f"POST {topic} JSON Failed: {response.status_code} {response.text}")

def test_post_with_text(topic, data):
    headers = {'Content-Type': 'text/plain'}
    response = requests.post(f'http://localhost:8080/{topic}', headers=headers, data=data)
    if response.status_code == 200:
        print(f"POST {topic} JSON Success")
    else:
        print(f"POST {topic} JSON Failed: {response.status_code} {response.text}")


if __name__ == '__main__':
    # Test JSON structure
    json_data = {
        "data": {
            "x": 1.0,
            "y": 2.0,
            "z": 3.1415
        }
    }

    json_data2 = {
        "data": {
            "x": 5.0,
            "y": 6.0,
            "z": 7.28
        }
    }

    json_data3 = {
        "data": {
            "x": 5.0,
            "y": 6.0
        }
    }

    #  testing existing subscriber 
    test_get_topic('topic_2')
    test_get_topic('topic_1')
    test_post_topic_json('topic_1', json_data)
    test_get_topic('topic_1')

    #  testing new subscriber
    test_post_topic_json('topic_2', json_data2)
    test_get_topic('topic_2')
    
    test_get_topic('topic_1')

    # Negative tests
    test_post_topic_json('topic_3', {})  # Posting empty JSON data
    test_post_with_text('topic4', "this is my test")
    test_post_topic_json('topic_4', json_data3)