import requests
import cv2
import numpy as np

# r  = requests.get('http://httpbin.org/get')
payload = {'key1': 'value1', 'key2': 'value2', 'key3': None}
r = requests.get('http://httpbin.org/get', params=payload)
# print(r.url, r.json())


img= np.zeros((1,1,3),np.uint8)
en= cv2.imencode('.jpg',img)
print(type(en), en[0], en[1].shape)

import json

data = {
    'name' : 'ACME',
    'shares' : 100,
    'price' : 542.23
}

json_str = json.dumps(data)
print(json.loads(json_str))