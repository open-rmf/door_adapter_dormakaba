
# Copyright 2022 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import requests
import json
import urllib3
import socket
import time
from rmf_door_msgs.msg import DoorMode

class DormakabaAPI:
    def __init__(self,url,api_key,api_value,door_id):
        self.url = url
        self.header = {api_key:api_value}
        self.data = {"id": door_id}

        count = 0
        self.connected = True
        while not self.check_connection():
            if count >= 5:
                print("Unable to connect to Dormakaba cloud API after several retries. Exiting...")
                self.connected = False
                break
            else:
                print("Unable to connect to Dormakaba cloud API. Attempting to reconnect...")
                count += 1
            time.sleep(1)

    def check_connection(self):
        # Test connectivity
        try:
            res = requests.post(url=self.url+"/rmf/status", headers=self.header, json=self.data, timeout=1.0)
            res.raise_for_status()
            return True
        except (socket.gaierror, urllib3.exceptions.NewConnectionError, urllib3.exceptions.MaxRetryError, requests.exceptions.HTTPError ,requests.exceptions.ReadTimeout, requests.exceptions.ConnectionError) as e:
            print(f"Connection Error: {e}")
            return False

    def open_door(self):
        try:
            response = requests.post(url=self.url+"/rmf/remoteopen",headers=self.header, json=self.data, timeout=1.0)
            if response:
                result = response.json()["body"]
                if (result.get("result") is not None):
                    return True
                else:
                    print("door could not perform open")
                    return False
            else:
                print("Invalid response received")
                return False
        except (socket.gaierror, urllib3.exceptions.NewConnectionError, urllib3.exceptions.MaxRetryError, requests.exceptions.HTTPError ,requests.exceptions.ReadTimeout, requests.exceptions.ConnectionError) as e:
            print("Connection Error. "+str(e))
            return False

    def get_mode(self):
        try:
            response = requests.post(url=self.url+"/rmf/status", headers=self.header, json=self.data, timeout=1.0)
            if response:
                state = response.json().get("body").get("doorState")
                if state is None:
                    return DoorMode.MODE_UNKNOWN
                elif state == "closed":
                    return DoorMode.MODE_CLOSED
                elif state == "opening" or state == "closing" or state == "betweenOpenAndClosed":
                    return DoorMode.MODE_MOVING
                elif state == "open":
                    return DoorMode.MODE_OPEN
                elif state == "OFFLINE":
                    return DoorMode.MODE_OFFLINE
                else:
                    return DoorMode.MODE_UNKNOWN
            else:
                return DoorMode.MODE_UNKNOWN
        except (socket.gaierror, urllib3.exceptions.NewConnectionError, urllib3.exceptions.MaxRetryError, requests.exceptions.HTTPError ,requests.exceptions.ReadTimeout, requests.exceptions.ConnectionError) as e:
            print("Connection Error. "+str(e))
            return DoorMode.MODE_UNKNOWN
