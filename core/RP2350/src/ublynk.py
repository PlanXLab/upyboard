import sys
import time
import json
import usocket
import ubinascii

import machine
from umqtt.robust2 import MQTTClient


__version__ = "1.0.0"
__author__ = "PlanXLab Development Team"

print("""
    ___  __          __
   / _ )/ /_ _____  / /__
  / _  / / // / _ \\/  '_/
 /____/_/\\_, /_//_/_/\\_\\2@MQTT
        /___/ for uPython v""" + __version__ + " (" + sys.platform + ")\n")


class BlynkMQTTClient:
    __DOWNLINK = "downlink/"
    __DOWNLINK_TOPIC_ALL = __DOWNLINK + "#"
    __DOWNLINK_TOPIC_DS_HEADER = __DOWNLINK + "ds"
    __DOWNLINK_TOPIC_UTC = __DOWNLINK + "utc/all/json"
    __DOWNLINK_TOPIC_REBOOT = __DOWNLINK + "reboot"
    __DOWNLINK_TOPIC_LOGLEVEL = __DOWNLINK + "loglevel"
    __DOWNLINK_TOPIC_PROPERTY = __DOWNLINK + "property"
    
    __UPLINK = "uplink/"
    __UPLINK_TOPIC_DS_HEADER = "ds"
    __UPLINK_TOPIC_UTC = "get/utc/all/json"
    __UPLINK_TOPIC_PROPERTY_DEVICE = "property/device"
    __UPLINK_TOPIC_HARDWARE = "hardware"
    __UPLINK_TOPIC_DEVICE_STATUS = "device/status"
    
    __USER_NAME = "device"

    def __init__(self, auth_token: str, *, 
                 server: str = "blynk.cloud", keepalive: int = 45, ssl: bool = False, verbose: bool = False) -> None:
        self.__blynk_server_name = server
        self.__blynk_server_port = 8883 if ssl else 1883
        self.__keepalive = keepalive
        self.__ssl = ssl
        self.__verbose = verbose
        
        try:
            usocket.getaddrinfo(self.__blynk_server_name, self.__blynk_server_port)
        except OSError as e:
            if self.__verbose:
                print(f"[MQTT] Failed to resolve server {self.__blynk_server_name}:{self.__blynk_server_port} - {e}")
            raise ValueError(f"Invalid server address: {self.__blynk_server_name}")
        
        self.__blynk_mqtt_client = MQTTClient(
            client_id=ubinascii.hexlify(machine.unique_id()).decode(),
            server=self.__blynk_server_name,
            port=self.__blynk_server_port,
            user=self.__USER_NAME,
            password=auth_token,
            keepalive=self.__keepalive,
            ssl=self.__ssl,
            ssl_params={"server_hostname": self.__blynk_server_name} if self.__ssl else {}
        )
        self.__blynk_mqtt_client.DEBUG = self.__verbose

        self.__downlink_callbacks = {}
        self.__is_connected = False
        self.__last_ping = 0
    
    def __publish(self, topic: str, payload: str, retain: bool = False, qos: int = 0) -> None:
        try:
            if not self.is_connected():
                if self.__verbose:
                    print(f"[MQTT] Not connected. Queuing message for topic: {topic}")
            
            self.__blynk_mqtt_client.publish(topic.encode(), payload.encode(), retain, qos)
            self.__last_ping = time.ticks_ms()
            if self.__verbose:
                print(f"[MQTT TX] Topic: {topic}, Payload: {payload}")
        except Exception as e:
            if self.__verbose:
                print(f"MQTT publish error: {e}")

    def __send_ping(self) -> None:
        if time.ticks_diff(time.ticks_ms(), self.__last_ping) > self.__keepalive * 1000 / 2:
            try:
                self.__blynk_mqtt_client.ping()
                self.__last_ping = time.ticks_ms()
            except Exception as e:
                if self.__verbose:
                    print(f"[MQTT] Ping failed: {e}")
                self.__is_connected = False # Mark as disconnected on ping failure
                    
    def __on_message(self, topic: bytes, payload: bytes, retained: bool, duplicate: bool) -> None:
        topic_str = topic.decode()
        payload_str = payload.decode()
        if self.__verbose:
            print(f"[MQTT RX] Topic: {topic_str}, Payload: {payload_str}")

        try:
            if "__all__" in self.__downlink_callbacks:
                self.__downlink_callbacks["__all__"](topic_str, payload_str)

            elif topic_str.startswith(self.__DOWNLINK_TOPIC_DS_HEADER + "/"):
                ds = topic_str.split("/")[-1]
                if ds in self.__downlink_callbacks:
                    self.__downlink_callbacks[ds](payload_str)

            elif topic_str == self.__DOWNLINK_TOPIC_UTC:
                if "utc" in self.__downlink_callbacks:
                    self.__downlink_callbacks["utc"](payload_str)

            elif topic_str == self.__DOWNLINK_TOPIC_REBOOT:
                if "reboot" in self.__downlink_callbacks:
                    self.__downlink_callbacks["reboot"]()
            
            elif topic_str == self.__DOWNLINK_TOPIC_LOGLEVEL:
                if "loglevel" in self.__downlink_callbacks:
                    self.__downlink_callbacks["loglevel"](payload_str)

            elif topic_str.startswith(self.__DOWNLINK_TOPIC_PROPERTY + "/"):
                if "property" in self.__downlink_callbacks:
                    parts = topic_str.split('/')
                    if len(parts) >= 4:
                        pin, prop_name = parts[2], parts[3]
                        self.__downlink_callbacks["property"](pin, prop_name, payload_str)
        except KeyError:
            if self.__verbose:
                print(f"[MQTT] No callback registered for topic: {topic_str}")
        except Exception as e:
            if self.__verbose:
                print(f"[MQTT] Error in callback for topic {topic_str}: {e}")

    def connect(self, clean_session: bool = True) -> bool:
        if self.__is_connected:
            return True
        
        lwt_payload = '{"status":"offline"}'
        self.__blynk_mqtt_client.set_last_will(
            self.__UPLINK_TOPIC_DEVICE_STATUS.encode(),
            lwt_payload.encode(),
            retain=True,
            qos=1
        )

        if self.__blynk_mqtt_client.connect(clean_session) is not None:
            self.__blynk_mqtt_client.set_callback(self.__on_message)
            self.__blynk_mqtt_client.subscribe(self.__DOWNLINK_TOPIC_ALL.encode(), 1)
            
            self.publish_device_status("online")
            
            self.__is_connected = True
            self.__last_ping = time.ticks_ms()
            if self.__verbose:
                print("Connected to Blynk MQTT broker and subscribed to downlink topics.")
            return True
        
        if self.__verbose:
            print("Failed to connect to Blynk MQTT broker.")
        self.__is_connected = False
        return False

    def disconnect(self) -> None:
        if self.__is_connected:
            self.publish_device_status("offline")
            time.sleep_ms(100) # Allow time for the message to be sent
            self.__blynk_mqtt_client.disconnect()
            self.__is_connected = False
            if self.__verbose:
                print("Disconnected from MQTT broker.")

    def is_connected(self) -> bool:
        return self.__is_connected

    def reconnect(self, max_attempts: int = 5) -> bool:
        if self.is_connected():
            return True
        
        for attempt in range(max_attempts):
            if self.__verbose:
                print(f"Reconnection attempt {attempt + 1}/{max_attempts}...")
            if self.connect(clean_session=False):
                if self.__verbose:
                    print("Reconnection successful.")
                return True
            # Exponential backoff
            time.sleep(2 ** attempt)
        
        if self.__verbose:
            print("Failed to reconnect after multiple attempts.")
        return False

    def add_subscribe_callback(self, datastream: str, callback: callable) -> None:
        self.__downlink_callbacks[datastream] = callback
        if self.__verbose:
            print(f"[SUB] Registered callback for '{datastream}'")

    def remove_subscribe_callback(self, datastream: str) -> None:
        if datastream in self.__downlink_callbacks:
            del self.__downlink_callbacks[datastream]
            if self.__verbose:
                print(f"[SUB] Removed callback for '{datastream}'")

    def publish(self, datastream: str, value: str | int | float, qos: int = 0) -> None:
        topic = f"{self.__UPLINK_TOPIC_DS_HEADER}/{datastream}"
        self.__publish(topic, str(value), qos=qos)

    def publish_device_status(self, status: str) -> None:
        if status not in ("online", "offline"):
            raise ValueError("Status must be 'online' or 'offline'")
        payload = json.dumps({"status": status})
        self.__publish(self.__UPLINK_TOPIC_DEVICE_STATUS, payload, retain=True, qos=1)

    def publish_hardware_info(self, info: dict) -> None:
        payload = json.dumps(info)
        self.__publish(self.__UPLINK_TOPIC_HARDWARE, payload, qos=1)

    def set_property(self, datastream: str, prop: str, value: str | int | float) -> None:
        topic = f"{self.__UPLINK_TOPIC_PROPERTY_DEVICE}/{datastream}/{prop}"
        self.__publish(topic, str(value), qos=1)

    def get_utc(self, timeout_ms: int = 5000) -> dict | None:
        data = None
        
        def __on_utc(payload):
            nonlocal data
            try:
                t_data = json.loads(payload)
                data = {"time": t_data.get("iso8601"), "zone": t_data.get("tz_name")}
            except (ValueError, KeyError) as e:
                if self.__verbose:
                    print(f"Error parsing UTC response: {e}")
                data = {} # Indicate a response was received but was invalid

        self.add_subscribe_callback("utc", __on_utc)
        self.__publish(self.__UPLINK_TOPIC_UTC, "", qos=1)
        
        start_time = time.ticks_ms()
        while data is None:
            self.loop() # Process messages
            if time.ticks_diff(time.ticks_ms(), start_time) > timeout_ms:
                if self.__verbose:
                    print(f"UTC time request timed out after {timeout_ms} ms")
                break
            time.sleep_ms(50)
        
        self.remove_subscribe_callback("utc")
        return data

    def loop(self) -> None:
        if not self.is_connected():
            # Attempt to reconnect if connection is lost
            self.reconnect(max_attempts=1)
            return

        try:
            self.__blynk_mqtt_client.check_msg()
            self.__send_ping()
            # Let robust2 handle sending queued messages
            self.__blynk_mqtt_client.send_queue()
        except Exception as e:
            if self.__verbose:
                print(f"Error in loop: {e}")
            self.__is_connected = False

    def loop_forever(self) -> None:
        while True:
            self.loop()
            time.sleep_ms(10)

