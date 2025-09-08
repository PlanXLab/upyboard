"""
Blynk MQTT Client Library

Advanced Blynk IoT platform integration for MicroPython devices with MQTT protocol.
Provides seamless connectivity to Blynk Cloud for real-time data exchange,
remote monitoring, and device control.

Features:

- MQTT-based communication with Blynk Cloud
- SSL/TLS secure connection support
- Datastream subscription and publishing
- Automatic connection management and keepalive
- UTC time synchronization from Blynk servers
- Remote device reboot capability
- Callback-based event handling system
- Robust error handling and reconnection
- Memory-efficient implementation for embedded systems
- Comprehensive logging and debugging support

Supported Operations:

- Publish sensor data to Blynk datastreams
- Subscribe to widget updates from mobile app
- Remote device control and configuration
- Real-time bidirectional communication
- Cloud-based data storage and visualization
- Mobile app notifications and alerts

Security:

- Device authentication via auth tokens
- Optional SSL/TLS encryption
- Secure server hostname verification
- Protected MQTT credential handling
"""

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
    """
    Blynk MQTT Client for secure IoT connectivity to Blynk Cloud platform.
    
    This class provides a robust MicroPython implementation for the Blynk IoT
    platform using MQTT protocol. It enables devices to communicate with
    Blynk Cloud for real-time data exchange, remote control, and monitoring
    through mobile applications and web dashboards.
    
    The client handles secure authentication, automatic connection management,
    datastream publishing/subscription, and provides callback-based event
    handling for incoming messages from the Blynk platform.
    
    Key Features:
    
        - Secure MQTT communication with SSL/TLS support
        - Automatic connection management and keepalive handling
        - Last Will and Testament (LWT) for offline status reporting
        - Robust reconnection logic with exponential backoff
        - Event-driven architecture using callbacks for datastreams
        - Support for publishing hardware info and setting widget properties
        - UTC time synchronization from Blynk servers
    
    Communication Flow:
    
        1. Connect to Blynk Cloud MQTT broker
        2. Subscribe to datastream updates from mobile app
        3. Publish sensor data to datastreams
        4. Handle remote control commands via callbacks
        5. Maintain connection with automatic ping/reconnect
    
    Security Features:
    
        - Device authentication via auth tokens
        - Optional SSL/TLS encryption with hostname verification
        - Secure MQTT credential handling
        - Protected message routing and validation
    
    """    
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

    def __init__(self, auth_token: str, server: str = "blynk.cloud", 
                 keepalive: int = 45, ssl: bool = False, verbose: bool = False) -> None:      
        """
        Initialize the Blynk MQTT Client with connection parameters.
        
        Creates a new Blynk MQTT client instance with the specified authentication
        token and connection settings. The client is ready to connect after
        initialization but does not establish a connection automatically.
        
        :param auth_token: Blynk authentication token from the Blynk Console
        :param server: Blynk server address (default: "blynk.cloud")
        :param keepalive: MQTT keepalive interval in seconds (default: 45)
        :param ssl: Enable SSL/TLS secure connection (default: False)
        :param verbose: Enable detailed logging for debugging (default: False)
        
        :raises ValueError: If the server address cannot be resolved
        
        Example
        --------
        ```python
            >>> # Basic client
            >>> client = BlynkMQTTClient(auth_token="YOUR_TOKEN")
            >>> 
            >>> # Secure client with logging
            >>> secure_client = BlynkMQTTClient(
            ...     auth_token="YOUR_TOKEN",
            ...     ssl=True,
            ...     verbose=True
            ... )
        ```
        """
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
        """Internal helper to publish messages, centralizing error handling and logging."""
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
        """Internal method to send MQTT ping and handle potential errors."""
        if time.ticks_diff(time.ticks_ms(), self.__last_ping) > self.__keepalive * 1000 / 2:
            try:
                self.__blynk_mqtt_client.ping()
                self.__last_ping = time.ticks_ms()
            except Exception as e:
                if self.__verbose:
                    print(f"[MQTT] Ping failed: {e}")
                self.__is_connected = False # Mark as disconnected on ping failure
                    
    def __on_message(self, topic: bytes, payload: bytes, retained: bool, duplicate: bool) -> None:
        """Internal callback to process all incoming MQTT messages."""
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
        """
        Connect to the Blynk MQTT broker with Last Will and Testament.
        
        Establishes a connection to the Blynk Cloud MQTT broker and subscribes
        to all downlink topics. Sets up LWT to automatically publish an "offline"
        status if disconnected unexpectedly.
        
        :param clean_session: Start a new session if True, or resume a previous one
        :return: True if connection is successful, False otherwise
        
        Example
        -------
        ```python
            >>> if client.connect():
            ...     print("Connected to Blynk")
            ... else:
            ...     print("Connection failed")
        ```
        """
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
        """
        Gracefully disconnect from the Blynk MQTT broker.
        
        Publishes an "offline" status before closing the connection to ensure
        proper cleanup and status reporting to the Blynk platform.
        
        Example
        -------
        ```python
            >>> client.disconnect()
            >>> print("Disconnected from Blynk")
        ```
        """
        if self.__is_connected:
            self.publish_device_status("offline")
            time.sleep_ms(100) # Allow time for the message to be sent
            self.__blynk_mqtt_client.disconnect()
            self.__is_connected = False
            if self.__verbose:
                print("Disconnected from MQTT broker.")

    def is_connected(self) -> bool:
        """
        Check if the client is currently connected to the Blynk MQTT broker.
        
        Returns the current connection status based on internal state tracking.
        Use this method to verify connectivity before performing operations.
        
        :return: True if connected, False otherwise
        
        Example
        -------
        ```python
            >>> if client.is_connected():
            ...     client.publish("V1", sensor_value)1
            ... else:
            ...     print("Not connected")
        ```
        """
        return self.__is_connected

    def reconnect(self, max_attempts: int = 5) -> bool:
        """
        Attempt to reconnect to the broker with exponential backoff.
        
        Tries to re-establish a lost connection using exponential backoff
        strategy. Useful for handling temporary network issues or server
        maintenance periods.
        
        :param max_attempts: Maximum number of reconnection attempts
        :return: True if reconnected successfully, False otherwise
        
        Example
        -------
        ```python
            >>> if not client.is_connected():
            ...     if client.reconnect():
            ...         print("Reconnected successfully")
            ...     else:
            ...         print("Failed to reconnect")
        ```
        """
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
        """
        Register a callback for a specific datastream or system event.
        
        Associates a callback function with a datastream to be called when
        data is received from the Blynk platform. Enables event-driven
        programming for handling remote control commands.
        
        :param datastream: Datastream name (e.g., "V1") or system event
                          ("reboot", "utc", "property", "loglevel", "__all__")
        :param callback: Function to call when message is received
        
        Callback Signatures:
        
            - Datastream: callback(value)
            - UTC: callback(payload)
            - Reboot: callback()
            - Property: callback(pin, property_name, value)
            - __all__: callback(topic, payload)
        
        Example
        -------
        ```python
            >>> def on_slider(value):
            ...     pwm.duty(int(float(value) * 10.23))
            >>> client.add_subscribe_callback("V1", on_slider)
        ```
        """
        self.__downlink_callbacks[datastream] = callback
        if self.__verbose:
            print(f"[SUB] Registered callback for '{datastream}'")

    def remove_subscribe_callback(self, datastream: str) -> None:
        """
        Remove a callback for a specific datastream.
        
        Removes the previously registered callback for the specified datastream.
        After removal, incoming data for this datastream will be ignored.
        
        :param datastream: Datastream name to remove callback for
        
        Example
        -------
        ```python
            >>> client.remove_subscribe_callback("V1")
        ```
        """
        if datastream in self.__downlink_callbacks:
            del self.__downlink_callbacks[datastream]
            if self.__verbose:
                print(f"[SUB] Removed callback for '{datastream}'")

    def publish(self, datastream: str, value: str | int | float, qos: int = 0) -> None:
        """
        Publish a value to a specific datastream.
        
        Sends data to the Blynk Cloud for a specific datastream. This data
        is displayed on the Blynk mobile app and can trigger events or
        notifications based on your project configuration.
        
        :param datastream: Datastream name (e.g., "V5")
        :param value: Value to publish (automatically converted to string)
        :param qos: MQTT Quality of Service level (0 or 1)
        
        Example
        -------
        ```python
            >>> # Publish sensor readings
            >>> client.publish("V2", temperature)
            >>> client.publish("V3", humidity)
        ```
        """
        topic = f"{self.__UPLINK_TOPIC_DS_HEADER}/{datastream}"
        self.__publish(topic, str(value), qos=qos)

    def publish_device_status(self, status: str) -> None:
        """
        Publish device online/offline status to Blynk.
        
        Updates the device status on the Blynk platform. This is handled
        automatically by connect() and disconnect() but can be called
        manually if needed.
        
        :param status: "online" or "offline"
        
        Example
        -------
        ```python
            >>> client.publish_device_status("online")
        ```
        """
        if status not in ("online", "offline"):
            raise ValueError("Status must be 'online' or 'offline'")
        payload = json.dumps({"status": status})
        self.__publish(self.__UPLINK_TOPIC_DEVICE_STATUS, payload, retain=True, qos=1)

    def publish_hardware_info(self, info: dict) -> None:
        """
        Publish device hardware information to Blynk.
        
        Sends hardware details about the device to the Blynk Cloud.
        Typically sent once after connecting to inform the platform
        about device capabilities.
        
        :param info: Dictionary containing hardware details
        
        Example
        -------
        ```python
            >>> hw_info = {"board": "RP2350", "firmware": "1.26.0"}
            >>> client.publish_hardware_info(hw_info)
        ```
        """
        payload = json.dumps(info)
        self.__publish(self.__UPLINK_TOPIC_HARDWARE, payload, qos=1)

    def set_property(self, datastream: str, prop: str, value: str | int | float) -> None:
        """
        Set a property for a specific datastream widget.
        
        Dynamically updates widget properties such as label, color, or
        min/max values. This allows runtime customization of the mobile
        app interface based on device state or user preferences.
        
        :param datastream: Target datastream name (e.g., "V2")
        :param prop: Property to change (e.g., "label", "color", "min", "max")
        :param value: New value for the property
        
        Example
        -------
        ```python
            >>> client.set_property("V2", "label", "CPU Temp")
            >>> client.set_property("V2", "color", "#FF0000")
        ```
        """
        topic = f"{self.__UPLINK_TOPIC_PROPERTY_DEVICE}/{datastream}/{prop}"
        self.__publish(topic, str(value), qos=1)

    def get_utc(self, timeout_ms: int = 5000) -> dict | None:
        """
        Get the current UTC time from the Blynk server with timeout.
        
        Requests and retrieves the current UTC time from the Blynk Cloud
        server. Useful for devices without a real-time clock to synchronize
        time. The method blocks until response is received or timeout occurs.
        
        :param timeout_ms: Maximum time to wait for response in milliseconds
        :return: Dictionary with "time" (ISO8601 format) and "zone", or None on timeout
        
        Example
        -------
        ```python
            >>> time_data = client.get_utc()
            >>> if time_data:
            ...     print(f"Current time: {time_data['time']}")
        ```
        """
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
        """
        Process pending messages and maintain connection in non-blocking manner.
        
        This method should be called regularly in your main loop to handle
        incoming data, send queued messages, and maintain the connection
        via pings. It includes automatic reconnection logic for lost connections.
        
        Example
        -------
        ```python
            >>> while True:
            ...     client.loop()
            ...     # Your application code here
            ...     time.sleep_ms(10)
        ```
        """
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
        """
        Run the client loop indefinitely (blocking).
        
        This method handles all communication automatically and will block
        the execution of subsequent code. Use this when you don't need to
        run other code alongside Blynk communication.
        
        Example
        -------
        ```python
            >>> # This will run forever, processing Blynk events
            >>> client.loop_forever()
        ```
        """
        while True:
            self.loop()
            time.sleep_ms(10)
