from utime import ticks_ms, ticks_diff
from . import simple2


class MQTTClient(simple2.MQTTClient):
    DEBUG = False

    # Information whether we store unsent messages with the flag QoS==0 in the queue.
    KEEP_QOS0 = True
    # Option, limits the possibility of only one unique message being queued.
    NO_QUEUE_DUPS = True
    # Limit the number of unsent messages in the queue.
    MSG_QUEUE_MAX = 5
    # How many PIDs we store for a sent message
    CONFIRM_QUEUE_MAX = 10
    # When you reconnect, all existing subscriptions are renewed.
    RESUBSCRIBE = True


    def __init__(self, client_id:str, server:str, port:int=0, user:str=None, password:str=None, keepalive:int=0,
                 ssl:bool=False, ssl_params:dict=None, socket_timeout:int=5, message_timeout:int=10):
        """
        initializes MQTTClient object.
        
        :param client_id: Unique MQTT ID attached to client. It must be unique for each client.
        :param server: MQTT host address.
        :param port: MQTT Port, typically 1883. If unset, the port number will default to 1883 of 8883 base on ssl.
        :param user: Username if your server requires it.
        :param password: Password if your server requires it.
        :param keepalive: The Keep Alive is a time interval measured in seconds since the last correct control packet was received.
        :param ssl: Require SSL for the connection.
        :param ssl_params: Required SSL parameters. Kwargs from function ssl.wrap_socket.
        :param socket_timeout: The time in seconds after which the socket interrupts the connection to the server when no data exchange takes place. None - socket blocking, positive number - seconds to wait.
        :param message_timeout: The time in seconds after which the library recognizes that a message with QoS=1 or topic subscription has not been received by the server.
        """
        super().__init__(client_id=client_id, server=server, port=port, user=user, password=password,
                         keepalive=keepalive, ssl=ssl, ssl_params=ssl_params, socket_timeout=socket_timeout,
                         message_timeout=message_timeout)
        self.subs = []  # List of stored subscriptions [ (topic, qos), ...]
        # Queue with list of messages to send
        self.msg_to_send = []  # [(topic, msg, retain, qos), (topic, msg, retain, qos), ... ]
        # Queue with list of subscriptions to send
        self.sub_to_send = []  # [(topic, qos), ...]
        # Queue with a list of messages waiting for the server to confirm of the message.
        self.msg_to_confirm = {}  # {(topic, msg, retain, qos): [pid, pid, ...]
        # Queue with a subscription list waiting for the server to confirm of the subscription
        self.sub_to_confirm = {}  # {(topic, qos): [pid, pid, ...]}
        self.conn_issue = None  # We store here if there is a connection problem.

    def is_keepalive(self):
        """
        It checks if the connection is active. If the connection is not active at the specified time,
        saves an error message and returns False.

        :return: If the connection is not active at the specified time returns False otherwise True.
        """
        time_from__last_cpackage = ticks_diff(ticks_ms(), self.last_cpacket) // 1000
        if 0 < self.keepalive < time_from__last_cpackage:
            self.conn_issue = (simple2.MQTTException(7), 9)
            return False
        return True

    def set_callback_status(self, f:callable):
        """
        See documentation for `umqtt.simple2.MQTTClient.set_callback_status()`
        """
        self._cbstat = f

    def cbstat(self, pid:int, stat:int):
        """
        Captured message statuses affect the queue here.

        :param pid: PID of the message or subscription
        :param stat: Status of the message or subscription
            stat == 0 - the message goes back to the message queue to be sent
            stat == 1 or 2 - the message is removed from the queue
        """
        try:
            self._cbstat(pid, stat)
        except AttributeError:
            pass

        for data, pids in self.msg_to_confirm.items():
            if pid in pids:
                if stat == 0:
                    if data not in self.msg_to_send:
                        self.msg_to_send.insert(0, data)
                    pids.remove(pid)
                    if not pids:
                        self.msg_to_confirm.pop(data)
                elif stat in (1, 2):
                    # A message has been delivered at least once, so we are not waiting for other confirmations
                    self.msg_to_confirm.pop(data)
                return

        for data, pids in self.sub_to_confirm.items():
            if pid in pids:
                if stat == 0:
                    if data not in self.sub_to_send:
                        self.sub_to_send.append(data)
                    pids.remove(pid)
                    if not pids:
                        self.sub_to_confirm.pop(data)
                elif stat in (1, 2):
                    # A message has been delivered at least once, so we are not waiting for other confirmations
                    self.sub_to_confirm.pop(data)

    def connect(self, clean_session:bool=True):
        """
        See documentation for `umqtt.simple2.MQTTClient.connect()`.
        If clean_session==True, then the queues are cleared.

        Connection problems are captured and handled by `is_conn_issue()`

        :param clean_session: If True, clears the message and subscription queues before connecting.
        :return: None or True if the connection is successful.
        """
        if clean_session:
            self.msg_to_send[:] = []
            self.msg_to_confirm.clear()
        try:
            out = super().connect(clean_session)
            self.conn_issue = None
            return out
        except (OSError, simple2.MQTTException) as e:
            self.conn_issue = (e, 1)

    def log(self):
        """
        This function is used to log connection issues.
        It prints the last connection issue to the console if DEBUG is enabled.
        The connection issue is stored in `self.conn_issue` and can be checked using `is_conn_issue()`.
        It is best to call this function after checking the connection issue with `is_conn_issue()`.
        """
        if self.DEBUG:
            if type(self.conn_issue) is tuple:
                conn_issue, issue_place = self.conn_issue
            else:
                conn_issue = self.conn_issue
                issue_place = 0
            place_str = ('?', 'connect', 'publish', 'subscribe',
                         'reconnect', 'sendqueue', 'disconnect', 'ping', 'wait_msg', 'keepalive', 'check_msg')
            print("MQTT (%s): %r" % (place_str[issue_place], conn_issue))

    def reconnect(self):
        """
        The function tries to resume the connection.

        Connection problems are captured and handled by `is_conn_issue()`
        """
        out = self.connect(False)
        if self.conn_issue:
            super().disconnect()
        return out

    def resubscribe(self):
        """
        Function from previously registered subscriptions, sends them again to the server.

        :return:
        """
        for topic, qos in self.subs:
            self.subscribe(topic, qos, False)

    def things_to_do(self):
        """
        The sum of all actions in the queues.

        When the value equals 0, it means that the library has sent and confirms the sending:
          * all messages
          * all subscriptions

        When the value equals 0, it means that the device can go into hibernation mode,
        assuming that it has not subscribed to some topics.

        :return: 0 (nothing to do) or int (number of things to do)
        """
        return len(self.msg_to_send) + \
            len(self.sub_to_send) + \
            sum([len(a) for a in self.msg_to_confirm.values()]) + \
            sum([len(a) for a in self.sub_to_confirm.values()])

    def add_msg_to_send(self, data:tuple):
        """
        By overwriting this method, you can control the amount of stored data in the queue.
        
        This is important because we do not have an infinite amount of memory in the devices.
        Currently, this method limits the queue length to MSG_QUEUE_MAX messages.
        The number of active messages is the sum of messages to be sent with messages awaiting confirmation.
        
        :param data: tuple (topic, msg, retain, qos) or (topic, qos) for subscriptions
        """
        # Before we add data to the queue, it is necessary to release the memory first.
        # Otherwise, we may fall into an infinite loop due to a lack of available memory.
        messages_count = len(self.msg_to_send)
        messages_count += sum(map(len, self.msg_to_confirm.values()))

        while messages_count >= self.MSG_QUEUE_MAX:
            min_msg_to_confirm = min(map(lambda x: x[0] if x else 65535, self.msg_to_confirm.values()), default=0)
            if 0 < min_msg_to_confirm < 65535:
                key_to_check = None
                for k, v in self.msg_to_confirm.items():
                    if v and v[0] == min_msg_to_confirm:
                        del v[0]
                        key_to_check = k
                        break
                if key_to_check and key_to_check in self.msg_to_confirm and not self.msg_to_confirm[key_to_check]:
                    self.msg_to_confirm.pop(key_to_check)
            else:
                self.msg_to_send.pop(0)
            messages_count -= 1

        self.msg_to_send.append(data)

    def disconnect(self):
        """
        See documentation for `umqtt.simple2.MQTTClient.disconnect()`
        Connection problems are captured and handled by `is_conn_issue()`
        """
        try:
            return super().disconnect()
        except (OSError, simple2.MQTTException) as e:
            self.conn_issue = (e, 6)

    def ping(self):
        """
        See documentation for `umqtt.simple2.MQTTClient.ping()`
        Connection problems are captured and handled by `is_conn_issue()`
        """
        if not self.is_keepalive():
            return
        try:
            return super().ping()
        except (OSError, simple2.MQTTException) as e:
            self.conn_issue = (e, 7)

    def publish(self, topic:str, msg:str, retain:bool=False, qos:int=0):
        """
        See documentation for `umqtt.simple2.MQTTClient.publish()`
        The function tries to send a message. If it fails, the message goes to the message queue for sending.
        The function does not support the `dup` parameter!
        When we have messages with the retain flag set, only one last message with that flag is sent!
        Connection problems are captured and handled by `is_conn_issue()`

        :param topic: Topic to publish the message to
        :param msg: Message to be sent
        :param retain: If True, the message will be retained by the broker
        :param qos: Quality of Service level (0, 1, or 2)
        :return: None od PID for QoS==1 (only if the message is sent immediately, otherwise it returns None)
        """
        data = (topic, msg, retain, qos)
        if retain:
            # We delete all previous messages for this topic with the retain flag set to True.
            # Only the last message with this flag is relevant.
            self.msg_to_send[:] = [m for m in self.msg_to_send if not (topic == m[0] and retain == m[2])]
        try:
            out = super().publish(topic, msg, retain, qos, False)
            if qos == 1:
                # We postpone the message in case it is not delivered to the server.
                # We will delete it when we receive a receipt.
                self.msg_to_confirm.setdefault(data, []).append(out)
                if len(self.msg_to_confirm[data]) > self.CONFIRM_QUEUE_MAX:
                    self.msg_to_confirm.pop(0)

            return out
        except (OSError, simple2.MQTTException) as e:
            self.conn_issue = (e, 2)
            # If the message cannot be sent, we put it in the queue to try to resend it.
            if self.NO_QUEUE_DUPS:
                if data in self.msg_to_send:
                    return
            if self.KEEP_QOS0 and qos == 0:
                self.add_msg_to_send(data)
            elif qos == 1:
                self.add_msg_to_send(data)

    def subscribe(self, topic:str, qos:int=0, resubscribe:bool=True):
        """
        See documentation for `umqtt.simple2.MQTTClient.subscribe()`
        The function tries to subscribe to the topic. If it fails,
        the topic subscription goes into the subscription queue.
        Connection problems are captured and handled by `is_conn_issue()`
        
        :param topic: Topic to subscribe to
        :param qos: Quality of Service level (0, 1, or 2)
        :param resubscribe: If True, the topic will be re-subscribed if it already exists in the subscription list.
                            This is useful when reconnecting to the broker.
        :return: PID of the subscription (if successful) or None if it fails
        """
        data = (topic, qos)

        if self.RESUBSCRIBE and resubscribe:
            if topic not in dict(self.subs):
                self.subs.append(data)

        # We delete all previous subscriptions for the same topic from the queue.
        # The most important is the last subscription.
        self.sub_to_send[:] = [s for s in self.sub_to_send if topic != s[0]]
        try:
            out = super().subscribe(topic, qos)
            self.sub_to_confirm.setdefault(data, []).append(out)
            if len(self.sub_to_confirm[data]) > self.CONFIRM_QUEUE_MAX:
                self.sub_to_confirm.pop(0)
            return out
        except (OSError, simple2.MQTTException) as e:
            self.conn_issue = (e, 3)
            if self.NO_QUEUE_DUPS:
                if data in self.sub_to_send:
                    return
            self.sub_to_send.append(data)

    def send_queue(self):
        """
        The function tries to send all messages and subscribe to all topics that are in the queue to send.

        :return: True if the queue's empty.
        """
        msg_to_del = []
        for data in self.msg_to_send:
            topic, msg, retain, qos = data
            try:
                out = super().publish(topic, msg, retain, qos, False)
                if qos == 1:
                    # We postpone the message in case it is not delivered to the server.
                    # We will delete it when we receive a receipt.
                    self.msg_to_confirm.setdefault(data, []).append(out)
                msg_to_del.append(data)
            except (OSError, simple2.MQTTException) as e:
                self.conn_issue = (e, 5)
                return False
        self.msg_to_send[:] = [m for m in self.msg_to_send if m not in msg_to_del]
        del msg_to_del

        sub_to_del = []
        for data in self.sub_to_send:
            topic, qos = data
            try:
                out = super().subscribe(topic, qos)
                self.sub_to_confirm.setdefault(data, []).append(out)
                sub_to_del.append(data)
            except (OSError, simple2.MQTTException) as e:
                self.conn_issue = (e, 5)
                return False
        self.sub_to_send[:] = [s for s in self.sub_to_send if s not in sub_to_del]

        return True

    def is_conn_issue(self):
        """
        With this function we can check if there is any connection problem.
        It is best to use this function with the reconnect() method to resume the connection when it is broken.

        You can also check the result of methods such as this:
        `connect()`, `publish()`, `subscribe()`, `reconnect()`, `send_queue()`, `disconnect()`, `ping()`, `wait_msg()`,
        `check_msg()`, `is_keepalive()`.

        The value of the last error is stored in self.conn_issue.

        :return: Connection problem
        """
        self.is_keepalive()

        if self.conn_issue:
            self.log()
        return bool(self.conn_issue)

    def wait_msg(self):
        """
        See documentation for `umqtt.simple2.MQTTClient.wait_msg()`

        Connection problems are captured and handled by `is_conn_issue()`
        """
        self.is_keepalive()
        try:
            return super().wait_msg()
        except (OSError, simple2.MQTTException) as e:
            self.conn_issue = (e, 8)

    def check_msg(self):
        """
        See documentation for `umqtt.simple2.MQTTClient.check_msg()`

        Connection problems are captured and handled by `is_conn_issue()`
        """
        self.is_keepalive()
        try:
            return super().check_msg()
        except (OSError, simple2.MQTTException) as e:
            self.conn_issue = (e, 10)
