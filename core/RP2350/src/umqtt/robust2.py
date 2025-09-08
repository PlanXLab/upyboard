from utime import ticks_ms, ticks_diff
from . import simple2


class MQTTClient(simple2.MQTTClient):
    DEBUG = False
    KEEP_QOS0 = True
    NO_QUEUE_DUPS = True
    MSG_QUEUE_MAX = 5
    CONFIRM_QUEUE_MAX = 10
    RESUBSCRIBE = True

    def __init__(self, client_id:str, server:str, port:int=0, user:str=None, password:str=None, keepalive:int=0,
                 ssl:bool=False, ssl_params:dict=None, socket_timeout:int=5, message_timeout:int=10):
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
        time_from__last_cpackage = ticks_diff(ticks_ms(), self.last_cpacket) // 1000
        if 0 < self.keepalive < time_from__last_cpackage:
            self.conn_issue = (simple2.MQTTException(7), 9)
            return False
        return True

    def set_callback_status(self, f:callable):
        self._cbstat = f

    def cbstat(self, pid:int, stat:int):
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
        out = self.connect(False)
        if self.conn_issue:
            super().disconnect()
        return out

    def resubscribe(self):
        for topic, qos in self.subs:
            self.subscribe(topic, qos, False)

    def things_to_do(self):
        return len(self.msg_to_send) + \
            len(self.sub_to_send) + \
            sum([len(a) for a in self.msg_to_confirm.values()]) + \
            sum([len(a) for a in self.sub_to_confirm.values()])

    def add_msg_to_send(self, data:tuple):
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
        try:
            return super().disconnect()
        except (OSError, simple2.MQTTException) as e:
            self.conn_issue = (e, 6)

    def ping(self):
        if not self.is_keepalive():
            return
        try:
            return super().ping()
        except (OSError, simple2.MQTTException) as e:
            self.conn_issue = (e, 7)

    def publish(self, topic:str, msg:str, retain:bool=False, qos:int=0):
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
        data = (topic, qos)

        if self.RESUBSCRIBE and resubscribe:
            if topic not in dict(self.subs):
                self.subs.append(data)

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
        self.is_keepalive()

        if self.conn_issue:
            self.log()
        return bool(self.conn_issue)

    def wait_msg(self):
        self.is_keepalive()
        try:
            return super().wait_msg()
        except (OSError, simple2.MQTTException) as e:
            self.conn_issue = (e, 8)

    def check_msg(self):
        self.is_keepalive()
        try:
            return super().check_msg()
        except (OSError, simple2.MQTTException) as e:
            self.conn_issue = (e, 10)
