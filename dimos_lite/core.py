import threading
import queue
import time


class StreamOut:
    def __init__(self, name):
        self.name = name
        self.subscribers = []

    def add_subscriber(self, stream_in):
        if stream_in not in self.subscribers:
            self.subscribers.append(stream_in)

    def publish(self, data):
        for sub in self.subscribers:
            sub.put(data)


class StreamIn:
    def __init__(self, name):
        self.name = name
        self._q = queue.Queue(maxsize=10)
        self._callback = None

    def put(self, data):
        while True:
            try:
                self._q.put_nowait(data)
                break
            except queue.Full:
                try:
                    self._q.get_nowait()
                except queue.Empty:
                    pass

    def subscribe(self, callback):
        self._callback = callback
        t = threading.Thread(target=self._worker, daemon=True)
        t.start()

    def _worker(self):
        while True:
            data = self._q.get()
            if self._callback:
                try:
                    self._callback(data)
                except Exception as e:
                    print(f"[{self.name}] Callback error: {e}")
            self._q.task_done()


class Module:
    def __init__(self, name):
        self.name = name

    def start(self):
        pass


def autoconnect(*modules):
    outputs = {}
    inputs = {}
    for mod in modules:
        for attr_name in dir(mod):
            attr = getattr(mod, attr_name)
            if isinstance(attr, StreamOut):
                outputs[attr.name] = attr
            elif isinstance(attr, StreamIn):
                inputs[attr.name] = attr
    connections = 0
    for name, stream_out in outputs.items():
        if name in inputs:
            stream_out.add_subscriber(inputs[name])
            connections += 1
            print(f"[Autoconnect] {name}")
    print(f"[Autoconnect] {connections} stream(s) wired.")
