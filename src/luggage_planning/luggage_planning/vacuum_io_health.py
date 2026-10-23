"""Fresh IO samples and bounded service discovery; no ROS or second CPS socket."""
import threading


class FreshDigitalInput:
    def __init__(self, now, max_age_sec=2.0):
        self.now, self.max_age_sec = now, float(max_age_sec)
        self.received_at, self.raw = None, None

    def update(self, value):
        self.raw, self.received_at = int(bool(value)), self.now()

    def age(self):
        return None if self.received_at is None else self.now() - self.received_at

    def value(self):
        age = self.age()
        return self.raw if age is not None and age <= self.max_age_sec else None


class VacuumIoTransport:
    def __init__(self, clients, request_factory, now, sleep, log,
                 service_timeout=2.0, call_timeout=2.0, connect_attempts=2, retry_period=.2):
        self.clients, self.request_factory = clients, request_factory
        self.now, self.sleep, self.log = now, sleep, log
        self.service_timeout, self.call_timeout = float(service_timeout), float(call_timeout)
        self.connect_attempts, self.retry_period = int(connect_attempts), float(retry_period)
        if min(self.service_timeout, self.call_timeout, self.connect_attempts) <= 0 or self.retry_period < 0:
            raise ValueError("invalid vacuum IO timeouts/attempt count")

    def set_do(self, bit, value):
        client = self.clients[int(bit)]
        start = self.now()
        for attempt in range(1, self.connect_attempts + 1):
            try:
                ready = client.wait_for_service(timeout_sec=self.service_timeout)
            except Exception as exc:
                message = "VACUUM_IO_DISCOVERY_EXCEPTION: %s" % exc
                self.log(dict(event="vacuum_io_result", bit=bit, value=value,
                              ok=False, message=message, elapsed_sec=self.now()-start))
                return False, message
            self.log(dict(event="vacuum_io_discovery", bit=bit, value=value, attempt=attempt, ready=ready))
            if ready:
                break
            if attempt < self.connect_attempts:
                self.sleep(self.retry_period)
        else:
            return False, "VACUUM_IO_SERVICE_UNAVAILABLE bit=%d attempts=%d" % (bit, self.connect_attempts)
        try:
            future = client.call_async(self.request_factory(bool(value)))
            done = threading.Event()
            future.add_done_callback(lambda _: done.set())
            if not done.wait(self.call_timeout):
                future.cancel()
                # An RPC timeout has unknown physical outcome. Never replay
                # an old pump-on after a later release command.
                message = "VACUUM_IO_CALL_TIMEOUT bit=%d value=%d; outcome unknown" % (bit, value)
                self.log(dict(event="vacuum_io_result", ok=False, message=message, elapsed_sec=self.now()-start))
                return False, message
            response = future.result()
            ok = response is not None and bool(response.success)
            message = str(response.message) if response is not None else "VACUUM_IO_EMPTY_RESPONSE"
        except Exception as exc:
            ok, message = False, "VACUUM_IO_EXCEPTION: %s" % exc
        self.log(dict(event="vacuum_io_result", bit=bit, value=value, ok=ok,
                      message=message, elapsed_sec=self.now()-start))
        return ok, message
