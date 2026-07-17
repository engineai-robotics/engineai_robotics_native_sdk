from .singleton_meta import SingletonMeta
import lcm


class LcmManager(metaclass=SingletonMeta):
    def __init__(self):
        self._lcm_handle = None
        self._subscribers = []

    @property
    def lcm_handle(self):
        return self._lcm_handle

    def connect(self, lcm_url="udpm://239.255.76.67:7667"):
        """
        Connect to LCM
        """
        try:
            self._lcm_handle = lcm.LCM(lcm_url)
            print(f"Connected to LCM: {lcm_url}")
            self._notify_connection_status(True)
            return True, "Connection successful"
        except Exception as e:
            return False, f"Connection failed: {str(e)}"

    def disconnect(self):
        """
        Disconnect from LCM
        """
        if self._lcm_handle:
            try:
                # Close the LCM connection safely.
                self._lcm_handle.close()
            except Exception as e:
                print(f"Error while closing LCM connection: {e}")
            finally:
                self._lcm_handle = None
                self._notify_connection_status(False)
                print("LCM connection disconnected")

    def safe_disconnect(self):
        """
        Disconnect from LCM safely with extra cleanup steps.
        """
        print("Starting safe LCM disconnect")
        if self._lcm_handle:
            try:
                # Try to process any pending messages first.
                try:
                    self._lcm_handle.handle_timeout(0)
                except:
                    pass

                # The LCM object does not expose close(), so clear the handle directly.
                self._lcm_handle = None
                print("LCM connection safely closed")
            except Exception as e:
                print(f"Error during safe LCM disconnect: {e}")
            finally:
                self._lcm_handle = None
                self._notify_connection_status(False)
                print("LCM resources cleaned up")

    def is_connected(self):
        """
        Check LCM connection status
        """
        return self._lcm_handle is not None

    def add_connection_listener(self, callback):
        """
        Add a connection status listener
        """
        if callback not in self._subscribers:
            self._subscribers.append(callback)

    def remove_connection_listener(self, callback):
        """
        Remove a connection status listener
        """
        if callback in self._subscribers:
            self._subscribers.remove(callback)

    def _notify_connection_status(self, connected):
        """
        Notify all listeners of connection status changes
        """
        for callback in self._subscribers:
            try:
                callback(connected)
            except Exception as e:
                print(f"Error while notifying connection listeners: {e}")

    def lcm_handle_with_blocking(self, timeout=200):
        return self._lcm_handle.handle_timeout(timeout)

    def safe_handle_timeout(self, timeout=10):
        """
        Safely handle an LCM timeout.
        """
        try:
            if self._lcm_handle:
                return self._lcm_handle.handle_timeout(timeout)
        except Exception as e:
            print(f"Error while handling LCM timeout: {e}")
        return 0
