import time
import sys
import signal
from eye import Eye


if __name__ == "__main__":
    e = Eye()

    def on_terminate(signal, frame):
        e.turn_off()
        sys.exit()

    signal.signal(signal.SIGINT, on_terminate)

    while(1):
        time.sleep(1)
