"""Display telemetry text forwarded by the terminal controller."""

import argparse
import socket


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", type=int, default=5011)
    args = parser.parse_args()

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(("127.0.0.1", args.port))
    print(f"Telemetry log listening on 127.0.0.1:{args.port}")
    print("Close this window to stop logging.\n")
    try:
        while True:
            data, _ = sock.recvfrom(4096)
            print(data.decode("utf-8", errors="replace"), flush=True)
    except (KeyboardInterrupt, OSError):
        pass
    finally:
        sock.close()


if __name__ == "__main__":
    main()

