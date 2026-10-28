"""Python 3 standard-library UDP parameter setter / brake sender."""
import argparse
import math
import socket
import struct
import time


def config_packet(full, release, hold):
    if not all(math.isfinite(x) for x in (full, release)):
        raise ValueError('Positions must be finite')
    f, r = round(full * 10), round(release * 10)
    if not (0 <= f < r <= 1000 and r - f > 100 and 0 <= hold <= 10000):
        raise ValueError('Require 0 <= full < release <= 100mm, travel > 10mm, hold 0..10000ms')
    data = b'BRC1' + struct.pack('!HHH', f, r, hold)
    return data + struct.pack('!H', sum(data))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--ip', default='10.177.21.3')
    parser.add_argument('--port', type=int, default=5000)
    sub = parser.add_subparsers(dest='action', required=True)
    cfg = sub.add_parser('set')
    cfg.add_argument('--full', type=float, required=True, help='absolute sensor position mm')
    cfg.add_argument('--release', type=float, required=True, help='absolute sensor position mm')
    cfg.add_argument('--hold-ms', type=int, required=True)
    on = sub.add_parser('on')
    on.add_argument('--seconds', type=float, default=5)
    sub.add_parser('off')
    args = parser.parse_args()
    target = (socket.gethostbyname(args.ip), args.port)
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
        sock.connect(target)  # filters responses to this peer; UDP has no handshake
        if args.action == 'set':
            packet = config_packet(args.full, args.release, args.hold_ms)
            sock.settimeout(2)
            sock.send(packet)
            try:
                reply = sock.recv(64)
            except socket.timeout:
                raise SystemExit('No acknowledgement: configuration result unknown. Check network/serial.')
            if len(reply) != 4 or reply[:3] != b'BRA':
                raise SystemExit('Unexpected acknowledgement')
            messages = {0: 'Applied (RAM only)', 1: 'Rejected: format/checksum',
                        2: 'Rejected: parameter range', 3: 'Rejected: busy; release brake and wait for stop'}
            print(messages.get(reply[3], 'Unknown status'))
            if reply[3]:
                raise SystemExit(1)
        elif args.action == 'off':
            sock.send(bytes(9))
            print('OFF sent; no command acknowledgement in legacy protocol')
        else:
            if not math.isfinite(args.seconds) or args.seconds <= 0:
                raise SystemExit('--seconds must be positive and finite')
            packet = bytes(8) + b'\x80'
            print('Sending ON at 10Hz; ASMS mode 1 required. OFF sent on exit.')
            try:
                until = time.monotonic() + args.seconds
                while time.monotonic() < until:
                    sock.send(packet)
                    time.sleep(0.1)
            except KeyboardInterrupt:
                pass
            finally:
                sock.send(bytes(9))
                print('OFF sent (UDP delivery is not guaranteed)')


if __name__ == '__main__':
    main()

