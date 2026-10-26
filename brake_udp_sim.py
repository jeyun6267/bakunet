import socket
import threading
import time

# ============================================================
# Brake Board UDP Simulator
# Arduino 코드에 맞춘 테스트용 Python 송신 프로그램
#
# Arduino 코드 기준:
# - ASMS mode 패킷: 5 bytes, rxBuf[0] = mode
# - PC brake 패킷: 9 bytes, rxBuf[8] bit7 = brake bit
#
# 사용 순서 예시:
#   1) m1      -> 자율주행 모드
#   2) bon     -> 브레이크 ON
#   3) boff    -> 브레이크 OFF
#   4) m0      -> 수동/대기 모드
# ============================================================

# ================= 사용자 설정 =================
ARDUINO_IP = "10.177.21.3"
ARDUINO_PORT = 5000

# Arduino 코드의 asmsIP와 같아야 함:
# IPAddress asmsIP(10, 177, 21, 5);
LOCAL_IP = "10.177.21.5"
# ===============================================

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

try:
    sock.bind((LOCAL_IP, 0))
    print(f"[OK] Local bind: {LOCAL_IP}")
except OSError as e:
    print("[WARNING] LOCAL_IP bind 실패")
    print(f"원인: {e}")
    print("PC 이더넷 IPv4 주소가 10.177.21.5로 설정되어 있는지 확인하세요.")
    print("이 상태에서는 Arduino가 ASMS mode 패킷을 무시할 가능성이 큽니다.")
    sock.bind(("", 0))

send_lock = threading.Lock()

normal_mode = 0

emg_stop = threading.Event()
emg_thread = None

brake_stop = threading.Event()
brake_thread = None


def send_packet(data: bytes, label: str):
    with send_lock:
        sock.sendto(data, (ARDUINO_IP, ARDUINO_PORT))
    print(label)


def send_asms_mode(mode: int):
    """
    Arduino 코드에서 packetSize == 5일 때 ASMS mode로 처리합니다.
    실제로는 rxBuf[0]만 사용합니다.
    """
    packet = bytearray(5)
    packet[0] = mode & 0xFF
    send_packet(bytes(packet), f"sent ASMS mode={mode}")


def send_pc_brake(on: bool):
    """
    Arduino 코드에서 packetSize == 9일 때 PC brake로 처리합니다.
    실제로는 rxBuf[8]의 bit7만 사용합니다.
    """
    packet = bytearray(9)

    if on:
        packet[8] = 0x80   # 1000 0000, brake bit ON
        label = "sent PC brake=ON"
    else:
        packet[8] = 0x00   # brake bit OFF
        label = "sent PC brake=OFF"

    send_packet(bytes(packet), label)


def start_emergency():
    """
    Arduino 코드는 mode=3이 0.5초 이상 안 들어오면 긴급정지를 해제합니다.
    따라서 긴급정지 중에는 m3를 반복 송신합니다.
    """
    global emg_thread

    if emg_thread is not None and emg_thread.is_alive():
        print("emergency already running")
        return

    emg_stop.clear()

    def loop():
        while not emg_stop.is_set():
            send_asms_mode(3)
            time.sleep(0.1)

    emg_thread = threading.Thread(target=loop, daemon=True)
    emg_thread.start()
    print("emergency keep-alive START")


def stop_emergency():
    global normal_mode

    emg_stop.set()
    time.sleep(0.15)
    send_asms_mode(normal_mode)
    print("emergency keep-alive STOP")


def start_brake():
    """
    Arduino 코드는 receivedMode == 1일 때만 PC brake를 인정합니다.
    또한 20초 동안 PC 패킷이 없으면 Rbreak를 0으로 만듭니다.
    따라서 brake ON 중에는 1초마다 brake ON 패킷을 반복 송신합니다.
    """
    global brake_thread

    if brake_thread is not None and brake_thread.is_alive():
        print("brake already running")
        return

    brake_stop.clear()

    def loop():
        while not brake_stop.is_set():
            send_pc_brake(True)
            time.sleep(1.0)

    brake_thread = threading.Thread(target=loop, daemon=True)
    brake_thread.start()
    print("brake keep-alive START")


def stop_brake():
    brake_stop.set()
    time.sleep(0.1)
    send_pc_brake(False)
    print("brake keep-alive STOP")


def print_help():
    print()
    print("Commands:")
    print("  m0      : manual / ready mode")
    print("  m1      : autonomous mode")
    print("  m3      : emergency keep-alive start")
    print("  e       : same as m3")
    print("  q       : emergency stop release")
    print("  bon     : brake ON keep-alive")
    print("  boff    : brake OFF")
    print("  test    : m1 -> brake ON 3 sec -> brake OFF")
    print("  exit    : quit")
    print()
    print("Tip:")
    print("  brake는 mode==1일 때만 인정됩니다. bon 전에 m1을 먼저 보내세요.")
    print()


def main():
    global normal_mode

    print_help()

    while True:
        try:
            cmd = input("> ").strip().lower()
        except KeyboardInterrupt:
            print()
            cmd = "exit"

        if cmd == "m0":
            normal_mode = 0
            stop_brake()
            send_asms_mode(0)

        elif cmd == "m1":
            normal_mode = 1
            send_asms_mode(1)

        elif cmd == "m3" or cmd == "e":
            start_emergency()

        elif cmd == "q":
            stop_emergency()

        elif cmd == "bon":
            if normal_mode != 1:
                print("주의: 현재 normal_mode가 1이 아닙니다. 먼저 m1을 보내는 게 안전합니다.")
            start_brake()

        elif cmd == "boff":
            stop_brake()

        elif cmd == "test":
            print("TEST: m1 -> bon 3sec -> boff")
            normal_mode = 1
            send_asms_mode(1)
            time.sleep(0.3)
            start_brake()
            time.sleep(3.0)
            stop_brake()

        elif cmd == "exit":
            stop_brake()
            emg_stop.set()
            print("bye")
            break

        else:
            print("unknown command")
            print_help()


if __name__ == "__main__":
    main()
