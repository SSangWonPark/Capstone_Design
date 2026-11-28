#!/usr/bin/env python3
"""
SparkFun Simultaneous RFID Reader - M7E Hecto (라즈베리파이용)
출력(Read Power) 조절로 태그 인식 거리 조정 예제

- 보드 내장 PCB 안테나 사용 (외부 안테나 없음)
- SparkFun Arduino 라이브러리의 시리얼 프로토콜을 Python(pyserial)으로 옮김
- 실행 중 키보드 명령으로 출력/필터를 실시간 변경
- 'c' 명령으로 출력 스윕(0 dBm -> 상한) 하여 거리별 최소 인식 출력 측정

연결 방법
  USB-C   : 보드 UART 스위치 "USB", 포트 /dev/ttyUSB0  (기본값)
  GPIO    : 보드 UART 스위치 "SER", 포트 /dev/serial0

실행 예
  python3 m7e_power_read.py
  python3 m7e_power_read.py --port /dev/serial0 --power 1500 --rssi -60

명령어 (입력 후 Enter)
  p<값>  : 읽기 출력 설정, 단위 0.01 dBm (예: p1500 -> 15.00 dBm)
  + / -  : 출력 1 dBm 증가 / 감소
  r<값>  : RSSI 임계값(dBm), 이보다 약한 태그 무시 (예: r-60), "r"만 입력 시 해제
  l<값>  : 출력 상한 변경 (예: l2700) - 외부 5V 전원 사용 시에만 2200 초과 권장
  s      : 읽기 시작 / 정지 토글
  c      : 출력 스윕 캘리브레이션 (태그 1개를 원하는 거리에 두고 실행)
  ?      : 현재 상태 / 도움말
  q      : 종료
"""

import argparse
import select
import struct
import sys
import time

import serial

# ---------------- 프로토콜 상수 ----------------
OP_VERSION = 0x03
OP_SET_BAUD_RATE = 0x06
OP_READ_TAG_ID_MULTIPLE = 0x22
OP_MULTI_PROTOCOL_TAG_OP = 0x2F
OP_SET_ANTENNA_PORT = 0x91
OP_SET_READ_TX_POWER = 0x92
OP_SET_TAG_PROTOCOL = 0x93
OP_SET_REGION = 0x97
OP_SET_READER_OPTIONAL_PARAMS = 0x9A

REGION_NORTHAMERICA = 0x01
REGION_EUROPE = 0x08
REGION_KOREA = 0x09
REGION_JAPAN = 0x05

POWER_MIN = 0        # 0.00 dBm
POWER_ABS_MAX = 2700  # 27.00 dBm (M7E 최대)
POWER_STEP = 100     # +/- 1회당 1 dBm

_CRC_TABLE = [
    0x0000, 0x1021, 0x2042, 0x3063, 0x4084, 0x50A5, 0x60C6, 0x70E7,
    0x8108, 0x9129, 0xA14A, 0xB16B, 0xC18C, 0xD1AD, 0xE1CE, 0xF1EF,
]


def tm_crc(data: bytes) -> int:
    """ThingMagic 변형 CRC16"""
    crc = 0xFFFF
    for b in data:
        crc = (((crc << 4) | (b >> 4)) ^ _CRC_TABLE[crc >> 12]) & 0xFFFF
        crc = (((crc << 4) | (b & 0x0F)) ^ _CRC_TABLE[crc >> 12]) & 0xFFFF
    return crc


def fmt_power(p: int) -> str:
    return f"{p / 100:.2f} dBm"


class M7E:
    def __init__(self, port: str, baud: int = 115200):
        self.ser = serial.Serial(port, baud, timeout=0)
        self._buf = bytearray()

    # ---------- 저수준 송수신 ----------
    def _build(self, opcode: int, data: bytes = b"") -> bytes:
        body = bytes([len(data), opcode]) + data
        crc = tm_crc(body)
        return b"\xFF" + body + bytes([crc >> 8, crc & 0xFF])

    def _read_frame(self, timeout: float):
        """FF로 시작하는 프레임 1개 수신, CRC 오류 시 'crc', 시간 초과 시 None"""
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            self._fill()
            frame = self._extract_frame()
            if frame is not None:
                return frame
            time.sleep(0.001)
        return None

    def _fill(self):
        n = self.ser.in_waiting
        if n:
            self._buf += self.ser.read(n)

    def _extract_frame(self):
        # 헤더(0xFF)까지 버림
        while self._buf and self._buf[0] != 0xFF:
            del self._buf[0]
        if len(self._buf) < 2:
            return None
        total = self._buf[1] + 7
        if len(self._buf) < total:
            return None
        frame = bytes(self._buf[:total])
        del self._buf[:total]
        crc = tm_crc(frame[1:total - 2])
        if frame[-2] != (crc >> 8) or frame[-1] != (crc & 0xFF):
            return "crc"
        return frame

    def flush_input(self):
        self.ser.reset_input_buffer()
        self._buf.clear()

    def send(self, opcode: int, data: bytes = b"", timeout: float = 2.0, wait: bool = True):
        """명령 송신. 성공 시 응답 프레임, 실패 시 None"""
        self.flush_input()
        self.ser.write(self._build(opcode, data))
        self.ser.flush()
        if not wait:
            return None
        frame = self._read_frame(timeout)
        if frame is None or frame == "crc":
            return None
        if frame[2] != opcode:
            return ("wrong_opcode", frame)
        status = (frame[3] << 8) | frame[4]
        if status != 0:
            print(f"[경고] opcode 0x{opcode:02X} 상태 코드 0x{status:04X}")
        return frame

    # ---------- 모듈 설정 ----------
    def get_version(self):
        return self.send(OP_VERSION)

    def set_baud(self, baud: int):
        self.send(OP_SET_BAUD_RATE, struct.pack(">I", baud), wait=False)

    def set_region(self, region: int):
        return self.send(OP_SET_REGION, bytes([region]))

    def set_tag_protocol(self, protocol: int = 0x05):  # 0x05 = GEN2
        return self.send(OP_SET_TAG_PROTOCOL, bytes([0x00, protocol]))

    def set_antenna_port(self):  # TX=1, RX=1 (내장 PCB 안테나 경로)
        return self.send(OP_SET_ANTENNA_PORT, bytes([0x01, 0x01]))

    def set_read_power(self, power: int):
        power = max(POWER_MIN, min(POWER_ABS_MAX, power))
        return self.send(OP_SET_READ_TX_POWER, struct.pack(">h", power))

    def _set_reader_config(self, opt1: int, opt2: int):
        return self.send(OP_SET_READER_OPTIONAL_PARAMS, bytes([0x01, opt1, opt2]))

    def start_reading(self):
        self._set_reader_config(0x0C, 0x00)  # 읽기 필터 해제 -> 같은 태그도 계속 보고
        blob = bytes([0x00, 0x00, 0x01, 0x22, 0x00, 0x00, 0x05, 0x07,
                      0x22, 0x10, 0x00, 0x1B, 0x03, 0xE8, 0x01, 0xFF])
        self.send(OP_MULTI_PROTOCOL_TAG_OP, blob)
        self._buf.clear()

    def stop_reading(self):
        self.send(OP_MULTI_PROTOCOL_TAG_OP, bytes([0x00, 0x00, 0x02]), wait=False)
        time.sleep(1.0)  # 정지까지 1~2초 소요
        self.flush_input()

    # ---------- 연속 읽기 응답 처리 ----------
    def poll(self):
        """수신된 프레임을 모두 해석해 이벤트 리스트로 반환"""
        self._fill()
        events = []
        while True:
            frame = self._extract_frame()
            if frame is None:
                break
            if frame == "crc":
                events.append(("bad_crc", None))
                continue
            events.append(self._parse(frame))
        return events

    @staticmethod
    def _parse(m: bytes):
        if m[2] != OP_READ_TAG_ID_MULTIPLE:
            return ("other", m)
        length = m[1]
        if length == 0x00:
            status = (m[3] << 8) | m[4]
            if status == 0x0400:
                return ("keepalive", None)
            if status == 0x0504:
                return ("temp_throttle", None)
            if status == 0x0505:
                return ("high_return_loss", None)
            return ("unknown", status)
        if length == 0x08:
            return ("unknown", None)
        if length == 0x0A:
            return ("temperature", None)

        # 태그 레코드
        rssi = m[12] - 256 if m[12] > 127 else m[12]
        freq_khz = (m[14] << 16) | (m[15] << 8) | m[16]
        timestamp = struct.unpack(">I", m[17:21])[0]
        data_bits = (m[24] << 8) | m[25]
        data_bytes = (data_bits + 7) // 8
        epc_bits = (m[27 + data_bytes] << 8) | m[28 + data_bytes]
        epc_bytes = epc_bits // 8 - 4  # PC 2바이트, EPC CRC 2바이트 제외
        start = 31 + data_bytes
        epc = m[start:start + epc_bytes]
        return ("tag", {"rssi": rssi, "freq_khz": freq_khz,
                        "timestamp": timestamp,
                        "epc": " ".join("{:02X}".format(b) for b in epc)})  # Python 3.7 호환

    def close(self):
        self.ser.close()


def connect(port: str) -> M7E:
    """115200으로 연결. 모듈이 다른 보레이트(예: Arduino에서 38400 설정)면 115200으로 되돌림"""
    target = 115200
    for baud in (target, 38400, 9600):
        r = M7E(port, baud)
        time.sleep(0.1)
        r.flush_input()  # 전원 직후 모듈이 보내는 버전 메시지 무시
        resp = r.get_version()

        if isinstance(resp, tuple) and resp[0] == "wrong_opcode":
            # 이전 실행에서 연속 읽기 상태로 남아 있음
            print("모듈이 읽는 중이어서 정지 요청")
            r.stop_reading()
            time.sleep(0.5)
            resp = r.get_version()

        if resp is not None and not isinstance(resp, tuple):
            if baud != target:
                r.set_baud(target)
                r.close()
                time.sleep(0.25)
                r = M7E(port, target)
                if r.get_version() is None:
                    r.close()
                    continue
            return r
        r.close()
    raise RuntimeError("모듈 응답 없음. 배선, UART 스위치(USB/SER), 포트 이름 확인")


class App:
    def __init__(self, reader: M7E, power: int, limit: int, rssi: int):
        self.r = reader
        self.power_limit = limit
        self.read_power = power
        self.rssi_threshold = rssi  # -128 = 필터 없음
        self.reading = False

    def start(self):
        self.r.start_reading()
        self.reading = True

    def stop(self):
        self.r.stop_reading()
        self.reading = False

    def apply_power(self, p: int):
        p = max(POWER_MIN, min(self.power_limit, p))
        was = self.reading
        if was:
            self.stop()  # 연속 읽기 중에는 설정 명령이 섞이므로 정지 후 변경
        self.r.set_read_power(p)
        self.read_power = p
        print(f"읽기 출력: {fmt_power(p)}")
        if was:
            self.start()

    def status(self):
        print("------------------------------")
        print(f"읽기 출력   : {fmt_power(self.read_power)}")
        print(f"출력 상한   : {fmt_power(self.power_limit)}")
        th = "없음" if self.rssi_threshold <= -128 else f"{self.rssi_threshold} dBm"
        print(f"RSSI 임계값 : {th}")
        print(f"상태        : {'읽는 중' if self.reading else '정지'}")
        print("명령: p<값> + - r<값> l<값> s c ? q")
        print("------------------------------")

    def sweep(self, dwell: float = 1.0):
        was = self.reading
        if was:
            self.stop()
        print("출력[dBm]\t인식횟수\t평균RSSI\t최대RSSI")
        for p in range(POWER_MIN, self.power_limit + 1, POWER_STEP):
            self.r.set_read_power(p)
            self.r.start_reading()
            rssis = []
            t0 = time.monotonic()
            while time.monotonic() - t0 < dwell:
                for kind, data in self.r.poll():
                    if kind == "tag":
                        rssis.append(data["rssi"])
                time.sleep(0.005)
            self.r.stop_reading()
            if rssis:
                print(f"{p / 100:.2f}\t\t{len(rssis)}\t\t{sum(rssis) // len(rssis)}\t\t{max(rssis)}")
            else:
                print(f"{p / 100:.2f}\t\t0\t\t-\t\t-")
        self.r.set_read_power(self.read_power)
        print(f"스윕 완료, 출력 복구: {fmt_power(self.read_power)}")
        if was:
            self.start()

    def handle(self, cmd: str) -> bool:
        cmd = cmd.strip()
        if not cmd:
            return True
        c, arg = cmd[0].lower(), cmd[1:].strip()
        try:
            if c == "p":
                self.apply_power(int(arg))
            elif c == "+":
                self.apply_power(self.read_power + POWER_STEP)
            elif c == "-" and not arg:
                self.apply_power(self.read_power - POWER_STEP)
            elif c == "r":
                self.rssi_threshold = -128 if not arg else int(arg)
                th = "없음" if self.rssi_threshold <= -128 else f"{self.rssi_threshold} dBm"
                print(f"RSSI 임계값: {th}")
            elif c == "l":
                self.power_limit = max(POWER_MIN, min(POWER_ABS_MAX, int(arg)))
                print(f"출력 상한: {fmt_power(self.power_limit)}")
                if self.power_limit > 2200:
                    print("주의: 22 dBm 초과는 외부 5V 전원(1A 이상) 필요, 장시간 사용 시 방열 필요")
                if self.read_power > self.power_limit:
                    self.apply_power(self.power_limit)
            elif c == "s":
                if self.reading:
                    self.stop()
                    print("읽기 정지")
                else:
                    self.start()
                    print("읽기 시작")
            elif c == "c":
                self.sweep()
            elif c == "?":
                self.status()
            elif c == "q":
                return False
            else:
                print("알 수 없는 명령 ('?' 입력 시 도움말)")
        except ValueError:
            print("숫자 형식 오류")
        return True

    def run(self):
        self.status()
        self.start()
        print("읽기 시작 (명령 입력 후 Enter)")
        while True:
            # 키보드 입력 확인 (블로킹 없음)
            if select.select([sys.stdin], [], [], 0)[0]:
                if not self.handle(sys.stdin.readline()):
                    break

            if self.reading:
                for kind, data in self.r.poll():
                    if kind == "tag":
                        if data["rssi"] < self.rssi_threshold:
                            continue  # 멀리 있는(약한) 태그 무시
                        print(f"RSSI[{data['rssi']}] PWR[{fmt_power(self.read_power)}] "
                              f"EPC[{data['epc']}]")
                    elif kind == "bad_crc":
                        print("Bad CRC")
                    elif kind == "high_return_loss":
                        print("High return loss - 안테나 주변 금속/장애물 확인")
                    elif kind == "temp_throttle":
                        print("온도 제한 동작 중 - 출력을 낮추거나 방열 필요")
            time.sleep(0.005)


def main():
    ap = argparse.ArgumentParser(description="M7E Hecto 출력 조절 예제 (Raspberry Pi)")
    ap.add_argument("--port", default="/dev/ttyUSB0",
                    help="USB-C: /dev/ttyUSB0, GPIO UART: /dev/serial0")
    ap.add_argument("--power", type=int, default=1000, help="시작 출력, 0.01 dBm 단위 (기본 1000)")
    ap.add_argument("--limit", type=int, default=2200, help="출력 상한, 0.01 dBm 단위 (기본 2200)")
    ap.add_argument("--rssi", type=int, default=-128, help="RSSI 임계값 dBm (기본 필터 없음)")
    args = ap.parse_args()

    reader = connect(args.port)
    print("모듈 연결 완료")
    reader.set_tag_protocol()
    reader.set_antenna_port()
    reader.set_region(REGION_KOREA)  # 국내 주파수 대역

    limit = max(POWER_MIN, min(POWER_ABS_MAX, args.limit))
    app = App(reader, power=min(args.power, limit), limit=limit, rssi=args.rssi)
    app.apply_power(app.read_power)

    try:
        app.run()
    except KeyboardInterrupt:
        pass
    finally:
        if app.reading:
            reader.stop_reading()  # 다음 실행을 위해 연속 읽기 정지
        reader.close()
        print("종료")


if __name__ == "__main__":
    main()
