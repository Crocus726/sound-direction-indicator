'''
sudo apt-get update
sudo apt-get install portaudio19-dev -y
sudo pip3 install pyaudio numpy
raspi-config 오디오 출력 3.5 mm 잭으로 설정
1개의 ESP32-S3만 작동시켜야 함
'''

import socket
import threading
import time
import numpy as np
import pyaudio
import sys

# --- 설정 ---
UDP_PORT = 115        # UDP 데이터를 수신할 포트
PKT_MAGIC = b"KPAY"
SAMPLE_RATE = 16000     # ESP32에서 보낸 샘플링 레이트와 동일하게 설정
CHANNELS = 2            # 스테레오 (좌, 우)
FORMAT = pyaudio.paInt16 # 16비트 오디오 데이터 형식
CHUNK = 256             # 한 번에 처리할 오디오 프레임 수 (ESP32의 CHUNK와 동일)

# 네트워크 지연(Jitter)을 흡수하기 위한 버퍼
# 숫자가 클수록 안정적이지만 소리가 들리기 시작하는 초기 지연 시간이 길어짐
MAX_BUFFER_SECONDS = 2

class AudioBuffer:
    # NpRing 클래스와 동일한 __init__ 함수
    def __init__(self, maxlen):
        self.maxlen = int(maxlen)
        self.buf = np.zeros(self.maxlen, dtype=np.int16)
        self.w = 0  # 데이터를 쓸 위치 (Write Pointer)
        self.filled = 0  # 버퍼에 채워진 데이터 양
        self.lock = threading.Lock() # 동시 접근을 막기 위한 잠금 장치
    # NpRing 클래스와 동일한 __append__ 함수
    # EPS32-S3로부터 받은 새로운 오디오 데이터 조각을 버퍼의 끝에 추가함
    def append(self, arr):
        a = np.frombuffer(arr, dtype=np.int16) if isinstance(arr, (bytes, bytearray, memoryview)) else np.asarray(arr, dtype=np.int16)
        n = len(a)
        if n == 0: return
        with self.lock:
            n = min(n, self.maxlen)
            r = self.maxlen - self.w
            if n <= r: self.buf[self.w:self.w+n] = a[-n:]
            else:
                k = r
                self.buf[self.w:] = a[-n:-n+k]
                self.buf[:n-k] = a[-(n-k):]
            self.w = (self.w + n) % self.maxlen
            self.filled = min(self.maxlen, self.filled + n)
    # NpRing 클래스에는 없는 새로운 read 함수
    # 버퍼에서 가장 오래된 데이터를 need만큼 읽어 반환함
    def read(self, need):
        need = int(need)
        with self.lock:
            if self.filled < need: return None # 데이터가 충분하지 않으면 None 반환
            
            # 읽을 데이터의 시작 위치 계산
            start = (self.w - self.filled) % self.maxlen
            if start + need <= self.maxlen:
                data = self.buf[start:start+need].copy()
            else: # 버퍼의 끝을 넘어가는 경우, 두 조각으로 나눠서 읽음
                k = self.maxlen - start
                data = np.concatenate((self.buf[start:], self.buf[:need-k])).copy()
            
            self.filled -= need # 읽은 만큼 데이터 양 감소
            return data

class UDPReader(threading.Thread):
    # 백그라운드에서 UDP 패킷 수신 / 채널별로 분리하여 버퍼에 저장하는 스레드
    def __init__(self, port):
        super().__init__(daemon=True)
        buffer_len = SAMPLE_RATE * MAX_BUFFER_SECONDS
        self.L_buf = AudioBuffer(buffer_len) # 왼쪽 채널 버퍼
        self.R_buf = AudioBuffer(buffer_len) # 오른쪽 채널 버퍼
        self.ok = False
        
        try:
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.sock.bind(("0.0.0.0", port)) # 모든 IP로부터 해당 포트로 오는 데이터 수신
            print(f"UDP 서버가 {port}번 포트에서 시작됨")
            self.ok = True
        except OSError as e:
            print(f"에러: UDP 포트 {port}를 열 수 없음 ({e})")

    def run(self):
        if not self.ok: return
        hdr_len = 11
        while True:
            try:
                data, addr = self.sock.recvfrom(2048) # 데이터 수신 대기

                # 패킷 유효성 검사
                if len(data) < hdr_len or data[:4] != PKT_MAGIC: continue
                
                h = data[:hdr_len]
                sid = h[4] # 채널 ID (0 또는 1)
                plen = int.from_bytes(h[5:7], "little") # 데이터 길이
                
                if len(data) < hdr_len + plen: continue
                payload = data[hdr_len : hdr_len + plen]
                
                # 채널 ID에 따라 맞는 버퍼에 저장
                if sid == 0: self.L_buf.append(payload)
                elif sid == 1: self.R_buf.append(payload)

            except Exception as e:
                print(f"UDP 수신 중 에러 발생: {e}")
                time.sleep(1)

def main():
    # UDP 수신 스레드 시작
    reader = UDPReader(UDP_PORT)
    if not reader.ok: return
    reader.start()

    # PyAudio 초기화 및 오디오 출력 스트림 열기
    p = pyaudio.PyAudio()
    stream = p.open(format=FORMAT,
                    channels=CHANNELS,
                    rate=SAMPLE_RATE,
                    output=True,
                    frames_per_buffer=CHUNK)

    print("오디오 재생 시작 (종료: Ctrl+C)")
    
    # 버퍼가 안정적으로 채워질 때까지 잠시 대기 (초기 지연)
    print("네트워크 버퍼링 중...")
    min_buffer_size = SAMPLE_RATE // 2 # 최소 0.5초 분량의 데이터가 쌓일 때까지
    while reader.L_buf.filled < min_buffer_size or reader.R_buf.filled < min_buffer_size:
        time.sleep(0.1)
    print("재생 시작")

    try:
        while True:
            # 각 채널 버퍼에서 오디오 데이터 읽기
            left_data = reader.L_buf.read(CHUNK)
            right_data = reader.R_buf.read(CHUNK)

            # 데이터가 부족하면(네트워크 끊김 등) 잠시 침묵을 재생하여 소리가 끊기는 것을 방지
            if left_data is None or right_data is None:
                silence = np.zeros(CHUNK * CHANNELS, dtype=np.int16)
                stream.write(silence.tobytes())
                continue
            
            # 좌/우 채널 데이터를 스테레오 데이터로 합치기 (Interleaving)
            # [L, R, L, R, L, R, ...] 형태로 만듦
            stereo_data = np.empty(len(left_data) * 2, dtype=np.int16)
            stereo_data[0::2] = left_data  # 짝수 인덱스에 왼쪽 채널
            stereo_data[1::2] = right_data # 홀수 인덱스에 오른쪽 채널
            
            # 합쳐진 스테레오 데이터를 오디오 잭으로 출력
            stream.write(stereo_data.tobytes())

    except KeyboardInterrupt:
        print("프로그램 종료")
    finally:
        # 프로그램 종료 시 스트림과 PyAudio 객체를 깔끔하게 닫음
        print("오디오 스트림 종료")
        stream.stop_stream()
        stream.close()
        p.terminate()

if __name__ == "__main__":
    main()
