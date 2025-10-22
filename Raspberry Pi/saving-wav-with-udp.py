import socket
import threading
import time
import numpy as np
import wave  # WAV 파일 처리를 위한 모듈
import sys
import datetime # 파일 이름에 타임스탬프 추가용

UDP_PORT = 12345  # ESP32 코드와 동일한 포트 번호 사용
PKT_MAGIC = b"KPAY"
SAMPLE_RATE = 16000
CHANNELS = 2
# PyAudio FORMAT = pyaudio.paInt16 에 해당하는 샘플 폭 (16 bits = 2 bytes)
SAMPLE_WIDTH = 2
CHUNK = 256
MAX_BUFFER_SECONDS = 4

# 현재 시각을 이용해 고유한 파일 이름 생성 (예: audio_record_20251022_140000.wav)
timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
OUTPUT_FILENAME = f"audio_record_{timestamp}.wav"

class AudioBuffer:
    # 스레드 환경에서 안전하게 오디오 데이터를 저장하고 읽기 위한 순환 버퍼
    def __init__(self, maxlen):
        self.maxlen = int(maxlen)
        self.buf = np.zeros(self.maxlen, dtype=np.int16)
        self.w = 0; self.filled = 0; self.lock = threading.Lock()

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

    def read(self, need):
        need = int(need)
        with self.lock:
            if self.filled < need: return None
            start = (self.w - self.filled) % self.maxlen
            if start + need <= self.maxlen:
                data = self.buf[start:start+need].copy()
            else:
                k = self.maxlen - start
                data = np.concatenate((self.buf[start:], self.buf[:need-k])).copy()
            self.filled -= need
            return data

class UDPReader(threading.Thread):
    # UDP 패킷을 수신하여 2개의 채널 버퍼에 저장하는 스레드
    def __init__(self, port):
        super().__init__(daemon=True)
        buffer_len = SAMPLE_RATE * MAX_BUFFER_SECONDS
        self.L_buf = AudioBuffer(buffer_len)
        self.R_buf = AudioBuffer(buffer_len)
        self.ok = False
        self.last_seq_L = None
        self.last_seq_R = None
        
        try:
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.sock.bind(("0.0.0.0", port))
            print(f"UDP 서버가 {port}번 포트에서 시작됨")
            self.ok = True
        except OSError as e:
            print(f"에러: UDP 포트 {port}를 열 수 없음 ({e})")

    def run(self):
        if not self.ok: return
        hdr_len = 11
        while True:
            try:
                data, addr = self.sock.recvfrom(2048)
                
                if len(data) < hdr_len or data[:4] != PKT_MAGIC: continue
                
                h = data[:hdr_len]
                sid = h[4]
                plen = int.from_bytes(h[5:7], "little")
                seq = int.from_bytes(h[7:11], "little")
                
                if len(data) < hdr_len + plen: continue
                payload = data[hdr_len : hdr_len + plen]
                
                if sid == 0: # 왼쪽 채널
                    if self.last_seq_L is None or seq > self.last_seq_L:
                        self.L_buf.append(payload)
                        self.last_seq_L = seq
                elif sid == 1: # 오른쪽 채널
                    if self.last_seq_R is None or seq > self.last_seq_R:
                        self.R_buf.append(payload)
                        self.last_seq_R = seq

            except Exception as e:
                print(f"UDP 수신 중 에러 발생: {e}")
                time.sleep(1)

def main():
    # UDP 수신 스레드 시작
    reader = UDPReader(UDP_PORT)
    if not reader.ok: return
    reader.start()

    # WAV 파일 쓰기 객체 열기
    try:
        wf = wave.open(OUTPUT_FILENAME, 'wb') # 쓰기 모드('wb')로 파일 열기
        wf.setnchannels(CHANNELS)       # 채널 수 설정 (스테레오)
        wf.setsampwidth(SAMPLE_WIDTH)   # 샘플 당 바이트 수 설정 (16bit = 2bytes)
        wf.setframerate(SAMPLE_RATE)    # 샘플링 레이트 설정
        print(f"오디오 녹음 시작 (파일명: {OUTPUT_FILENAME}) (종료: Ctrl+C)")
    except Exception as e:
        print(f"에러: WAV 파일을 열 수 없음 ({e})")
        sys.exit(1)

    # 버퍼가 안정적으로 채워질 때까지 잠시 대기
    print("...네트워크 버퍼링 중...")
    min_buffer_size = SAMPLE_RATE // 4 # 최소 0.25초 분량 데이터 대기
    while reader.L_buf.filled < min_buffer_size or reader.R_buf.filled < min_buffer_size:
        time.sleep(0.1)
    print("녹음 시작")

    frames_written = 0
    start_time = time.time()

    try:
        while True:
            # 좌/우 채널 버퍼에서 데이터 읽기
            left_data = reader.L_buf.read(CHUNK)
            right_data = reader.R_buf.read(CHUNK)

            # 데이터가 부족하면 잠시 대기 (패킷 손실 시 잠시 끊길 수 있음)
            if left_data is None or right_data is None:
                time.sleep(0.005) # 너무 빨리 루프 돌지 않도록 잠시 대기
                continue
            
            # 좌/우 데이터를 스테레오 데이터로 합치기 (Interleaving)
            stereo_data = np.empty(CHUNK * 2, dtype=np.int16)
            stereo_data[0::2] = left_data  # 짝수 인덱스에 왼쪽 채널
            stereo_data[1::2] = right_data # 홀수 인덱스에 오른쪽 채널
            
            # 합쳐진 스테레오 데이터를 WAV 파일에 쓰기
            wf.writeframes(stereo_data.tobytes())
            frames_written += CHUNK

            # 진행 상황 표시 (선택 사항)
            elapsed_time = time.time() - start_time
            if int(elapsed_time) % 5 == 0: # 5초마다 출력
                print(f"\r녹음 진행 중... {elapsed_time:.1f}초 ({frames_written} 프레임)", end="")


    except KeyboardInterrupt:
        print("\n녹음 종료")
    finally:
        # 7. 프로그램 종료 시 WAV 파일을 정상적으로 닫음 (매우 중요!)
        print(f"\nWAV 파일 닫는 중... 총 {frames_written} 프레임 저장됨")
        wf.close()
        print(f"녹음 완료: {OUTPUT_FILENAME}")

if __name__ == "__main__":
    main()
