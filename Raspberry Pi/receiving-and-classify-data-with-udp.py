import os
import urllib.request
import struct
import time
import sys
import threading
import socket
import numpy as np
from scipy.signal import resample_poly, butter, lfilter

# 네오픽셀 제어 모듈 포함
import board
import neopixel

try:
    import tflite_runtime.interpreter as tflite
except ModuleNotFoundError:
    import tensorflow.lite as tflite

# UDP 통신 관련 설정
UDP_PORT = 115 # UDP 통신 포트 번호
PKT_MAGIC = b"KPAY" # 각 데이터 패킷 시작 헤더

# ESP32 IP DHCP 할당 후 입력
ESP32_IPS = {
    "front_back_esp32_ip": "10.168.163.000",
    "left_right_esp32_ip": "10.168.163.000"
}

BASE = os.path.dirname(os.path.abspath(__file__))

# 모델/라벨 파일이 없을 경우 인터넷에서 다운로드 하는 데 사용되는 변수 설정 (ensure_assets 함수에서 사용)
MODEL_PATH = os.path.join(BASE, "yamnet.tflite")
LABELS_PATH = os.path.join(BASE, "yamnet_label_list.txt")
MODEL_URL = "https://storage.googleapis.com/mediapipe-models/audio_classifier/yamnet/float32/1/yamnet.tflite"
LABELS_URL = "https://storage.googleapis.com/mediapipe-tasks/audio_classifier/yamnet_label_list.txt"

SILENCE_RMS = 0.004
LOW_BAND = (50, 400)
MID_BAND = (300, 3000)
WEIGHT_K = 0.35

# 네오픽셀 설정
PIXELS_PER_RING = 15 # 네오픽셀 1개당 LED 수
COLOR_RED = (255, 0, 0)
COLOR_BLUE = (0, 0, 255)
COLOR_OFF = (0, 0, 0)

# 네오픽셀별 핀 번호 설정
LEFT_RING_PIN = board.D4
FRONT_RING_PIN = board.D17
RIGHT_RING_PIN = board.D27
BACK_RING_PIN = board.D22

# 네오픽셀 초기화 실행
try:
    rings = [
        neopixel.NeoPixel(LEFT_RING_PIN, PIXELS_PER_RING, auto_write = False),
        neopixel.NeoPixel(FRONT_RING_PIN, PIXELS_PER_RING, auto_write = False),
        neopixel.NeoPixel(RIGHT_RING_PIN, PIXELS_PER_RING, auto_write = False),
        neopixel.NeoPixel(BACK_RING_PIN, PIXELS_PER_RING, auto_write = False)
    ]
    print("네오픽셀 초기화 완료")
except Exception as e:
    print(f"네오픽셀 초기화 실패: {e}")
    exit()

def _is_tflite(p):
    try:
        if not os.path.exists(p): return False
        if os.path.getsize(p) < 3000000: return False
        with open(p,"rb") as f: return f.read(4) == b"TFL3"
    except: return False

# 모델/라벨 파일이 없을 경우 인터넷에서 다운로드 함
def ensure_assets():
    if not _is_tflite(MODEL_PATH):
        urllib.request.urlretrieve(MODEL_URL, MODEL_PATH)
    if not os.path.exists(LABELS_PATH) or os.path.getsize(LABELS_PATH) < 5000:
        urllib.request.urlretrieve(LABELS_URL, LABELS_PATH)

# 오디오 데이터를 입력받아 어떤 종류의 소리인지 추론
class YAMNet:
    # 모델 파일과 라벨 파일을 로드
    def __init__(self, model = MODEL_PATH, labels = LABELS_PATH):
        ensure_assets()
        self.labels = [l.strip() for l in open(labels, "r", encoding="utf-8")]
        self.interp = tflite.Interpreter(model_path=model)
        self.interp.allocate_tensors()
        inp = self.interp.get_input_details()[0]; outp = self.interp.get_output_details()[0]
        self.in_idx = inp['index']; self.out_idx = outp['index']
        self.rank = len(inp['shape']); self.need = int(inp['shape'][-1]); self.sr = 16000
    # 오디오 데이터(변수 x)를 입력받아 모델에 입력하고 521가지 소리 각각에 대한 확률 값을 배열(아마도 변수 s) 형태로 반환
    def infer(self, x):
        if len(x) < self.need: x = np.pad(x, (0, self.need - len(x)))
        elif len(x) > self.need: x = x[:self.need]
        if self.rank == 1: self.interp.set_tensor(self.in_idx, x.astype(np.float32))
        else: self.interp.set_tensor(self.in_idx, x[np.newaxis, :].astype(np.float32))
        self.interp.invoke()
        s = self.interp.get_tensor(self.out_idx)[0]
        return s.mean(axis=0) if s.ndim == 2 else s

# 실시간으로 들어오는 오디오 데이터를 순환 버퍼를 저장
class NpRing:
    def __init__(self, maxlen):
        self.maxlen = int(maxlen)
        self.buf = np.zeros(self.maxlen, dtype=np.int16)
        self.w = 0; self.filled = 0; self.lock = threading.Lock()
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
    # 버퍼에서 AI 분석에 필요한 만큼의 가장 최신 오디오 데이터만을 꺼내옴
    def latest(self, need):
        need = int(need)
        with self.lock:
            if self.filled < need: return None
            start = (self.w - need) % self.maxlen
            if start + need <= self.maxlen: return self.buf[start:start+need].copy()
            k = self.maxlen - start
            return np.concatenate((self.buf[start:], self.buf[:need-k])).copy()

# SerialReader 대체하는 UDPReader
class UDPReader(threading.Thread):
    def __init__(self, port, maxlen_s=4):
        super().__init__(daemon=True)
        self.sr = 16000  # 샘플링 레이트는 16kHz로 고정
        
        # 4개 방향 채널을 위한 순환 버퍼 생성
        self.F = NpRing(self.sr * maxlen_s); self.B = NpRing(self.sr * maxlen_s); self.L = NpRing(self.sr * maxlen_s); self.R = NpRing(self.sr * maxlen_s)
        self.ok = False
        
        try:
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.sock.bind(("0.0.0.0", port)) # 모든 IP로부터 해당 포트로 오는 데이터 수신
            print(f"UDP 서버가 {port}번 포트에서 시작됨")
            self.ok = True
        except OSError as e:
            print(f"에러: UDP 포트 {port}를 열 수 없음 ({e})")
            self.ok = False

    def run(self):
        if not self.ok: return
        hdr_len = 11
        while True:
            try:
                # UDP는 패킷 단위로 데이터가 오므로, 한 번에 하나의 패킷을 읽음
                data, addr = self.sock.recvfrom(2048)
                
                if len(data) < hdr_len: continue
                if data[:4] != PKT_MAGIC: continue
                
                h = data[:hdr_len]
                sid = h[4]
                plen = int.from_bytes(h[5:7], "little")
                
                if len(data) < hdr_len + plen: continue
                payload = data[hdr_len : hdr_len + plen]
                
                # 보낸 곳의 IP 주소를 확인하여 어떤 버퍼에 넣을지 결정
                ip_addr = addr[0]
                if ip_addr == ESP32_IPS["front_back_esp32_ip"]:
                    if sid == 0: self.F.append(payload)
                    elif sid == 1: self.B.append(payload)
                elif ip_addr == ESP32_IPS["left_right_esp32_ip"]:
                    if sid == 0: self.L.append(payload)
                    elif sid == 1: self.R.append(payload)
            except Exception as e:
                print(f"UDP 수신 중 에러 발생: {e}")
                time.sleep(1)

def to3(probs,labels):
    low = [l.lower() for l in labels]
    def pick(keys):
        ks = [k.lower() for k in keys]
        idx = [i for i,n in enumerate(low) if any(k in n for k in ks)]
        return float(np.sum([probs[i] for i in idx])) if idx else 0.0
    human = pick(["speech", "conversation", "narration", "babbling", "singing", "shout", "scream", "laugh"])
    vehicle = pick(["vehicle", "car", "engine", "truck", "bus", "motorcycle", "siren", "train", "aircraft", "horn"])
    other = max(0.0, 1.0 - human - vehicle); s = human + vehicle + other + 1e-9
    return {"human": human / s, "vehicle": vehicle / s, "other": other / s}

def _band_energy(x, sr, lo, hi):
    b, a = butter(4, [lo / (sr / 2), hi / (sr / 2)], btype='band'); y = lfilter(b, a, x)
    return float(np.sqrt(np.mean(y * y) + 1e-12))

def post_weight_three(three, x16k, sr = 16000):
    p_h, p_v, p_o = three['human'], three['vehicle'], three['other']
    e_low = _band_energy(x16k, sr, *LOW_BAND); e_mid = _band_energy(x16k, sr, *MID_BAND)
    s_h = p_h * (1.0 + WEIGHT_K * (e_mid / (e_low + 1e-9)))
    s_v = p_v * (1.0 + WEIGHT_K * (e_low / (e_mid + 1e-9)))
    s_o = p_o; s = s_h + s_v + s_o + 1e-9
    return {'human': s_h / s, 'vehicle': s_v / s, 'other': s_o / s}

# sig_i16에 오디오 데이터를, sr_src에 샘플링 레이트 변수를 입력받아서 실제 AI 분류 후 결과 반환
def classify_label(sig_i16, sr_src, yam):
    x = sig_i16.astype(np.float32) / 32768.0
    if sr_src != yam.sr: x = resample_poly(x, yam.sr, sr_src)
    need = getattr(yam, 'need', int(0.96 * yam.sr))
    use = max(1, need // 2)
    cur = x[-use:]
    if len(cur) < use: cur = np.pad(cur, (use-len(cur), 0))
    rms = float(np.sqrt(np.mean(cur*cur)+1e-12))
    if rms < SILENCE_RMS: return "silent"
    pad = np.zeros(need, dtype=np.float32)
    pad[-use:] = cur
    probs = yam.infer(pad)
    three = to3(probs, yam.labels)
    three = post_weight_three(three, pad, sr=yam.sr)
    return max(three.items(), key=lambda kv: kv[1])[0]
    # 반환 결과 예시: "human", "vehicle", "silent"

# 네오픽셀 번호(LEFT/FRONT/RIGHT/BACK 순으로 1 2 3 4)와 color를 입력받으면 해당 네오픽셀의 색을 설정하고 켜는 함수
def set_ring_color(ring_index, color):
    # ring_index가 rings 배열 요소 개수보다 크거나 작지 않은지 검증
    if 0 <= ring_index < len(rings):
        rings[ring_index].fill(color) # neopixel.NeoPixel().fill(color) / 네오픽셀 색 설정
        rings[ring_index].show() # 설정된 색으로 네오픽셀 점등

def _set_pair(ring_num, label):
    if label == "vehicle": set_ring_color(ring_num, COLOR_RED)
    elif label == "human": set_ring_color(ring_num, COLOR_BLUE)
    else: set_ring_color(ring_num, COLOR_OFF)

def drive_outputs(front, back, left, right):
    _set_pair(0, left); _set_pair(1, front); _set_pair(2, right); _set_pair(3, back)

def main():
    # UDP 리더 생성
    reader = UDPReader(UDP_PORT)
    if not reader.ok: sys.exit(1)
    reader.start()

    # AI 추론 시작
    sr = reader.sr
    yam = YAMNet() # 샘플링 레이트 초기화 및 AI 모델을 메모리로 로드함
    win_src = int(np.ceil((yam.need // 2) * (sr / float(yam.sr)))) # 최소 오디오 데이터 길이(yam.need)를 바탕으로 분석에 사용할 오디오 데이터 길이 계산

    last_print = time.time() # 마지막 상태 출력 시각을 저장할 last_print 변수를 현재 시각으로 초기화

    try:
        while True:
            # NpRing 버퍼에서 분석에 필요한 만큼의 최신 오디오 데이터를 꺼내 chL, chR, chF, chB 변수에 저장
            chF = reader.F.latest(win_src); chB = reader.B.latest(win_src); chL = reader.L.latest(win_src); chR = reader.R.latest(win_src)

            # 네 개의 채널 중 하나라도 분석에 필요한 만큼의 데이터가 없으면 0.005초 대기 후 처음부터 다시 시도
            if any(v is None for v in [chL, chR, chF, chB]):
                time.sleep(0.005); continue

            # 네 개의 채널 각각에 대해 AI 분류 수행 (각 변수에 "vehicle", "human", "silent" 중 하나가 저장됨)
            left = classify_label(chL, sr, yam)
            right = classify_label(chR, sr, yam)
            front = classify_label(chF, sr, yam)
            back = classify_label(chB, sr, yam)

            # 분류 결과에 따라 GPIO 핀 제어
            drive_outputs(front, back, left, right)

            now = time.time() # 현재 시각을 now 변수에 저장

            # 터미널에 결과를 0.5초가 지났을 때만 출력해서 결과를 천천히 출력
            if now - last_print > 0.5:
                print(f"front:{front} back:{back} left:{left} right:{right}")
                last_print = now

            time.sleep(0.005) # 0.005초 대기

    # Ctrl+C 입력 시 실행
    finally:
        for i in range(4): set_ring_color(i, COLOR_OFF) # GPIO 핀 초기화

if __name__ == "__main__":
    main()
