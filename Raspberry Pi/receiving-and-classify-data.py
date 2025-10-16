import os, urllib.request, serial, struct, time, sys, glob, threading
import numpy as np
from scipy.signal import resample_poly, butter, lfilter
import RPi.GPIO as GPIO

try:
    import tflite_runtime.interpreter as tflite
except ModuleNotFoundError:
    import tensorflow.lite as tflite

BAUD=921600
HDR_MAGIC=b"YAWAV1M"
PKT_MAGIC=b"KPAY"

BASE=os.path.dirname(os.path.abspath(__file__))

# 모델/라벨 파일이 없을 경우 인터넷에서 다운로드 하는 데 사용되는 변수 설정 (ensure_assets 함수에서 사용)
MODEL_PATH=os.path.join(BASE,"yamnet.tflite")
LABELS_PATH=os.path.join(BASE,"yamnet_label_list.txt")
MODEL_URL="https://storage.googleapis.com/mediapipe-models/audio_classifier/yamnet/float32/1/yamnet.tflite"
LABELS_URL="https://storage.googleapis.com/mediapipe-tasks/audio_classifier/yamnet_label_list.txt"

SILENCE_RMS=0.004
LOW_BAND=(50,400)
MID_BAND=(300,3000)
WEIGHT_K=0.35

PINS = {
    "left_red": 17,
    "left_blue": 27,
    "front_red": 22,
    "front_blue": 23,
    "right_red": 24,
    "right_blue": 25,
    "back_red": 5,
    "back_blue": 12,
}

def _is_tflite(p):
    try:
        if not os.path.exists(p): return False
        if os.path.getsize(p)<3000000: return False
        with open(p,"rb") as f: return f.read(4)==b"TFL3"
    except: return False

# 모델/라벨 파일이 없을 경우 인터넷에서 다운로드 함
def ensure_assets():
    if not _is_tflite(MODEL_PATH):
        urllib.request.urlretrieve(MODEL_URL, MODEL_PATH)
    if not os.path.exists(LABELS_PATH) or os.path.getsize(LABELS_PATH)<5000:
        urllib.request.urlretrieve(LABELS_URL, LABELS_PATH)

# 오디오 데이터를 입력받아 어떤 종류의 소리인지 추론
class YAMNet:
    # 모델 파일과 라벨 파일을 로드
    def __init__(self,model=MODEL_PATH,labels=LABELS_PATH):
        ensure_assets()
        self.labels=[l.strip() for l in open(labels,"r",encoding="utf-8")]
        self.interp=tflite.Interpreter(model_path=model)
        self.interp.allocate_tensors()
        inp=self.interp.get_input_details()[0]; outp=self.interp.get_output_details()[0]
        self.in_idx=inp['index']; self.out_idx=outp['index']
        self.rank=len(inp['shape']); self.need=int(inp['shape'][-1]); self.sr=16000
    # 오디오 데이터(변수 x)를 입력받아 모델에 입력하고 521가지 소리 각각에 대한 확률 값을 배열(아마도 변수 s) 형태로 반환
    def infer(self,x):
        if len(x)<self.need: x=np.pad(x,(0,self.need-len(x)))
        elif len(x)>self.need: x=x[:self.need]
        if self.rank==1: self.interp.set_tensor(self.in_idx,x.astype(np.float32))
        else: self.interp.set_tensor(self.in_idx,x[np.newaxis,:].astype(np.float32))
        self.interp.invoke()
        s=self.interp.get_tensor(self.out_idx)[0]
        return s.mean(axis=0) if s.ndim==2 else s

def discover_ports(maxn=2):
    ids=sorted(glob.glob("/dev/serial/by-id/*"))
    devs=sorted(glob.glob("/dev/ttyACM*")+glob.glob("/dev/ttyUSB*"))
    ports=ids if ids else devs
    return ports[:maxn]

# 실시간으로 들어오는 오디오 데이터를 순환 버퍼를 저장
class NpRing:
    def __init__(self, maxlen):
        self.maxlen = int(maxlen)
        self.buf = np.zeros(self.maxlen, dtype=np.int16)
        self.w = 0
        self.filled = 0
        self.lock = threading.Lock()
    # EPS32-S3로부터 받은 새로운 오디오 데이터 조각을 버퍼의 끝에 추가함
    def append(self, arr):
        a = np.frombuffer(arr, dtype=np.int16) if isinstance(arr, (bytes, bytearray, memoryview)) else np.asarray(arr, dtype=np.int16)
        n = len(a)
        if n == 0: return
        with self.lock:
            n = min(n, self.maxlen)
            r = self.maxlen - self.w
            if n <= r:
                self.buf[self.w:self.w+n] = a[-n:]
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
            if start + need <= self.maxlen:
                return self.buf[start:start+need].copy()
            k = self.maxlen - start
            return np.concatenate((self.buf[start:], self.buf[:need-k])).copy()
    def clear(self):
        with self.lock:
            self.w = 0; self.filled = 0

def sync_header(ser,timeout=8):
    ser.reset_input_buffer(); t0=time.time(); buf=bytearray()
    while time.time()-t0<timeout:
        ser.write(b'!'); chunk=ser.read(1024)
        if chunk: buf+=chunk
        i=buf.find(HDR_MAGIC)
        if i!=-1:
            rest=buf[i+len(HDR_MAGIC):]
            while len(rest)<5:
                c=ser.read(5-len(rest))
                if not c: ser.write(b'!'); continue
                rest+=c
            sr=struct.unpack("<I",rest[:4])[0]; streams=rest[4]
            return sr,streams,bytes(rest[5:])
        if len(buf)>65536: buf=buf[-32768:]
        time.sleep(0.005)
    raise RuntimeError("header not found")

# 각 USB 포트마다 하나씩 생성, 백그라운드에서 EPS32-S3로부터 오디오 데이터를 끊임 없이 수신하여 NpRing 버퍼에 저장
class SerialReader(threading.Thread):
    def __init__(self,port,idx,maxlen_s=4):
        super().__init__(daemon=True)
        self.port=port; self.idx=idx; self.ser=None; self.sr=None
        self.A=None; self.B=None; self.ok=False; self.maxlen_s=maxlen_s
    def run(self):
        self.ser=serial.Serial(self.port,BAUD,timeout=1.0)
        sr,streams,rest=sync_header(self.ser,timeout=8)
        if streams!=2: self.ser.close(); return
        self.sr=sr; self.A=NpRing(sr*self.maxlen_s); self.B=NpRing(sr*self.maxlen_s)
        buf=bytearray(rest); self.ok=True; hdr_len=11
        while True:
            chunk=self.ser.read(1024)
            if chunk: buf+=chunk
            j=buf.find(PKT_MAGIC)
            if j==-1:
                if len(buf)>8192: buf=buf[-4096:]
                continue
            if len(buf)<j+hdr_len: continue
            h=buf[j:j+hdr_len]; sid=h[4]; plen=int.from_bytes(h[5:7],"little")
            if len(buf)<j+hdr_len+plen: continue
            payload=buf[j+hdr_len:j+hdr_len+plen]; arr=np.frombuffer(payload,dtype="<i2")
            if sid==0: self.A.append(arr)
            elif sid==1: self.B.append(arr)
            buf=buf[j+hdr_len+plen:]

def to3(probs,labels):
    low=[l.lower() for l in labels]
    def pick(keys):
        ks=[k.lower() for k in keys]
        idx=[i for i,n in enumerate(low) if any(k in n for k in ks)]
        return float(np.sum([probs[i] for i in idx])) if idx else 0.0
    human=pick(["speech","conversation","narration","babbling","singing","shout","scream","laugh"])
    vehicle=pick(["vehicle","car","engine","truck","bus","motorcycle","siren","train","aircraft","horn"])
    other=max(0.0,1.0-human-vehicle); s=human+vehicle+other+1e-9
    return {"human":human/s,"vehicle":vehicle/s,"other":other/s}

def _band_energy(x, sr, lo, hi):
    b,a=butter(4,[lo/(sr/2),hi/(sr/2)],btype='band'); y=lfilter(b,a,x)
    return float(np.sqrt(np.mean(y*y)+1e-12))

def post_weight_three(three, x16k, sr=16000):
    p_h, p_v, p_o = three['human'], three['vehicle'], three['other']
    e_low=_band_energy(x16k, sr, *LOW_BAND); e_mid=_band_energy(x16k, sr, *MID_BAND)
    s_h=p_h*(1.0+WEIGHT_K*(e_mid/(e_low+1e-9)))
    s_v=p_v*(1.0+WEIGHT_K*(e_low/(e_mid+1e-9)))
    s_o=p_o; s=s_h+s_v+s_o+1e-9
    return {'human':s_h/s,'vehicle':s_v/s,'other':s_o/s}

# 실제 AI 분류 총괄
def classify_label(sig_i16, sr_src, yam):
    x = sig_i16.astype(np.float32)/32768.0
    if sr_src!=yam.sr: x = resample_poly(x, yam.sr, sr_src)
    need = getattr(yam,'need', int(0.96*yam.sr))
    use = max(1, need//2)
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

# GPIO 초기화 (BCM 모드로 설정, 모든 핀을 출력 모드로 설정)
def init_gpio():
    GPIO.setmode(GPIO.BCM)
    for p in PINS.values():
        GPIO.setup(p, GPIO.OUT, initial=GPIO.LOW)

def _set_pair(red_pin, blue_pin, label):
    if label == "vehicle":
        GPIO.output(red_pin, GPIO.HIGH)
        GPIO.output(blue_pin, GPIO.LOW)
    elif label == "human":
        GPIO.output(red_pin, GPIO.LOW)
        GPIO.output(blue_pin, GPIO.HIGH)
    else:
        GPIO.output(red_pin, GPIO.LOW)
        GPIO.output(blue_pin, GPIO.LOW)

def drive_outputs(front, back, left, right):
    _set_pair(PINS["left_red"],  PINS["left_blue"],  left)
    _set_pair(PINS["front_red"], PINS["front_blue"], front)
    _set_pair(PINS["right_red"], PINS["right_blue"], right)
    _set_pair(PINS["back_red"],  PINS["back_blue"],  back)

def main():
    # 시리얼 통신 설정
    ports=sys.argv[1:] if len(sys.argv)>1 else discover_ports(2) # 사용할 USB 포트 결정
    if len(ports)<2: print("need 2 ports"); sys.exit(1) # 포트 개수가 2개가 아니라면 종료
    r0=SerialReader(ports[0],0); r1=SerialReader(ports[1],1) # r0은 ports[0] 포트에서, r1은 ports[1] 포트에서 시리얼 통신으로 데이터 수신(스레드)
    r0.start(); r1.start() # 시리얼 데이터 수신 시작
    while not (r0.ok and r1.ok): time.sleep(0.01) # r0, r1이 정상적으로 수신 준비될 때까지 대기
    if r0.sr!=r1.sr: print("sample rates mismatch"); sys.exit(1) # 샘플링 속도가 다르면 종료

    # AI 추론 시작
    sr=r0.sr; yam=YAMNet() # 샘플링 레이트 초기화 및 AI 모델을 메모리로 로드함
    win_src=int(np.ceil((yam.need//2)*(sr/float(yam.sr)))) # 최소 오디오 데이터 길이(yam.need)를 바탕으로 분석에 사용할 오디오 데이터 길이 계산

    init_gpio() # GPIO 초기화
    last_print=time.time() # 마지막 상태 출력 시각을 저장할 last_print 변수를 현재 시각으로 초기화
    
    try:
        while True:
            # r0, r1의 NpRing 버퍼에서 분석에 필요한 만큼의 최신 오디오 데이터를 꺼내 chL, chR, chF, chB 변수에 저장
            chL=r0.A.latest(win_src); chR=r0.B.latest(win_src); chF=r1.A.latest(win_src); chB=r1.B.latest(win_src)
            
            # 네 개의 채널 중 하나라도 분석에 필요한 만큼의 데이터가 없으면 0.005초 대기 후 처음부터 다시 시도
            if any(v is None for v in [chL,chR,chF,chB]):
                time.sleep(0.005); continue
            
            # 네 개의 채널 각각에 대해 AI 분류 수행
            left=classify_label(chL,sr,yam)
            right=classify_label(chR,sr,yam)
            front=classify_label(chF,sr,yam)
            back=classify_label(chB,sr,yam)

            # 분류 결과에 따라 GPIO 핀 제어
            drive_outputs(front, back, left, right)
            
            now=time.time() # 현재 시각을 now 변수에 저장

            # 터미널에 결과를 0.5초가 지났을 때만 출력해서 결과를 천천히 출력
            if now-last_print>0.5:
                print(f"front:{front} back:{back} left:{left} right:{right}")
                last_print=now

            time.sleep(0.005) # 0.005초 대기

    # Ctrl+C 입력 시 실행
    finally:
        GPIO.cleanup() # GPIO 핀 초기화

if __name__=="__main__":
    main()
