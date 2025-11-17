# 소리 인식 기반 운전 보조 디스플레이

# 라즈베리 파이 설정
Python 3.11 버전의 가상환경 생성 후 접속
```
conda create --name indicator_env python=3.11 -y
conda activate indicator_env
```
텐서플로 2.13 버전 설치 후 버전 확인
```
pip install tensorflow==2.13.0 tensorflow-hub==0.16.1
python -c "import tensorflow as tf; print(tf.__version__)"
```
numpy, scipy, 네오픽셀 모듈 설치 (ws281x 모듈은 자동으로 설치됨)
```
pip install numpy==1.26.4 scipy
pip install adafruit-circuitpython-neopixel
```
ESP32-S3의 IP 파악 후 Raspberry Pi/receiving-and-classify-data-with-udp.py 소스 코드 수정
```
ESP32_IPS = {
  "front_back": "111.111.111.AAA",
  "left_right": "111.111.111.BBB"
}
```

# ESP32-S3 Wi-Fi 설정
ssid = 핫스팟 Wi-Fi 이름
password = 핫스팟 Wi-Fi 비밀번호
```
const char* ssid = "Arendelle";
const char* password = "12220621";
```
host = 핫스팟이 라즈베리 파이에 부여한 IP
port = 라즈베리 파이 코드와 동일하게 설정(기본: 12345)
```
const char* host = "111.111.111.111";
const uint16_t port = 12345;
```
local_IP = 핫스팟이 ESP32-S3에 부여할 IP(ESP32-S3의 IP를 임의 설정)
gateway = 핫스팟 기기의 IP(핫스팟에 연결한 Windows PC 명령 프롬프트에 'ipconfig' 입력 후 나오는 '기본 게이트웨이' IP 사용)
subnet = (255, 255, 255, 0)으로 고정
```
IPAddress local_IP(111, 111, 111, 111);
IPAddress gateway(111, 111, 111, 111);
IPAddress subnet(255, 255, 255, 0);
```

# ESP32-S3 내장 RGB LED로 상태 확인
* 보라색 점등: 최초 전원 인가 후 초기화 중
* 파란색 점멸: 초기화 후 Wi-Fi AP에 연결 중
* 초록색 점등: Wi-Fi AP에 접속 성공
* 초록색 점멸: Wi-Fi AP에 접속 및 데이터 전송 중
* 빨간색 점등: Wi-Fi AP에 접속 실패
