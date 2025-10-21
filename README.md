# 소리 인식 기반 운전 보조 디스플레이

# 라즈베리 파이 설정
Python 3.11 버전의 가상환경 생성
```
conda create --name indicator_env python=3.11 -y
```
가상환경 접속
```
conda activate indicator_env
```
텐서플로 2.13 버전 설치
```
pip install tensorflow==2.13.0 tensorflow-hub==0.16.1
```
다음 명령어 입력 후 버전이 뜨는지 확인
```
python -c "import tensorflow as tf; print(tf.__version__)"
```
numpy, scipy 모듈 설치 (numpy 버전 확인)
```
pip install numpy==1.26.4 scipy
```
네오픽셀 모듈 설치 (ws281x 모듈은 자동으로 설치됨)
```
pip install adafruit-circuitpython-neopixel
```
ESP32-S3의 IP 파악 후 Raspberry Pi/receiving-and-classify-data-with-udp.py 소스 코드 수정
```
ESP32_IPS = {
  "front_back": "10.168.163.000",
  "left_right": "10.168.163.000"
}
```
