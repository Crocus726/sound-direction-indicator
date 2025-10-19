import time
import board
import neopixel

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

# 네오픽셀 번호(LEFT/FRONT/RIGHT/BACK 순으로 1 2 3 4)와 color를 입력받으면 해당 네오픽셀의 색을 설정하고 켜는 함수
def set_ring_color(ring_index, color):
    # ring_index가 rings 배열 요소 개수보다 크거나 작지 않은지 검증
    if 0 <= ring_index < len(rings):
        rings[ring_index].fill(color) # neopixel.NeoPixel().fill(color) / 네오픽셀 색 설정
        rings[ring_index].show() # 설정된 색으로 네오픽셀 점등

try:
    print("프로그램 시작")
    while True:
        # 4번 반복
        for i in range(4):
            # 현재 번호의 네오픽셀을 빨간색으로 점등 후 1초 대기
            print(f"링 {i+1}번에 빨간색 점등")
            set_ring_color(i, COLOR_RED)
            time.sleep(1)

            # 현재 번호의 네오픽셀을 파란색으로 점등 후 1초 대기
            print(f"링 {i+1}번에 파란색 점등")
            set_ring_color(i, COLOR_BLUE)
            time.sleep(1)

            # 현재 번호의 네오픽셀을 끔
            set_ring_color(i, COLOR_OFF)

# 종료 시 (Ctrl + C)
except KeyboardInterrupt:
    print("\n프로그램 종료")
    for i in range(4):
        set_ring_color(i, COLOR_OFF)
