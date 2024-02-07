
# 필요한 라이브러리를 가져온다.
import os
import cv2

# FFMPEG 라이브러리의 CAPTURE_OPTIONS 환경 변수를 설정하여 RTSP 스트리밍에 사용할 프로토콜을 지정.
os.environ["OPENCV_FFMPEG_CAPTURE_OPTIONS"] = "rtsp_transport;udp"

# VideoCapture 객체를 생성하고 RTSP URL을 지정하여 카메라에 연결.
cap = cv2.VideoCapture('rtsp://admin:admin0102@192.168.10.2:554/stream1')

# 카메라 연결 여부 확인 메세지.
if not cap.isOpened():
    print("카메라 연결 실패...")
else:
    print("카메라 연결 성공!!")

# 스트리밍 창의 초기 크기 설정
cv2.namedWindow('frame', cv2.WINDOW_NORMAL)
cv2.resizeWindow('frame', 400, 300)  # 원하는 가로(width), 세로(height) 크기로 조절

while cap.isOpened():

    # 카메라로부터 프레임을 읽어온다.
    ret, frame = cap.read()

    # 프레임이 정상적으로 읽어졌는지 확인.
    if not ret:
        print("Can't receive frame (stream end?). Exiting ...")
        break

    # 이미지 크기를 조절하여 스트리밍 창에 표시
    frame_resized = cv2.resize(frame, (400, 300))  # 원하는 가로(width), 세로(height) 크기로 조절
    cv2.imshow('frame', frame_resized)

    # 읽어온 프레임을 화면에 표시.
    cv2.imshow('frame', frame)

    # 'Esc' 키를 누르면 루프를 종료.
    key = cv2.waitKey(1)
    if key == 27 or cv2.getWindowProperty('frame', cv2.WND_PROP_VISIBLE) < 1:
        break

# 작업이 끝나면 VideoCapture 객체를 해제하고 창을 닫습니다.
cap.release()
cv2.destroyAllWindows()

