import socket
import _41_FILE_record as rec

# 접속할 서버 주소와 포트 번호 지정
HOST = '192.168.0.3'
PORT = 5560     # Prosessing의 Sensor_Visualizer_V2의 번호

# 1) 소켓 객체 생성
client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    
def initialize(channel, sensor_name, sensor_min, sensor_max):
    rec.initialize(channel, sensor_name, sensor_min, sensor_max)
	
    # 2) 지정한 서버 주소(HOST)와 포트 번호를 이용해서 접속
    client_socket.connect((HOST, PORT))

    # 3) Sensor Visualizer V2 초기화 메시지 생성
    # 초기 메시지 구성
    msg = 'init ' + str(channel)
    for i in range(channel):
        msg += ' ' + sensor_name[i] +\
               ' ' + str(sensor_min[i]) +\
               ' ' + str(sensor_max[i])
    msg += '\n'

    # 4) Sensor Visualizer V2 초기화 메시지(msg) 전송
    # msg = 'init 9 ROLL(X) -180 180 ~~~~ \n'
    client_socket.sendall(msg.encode())


def msg_send(msg, data):
    rec.msg_send(msg, data)
    for i in data:
        msg += ' ' + i
    msg += '\n'
    client_socket.sendall(msg.encode())


def close():
    # 소켓 닫기
    client_socket.close()

