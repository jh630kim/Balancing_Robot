# -*- coding: utf-8 -*-

# https://category.yahboom.net/products/balancecar 를 참고해서 작성
# 홈페이지에 게시된 App을 이용하도록 작성함
# 따라서!!! Yahboom app 의 명령을 파싱한다.
# Document 에 앱, 프로토콜, 코드까지 다 있다.

import bluetooth        # pip3 install pybluez 로 설치 필요.
import time
import threading
import math


class Comm_BT:
    # 블루투스 초기화
    def __init__(self):
        self.server_socket = 0
        self.client_socket = 0
        self.address = 0
        self.port = 0
        self.Kp = 0
        self.Kd = 0
        self.Kp2 = 0
        self.Ki2 = 0

        # 상태
        self.newline = False
        self.command = ''
        self.auto_up = False
        self.reset_pid = False
        self.set_pid = False

        # 파싱한 명령
        self.car_cmd0 = {'1': 'FORWARD',
                         '2': 'BACKWARD',
                         '3': 'LEFT',
                         '4': 'RIGHT',
                         '0': 'STOP'}
        self.car_cmd1 = {'1': 'TURN_LEFT',
                         '2': 'TURN_RIGHT',
                         '0': 'STOP'}

        self.BT_Init()

    # BT 초기화
    def BT_Init(self):
        print("Wating BT connection...")
        # Server Socket 설정
        self.server_socket = bluetooth.BluetoothSocket(bluetooth.RFCOMM)
        self.port = 1
        # Server Socket과 RFCOMM port를 bind
        self.server_socket.bind(("", self.port))
        # Client를 기다림
        self.server_socket.listen(1)
        # Client 연결
        self.client_socket, self.address = self.server_socket.accept()
        print("Accepted connection from ", self.address)

        # 연결 메시지 전송
        data = "$$$U,9600,N"
        self.client_socket.send(data)

    '''
    # 소켓 연결 Thread 시작
    def wait_client_thread(self):
        waitThread = threading.Thread(target=self.wait_client_BT)
        waitThread.start()
    
    def wait_client_BT(self):
    '''

    # 데이터 수신 Thread 시작
    def start_comm_thread(self):
        commThread = threading.Thread(target=self.data_receiving_BT)
        commThread.start()

    # 데이터 수신 쓰레드
    def data_receiving_BT(self):
        # "$0,0,0,0,0,0,0,0,0,0,0,0cm,8.2V#";
        while True:
            incomingStr = self.client_socket.recv(1024)
            temp = incomingStr.decode()
            # 첫번째 바이트는 '$', 마지막 바이트는 "#"
            if (temp[0] != "$") or (temp[-1] != "#"):
                # 자꾸 에러가 나면... 파싱 구문을 보완해야 한다. $와 #을 찾도록...
                print("Parsing Error in Bluetooth Thread!!!")
                print(temp)
            else:
                # 가끔 두개가 연달아 들어오는 경우가 있다. 첫번째 것만 처리
                # 첫 문자 $를 빼고 데이터만 전송
                self.command = temp.split('#')[0][1:]
                self.newline = True
                # print(self.command)
            time.sleep(0.01)

    # 명령 파싱
    def parsing(self, command, Kp=0, Kd=0, Kp2=0, Ki2=0):
        # parsing 시작
        cmd_list = command.split(',')

        # 1st byte : 전/후/좌/우/정지 명령
        try:
            car_state = self.car_cmd0[cmd_list[0]]
            # print(car_state)
        except KeyError:
            car_state = self.car_cmd0['0']
            # print("error1", car_state)

        # 2nd byte : 좌/우 회전
        if cmd_list[1] != '0':
            try:
                car_state = self.car_cmd1[cmd_list[1]]
                # print(car_state)
            except KeyError:
                car_state = self.car_cmd1['0']
                # print("error2", car_state)

        # 3rd byte : PID 계수 전송
        if cmd_list[2] == '1':
            return_data = "$0,0,0,0,0,0,AP{},AD{},VP{},VI{}#".format(Kp, Kd, Kp2, Ki2)
            print(return_data)
            self.client_socket.send(return_data)
        # 3rd byte : PID 계수 초기화
        elif cmd_list[2] == '2':
            self.reset_pid = True
            # print(self.reset_pid)
            return_data = "$OK#"
            self.client_socket.send(return_data)

        # 4th byte : 데이터 자동 전송
        if cmd_list[3] == '1':
            self.auto_up = True
            # print(self.auto_up)
            return_data = "$OK#"
            self.client_socket.send(return_data)
        elif cmd_list[3] == '2':
            self.auto_up = False
            # print(self.auto_up)
            return_data = "$OK#"
            self.client_socket.send(return_data)

        # 5th byte : Angle PID update
        # 6th byte : Speed PID update
        if (cmd_list[4] == '1') or (cmd_list[5] == '1'):
            self.Kp = float(cmd_list[6][2:])
            self.Kd = float(cmd_list[7][2:])
            self.Kp2 = float(cmd_list[8][2:])
            self.Ki2 = float(cmd_list[9][2:])
            self.set_pid = True
            # print(self.Kp, self.Kd, self.Kp2, self.Ki2)
            return_data = "$OK#"
            self.client_socket.send(return_data)

        return car_state

    # 주의!!!!
    # Accel_Angle, Gyro_Angle, Battery 만 표기되는 것 같다.
    # 그리고, Accel 과 Gyro 는 x10이 되는 것 같다.
    def auto_update(self, Accel_Angle=0, Gyro_Angle=0, Battery=0,
                    left_speed=0, right_speed=0, CSB=0):
        # 자동 Update 실시
        msg = "$LV{},RV{},AC{},GY{},CSB{},VT{}#"
        return_data = msg.format(left_speed, right_speed,
                                 Accel_Angle, Gyro_Angle,
                                 CSB, Battery)
        # print(return_data)
        self.client_socket.send(return_data)

    # 새로운 명령이 있는지 확인 및 초기화
    def get_command(self):
        return self.newline, self.command

    def clear_command(self):
        self.newline = False
        self.command = ''

    # 현재 값을 자동을 update 할 것인지 확인
    # reset은 BT 명령에 의해 된다.
    def get_auto_up(self):
        return self.auto_up

    # PID 값을 Reset 할 것이가?
    def get_reset_pid(self):
        return self.reset_pid

    def clear_reset_pid(self):
        self.reset_pid = False

    # PID 값을 재설정 할 것인가?
    def get_new_pid(self):
        return self.set_pid, self.Kp, self.Kd, self.Kp2, self.Ki2

    def clear_set_pid(self):
        self.set_pid = False

    # 외부에서 직접 메시지를 보낼 경우
    def msg_send(self, msg):
        self.client_socket.send(msg)


if __name__ == '__main__':
    bt = Comm_BT()
    bt.start_comm_thread()

    Kp, Kd, Kp2, Ki2 = (1.1, 2.2, 3.3, 4.4)

    while True:
        # Example1) 새로운 명령이 있는지 확인 후 파싱
        status, command = bt.get_command()
        if status:
            print("command = ", command)
            # 전/후/좌/우/좌턴/우턴/정지 명령, PID 계수 확인
            car_state = bt.parsing(command, Kp, Kd, Kp2, Ki2)
            print("CarState = ", car_state)
            bt.clear_command()

        # Example2) BT로 송신할 상태 명령, 현재는 3개만 Display된다.
        A = 0   # Accel_Angle
        B = 0   # Gyro_Angle
        C = 0   # Battery
        auto_up = bt.get_auto_up()
        if auto_up:
            A = math.sin(time.time()) * 10      # display할 때 x 10이 된다.
            B = math.cos(time.time()) * 10      # display할 때 x 10이 된다.
            C = math.sin(time.time()) * 100     # display할 때 x 1이 된다.
            bt.auto_update(A, B, C)

        # Example3) PID값 초기화
        reset_pid = bt.get_reset_pid()
        if reset_pid:
            Kp, Kd, Kp2, Ki2 = (1.1, 2.2, 3.3, 4.4)
            print("Kp, Kd, Kp2, Ki2 =", Kp, Kd, Kp2, Ki2)
            bt.clear_reset_pid()

        # Example4) PID값 재설정
        set_pid, _Kp, _Kd, _Kp2, _Ki2 = bt.get_new_pid()
        if set_pid:
            Kp, Kd, Kp2, Ki2 = (_Kp, _Kd, _Kp2, _Ki2)
            print("Kp, Kd, Kp2, Ki2 =", Kp, Kd, Kp2, Ki2)
            bt.clear_set_pid()

        time.sleep(0.01)

    bt.client_socket.close()
    bt.server_socket.close()