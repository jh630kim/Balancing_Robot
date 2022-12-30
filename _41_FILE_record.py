import time

file_name = './sensor_' + str(int(time.time())) + '.csv'

def initialize(channel, sensor_name, sensor_min, sensor_max):
    file=open(file_name, 'w')
    for i in sensor_name:
        file.write('%s ' % i)
    file.write('\n')
    file.close()

def msg_send(msg, data):
    file=open(file_name, 'a')
    for i in data:
        file.write('%s\n' % i)
    file.close()

def close():
    pass
