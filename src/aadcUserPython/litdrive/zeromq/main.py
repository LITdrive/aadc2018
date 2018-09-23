import zmq

from zeromq.util.zmqParser import *

# connect to the car
context = zmq.Context()
socket = context.socket(zmq.REP)
socket.bind("tcp://*:5555")
print("connected")

while True:
    req = socket.recv_multipart()
    req_ = parseFromZMQ(req, ['tSignalValue', 'tBoolSignalValue', 'tWheelData', 'tInerMeasUnitData', 'tUltrasonicStruct', 'tVoltageStruct'])
    print("Received: ")
    print(req_)

    rep = [[42, 1337], [45, True]]
    rep_ = parseToZMQMultiple(rep, ['tSignalValue', 'tBoolSignalValue'])
    socket.send_multipart(rep_)
    print("Sent: ", end="")
    print(rep)
