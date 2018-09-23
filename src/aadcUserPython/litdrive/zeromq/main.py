import zmq

# connect to the car
context = zmq.Context()
socket = context.socket(zmq.REP)
socket.bind("tcp://*:5555")

# [516556, True] + [516558, False]
boolSignals = [b'\xcc\xe1\x07\x00\x01', b'\xce\xe1\x07\x00\x00']
bi = 0

# [516556, 9999] + [516558, 70135]
floatSignals = [b'\xcc\xe1\x07\x00\x00<\x1cF', b'\xce\xe1\x07\x00\x80\xfb\x88G']
fi = 0

while True:
    msg = socket.recv_multipart()
    print("Received: ", end="")
    print(msg)

    print("Sent: ", end="")
    sending = [boolSignals[bi], floatSignals[fi]]
    print(sending)

    socket.send_multipart(sending)

    bi = (bi + 1) % 2
    fi = (fi + 1) % 2
