#! /usr/local/bin/python3
import zmq
import time
import random
import json
import flock_control as fc

TIMEOUT = 10000
POLL_TIME = 1/30
WAIT_TIME = .5
flock_control = None

def getEventFromSocket(socket_listen):
    socket_listen.send_string("request")
    poller = zmq.Poller()
    poller.register(socket_listen, zmq.POLLIN)
    evt = dict(poller.poll(TIMEOUT))
    return evt

def handleEvent(evt,socket_listen): 
    global flock_control
    if evt.get(socket_listen) != zmq.POLLIN:
        return False

    
    response_raw = socket_listen.recv(zmq.NOBLOCK)
    response = json.loads(response_raw.decode("utf-8"))
    if not flock_control:
        flock_control = fc.FlockControl(len(response["birds"]))
    command = flock_control.make_decisions(response)
    print(json.dumps(command).encode("utf-8"))
    return True

def closeSocketAndWait(socket_listen):
    time.sleep(WAIT_TIME)
    socket_listen.close()
    socket_listen = context.socket_listen(zmq.REQ)
    socket_listen.connect("tcp://localhost:12346")

def main():
    context = zmq.Context()
    socket_listen = context.socket(zmq.REQ)
    socket_listen.connect("tcp://localhost:12346")

    while True:
        evt = getEventFromSocket(socket_listen)
        if evt and handleEvent(evt,socket_listen):
            time.sleep(POLL_TIME)
            continue
        closeSocketAndWait(socket_listen)

main()