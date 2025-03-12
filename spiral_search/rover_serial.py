import serial

def connect_rover(port:str, baudrate=9600):
    board = serial.Serial(port, baudrate)
    return board

def stop(board:serial.Serial):
    board.write('S'.encode())

def move_forward(board:serial.Serial):
    board.write('F'.encode())

def move_backward(board:serial.Serial):
    board.write('B'.encode())

def move_left(board:serial.Serial):
    board.write('L'.encode())

def move_right(board:serial.Serial):
    board.write('R'.encode())

def move_forward_left(board:serial.Serial):
    board.write('I'.encode())

def move_forward_right(board:serial.Serial):
    board.write('J'.encode())

def printFromBoard(board:serial.Serial):
    if board.in_waiting > 0:
        data = board.readline().decode('utf-8').strip()
        return data