from pyfirmata import Board, Arduino

# Need to change em according to rover pins
left_motor_forward_pin = 9
left_motor_backward_pin = 8
right_motor_forward_pin = 6
right_motor_backward_pin = 7
full_throttle_threshold = 0.3
half_throttle_threshold = 0.15


def connect_rover(port:str):
    board = Arduino(port)
    # Setup motor pins as outputs
    board.digital[left_motor_forward_pin].mode = 1  # OUTPUT
    board.digital[left_motor_backward_pin].mode = 1  # OUTPUT
    board.digital[right_motor_forward_pin].mode = 1  # OUTPUT
    board.digital[right_motor_backward_pin].mode = 1  # OUTPUT
    
    return board

# Function to stop the rover
def stop(board:Board):
    board.digital[left_motor_forward_pin].write(0)
    board.digital[right_motor_forward_pin].write(0)
    board.digital[left_motor_backward_pin].write(0)
    board.digital[right_motor_backward_pin].write(0)

# Function to move the rover forward
def move_forward(board:Board):
    stop(board)
    board.digital[left_motor_forward_pin].write(full_throttle_threshold)
    board.digital[right_motor_forward_pin].write(full_throttle_threshold)
    board.digital[left_motor_backward_pin].write(0)
    board.digital[right_motor_backward_pin].write(0)

# Function to move the rover backward
def move_backward(board:Board):
    stop(board)
    board.digital[left_motor_backward_pin].write(full_throttle_threshold)
    board.digital[right_motor_backward_pin].write(full_throttle_threshold)
    board.digital[left_motor_forward_pin].write(0)
    board.digital[right_motor_forward_pin].write(0)

# Function to move the rover left (turn in place)
def move_left(board:Board):
    stop(board)
    board.digital[left_motor_backward_pin].write(full_throttle_threshold)
    board.digital[right_motor_forward_pin].write(full_throttle_threshold)
    board.digital[left_motor_forward_pin].write(0)
    board.digital[right_motor_backward_pin].write(0)

# Function to move the rover right (turn in place)
def move_right(board:Board):
    stop(board)
    board.digital[left_motor_forward_pin].write(full_throttle_threshold)
    board.digital[right_motor_backward_pin].write(full_throttle_threshold)
    board.digital[left_motor_backward_pin].write(0)
    board.digital[right_motor_forward_pin].write(0)

# Function to move the rover forward-left (left wheel slower)
def move_forward_left(board:Board):
    stop(board)
    board.digital[left_motor_forward_pin].write(half_throttle_threshold)  # Half speed left motor
    board.digital[right_motor_forward_pin].write(full_throttle_threshold)   # Full speed right motor
    board.digital[left_motor_backward_pin].write(0)
    board.digital[right_motor_backward_pin].write(0)


# Function to move the rover forward-right (right wheel slower)
def move_forward_right(board:Board):
    stop(board)
    board.digital[left_motor_forward_pin].write(full_throttle_threshold)    # Full speed left motor
    board.digital[right_motor_forward_pin].write(half_throttle_threshold)  # Half speed right motor
    board.digital[left_motor_backward_pin].write(0)
    board.digital[right_motor_backward_pin].write(0)
