import sys
import tty
import termios

def up_arrow():
    print("Up arrow pressed")

def down_arrow():
    print("Down arrow pressed")

def left_arrow():
    print("Left arrow pressed")

def right_arrow():
    print("Right arrow pressed")

def get_key():
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(sys.stdin.fileno())
        ch = sys.stdin.read(1)
        if ch == '\x1b':
            ch = sys.stdin.read(2)
            return ch
        return ch
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)

def main():
    print("Press arrow keys (q to quit)...")
    while True:
        key = get_key()
        
        if key == '[A':
            up_arrow()
        elif key == '[B':
            down_arrow()
        elif key == '[C':
            right_arrow()
        elif key == '[D':
            left_arrow()
        elif key == 'q':
            break
        
        sys.stdout.flush()

if __name__ == "__main__":
    main()