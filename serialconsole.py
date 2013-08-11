import os,serial
import thread

import select,sys
import tty
import termios
import time

class NonBlockingConsole(object):
    def __enter__(self):
        self.old_settings = termios.tcgetattr(sys.stdin)
        tty.setcbreak(sys.stdin.fileno())
        return self

    def __exit__(self, type, value, traceback):
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.old_settings)


    def get_data(self):
        if select.select([sys.stdin], [], [], 0) == ([sys.stdin], [], []):
            return sys.stdin.read(1)
        return False

PORT="/dev/ttyUSB0"


# Does serial IO and key commands, quits if any files written locally (e.g. src code changed)
class MyTerminal(object):

    def __init__(self,watch_files):
        self.watch_files=watch_files
        self.last_mod={}
        self.console=NonBlockingConsole()
        self.port= serial.Serial(
            port=PORT,
            baudrate=57600,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            bytesize=serial.EIGHTBITS,
            timeout=0,
            )

    def check_files(self):
        changed=False
        for f in self.watch_files:
            now=os.path.getmtime(f)
            lm=self.last_mod.get(f,None)
            if lm<now:
                self.last_mod[f]=now
                changed=True
        return changed

    def keyboard_char(self,char):
        self.port.write(c)
        echo=False
        if echo:
            sys.stdout.write(">%s" % c)


        
    def start(self,):
        with(self.console):
            last_check=time.time()
            self.check_files()
            while(1):
                did=False
                now=time.time()
                if now-last_check > 1:
                    if self.check_files():
                        return

                t=self.port.read()
                if t:
                    did=True
                    sys.stdout.write(t)

                c=self.console.get_data()
                if c:
                    if ord(c)==27:  #esc
                        return
                    self.keyboard_char(c)
                    did=True

                if not did:
                    sys.stdout.flush()
                    time.sleep(0.05)




b=MyTerminal(watch_files=sys.argv[1:])
b.start()
