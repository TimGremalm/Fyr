import threading
from time import sleep, time
from enum import Enum
from pythonosc.udp_client import SimpleUDPClient
import colorsys

class Sector(threading.Thread):
    def __init__(self, fyr):
        self.center = 0.0
        self.offset = 0.2
        self.width = 0.2
        self.rotate_time = 3.0
        self.step_time = 0.100
        self.fyr = fyr
        self.fyr.sector = self
        self.now = 0

        threading.Thread.__init__(self)
        self.setDaemon(True)
        self.quit = False

    def draw(self):
        steps = self.rotate_time / self.step_time
        step_increment = 1.0 / steps
        self.now += step_increment
        if self.now > 1.0:
            self.now = 0
        # Add position + center of sector + cllibration offset. Modula by 1.
        # print(self.now)
        self.fyr.pan = (self.now + self.center + self.offset) % 1
        outer_width = (1 - self.width) / 2
        if self.now >= 0 and self.now < outer_width:
            # print("before")
            self.fyr.red = 0.0
            self.fyr.green = 1.0
            self.fyr.blue = 0.0
            self.fyr.white = 0.0
        elif self.now >= outer_width and self.now < (outer_width + self.width):
            # print("sector")
            self.fyr.red = 0.5
            self.fyr.green = 0.5
            self.fyr.blue = 0.2
            self.fyr.white = 1.0
        else:
            # print("after")
            self.fyr.red = 1.0
            self.fyr.green = 0.0
            self.fyr.blue = 0.0
            self.fyr.white = 0.0

    def run(self):
        while True:
            if self.quit:
                return
            self.draw()
            sleep(self.step_time)

class Fyr(threading.Thread):
    def __init__(self):
        # Variables
        self.osc_ip = "127.0.0.1"
        self.osc_port = 7700
        self.osc_incoming_port = 9000
        self.dimmer = 1.0
        self.red = 0.0
        self.green = 0.0
        self.blue = 0.0
        self.white = 0.0
        self.pan = 0.0
        self.pan_fine = 0.0
        self.speed_min = 0.8
        self.speed_max = 0.5
        self.ignoresend = False

        threading.Thread.__init__(self)
        self.setDaemon(True)
        self.quit = False

    def run(self):
        self.osc = SimpleUDPClient(self.osc_ip, self.osc_port)
        while True:
            if self.quit:
                return
            # self.draw()
            if not self.ignoresend:
                self.send()
            # Cap to 50 FPS
            sleep(0.020)

    def send(self):
        self.send_osc_dimmer(self.dimmer)
        self.send_osc_red(self.red)
        self.send_osc_green(self.green)
        self.send_osc_blue(self.blue)
        self.send_osc_white(self.white)
        self.send_osc_pan(self.pan)
        self.send_osc_pan_fine(self.pan_fine)
        self.send_osc_speed_min(self.speed_min)
        self.send_osc_speed_max(self.speed_max)

    def send_osc_dimmer(self, value: float):
        self.osc.send_message("/dimmer", value)

    def send_osc_red(self, value: float):
        self.osc.send_message("/red", value)

    def send_osc_green(self, value: float):
        self.osc.send_message("/green", value)

    def send_osc_blue(self, value: float):
        self.osc.send_message("/blue", value)

    def send_osc_white(self, value: float):
        self.osc.send_message("/white", value)

    def send_osc_pan(self, value: float):
        self.osc.send_message("/pan", value)

    def send_osc_pan_fine(self, value: float):
        self.osc.send_message("/pan_fine", value)

    def send_osc_speed_min(self, value: float):
        self.osc.send_message("/speed_min", value)

    def send_osc_speed_max(self, value: float):
        self.osc.send_message("/speed_max", value)

if __name__ == '__main__':
    title_short = "Fyr"
    title_long = "Fyr Controller"
    print(title_long)
    from pysh.shell import Pysh  # https://github.com/TimGremalm/pysh

    f = Fyr()
    f.start()

    s = Sector(f)
    s.start()

    banner = [f"{title_long} Shell",
              'You may leave this shell by typing `exit`, `q` or pressing Ctrl+D']
    Pysh(dict_to_include={'fyr': f},
         prompt=f"{title_short}$ ",
         banner=banner)
    f.quit = True
    s.quit = True

