import threading
from time import sleep, time
from enum import Enum
from pythonosc.udp_client import SimpleUDPClient
import colorsys
import numpy as np

def vector_calc(lat, long, ht):
    '''
    Calculates the vector from a specified point on the Earth's surface to the North Pole.
    '''
    a = 6378137.0  # Equatorial radius of the Earth
    b = 6356752.314245  # Polar radius of the Earth

    e_squared = 1 - ((b ** 2) / (a ** 2))  # e is the eccentricity of the Earth
    n_phi = a / (np.sqrt(1 - (e_squared * (np.sin(lat) ** 2))))

    x = (n_phi + ht) * np.cos(lat) * np.cos(long)
    y = (n_phi + ht) * np.cos(lat) * np.sin(long)
    z = ((((b ** 2) / (a ** 2)) * n_phi) + ht) * np.sin(lat)

    x_npole = 0.0
    y_npole = 6378137.0
    z_npole = 0.0

    v = ((x_npole - x), (y_npole - y), (z_npole - z))

    return v

def angle_calc(lat1, long1, lat2, long2, ht1=0, ht2=0):
    '''
    Calculates the angle between the vectors from 2 points to the North Pole.
    '''
    # Convert from degrees to radians
    lat1_rad = (lat1 / 180) * np.pi
    long1_rad = (long1 / 180) * np.pi
    lat2_rad = (lat2 / 180) * np.pi
    long2_rad = (long2 / 180) * np.pi

    v1 = vector_calc(lat1_rad, long1_rad, ht1)
    v2 = vector_calc(lat2_rad, long2_rad, ht2)

    # The angle between two vectors, vect1 and vect2 is given by:
    # arccos[vect1.vect2 / |vect1||vect2|]
    dot = np.dot(v1, v2)  # The dot product of the two vectors
    v1_mag = np.linalg.norm(v1)  # The magnitude of the vector v1
    v2_mag = np.linalg.norm(v2)  # The magnitude of the vector v2

    theta_rad = np.arccos(dot / (v1_mag * v2_mag))
    # Convert radians back to degrees
    theta = (theta_rad / np.pi) * 180

    return theta

class CoordinateFetcher(threading.Thread):
    def __init__(self, fyr, sector):
        self.fyr = fyr
        self.sector = sector
        self.fyr.fetcher = self

        threading.Thread.__init__(self)
        self.setDaemon(True)
        self.quit = False

    def fetch(self):
        pass

    def run(self):
        while True:
            if self.quit:
                return
            self.fetch()
            sleep(2)

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

    c = CoordinateFetcher(f, s)
    c.start()

    banner = [f"{title_long} Shell",
              'You may leave this shell by typing `exit`, `q` or pressing Ctrl+D']
    Pysh(dict_to_include={'fyr': f},
         prompt=f"{title_short}$ ",
         banner=banner)
    f.quit = True
    s.quit = True
    c.quit = True

