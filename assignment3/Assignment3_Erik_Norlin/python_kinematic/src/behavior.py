import csv

class Behavior():
    def __init__(self, dt, R):
        self.t = []
        self.v = []
        self.phiDot = []
        self.readCSV(R)
        self.dt = dt


    def get_v_phiDot(self, t):
        t_idx = int(t/self.dt)
        return self.v[t_idx], self.phiDot[t_idx]


    def readCSV(self, R):
        path = r'data/wheelspeeds-1.csv'
        line = 0
        counter = 20

        with open(path) as file: # Grab every 20th element to mimic dt=0.1
            reader = csv.reader(file)
            for row in reader:
                if line > 0:
                    if counter == 20:

                        t = float(row[0])
                        vR = float(row[1])
                        vL = float(row[2])

                        v = (vR+vL)/2
                        phiDot = (vR-vL)/(2*R)

                        self.t.append(t)
                        self.v.append(v)
                        self.phiDot.append(phiDot)

                        counter = 0
                    counter += 1
                line += 1