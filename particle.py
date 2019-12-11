class Particle:
    def __init__(pose0, weight0, map0):
        self.pose   = pose0 # 3 element np array
        self.weight = weight0
        self.map    = map0
