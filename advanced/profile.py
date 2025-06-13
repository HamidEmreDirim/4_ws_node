class LinearProfile:
    def __init__(self, start, end):
        self.start, self.end = start, end

    def value_at(self, t_rel):
        t = min(max(t_rel, 0.0), 1.0)
        return self.start + (self.end - self.start) * t
