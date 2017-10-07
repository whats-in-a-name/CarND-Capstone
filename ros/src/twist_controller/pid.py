
MIN_NUM = float('-inf')
MAX_NUM = float('inf')


class PID(object):
    def __init__(self, k_p, k_i, k_d, y_min=MIN_NUM, y_max=MAX_NUM):
        self.k_p = k_p
        self.k_i = k_i
        self.k_d = k_d
        self.y_min = y_min
        self.y_max = y_max

        self.reset()

    def reset(self):
        self.e_p = None
        self.e_i = 0.0
        self.e_d = 0.0

    def step(self, e, dt):
        if self.e_p is None:
            self.e_p = e

        self.e_d = (e - self.e_p) / dt
        self.e_i += e * dt
        self.e_p = e

        y = (
            self.k_p * self.e_p +
            self.k_i * self.e_i +
            self.k_d * self.e_d
        )

        # Truncate y to the valid range.
        y = min(y, self.y_max)
        y = max(y, self.y_min)

        return y
