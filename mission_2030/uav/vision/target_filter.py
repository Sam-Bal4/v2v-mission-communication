class TargetFilter:
    def __init__(self, alpha=0.3):
        self.alpha = alpha
        self.val = None

    def update(self, new_val):
        """ EMA Filter: tuple of floats expected """
        if self.val is None:
            self.val = new_val
        else:
            self.val = tuple(self.alpha * n + (1 - self.alpha) * o for n, o in zip(new_val, self.val))
        return self.val
