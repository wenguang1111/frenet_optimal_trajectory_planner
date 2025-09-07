class BetaAnnealer:
    def __init__(self, beta_start=0.0, beta_end=1.0, n_steps=10000):
        self.beta_start = beta_start
        self.beta_end = beta_end
        self.n_steps = n_steps
        self.step_count = 0

    def step(self):
        self.step_count += 1
        progress = min(1.0, self.step_count / self.n_steps)
        return self.beta_start + progress * (self.beta_end - self.beta_start)
