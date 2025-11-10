import math

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


class CyclicalAnnealer:
    """
    Cyclical Annealing Schedule for KL divergence weight (beta).
    
    Based on: Fu et al. "Cyclical Annealing Schedule: A Simple Approach to 
    Mitigating KL Vanishing" (NAACL 2019)
    
    The schedule consists of multiple cycles where beta increases from 0 to 1
    within each cycle, then resets. This prevents KL vanishing and allows the
    model to alternate between focusing on reconstruction and regularization.
    
    Args:
        n_cycles (int): Number of annealing cycles
        n_steps (int): Total number of training steps
        ratio (float): Proportion of cycle spent increasing beta (0 < ratio <= 1)
                      Default: 0.5 (half cycle for annealing, half at beta=1)
        beta_start (float): Starting beta value in each cycle
        beta_end (float): Ending beta value in each cycle
        schedule (str): Annealing function within each cycle
                       Options: 'linear', 'cosine', 'sigmoid'
    
    Example:
        >>> # 4 cycles over 20000 steps, each cycle uses half time to ramp up
        >>> annealer = CyclicalAnnealer(n_cycles=4, n_steps=20000, ratio=0.5)
        >>> for step in range(20000):
        >>>     beta = annealer.step()
    """
    
    def __init__(self, n_cycles=4, n_steps=10000, ratio=0.5, 
                 beta_start=0.0, beta_end=1.0, schedule='linear'):
        assert 0 < ratio <= 1, "ratio must be in (0, 1]"
        assert schedule in ['linear', 'cosine', 'sigmoid'], \
            "schedule must be 'linear', 'cosine', or 'sigmoid'"
        
        self.n_cycles = n_cycles
        self.n_steps = n_steps
        self.ratio = ratio
        self.beta_start = beta_start
        self.beta_end = beta_end
        self.schedule = schedule
        
        self.step_count = 0
        self.steps_per_cycle = n_steps / n_cycles
        self.ramp_steps = self.steps_per_cycle * ratio
        
    def step(self):
        """Compute beta value for current step and increment counter."""
        beta = self._get_beta()
        self.step_count += 1
        return beta
    
    def _get_beta(self):
        """Calculate beta based on current position in cycle."""
        # Position within current cycle
        cycle_position = self.step_count % self.steps_per_cycle
        
        if cycle_position < self.ramp_steps:
            # Annealing phase: beta increases from start to end
            progress = cycle_position / self.ramp_steps
            
            if self.schedule == 'linear':
                beta = self._linear_schedule(progress)
            elif self.schedule == 'cosine':
                beta = self._cosine_schedule(progress)
            elif self.schedule == 'sigmoid':
                beta = self._sigmoid_schedule(progress)
        else:
            # Constant phase: beta stays at end value
            beta = self.beta_end
        
        return beta
    
    def _linear_schedule(self, progress):
        """Linear annealing from beta_start to beta_end."""
        return self.beta_start + progress * (self.beta_end - self.beta_start)
    
    def _cosine_schedule(self, progress):
        """Cosine annealing (smooth acceleration then deceleration)."""
        cosine_progress = (1 - math.cos(progress * math.pi)) / 2
        return self.beta_start + cosine_progress * (self.beta_end - self.beta_start)
    
    def _sigmoid_schedule(self, progress):
        """Sigmoid annealing (slow start, rapid middle, slow end)."""
        # Map progress [0,1] to sigmoid input [-6, 6] for smooth curve
        x = (progress - 0.5) * 12
        sigmoid_progress = 1 / (1 + math.exp(-x))
        return self.beta_start + sigmoid_progress * (self.beta_end - self.beta_start)
    
    def reset(self):
        """Reset the step counter (useful for multi-run experiments)."""
        self.step_count = 0
    
    def get_cycle_number(self):
        """Return current cycle number (0-indexed)."""
        return int(self.step_count / self.steps_per_cycle)
    
    def get_position_in_cycle(self):
        """Return current position within cycle [0, 1]."""
        return (self.step_count % self.steps_per_cycle) / self.steps_per_cycle

