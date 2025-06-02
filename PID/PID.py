class PID:
    def __init__(self, T, kp=0, ki=0, kd=0, saturation=None):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.T = T
        self.saturation = saturation
        self.last_error = 0
        self.integral_error = 0

    def control(self, reference, state):
        error = reference - state

        # Integrativo (trapezoidal)
        self.integral_error += (error + self.last_error) * self.T / 2

        # Derivativo (diferença finita)
        derivative_error = (error - self.last_error) / self.T

        # PID control law
        u = (
            self.kp * error
            + self.ki * self.integral_error
            + self.kd * derivative_error
        )

        self.last_error = error

        # Saturation (if defined)
        if self.saturation is not None:
            u = max(-self.saturation, min(u, self.saturation))

        return u
