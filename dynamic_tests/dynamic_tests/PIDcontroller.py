class PIDController:
    def __init__(self, kp, ki, kd, dt):
        """
        Simple PID Controller initialization.

        :param kp: Proportional gain.
        :param ki: Integral gain.
        :param kd: Derivative gain.
        """
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.dt = dt

        self.integral = 0.0
        self.previous_error = 0.0

    def compute(self, target, current):
        """
        Compute the control effort based on the target and current values.

        :param target: Desired setpoint value.
        :param current: Current measured value.
        :param dt: Time difference since the last computation.
        :return: Control effort (e.g., throttle command).
        """
        error = target - current

        # Proportional term
        P = self.kp * error

        # Integral term
        self.integral += error *self.dt
        I = self.ki * self.integral

        # Derivative term
        derivative = (error - self.previous_error) /self.dt if self.dt > 0 else 0.0
        D = self.kd * derivative

        # Compute total output
        output = P + I + D

        # Update previous error
        self.previous_error = error

        return output
