import numpy as np

class KalmanFilterPID:
    def __init__(self, Kp, Ki, Kd, dt):
        # PID gains
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd

        # Sampling time
        self.dt = dt

        # Kalman filter variables
        self.x = np.zeros(2)  # State vector [position, velocity]
        self.P = np.eye(2)  # Error covariance matrix
        self.Q = np.eye(2)  # Process noise covariance matrix
        self.R = np.eye(1)  # Measurement noise covariance matrix

        # PID variables
        self.error_sum = 0.0
        self.prev_error = 0.0

    def update(self, setpoint, measurement):
        # Prediction step
        self.x[0] += self.dt * self.x[1]  # Predicted position
        self.P[0, 0] += self.dt * (self.P[1, 1] + self.P[0, 1] + self.P[1, 0] + self.Q[0, 0])  # Predicted error covariance

        # Kalman gain calculation
        K = self.P[0, 0] / (self.P[0, 0] + self.R[0, 0])

        # Update step
        self.x[0] += K * (measurement - self.x[0])  # Updated position
        self.x[1] = self.x[1] + K * (measurement - self.x[0]) / self.dt  # Updated velocity
        self.P[0, 0] -= K * self.P[0, 0]  # Updated error covariance

        # PID control calculation
        error = setpoint - self.x[0]
        self.error_sum += error * self.dt
        error_rate = (error - self.prev_error) / self.dt

        output = self.Kp * error + self.Ki * self.error_sum + self.Kd * error_rate

        # Update previous error
        self.prev_error = error

        return output
