class WallFollower:
    """Class to safely explore an environment (without crashing) when the pose is unknown."""

    def __init__(self, dt: float) -> None:
        """Wall following class initializer.

        Args:
            dt: Sampling period [s].

        """
        self._dt: float = dt

        self.forward = 1
        self.rotate_90_right = 2
        self.rotate_90_left = 3
        self.rotate_180 = 4
        self.state = self.forward

        self.follow_right_wall = True
        self.distance = 0.25
        self.wall_distance = 0.2

        self.Kp = 4
        self.Ki = 0
        self.Kd = 4

        self.integral_error = 0
        self.error_anterior = 0

    def compute_commands(self, z_scan: list[float], z_v: float, z_w: float) -> tuple[float, float]:
        """Wall following exploration algorithm.

        Args:
            z_scan: Distance from every LiDAR ray to the closest obstacle [m].
            z_v: Odometric estimate of the linear velocity of the robot center [m/s].
            z_w: Odometric estimate of the angular velocity of the robot center [rad/s].

        Returns:
            v: Linear velocity [m/s].
            w: Angular velocity [rad/s].

        """
        # 2.14. Complete the function body with your code (i.e., compute v and w).
        min_front = z_scan[0]
        min_left = z_scan[len(z_scan) // 4]
        min_right = z_scan[len(z_scan) * 3 // 4]

        if self.follow_right_wall:
            min_diagonal = z_scan[len(z_scan) * 7 // 8]
        else:
            min_diagonal = z_scan[len(z_scan) // 8]

        if min_right > 0.45 and self.follow_right_wall:
            self.follow_right_wall = False
        elif min_left > 0.45 and not self.follow_right_wall:
            self.follow_right_wall = True

        if self.state is self.forward:
            if min_front < self.distance:
                max_distance = max(min_left, min_right)
                if max_distance < self.distance:
                    self.state = self.rotate_180
                else:
                    if max_distance == min_right:
                        self.state = self.rotate_90_right
                    else:
                        self.state = self.rotate_90_left
                self.integral_error = 0
                self.error_anterior = 0

        elif (
            self.state is self.rotate_90_right
            or self.state is self.rotate_90_left
            or self.state is self.rotate_180
        ):
            if min_front > self.distance and min_diagonal > self.distance:
                self.state = self.forward

        # Transiciones
        if self.state is self.forward:
            # Necesitamos control PID para que vaya recto sin chocarse. Que siga a una pared
            if self.follow_right_wall:
                if not min_right:
                    min_right = 0.15
                current_distance = min_right
                factor = 1
            else:
                if not min_left:
                    min_left = 0.15
                current_distance = min_left
                factor = -1

            error = self.wall_distance - current_distance
            self.integral_error += error * self._dt
            derivada_error = (error - self.error_anterior) / self._dt
            v = 0.14  # velocidad exterior
            # v = 0.04  # velocidad interior
            w = factor * (
                self.Kp * error + self.Ki * self.integral_error + self.Kd * derivada_error
            )
            self.error_anterior = error

        elif self.state is self.rotate_90_right:
            v = 0.0
            # w = -0.5  # velocidad exterior
            w = -0.5  # velocidad interior

        elif self.state is self.rotate_90_left:
            v = 0.0
            # w = 0.5    # velocidad exterior
            w = 0.5  # velocidad interior

        elif self.state is self.rotate_180:
            v = 0.0
            # w = 0.5    # velocidad exterior
            w = 0.5  # velocidad interior

        else:
            v = 0.0
            w = 0.0

        return v, w, self.state
