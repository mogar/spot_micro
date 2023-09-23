


# TODO: smooth all joints together, instead of one at a time

# TODO: pose identity with tolerance


def one_step_interp(start_angle, end_angle, max_angle_delta):
    max_positive_increment = max_angle_delta
    max_negative_increment = -1 * max_positive_increment

    delta_angle = end_angle - start_angle
    # clamp
    delta_angle = max(max_negative_increment, min(delta_angle, max_positive_increment))

    return start_angle + delta_angle
