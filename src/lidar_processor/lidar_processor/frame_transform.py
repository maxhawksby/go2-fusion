"""Apply a ROS transform to points. No ROS imports, so tests can call it.

Points stay stored in the cloud frame (odom). The .ply is written in the
map frame using the latest map <- odom transform, which is the same
transform slam_toolbox updates on a loop closure. Re-applying that
transform at each save moves the whole file with the correction. The
in-memory cloud is not rewritten, and it is not cleared to drop obstacles
that have moved.
"""


def rotate_point(x, y, z, quaternion):
    """Rotate one point by a quaternion (x, y, z, w)."""

    qx, qy, qz, qw = quaternion
    tx = 2.0 * (qy * z - qz * y)
    ty = 2.0 * (qz * x - qx * z)
    tz = 2.0 * (qx * y - qy * x)
    return (
        x + qw * tx + (qy * tz - qz * ty),
        y + qw * ty + (qz * tx - qx * tz),
        z + qw * tz + (qx * ty - qy * tx),
    )


def transform_points(points, translation, quaternion):
    """Points expressed in the parent frame of `translation` and `quaternion`.

    `translation` is (x, y, z). `quaternion` is (x, y, z, w), the rotation
    of the source frame in the target frame, as tf2's lookup of
    target <- source reports it.
    """

    tx, ty, tz = translation
    transformed = []
    for x, y, z in points:
        rx, ry, rz = rotate_point(x, y, z, quaternion)
        transformed.append((rx + tx, ry + ty, rz + tz))
    return transformed
