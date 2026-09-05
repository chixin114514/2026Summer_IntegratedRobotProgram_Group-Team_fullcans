import math


# ============================================================
# Exceptions
#
# task_manager catches these exceptions and converts them into
# a SAFE_STOP instead of allowing dangerous robot motion.
# ============================================================

class KinematicsError(Exception):
    pass


class UnreachableTargetError(
    KinematicsError
):
    pass


class NoIKSolutionError(
    KinematicsError
):
    pass


class JointLimitError(
    KinematicsError
):
    pass


# ============================================================
# Matrix utilities
# ============================================================

def identity4():

    return [
        [1.0, 0.0, 0.0, 0.0],
        [0.0, 1.0, 0.0, 0.0],
        [0.0, 0.0, 1.0, 0.0],
        [0.0, 0.0, 0.0, 1.0],
    ]


def mat_mul(
    a,
    b,
):

    rows = len(a)

    columns = len(
        b[0]
    )

    inner = len(b)

    result = [
        [
            0.0
            for _ in range(
                columns
            )
        ]
        for _ in range(
            rows
        )
    ]

    for row in range(
        rows
    ):

        for column in range(
            columns
        ):

            for index in range(
                inner
            ):

                result[
                    row
                ][
                    column
                ] += (
                    a[
                        row
                    ][
                        index
                    ]
                    *
                    b[
                        index
                    ][
                        column
                    ]
                )

    return result


def translation(
    x,
    y,
    z,
):

    result = identity4()

    result[0][3] = x
    result[1][3] = y
    result[2][3] = z

    return result


def rot_x(angle):

    c = math.cos(
        angle
    )

    s = math.sin(
        angle
    )

    return [
        [1.0, 0.0, 0.0, 0.0],
        [0.0, c, -s, 0.0],
        [0.0, s, c, 0.0],
        [0.0, 0.0, 0.0, 1.0],
    ]


def rot_y(angle):

    c = math.cos(
        angle
    )

    s = math.sin(
        angle
    )

    return [
        [c, 0.0, s, 0.0],
        [0.0, 1.0, 0.0, 0.0],
        [-s, 0.0, c, 0.0],
        [0.0, 0.0, 0.0, 1.0],
    ]


def rot_z(angle):

    c = math.cos(
        angle
    )

    s = math.sin(
        angle
    )

    return [
        [c, -s, 0.0, 0.0],
        [s, c, 0.0, 0.0],
        [0.0, 0.0, 1.0, 0.0],
        [0.0, 0.0, 0.0, 1.0],
    ]


def rpy_matrix(
    roll,
    pitch,
    yaw,
):

    return mat_mul(
        mat_mul(
            rot_z(
                yaw
            ),
            rot_y(
                pitch
            ),
        ),
        rot_x(
            roll
        ),
    )


def origin_matrix(
    xyz,
    rpy,
):

    return mat_mul(
        translation(
            xyz[0],
            xyz[1],
            xyz[2],
        ),
        rpy_matrix(
            rpy[0],
            rpy[1],
            rpy[2],
        ),
    )


def transpose3(matrix):

    return [
        [
            matrix[
                column
            ][
                row
            ]
            for column in range(
                3
            )
        ]
        for row in range(
            3
        )
    ]


def mat3_mul(
    a,
    b,
):

    return [
        [
            sum(
                a[row][index]
                *
                b[index][column]
                for index in range(
                    3
                )
            )
            for column in range(
                3
            )
        ]
        for row in range(
            3
        )
    ]


# ============================================================
# MechArm 270 kinematic model
#
# Each tuple contains:
#
#   joint origin XYZ
#   joint origin RPY
#
# Joint motion itself is about the local Z axis.
# ============================================================

JOINT_GEOMETRY = [

    (
        (
            0.0,
            0.0,
            0.100,
        ),
        (
            0.0,
            0.0,
            0.0,
        ),
    ),

    (
        (
            0.0,
            0.0,
            0.038,
        ),
        (
            -math.pi / 2.0,
            0.0,
            0.0,
        ),
    ),

    (
        (
            0.0,
            -0.100,
            0.0,
        ),
        (
            0.0,
            0.0,
            0.0,
        ),
    ),

    (
        (
            0.108,
            -0.005,
            -0.001,
        ),
        (
            0.0,
            math.pi / 2.0,
            0.0,
        ),
    ),

    (
        (
            -0.001,
            0.0,
            0.0,
        ),
        (
            0.0,
            -math.pi / 2.0,
            0.0,
        ),
    ),

    (
        (
            0.060,
            0.0,
            0.0,
        ),
        (
            0.0,
            math.pi / 2.0,
            0.0,
        ),
    ),
]


class MechArmKinematics:

    def __init__(
        self,
        config,
    ):

        self.config = config

        task = (
            config.task
        )

        safety = (
            config.safety
        )

        ik = (
            task[
                'kinematics'
            ]
        )

        solver = (
            ik[
                'solver'
            ]
        )

        self.tool_offset = float(
            ik[
                'tool_offset_m'
            ]
        )

        self.maximum_iterations = int(
            solver[
                'maximum_iterations'
            ]
        )

        self.numerical_step = float(
            solver[
                'numerical_step_rad'
            ]
        )

        self.damping = float(
            solver[
                'damping'
            ]
        )

        self.maximum_iteration_step = float(
            solver[
                'maximum_iteration_step_rad'
            ]
        )

        cartesian = (
            safety[
                'cartesian'
            ]
        )

        self.position_tolerance = float(
            cartesian[
                'ik_position_tolerance_m'
            ]
        )

        self.orientation_tolerance = (
            math.radians(
                float(
                    cartesian[
                        'ik_orientation_tolerance_deg'
                    ]
                )
            )
        )

        # -----------------------------------------------------
        # Joint limits come only from safety.yaml.
        #
        # They are NOT duplicated in task_manager.py.
        # -----------------------------------------------------

        limit_config = (
            safety[
                'joint_limits_deg'
            ]
        )

        self.lower_limits = []

        self.upper_limits = []

        for index in range(
            1,
            7,
        ):

            joint = (
                limit_config[
                    f'joint{index}'
                ]
            )

            self.lower_limits.append(
                math.radians(
                    float(
                        joint[
                            'min'
                        ]
                    )
                )
            )

            self.upper_limits.append(
                math.radians(
                    float(
                        joint[
                            'max'
                        ]
                    )
                )
            )

    # ========================================================
    # Joint validation
    # ========================================================

    def validate_joints(
        self,
        joints,
    ):

        if len(joints) != 6:

            raise JointLimitError(
                'Expected exactly six joints.'
            )

        for index, (
            value,
            lower,
            upper,
        ) in enumerate(
            zip(
                joints,
                self.lower_limits,
                self.upper_limits,
            ),
            start=1,
        ):

            if not math.isfinite(
                value
            ):

                raise JointLimitError(
                    f'Joint {index} is not finite.'
                )

            if (
                value < lower
                or
                value > upper
            ):

                raise JointLimitError(
                    f'Joint {index} exceeds limit: '
                    f'{math.degrees(value):.2f} deg '
                    f'not in '
                    f'['
                    f'{math.degrees(lower):.2f}, '
                    f'{math.degrees(upper):.2f}'
                    f'] deg'
                )

    def clamp_joint(
        self,
        value,
        index,
    ):

        return max(
            self.lower_limits[
                index
            ],
            min(
                self.upper_limits[
                    index
                ],
                value,
            ),
        )

    # ========================================================
    # Forward kinematics
    # ========================================================

    def forward_transform(
        self,
        joints,
    ):

        self.validate_joints(
            joints
        )

        transform = identity4()

        for index in range(
            6
        ):

            xyz, rpy = (
                JOINT_GEOMETRY[
                    index
                ]
            )

            transform = mat_mul(
                transform,
                origin_matrix(
                    xyz,
                    rpy,
                ),
            )

            transform = mat_mul(
                transform,
                rot_z(
                    joints[
                        index
                    ]
                ),
            )

        transform = mat_mul(
            transform,
            translation(
                0.0,
                0.0,
                self.tool_offset,
            ),
        )

        return transform

    def forward_position(
        self,
        joints,
    ):

        transform = (
            self.forward_transform(
                joints
            )
        )

        return [
            transform[0][3],
            transform[1][3],
            transform[2][3],
        ]

    def forward_rotation(
        self,
        joints,
    ):

        transform = (
            self.forward_transform(
                joints
            )
        )

        return [
            [
                transform[
                    row
                ][
                    column
                ]
                for column in range(
                    3
                )
            ]
            for row in range(
                3
            )
        ]

    # ========================================================
    # Orientation error
    # ========================================================

    @staticmethod
    def orientation_error(
        current_rotation,
        target_rotation,
    ):

        error_matrix = (
            mat3_mul(
                target_rotation,
                transpose3(
                    current_rotation
                ),
            )
        )

        return [

            0.5
            *
            (
                error_matrix[2][1]
                -
                error_matrix[1][2]
            ),

            0.5
            *
            (
                error_matrix[0][2]
                -
                error_matrix[2][0]
            ),

            0.5
            *
            (
                error_matrix[1][0]
                -
                error_matrix[0][1]
            ),
        ]

    # ========================================================
    # Generic linear solver
    # ========================================================

    @staticmethod
    def solve_linear_system(
        matrix,
        vector,
    ):

        size = len(
            vector
        )

        augmented = [
            [
                float(
                    matrix[
                        row
                    ][
                        column
                    ]
                )
                for column in range(
                    size
                )
            ]
            +
            [
                float(
                    vector[
                        row
                    ]
                )
            ]
            for row in range(
                size
            )
        ]

        for column in range(
            size
        ):

            pivot = max(
                range(
                    column,
                    size,
                ),
                key=lambda row: abs(
                    augmented[
                        row
                    ][
                        column
                    ]
                ),
            )

            if abs(
                augmented[
                    pivot
                ][
                    column
                ]
            ) < 1e-10:

                return None

            (
                augmented[column],
                augmented[pivot],
            ) = (
                augmented[pivot],
                augmented[column],
            )

            divisor = (
                augmented[
                    column
                ][
                    column
                ]
            )

            for index in range(
                column,
                size + 1,
            ):

                augmented[
                    column
                ][
                    index
                ] /= divisor

            for row in range(
                size
            ):

                if row == column:
                    continue

                factor = (
                    augmented[
                        row
                    ][
                        column
                    ]
                )

                for index in range(
                    column,
                    size + 1,
                ):

                    augmented[
                        row
                    ][
                        index
                    ] -= (
                        factor
                        *
                        augmented[
                            column
                        ][
                            index
                        ]
                    )

        return [
            augmented[
                index
            ][
                size
            ]
            for index in range(
                size
            )
        ]

    # ========================================================
    # Numerical Jacobian
    #
    # position_only:
    #   3 x 6
    #
    # full pose:
    #   6 x 6
    # ========================================================

    def numerical_jacobian(
        self,
        joints,
        include_orientation,
    ):

        current_position = (
            self.forward_position(
                joints
            )
        )

        current_rotation = (
            self.forward_rotation(
                joints
            )
        )

        row_count = (
            6
            if include_orientation
            else
            3
        )

        jacobian = [
            [
                0.0
                for _ in range(
                    6
                )
            ]
            for _ in range(
                row_count
            )
        ]

        for joint_index in range(
            6
        ):

            test_joints = list(
                joints
            )

            test_joints[
                joint_index
            ] = self.clamp_joint(
                test_joints[
                    joint_index
                ]
                +
                self.numerical_step,
                joint_index,
            )

            actual_delta = (
                test_joints[
                    joint_index
                ]
                -
                joints[
                    joint_index
                ]
            )

            if abs(
                actual_delta
            ) < 1e-12:

                continue

            test_position = (
                self.forward_position(
                    test_joints
                )
            )

            for row in range(
                3
            ):

                jacobian[
                    row
                ][
                    joint_index
                ] = (
                    test_position[
                        row
                    ]
                    -
                    current_position[
                        row
                    ]
                ) / actual_delta

            if include_orientation:

                test_rotation = (
                    self.forward_rotation(
                        test_joints
                    )
                )

                rotation_delta = (
                    self.orientation_error(
                        current_rotation,
                        test_rotation,
                    )
                )

                for row in range(
                    3
                ):

                    jacobian[
                        row + 3
                    ][
                        joint_index
                    ] = (
                        rotation_delta[
                            row
                        ]
                        /
                        actual_delta
                    )

        return jacobian

    # ========================================================
    # Damped least-squares step
    # ========================================================

    def damped_least_squares(
        self,
        jacobian,
        error,
    ):

        rows = len(
            jacobian
        )

        matrix = [
            [
                0.0
                for _ in range(
                    rows
                )
            ]
            for _ in range(
                rows
            )
        ]

        for row in range(
            rows
        ):

            for column in range(
                rows
            ):

                matrix[
                    row
                ][
                    column
                ] = sum(
                    jacobian[
                        row
                    ][
                        joint
                    ]
                    *
                    jacobian[
                        column
                    ][
                        joint
                    ]
                    for joint in range(
                        6
                    )
                )

                if row == column:

                    matrix[
                        row
                    ][
                        column
                    ] += (
                        self.damping
                        *
                        self.damping
                    )

        intermediate = (
            self.solve_linear_system(
                matrix,
                error,
            )
        )

        if intermediate is None:

            raise NoIKSolutionError(
                'IK Jacobian became singular.'
            )

        joint_delta = [

            sum(
                jacobian[
                    row
                ][
                    joint
                ]
                *
                intermediate[
                    row
                ]
                for row in range(
                    rows
                )
            )

            for joint in range(
                6
            )
        ]

        maximum = max(
            abs(value)
            for value in joint_delta
        )

        if (
            maximum
            >
            self.maximum_iteration_step
        ):

            scale = (
                self.maximum_iteration_step
                /
                maximum
            )

            joint_delta = [
                value
                *
                scale
                for value in joint_delta
            ]

        return joint_delta

    # ========================================================
    # Position-only IK
    # ========================================================

    def solve_position(
        self,
        target_xyz,
        seed,
        position_tolerance=None,
    ):

        if position_tolerance is None:

            position_tolerance = (
                self.position_tolerance
            )

        if len(
            target_xyz
        ) != 3:

            raise UnreachableTargetError(
                'Cartesian target must contain X, Y, Z.'
            )

        if target_xyz[2] < 0.0:

            raise UnreachableTargetError(
                'Target Z is below the robot base plane.'
            )

        joints = list(
            seed
        )

        self.validate_joints(
            joints
        )

        for _ in range(
            self.maximum_iterations
        ):

            current = (
                self.forward_position(
                    joints
                )
            )

            error = [
                target_xyz[index]
                -
                current[index]
                for index in range(
                    3
                )
            ]

            error_norm = math.sqrt(
                sum(
                    value
                    *
                    value
                    for value in error
                )
            )

            if (
                error_norm
                <=
                position_tolerance
            ):

                self.validate_joints(
                    joints
                )

                return list(
                    joints
                )

            jacobian = (
                self.numerical_jacobian(
                    joints,
                    include_orientation=False,
                )
            )

            delta = (
                self.damped_least_squares(
                    jacobian,
                    error,
                )
            )

            for index in range(
                6
            ):

                joints[
                    index
                ] = self.clamp_joint(
                    joints[
                        index
                    ]
                    +
                    delta[
                        index
                    ],
                    index,
                )

        final_position = (
            self.forward_position(
                joints
            )
        )

        final_error = math.sqrt(
            sum(
                (
                    target_xyz[
                        index
                    ]
                    -
                    final_position[
                        index
                    ]
                )
                ** 2
                for index in range(
                    3
                )
            )
        )

        raise NoIKSolutionError(
            'Position IK failed: '
            f'target={target_xyz}, '
            f'best_error='
            f'{final_error * 1000.0:.1f} mm'
        )

    # ========================================================
    # Position + orientation IK
    # ========================================================

    def solve_pose(
        self,
        target_xyz,
        target_rotation,
        seed,
        position_tolerance=None,
        orientation_tolerance=None,
    ):

        if position_tolerance is None:

            position_tolerance = (
                self.position_tolerance
            )

        if orientation_tolerance is None:

            orientation_tolerance = (
                self.orientation_tolerance
            )

        joints = list(
            seed
        )

        self.validate_joints(
            joints
        )

        for _ in range(
            self.maximum_iterations
        ):

            current_position = (
                self.forward_position(
                    joints
                )
            )

            current_rotation = (
                self.forward_rotation(
                    joints
                )
            )

            position_error = [
                target_xyz[index]
                -
                current_position[
                    index
                ]
                for index in range(
                    3
                )
            ]

            orientation_error = (
                self.orientation_error(
                    current_rotation,
                    target_rotation,
                )
            )

            position_norm = math.sqrt(
                sum(
                    value
                    *
                    value
                    for value in (
                        position_error
                    )
                )
            )

            orientation_norm = (
                math.sqrt(
                    sum(
                        value
                        *
                        value
                        for value in (
                            orientation_error
                        )
                    )
                )
            )

            if (
                position_norm
                <=
                position_tolerance
                and
                orientation_norm
                <=
                orientation_tolerance
            ):

                self.validate_joints(
                    joints
                )

                return list(
                    joints
                )

            error = (
                position_error
                +
                orientation_error
            )

            jacobian = (
                self.numerical_jacobian(
                    joints,
                    include_orientation=True,
                )
            )

            delta = (
                self.damped_least_squares(
                    jacobian,
                    error,
                )
            )

            for index in range(
                6
            ):

                joints[
                    index
                ] = self.clamp_joint(
                    joints[
                        index
                    ]
                    +
                    delta[
                        index
                    ],
                    index,
                )

        final_position = (
            self.forward_position(
                joints
            )
        )

        final_rotation = (
            self.forward_rotation(
                joints
            )
        )

        final_position_error = (
            math.sqrt(
                sum(
                    (
                        target_xyz[
                            index
                        ]
                        -
                        final_position[
                            index
                        ]
                    )
                    ** 2
                    for index in range(
                        3
                    )
                )
            )
        )

        final_orientation_error = (
            self.orientation_error(
                final_rotation,
                target_rotation,
            )
        )

        final_orientation_norm = (
            math.sqrt(
                sum(
                    value
                    *
                    value
                    for value in (
                        final_orientation_error
                    )
                )
            )
        )

        raise NoIKSolutionError(
            'Full-pose IK failed: '
            f'target={target_xyz}, '
            f'position_error='
            f'{final_position_error * 1000.0:.1f} mm, '
            f'orientation_error='
            f'{math.degrees(final_orientation_norm):.2f} deg'
        )

    # ========================================================
    # Configuration helpers
    # ========================================================

    @staticmethod
    def degrees_to_radians(
        values,
    ):

        return [
            math.radians(
                float(value)
            )
            for value in values
        ]

    def pick_seed(self):

        return (
            self.degrees_to_radians(
                self.config.task[
                    'kinematics'
                ][
                    'pick_seed_deg'
                ]
            )
        )

    def place_seed(self):

        return (
            self.degrees_to_radians(
                self.config.task[
                    'kinematics'
                ][
                    'place_seed_deg'
                ]
            )
        )
