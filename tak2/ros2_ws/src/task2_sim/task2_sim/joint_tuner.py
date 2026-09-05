import json
import math
import threading
from http.server import (
    BaseHTTPRequestHandler,
    ThreadingHTTPServer,
)
from pathlib import Path

import rclpy
from rclpy.node import Node

from std_msgs.msg import Float64


# ============================================================
# Small matrix helpers
# ============================================================

def mat_mul(a, b):

    rows = len(a)
    cols = len(b[0])
    inner = len(b)

    result = [
        [0.0 for _ in range(cols)]
        for _ in range(rows)
    ]

    for i in range(rows):
        for j in range(cols):
            for k in range(inner):

                result[i][j] += (
                    a[i][k]
                    *
                    b[k][j]
                )

    return result


def identity4():

    return [
        [1.0, 0.0, 0.0, 0.0],
        [0.0, 1.0, 0.0, 0.0],
        [0.0, 0.0, 1.0, 0.0],
        [0.0, 0.0, 0.0, 1.0],
    ]


def translation(x, y, z):

    t = identity4()

    t[0][3] = x
    t[1][3] = y
    t[2][3] = z

    return t


def rot_x(a):

    c = math.cos(a)
    s = math.sin(a)

    return [
        [1, 0, 0, 0],
        [0, c, -s, 0],
        [0, s, c, 0],
        [0, 0, 0, 1],
    ]


def rot_y(a):

    c = math.cos(a)
    s = math.sin(a)

    return [
        [c, 0, s, 0],
        [0, 1, 0, 0],
        [-s, 0, c, 0],
        [0, 0, 0, 1],
    ]


def rot_z(a):

    c = math.cos(a)
    s = math.sin(a)

    return [
        [c, -s, 0, 0],
        [s, c, 0, 0],
        [0, 0, 1, 0],
        [0, 0, 0, 1],
    ]


def rpy_matrix(r, p, y):

    return mat_mul(
        mat_mul(
            rot_z(y),
            rot_y(p),
        ),
        rot_x(r),
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


# ============================================================
# MechArm kinematics
# ============================================================

JOINTS = [

    (
        (0.0, 0.0, 0.100),
        (0.0, 0.0, 0.0),
    ),

    (
        (0.0, 0.0, 0.038),
        (-math.pi / 2, 0.0, 0.0),
    ),

    (
        (0.0, -0.100, 0.0),
        (0.0, 0.0, 0.0),
    ),

    (
        (0.108, -0.005, -0.001),
        (0.0, math.pi / 2, 0.0),
    ),

    (
        (-0.001, 0.0, 0.0),
        (0.0, -math.pi / 2, 0.0),
    ),

    (
        (0.060, 0.0, 0.0),
        (0.0, math.pi / 2, 0.0),
    ),
]


LOWER = [
    -2.792527,
    -1.3089,
    -3.0543,
    -2.7052,
    -2.0071,
    -3.14,
]


UPPER = [
    2.792527,
    2.0943,
    1.1344,
    2.7052,
    2.0071,
    3.14,
]


HOME = [
    math.radians(0),
    math.radians(-20),
    math.radians(-70),
    math.radians(0),
    math.radians(90),
    math.radians(0),
]


def clamp_joint(
    value,
    index,
):

    return max(
        LOWER[index],
        min(
            UPPER[index],
            value,
        ),
    )


def forward_kinematics(q):

    t = identity4()

    for index in range(6):

        xyz, rpy = JOINTS[index]

        t = mat_mul(
            t,
            origin_matrix(
                xyz,
                rpy,
            ),
        )

        # All official MechArm joints use local Z axis.
        t = mat_mul(
            t,
            rot_z(
                q[index]
            ),
        )

    # Approximate tool centre point.
    # Offset is along local Z so J6 rotation does not change
    # Cartesian position.
    t = mat_mul(
        t,
        translation(
            0.0,
            0.0,
            0.045,
        ),
    )

    return [
        t[0][3],
        t[1][3],
        t[2][3],
    ]


# ============================================================
# 3x3 solver for damped least-squares IK
# ============================================================

def solve_3x3(a, b):

    m = [
        [
            float(a[i][j])
            for j in range(3)
        ]
        +
        [float(b[i])]
        for i in range(3)
    ]

    for col in range(3):

        pivot = max(
            range(
                col,
                3,
            ),
            key=lambda r: abs(
                m[r][col]
            ),
        )

        if abs(
            m[pivot][col]
        ) < 1e-12:

            return None

        m[col], m[pivot] = (
            m[pivot],
            m[col],
        )

        divisor = m[col][col]

        for j in range(
            col,
            4,
        ):

            m[col][j] /= divisor

        for row in range(3):

            if row == col:
                continue

            factor = m[row][col]

            for j in range(
                col,
                4,
            ):

                m[row][j] -= (
                    factor
                    *
                    m[col][j]
                )

    return [
        m[0][3],
        m[1][3],
        m[2][3],
    ]


def solve_ik(
    target,
    seed,
):

    q = list(seed)

    damping = 0.05
    epsilon = 0.002

    for _ in range(250):

        current = (
            forward_kinematics(
                q
            )
        )

        error = [
            target[i]
            -
            current[i]
            for i in range(3)
        ]

        error_norm = math.sqrt(
            sum(
                value * value
                for value in error
            )
        )

        if error_norm < 0.004:

            return (
                True,
                q,
                error_norm,
            )

        # Numerical 3x6 Jacobian.
        j = [
            [0.0 for _ in range(6)]
            for _ in range(3)
        ]

        for joint in range(6):

            test_q = list(q)

            test_q[joint] += (
                epsilon
            )

            test_q[joint] = (
                clamp_joint(
                    test_q[joint],
                    joint,
                )
            )

            test_xyz = (
                forward_kinematics(
                    test_q
                )
            )

            delta = (
                test_q[joint]
                -
                q[joint]
            )

            if abs(delta) < 1e-9:
                continue

            for row in range(3):

                j[row][joint] = (
                    (
                        test_xyz[row]
                        -
                        current[row]
                    )
                    /
                    delta
                )

        # A = J J^T + lambda^2 I
        a = [
            [0.0 for _ in range(3)]
            for _ in range(3)
        ]

        for r in range(3):

            for c in range(3):

                a[r][c] = sum(
                    j[r][k]
                    *
                    j[c][k]
                    for k in range(6)
                )

                if r == c:

                    a[r][c] += (
                        damping
                        *
                        damping
                    )

        y = solve_3x3(
            a,
            error,
        )

        if y is None:

            break

        dq = [
            sum(
                j[row][joint]
                *
                y[row]
                for row in range(3)
            )
            for joint in range(6)
        ]

        # Limit one IK iteration to a small physical change.
        max_step = max(
            abs(value)
            for value in dq
        )

        if max_step > 0.10:

            scale = (
                0.10
                /
                max_step
            )

            dq = [
                value * scale
                for value in dq
            ]

        for joint in range(6):

            q[joint] = (
                clamp_joint(
                    q[joint]
                    +
                    dq[joint],
                    joint,
                )
            )

    final_xyz = (
        forward_kinematics(
            q
        )
    )

    final_error = math.sqrt(
        sum(
            (
                target[i]
                -
                final_xyz[i]
            ) ** 2
            for i in range(3)
        )
    )

    return (
        False,
        q,
        final_error,
    )


# ============================================================
# ROS controller
# ============================================================

class JointTuner(Node):

    def __init__(self):

        super().__init__(
            'task2_joint_tuner'
        )

        self.lock = (
            threading.Lock()
        )

        self.publishers_cmd = []

        for index in range(1, 7):

            self.publishers_cmd.append(
                self.create_publisher(
                    Float64,
                    (
                        f'/task2/'
                        f'joint{index}/'
                        f'cmd_pos'
                    ),
                    10,
                )
            )

        self.current_pose = [
            0.0
            for _ in range(6)
        ]

        self.start_pose = list(
            self.current_pose
        )

        self.target_pose = list(
            self.current_pose
        )

        self.motion_start = (
            self.now_seconds()
        )

        self.motion_duration = 0.0

        self.timer = self.create_timer(
            0.05,
            self.update_motion,
        )

        self.preset_file = (
            Path.home()
            /
            'task2_tuned_waypoints.json'
        )

        self.presets = {}

        if self.preset_file.exists():

            try:

                self.presets = json.loads(
                    self.preset_file
                    .read_text()
                )

            except Exception:

                self.presets = {}

        self.get_logger().info(
            'Task 2 joint tuner started.'
        )

    def now_seconds(self):

        return (
            self.get_clock()
            .now()
            .nanoseconds
            /
            1e9
        )

    def command_pose(
        self,
        pose,
        duration=3.0,
    ):

        pose = [
            clamp_joint(
                pose[i],
                i,
            )
            for i in range(6)
        ]

        with self.lock:

            self.start_pose = list(
                self.current_pose
            )

            self.target_pose = list(
                pose
            )

            self.motion_start = (
                self.now_seconds()
            )

            self.motion_duration = max(
                0.2,
                float(duration),
            )

    def update_motion(self):

        with self.lock:

            elapsed = (
                self.now_seconds()
                -
                self.motion_start
            )

            if (
                self.motion_duration
                <= 0.0
            ):

                pose = list(
                    self.target_pose
                )

            else:

                progress = min(
                    1.0,
                    max(
                        0.0,
                        (
                            elapsed
                            /
                            self.motion_duration
                        ),
                    ),
                )

                # Cubic smoothstep.
                smooth = (
                    3.0
                    * progress
                    * progress
                    -
                    2.0
                    * progress
                    * progress
                    * progress
                )

                pose = [
                    (
                        self.start_pose[i]
                        +
                        (
                            self.target_pose[i]
                            -
                            self.start_pose[i]
                        )
                        *
                        smooth
                    )
                    for i in range(6)
                ]

            self.current_pose = list(
                pose
            )

        for publisher, value in zip(
            self.publishers_cmd,
            pose,
        ):

            msg = Float64()

            msg.data = float(
                value
            )

            publisher.publish(
                msg
            )

    def state(self):

        with self.lock:

            q = list(
                self.current_pose
            )

            target = list(
                self.target_pose
            )

        xyz = (
            forward_kinematics(
                q
            )
        )

        return {
            'joints_deg': [
                math.degrees(v)
                for v in q
            ],

            'target_deg': [
                math.degrees(v)
                for v in target
            ],

            'xyz': xyz,

            'lower_deg': [
                math.degrees(v)
                for v in LOWER
            ],

            'upper_deg': [
                math.degrees(v)
                for v in UPPER
            ],

            'presets':
                self.presets,
        }

    def save_preset(
        self,
        name,
    ):

        with self.lock:

            pose = list(
                self.target_pose
            )

        self.presets[name] = [
            math.degrees(v)
            for v in pose
        ]

        self.preset_file.write_text(
            json.dumps(
                self.presets,
                indent=2,
            )
        )


# ============================================================
# Web interface
# ============================================================

HTML = r'''
<!DOCTYPE html>
<html>

<head>

<meta charset="utf-8">

<title>Task 2 MechArm Tuner</title>

<style>

body {
    font-family: sans-serif;
    max-width: 1000px;
    margin: 24px auto;
    background: #f5f5f5;
}

.card {
    background: white;
    padding: 18px;
    margin-bottom: 16px;
    border-radius: 10px;
}

.row {
    display: grid;
    grid-template-columns:
        60px 1fr 90px;
    gap: 12px;
    align-items: center;
    margin: 10px 0;
}

input[type=range] {
    width: 100%;
}

.xyz {
    font-size: 22px;
    font-weight: bold;
}

button {
    padding: 10px 16px;
    margin: 5px;
}

.coord input {
    width: 90px;
    padding: 7px;
}

#status {
    white-space: pre-wrap;
    font-family: monospace;
}

</style>

</head>


<body>

<h1>Task 2 · MechArm Control Tuner</h1>


<div class="card">

<h2>Joint control</h2>

<div id="joints"></div>

<button onclick="sendJoints()">
Send joint pose
</button>

<button onclick="home()">
HOME
</button>

</div>


<div class="card">

<h2>Estimated tool position</h2>

<div class="xyz" id="xyz">
X=0 Y=0 Z=0
</div>

<p>
Coordinates are relative to the robot base frame.
</p>

</div>


<div class="card coord">

<h2>Cartesian target A</h2>

X
<input id="ax" value="0.18">

Y
<input id="ay" value="0.10">

Z
<input id="az" value="0.05">

<button onclick="moveXYZ('A')">
Move A by IK
</button>

<button onclick="savePreset('PICK_A')">
Save current as PICK_A
</button>

</div>


<div class="card coord">

<h2>Cartesian target B</h2>

X
<input id="bx" value="0.18">

Y
<input id="by" value="-0.10">

Z
<input id="bz" value="0.05">

<button onclick="moveXYZ('B')">
Move B by IK
</button>

<button onclick="savePreset('PLACE_B')">
Save current as PLACE_B
</button>

</div>


<div class="card">

<h2>Status</h2>

<div id="status">
Loading...
</div>

</div>


<script>

let state = null;


async function api(
    path,
    data={}
) {

    const response = await fetch(
        path,
        {
            method: 'POST',
            headers: {
                'Content-Type':
                    'application/json'
            },
            body: JSON.stringify(data)
        }
    );

    return await response.json();
}


function buildJointUI(s) {

    const box =
        document.getElementById(
            'joints'
        );

    box.innerHTML = '';

    for (
        let i = 0;
        i < 6;
        i++
    ) {

        const row =
            document.createElement(
                'div'
            );

        row.className = 'row';

        const label =
            document.createElement(
                'b'
            );

        label.innerText =
            'J' + (i + 1);

        const slider =
            document.createElement(
                'input'
            );

        slider.type = 'range';

        slider.min =
            s.lower_deg[i];

        slider.max =
            s.upper_deg[i];

        slider.step = '1';

        slider.value =
            s.target_deg[i];

        slider.id =
            'j' + (i + 1);

        const number =
            document.createElement(
                'input'
            );

        number.type = 'number';

        number.step = '1';

        number.value =
            Number(
                s.target_deg[i]
            ).toFixed(1);

        number.id =
            'n' + (i + 1);

        slider.oninput = () => {
            number.value =
                slider.value;
        };

        number.oninput = () => {
            slider.value =
                number.value;
        };

        row.appendChild(label);
        row.appendChild(slider);
        row.appendChild(number);

        box.appendChild(row);
    }
}


async function initialLoad() {

    state = await (
        await fetch('/state')
    ).json();

    buildJointUI(state);

    refresh();
}


async function refresh() {

    try {

        state = await (
            await fetch('/state')
        ).json();

        document.getElementById(
            'xyz'
        ).innerText =
            'X='
            +
            state.xyz[0].toFixed(3)
            +
            ' m   Y='
            +
            state.xyz[1].toFixed(3)
            +
            ' m   Z='
            +
            state.xyz[2].toFixed(3)
            +
            ' m';

    } catch(e) {}

    setTimeout(
        refresh,
        250
    );
}


async function sendJoints() {

    const joints = [];

    for (
        let i = 1;
        i <= 6;
        i++
    ) {

        joints.push(
            Number(
                document.getElementById(
                    'n' + i
                ).value
            )
        );
    }

    const result = await api(
        '/set',
        {
            joints_deg: joints,
            duration: 3.0
        }
    );

    document.getElementById(
        'status'
    ).innerText =
        JSON.stringify(
            result,
            null,
            2
        );
}


async function home() {

    const result =
        await api('/home');

    document.getElementById(
        'status'
    ).innerText =
        JSON.stringify(
            result,
            null,
            2
        );
}


async function moveXYZ(which) {

    const p =
        which.toLowerCase();

    const x =
        Number(
            document.getElementById(
                p + 'x'
            ).value
        );

    const y =
        Number(
            document.getElementById(
                p + 'y'
            ).value
        );

    const z =
        Number(
            document.getElementById(
                p + 'z'
            ).value
        );

    const result = await api(
        '/ik',
        {
            x: x,
            y: y,
            z: z,
            duration: 4.0
        }
    );

    document.getElementById(
        'status'
    ).innerText =
        JSON.stringify(
            result,
            null,
            2
        );

    if (
        result.joints_deg
    ) {

        for (
            let i = 0;
            i < 6;
            i++
        ) {

            document.getElementById(
                'j' + (i + 1)
            ).value =
                result.joints_deg[i];

            document.getElementById(
                'n' + (i + 1)
            ).value =
                Number(
                    result.joints_deg[i]
                ).toFixed(1);
        }
    }
}


async function savePreset(name) {

    const result = await api(
        '/save',
        {
            name: name
        }
    );

    document.getElementById(
        'status'
    ).innerText =
        JSON.stringify(
            result,
            null,
            2
        );
}


initialLoad();

</script>

</body>

</html>
'''


class Handler(
    BaseHTTPRequestHandler
):

    tuner = None

    def send_json(
        self,
        data,
        status=200,
    ):

        raw = json.dumps(
            data
        ).encode(
            'utf-8'
        )

        self.send_response(
            status
        )

        self.send_header(
            'Content-Type',
            'application/json',
        )

        self.send_header(
            'Content-Length',
            str(len(raw)),
        )

        self.end_headers()

        self.wfile.write(
            raw
        )

    def do_GET(self):

        if self.path == '/':

            raw = HTML.encode(
                'utf-8'
            )

            self.send_response(200)

            self.send_header(
                'Content-Type',
                'text/html; charset=utf-8',
            )

            self.send_header(
                'Content-Length',
                str(len(raw)),
            )

            self.end_headers()

            self.wfile.write(
                raw
            )

            return

        if self.path == '/state':

            self.send_json(
                self.tuner.state()
            )

            return

        self.send_error(404)

    def read_json(self):

        length = int(
            self.headers.get(
                'Content-Length',
                '0',
            )
        )

        if length <= 0:
            return {}

        return json.loads(
            self.rfile.read(
                length
            ).decode(
                'utf-8'
            )
        )

    def do_POST(self):

        try:

            data = (
                self.read_json()
            )

            if self.path == '/set':

                degrees = (
                    data[
                        'joints_deg'
                    ]
                )

                if len(degrees) != 6:

                    raise ValueError(
                        'Need six joint angles.'
                    )

                pose = [
                    math.radians(
                        float(v)
                    )
                    for v in degrees
                ]

                self.tuner.command_pose(
                    pose,
                    data.get(
                        'duration',
                        3.0,
                    ),
                )

                self.send_json({
                    'ok': True,
                    'joints_deg':
                        degrees,
                })

                return

            if self.path == '/home':

                self.tuner.command_pose(
                    HOME,
                    4.0,
                )

                self.send_json({
                    'ok': True,
                    'state': 'HOME',
                })

                return

            if self.path == '/ik':

                target = [
                    float(data['x']),
                    float(data['y']),
                    float(data['z']),
                ]

                with (
                    self.tuner.lock
                ):

                    seed = list(
                        self.tuner.target_pose
                    )

                (
                    success,
                    q,
                    error,
                ) = solve_ik(
                    target,
                    seed,
                )

                if success:

                    self.tuner.command_pose(
                        q,
                        data.get(
                            'duration',
                            4.0,
                        ),
                    )

                self.send_json({
                    'ok': success,

                    'target_xyz':
                        target,

                    'error_m':
                        error,

                    'joints_deg': [
                        math.degrees(v)
                        for v in q
                    ],

                    'estimated_xyz':
                        forward_kinematics(q),
                })

                return

            if self.path == '/save':

                name = str(
                    data['name']
                )

                self.tuner.save_preset(
                    name
                )

                self.send_json({
                    'ok': True,

                    'saved':
                        name,

                    'file':
                        str(
                            self.tuner
                            .preset_file
                        ),

                    'presets':
                        self.tuner
                        .presets,
                })

                return

            self.send_error(404)

        except Exception as exc:

            self.send_json(
                {
                    'ok': False,
                    'error':
                        str(exc),
                },
                400,
            )

    def log_message(
        self,
        format,
        *args,
    ):

        return


def main(args=None):

    rclpy.init(
        args=args
    )

    tuner = JointTuner()

    Handler.tuner = tuner

    server = ThreadingHTTPServer(
        (
            '127.0.0.1',
            8080,
        ),
        Handler,
    )

    thread = threading.Thread(
        target=server.serve_forever,
        daemon=True,
    )

    thread.start()

    tuner.get_logger().info(
        'Open http://127.0.0.1:8080 '
        'on the Jetson.'
    )

    try:

        rclpy.spin(
            tuner
        )

    except KeyboardInterrupt:

        pass

    finally:

        server.shutdown()

        tuner.destroy_node()

        rclpy.shutdown()


if __name__ == '__main__':

    main()


# ============================================================
# Full end-effector pose helpers
# ============================================================

def forward_transform(q):

    t = identity4()

    for index in range(6):

        xyz, rpy = JOINTS[index]

        t = mat_mul(
            t,
            origin_matrix(
                xyz,
                rpy,
            ),
        )

        t = mat_mul(
            t,
            rot_z(
                q[index]
            ),
        )

    t = mat_mul(
        t,
        translation(
            0.0,
            0.0,
            0.045,
        ),
    )

    return t


def rotation_from_transform(t):

    return [
        [
            t[r][c]
            for c in range(3)
        ]
        for r in range(3)
    ]


def transpose3(a):

    return [
        [
            a[c][r]
            for c in range(3)
        ]
        for r in range(3)
    ]


def mat3_mul(a, b):

    return [
        [
            sum(
                a[r][k] * b[k][c]
                for k in range(3)
            )
            for c in range(3)
        ]
        for r in range(3)
    ]


def orientation_error(
    current_r,
    target_r,
):

    # Rotation error matrix:
    #
    # R_error = R_target * R_current^T

    r_err = mat3_mul(
        target_r,
        transpose3(
            current_r
        ),
    )

    # Small-angle orientation error vector.
    #
    # For the small corrections used during numerical IK this
    # gives a stable rotation-vector approximation.

    return [
        0.5
        *
        (
            r_err[2][1]
            -
            r_err[1][2]
        ),

        0.5
        *
        (
            r_err[0][2]
            -
            r_err[2][0]
        ),

        0.5
        *
        (
            r_err[1][0]
            -
            r_err[0][1]
        ),
    ]


def solve_linear_system(
    matrix,
    vector,
):

    n = len(vector)

    a = [
        [
            float(matrix[r][c])
            for c in range(n)
        ]
        +
        [
            float(vector[r])
        ]
        for r in range(n)
    ]

    for col in range(n):

        pivot = max(
            range(
                col,
                n,
            ),
            key=lambda r: abs(
                a[r][col]
            ),
        )

        if abs(
            a[pivot][col]
        ) < 1e-10:

            return None

        a[col], a[pivot] = (
            a[pivot],
            a[col],
        )

        divisor = a[col][col]

        for c in range(
            col,
            n + 1,
        ):

            a[col][c] /= divisor

        for r in range(n):

            if r == col:
                continue

            factor = a[r][col]

            for c in range(
                col,
                n + 1,
            ):

                a[r][c] -= (
                    factor
                    *
                    a[col][c]
                )

    return [
        a[i][n]
        for i in range(n)
    ]


def solve_pose_ik(
    target_xyz,
    target_rotation,
    seed,
):

    q = list(seed)

    epsilon = 0.001

    position_weight = 1.0
    orientation_weight = 0.35

    damping = 0.03

    for _ in range(400):

        current_t = (
            forward_transform(
                q
            )
        )

        current_xyz = [
            current_t[0][3],
            current_t[1][3],
            current_t[2][3],
        ]

        current_rotation = (
            rotation_from_transform(
                current_t
            )
        )

        pos_error = [
            target_xyz[i]
            -
            current_xyz[i]
            for i in range(3)
        ]

        rot_error = (
            orientation_error(
                current_rotation,
                target_rotation,
            )
        )

        position_norm = math.sqrt(
            sum(
                e * e
                for e in pos_error
            )
        )

        orientation_norm = math.sqrt(
            sum(
                e * e
                for e in rot_error
            )
        )

        if (
            position_norm < 0.003
            and
            orientation_norm < 0.025
        ):

            return (
                True,
                q,
                position_norm,
                orientation_norm,
            )

        error = [

            pos_error[0]
            * position_weight,

            pos_error[1]
            * position_weight,

            pos_error[2]
            * position_weight,

            rot_error[0]
            * orientation_weight,

            rot_error[1]
            * orientation_weight,

            rot_error[2]
            * orientation_weight,
        ]

        # 6 x 6 numerical Jacobian
        j = [
            [
                0.0
                for _ in range(6)
            ]
            for _ in range(6)
        ]

        for joint in range(6):

            q_test = list(q)

            q_test[joint] = (
                clamp_joint(
                    q_test[joint]
                    +
                    epsilon,
                    joint,
                )
            )

            actual_delta = (
                q_test[joint]
                -
                q[joint]
            )

            if abs(
                actual_delta
            ) < 1e-10:

                continue

            test_t = (
                forward_transform(
                    q_test
                )
            )

            test_xyz = [
                test_t[0][3],
                test_t[1][3],
                test_t[2][3],
            ]

            test_rotation = (
                rotation_from_transform(
                    test_t
                )
            )

            delta_orientation = (
                orientation_error(
                    current_rotation,
                    test_rotation,
                )
            )

            for row in range(3):

                j[row][joint] = (
                    (
                        test_xyz[row]
                        -
                        current_xyz[row]
                    )
                    /
                    actual_delta
                    *
                    position_weight
                )

                j[row + 3][joint] = (
                    delta_orientation[row]
                    /
                    actual_delta
                    *
                    orientation_weight
                )

        # Damped least squares:
        #
        # dq = J^T (J J^T + λ²I)^-1 e

        a = [
            [
                0.0
                for _ in range(6)
            ]
            for _ in range(6)
        ]

        for r in range(6):

            for c in range(6):

                a[r][c] = sum(
                    j[r][k]
                    *
                    j[c][k]
                    for k in range(6)
                )

                if r == c:

                    a[r][c] += (
                        damping
                        *
                        damping
                    )

        y = solve_linear_system(
            a,
            error,
        )

        if y is None:

            break

        dq = [
            sum(
                j[row][joint]
                *
                y[row]
                for row in range(6)
            )
            for joint in range(6)
        ]

        max_step = max(
            abs(v)
            for v in dq
        )

        if max_step > 0.08:

            scale = (
                0.08
                /
                max_step
            )

            dq = [
                v * scale
                for v in dq
            ]

        for joint in range(6):

            q[joint] = (
                clamp_joint(
                    q[joint]
                    +
                    dq[joint],
                    joint,
                )
            )

    final_t = (
        forward_transform(
            q
        )
    )

    final_xyz = [
        final_t[0][3],
        final_t[1][3],
        final_t[2][3],
    ]

    final_rotation = (
        rotation_from_transform(
            final_t
        )
    )

    final_pos_error = math.sqrt(
        sum(
            (
                target_xyz[i]
                -
                final_xyz[i]
            )
            ** 2
            for i in range(3)
        )
    )

    final_rot_vec = (
        orientation_error(
            final_rotation,
            target_rotation,
        )
    )

    final_rot_error = math.sqrt(
        sum(
            v * v
            for v in final_rot_vec
        )
    )

    return (
        False,
        q,
        final_pos_error,
        final_rot_error,
    )
