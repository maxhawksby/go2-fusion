import ast
from pathlib import Path


FORBIDDEN = {'/cmd_vel', '/cmd_vel_out', '/webrtc_req'}


def test_scoreboard_never_creates_a_control_publisher():
    root = Path(__file__).parents[1] / 'go2_scoreboard'
    for path in root.glob('*.py'):
        tree = ast.parse(path.read_text(encoding='utf-8'))
        for node in ast.walk(tree):
            if not isinstance(node, ast.Call):
                continue
            function = node.func
            if not (isinstance(function, ast.Attribute)
                    and function.attr == 'create_publisher'):
                continue
            strings = {
                arg.value for arg in node.args
                if isinstance(arg, ast.Constant) and isinstance(arg.value, str)
            }
            assert not strings & FORBIDDEN, f'{path} publishes a control topic'
