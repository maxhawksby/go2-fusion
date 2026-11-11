"""Navigation frames belong to localization, not the rigid robot model."""

from pathlib import Path
import xml.etree.ElementTree as ET

import pytest


MODELS = Path(__file__).resolve().parents[1] / 'urdf'


@pytest.mark.parametrize('filename,prefix', [
    ('go2.urdf', ''), ('go2_with_realsense.urdf', ''),
    ('multi_go2.urdf', 'robot0/'), ('multi_go2.urdf', 'robot1/'),
])
def test_model_is_a_connected_body_tree_without_navigation_frames(filename, prefix):
    text = (MODELS / filename).read_text()
    if prefix:
        text = text.format(robot_num=prefix.rstrip('/'))
    root = ET.fromstring(text)
    links = {link.attrib['name'] for link in root.findall('link')}
    assert 'map' not in links and 'odom' not in links
    parents = {}
    for joint in root.findall('joint'):
        parent = joint.find('parent').attrib['link']
        child = joint.find('child').attrib['link']
        assert parent in links and child in links
        assert child not in parents, f'{child} has multiple URDF parents'
        parents[child] = parent
    body = prefix + 'base_link'
    assert links - parents.keys() == {body}
    for link in links:
        seen = set()
        while link in parents:
            assert link not in seen, 'cycle in robot model'
            seen.add(link)
            link = parents[link]
        assert link == body
    assert prefix + 'base_footprint' in links
    assert prefix + 'radar' in links
