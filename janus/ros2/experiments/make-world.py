#!/usr/bin/env python3
"""Prepare reproducible geometric QR variants of the public JANUS world."""
import json
import math
from pathlib import Path
import sys
import xml.etree.ElementTree as ET


def main():
    case_id, source, output, variant = sys.argv[1:]
    output = Path(output)
    world = ET.parse(source)
    includes = {include.findtext('uri'): include.find('pose')
                for include in world.findall('./world/include')}
    robot = [0., 0., .077, 0., 0., 0.]
    marker = [1.50, 0., .31, 0., 0., 0.]
    target_u = 320.0
    if case_id in ('Q02', 'Q03'):
        marker[1] = .28 if case_id == 'Q02' else -.28
    elif case_id == 'Q04':
        robot[1] = .13
        robot[5] = math.radians(-12)
    elif case_id == 'Q05':
        robot[5] = math.radians(55)
    elif case_id == 'Q19':
        robot[5] = math.radians(-10)
        marker[1] = .035
        marker[5] = math.radians(10)
        # At the dock pose, the camera looks down +X and the marker is .035 m left.
        fx = 320 / math.tan(math.pi/6)
        target_u = 320 - fx * marker[1] / .265
    if variant == 'near_start':
        robot[0] = {'Q01':.76,'Q02':.45,'Q03':.45,'Q04':.60,'Q05':.65,'Q06':.67,'Q07':.67,
                    'Q13':.67,'Q17':.55,'Q19':.65}.get(case_id,0.)
        # Leave the 1 ms physics step and simulated frame rate unchanged. Bound
        # wall demand on the VM so ROS camera delivery can catch up.
        world.find('./world/physics/real_time_factor').text = '0.25'
    includes['model://arche_janus'].text = ' '.join(map(str,robot))
    includes['model://janus_dock'].text = ' '.join(map(str,marker))
    world.write(output/'world.sdf', encoding='utf-8', xml_declaration=True)
    data = {'case_id':case_id, 'variant':variant, 'spawn_xyz_rpy':robot, 'marker_xyz_rpy':marker, 'target_u_px':target_u,
            'dock_nominal_axle_xy_yaw':[marker[0]-.25-.265, 0., 0.],
            'target_real_time_factor':float(world.findtext('./world/physics/real_time_factor')),
            'scope':'nominal synthetic mount; physical dock transform requires measurement'}
    (output/'fixture.json').write_text(json.dumps(data, indent=2)+'\n')
    print(target_u)


if __name__ == '__main__':
    main()
