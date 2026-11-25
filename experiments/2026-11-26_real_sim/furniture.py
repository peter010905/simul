"""Count explicit fixed-table collision elements, not SLAM blobs or chair legs."""
import argparse
import hashlib
import json
from pathlib import Path
import re
import xml.etree.ElementTree as ET


def count(world):
    root = ET.parse(world).getroot()
    names = [c.get('name', '') for m in root.findall('.//world/model')
             if m.get('name') == 'demo_furniture' for c in m.findall('.//collision')]
    return dict(world_sha256=hashlib.sha256(world.read_bytes()).hexdigest(),
        table_tops=[n for n in names if re.fullmatch(r'table_(north|south|middle)_\d+_top', n)],
        table_leg_collisions=[n for n in names if 'leg' in n],
        semantic_physical_table_leg_count=None,
        note='Explicit simulator geometry count. Physical SLAM components do not identify table legs.')


if __name__ == '__main__':
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('world', type=Path)
    p.add_argument('output', type=Path)
    a = p.parse_args()
    a.output.write_text(json.dumps(count(a.world), indent=2)+'\n')
