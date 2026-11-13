"""Deterministic, offline Gazebo world and genuine ArUco textures."""
import argparse
import math
from pathlib import Path
import random
import xml.etree.ElementTree as ET
import cv2
import numpy as np
import yaml


def generate(scenario, output, recording=False):
    cfg = yaml.safe_load(Path(scenario).read_text())
    out = Path(output).resolve()
    out.mkdir(parents=True, exist_ok=True)
    rng = random.Random(cfg.get('seed', 7))
    targets = cfg['targets']
    if cfg.get('randomize'):
        targets = [dict(id=i, x=x, y=y, yaw=a, size=rng.uniform(.16, .30))
                   for i, (x, y, a) in zip(rng.sample(range(1024), 3), rng.sample([
                       (4., .15, math.pi/2), (12., .15, math.pi/2),
                       (19.98, 4., math.pi), (19.98, 8., math.pi),
                       (5., 11.18, -math.pi/2), (14., 11.18, -math.pi/2)], 3))]
    parts = ['''<sdf version="1.9"><world name="osp_studio">
    <physics name="physics" type="ignored"><max_step_size>0.001</max_step_size><real_time_factor>1</real_time_factor></physics>
    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
    <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>
    <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
    <plugin filename="gz-sim-contact-system" name="gz::sim::systems::Contact"/>
    <plugin filename="gz-sim-imu-system" name="gz::sim::systems::Imu"/>

    <scene><ambient>0.8 0.8 0.8 1</ambient><background>0.65 0.7 0.75 1</background><shadows>false</shadows></scene>
    <light name="sun" type="directional"><pose>0 0 10 0 0 0</pose><diffuse>0.8 0.8 0.8 1</diffuse><direction>0 0 -1</direction></light>''']
    def box(name, x, y, z, sx, sy, sz, color, collision=True, yaw=0):
        geom = f'<geometry><box><size>{sx} {sy} {sz}</size></box></geometry>'
        parts.append(f'<model name="{name}"><static>true</static><pose>{x} {y} {z} 0 0 {yaw}</pose><link name="link">' +
                     (f'<collision name="collision">{geom}</collision>' if collision else '') +
                     f'<visual name="visual">{geom}<material><diffuse>{color} 1</diffuse><ambient>{color} 1</ambient></material></visual></link></model>')
    if cfg.get('arena'):
        arena=cfg['arena'];w,h=arena['width_m'],arena['height_m']
        box('floor',w/2,h/2,-.06,w,h,.12,'.24 .28 .31')
        for j,wall in enumerate(arena['walls']):
            box(f'arena_wall{j}',wall['x'],wall['y'],wall['height']/2,
                wall['sx'],wall['sy'],wall['height'],'.75 .77 .78',yaw=wall['yaw'])
    else:
        box('floor', 10.075, 5.675, -.06, 20.15, 11.35, .12, '.24 .28 .31')
        for name, x, y, sx, sy in [('south',10.075,0,20.15,.12),('north',10.075,11.35,20.15,.12),
                                  ('west',0,5.675,.12,11.35),('east',20.15,5.675,.12,11.35)]:
            box(name,x,y,.7,sx,sy,1.4,'.75 .77 .78')
        # Approximate wall projections from the supplied raster drawing.
        for j, x in enumerate([.5, 4., 8., 12., 16., 19.3]):
            box(f'north_pier{j}',x,11.15,.7,.5,.35,1.4,'.55 .58 .6')
        for j, x in enumerate([.5, 6.4, 15.9, 17.6]):
            box(f'south_pier{j}',x,.2,.7,.5,.4,1.4,'.55 .58 .6')
    for j,o in enumerate(cfg.get('obstacles',[])):
        if o['type']=='cylinder':
            geom=f'<geometry><cylinder><radius>{o["radius"]}</radius><length>{o["height"]}</length></cylinder></geometry>'
            parts.append(f'<model name="obstacle{j}"><static>true</static><pose>{o["x"]} {o["y"]} {o["height"]/2} 0 0 0</pose><link name="link"><collision name="collision">{geom}</collision><visual name="visual">{geom}<material><ambient>.25 .45 .65 1</ambient><diffuse>.25 .45 .65 1</diffuse></material></visual></link></model>')
        else:
            color='.55 .35 .18' if o.get('kind')=='rack' else '.55 .25 .22'
            box(f'obstacle{j}',o['x'],o['y'],o['height']/2,o['sx'],o['sy'],o['height'],color,yaw=o.get('yaw',0.))
    # Line markings are visual only. Rounded west turns and the east chicane.
    line_idx = 0
    def line(a,b,width=.015):
        nonlocal line_idx
        dx,dy=b[0]-a[0],b[1]-a[1]
        box(f'paint{line_idx}',(a[0]+b[0])/2,(a[1]+b[1])/2,.002,
            math.hypot(dx,dy),width,.002,'.9 .91 .85',False,math.atan2(dy,dx))
        line_idx+=1
    if not cfg.get('arena'):
        for inset in [0., .845]:
            x0,y0,x1,y1=1.1+inset,.95+inset,17.7-inset,10.4-inset
            r=1.5-inset
            pts=[]
            for cx,cy,start in [(x1-r,y0+r,-90),(x1-r,y1-r,0),(x0+r,y1-r,90),(x0+r,y0+r,180)]:
                pts.extend([(cx+r*math.cos(math.radians(a)),cy+r*math.sin(math.radians(a))) for a in np.linspace(start,start+90,20)])
            # Slight inward chicane on the right edge; no collision geometry.
            pts=[(x-(.65*math.sin((y-3)*math.pi/2)**2 if x>x1-.1 and 3<y<8 else 0),y) for x,y in pts]
            for a,b in zip(pts,pts[1:]+pts[:1]): line(a,b)
        for y in np.arange(2.5,9.5,.62):
            line((2.6,y),(15.6,y),.012)
            for x in np.arange(2.6,15.4,.25): line((x,y),(min(x+.6,15.6),y+.5),.008)
    dictionary=cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_ARUCO_ORIGINAL)
    for j,t in enumerate(targets):
        if not 0 <= int(t['id']) < 1024 or not .05 <= t['size'] <= .8:
            raise ValueError('Invalid Original ArUco id or size')
        img=cv2.aruco.drawMarker(dictionary,int(t['id']),512)
        img=cv2.copyMakeBorder(img,64,64,64,64,cv2.BORDER_CONSTANT,value=255)
        cv2.imwrite(str(out/f'marker{j}.png'),cv2.cvtColor(img,cv2.COLOR_GRAY2BGR))
        s=t['size']*1.25
        x,y,a=t['x'],t['y'],t['yaw']
        box(f'target_support{j}',x-.05*math.cos(a),y-.05*math.sin(a),.45,.08,s,.9,'.8 .8 .8',yaw=a)
        # Native geometry avoids texture streaming differences between GPU drivers.
        cell=t['size']/7
        pattern=cv2.aruco.drawMarker(dictionary,int(t['id']),7)
        visuals=[]
        for row in range(7):
            for col in range(7):
                if pattern[row,col]==0:
                    visuals.append(f'<visual name="cell{row}_{col}"><pose>.002 {(col-3)*cell} {(3-row)*cell} 0 0 0</pose><geometry><box><size>.002 {cell} {cell}</size></box></geometry><material><ambient>0 0 0 1</ambient><diffuse>0 0 0 1</diffuse></material></visual>')
        white=f'<visual name="white"><geometry><box><size>.002 {s} {s}</size></box></geometry><material><ambient>1 1 1 1</ambient><diffuse>1 1 1 1</diffuse></material></visual>'
        parts.append(f'<model name="target{j}"><static>true</static><pose>{x} {y} .45 0 0 {a}</pose><link name="link">'+white+''.join(visuals)+'</link></model>')
    parts.append('</world></sdf>')
    world=out/'osp_studio.sdf'
    root=ET.fromstring('\n'.join(parts))
    scene=root.find('world')
    if recording:
        # Optional observer only: no collisions, TF or inputs to the policy.
        # The image is rendered by Gazebo's Sensors system, not a 2D replay.
        scene.append(ET.fromstring('''<model name="mission_observer">
          <static>true</static><pose>10.075 -7 22 0 1.04719755 1.57079633</pose>
          <link name="camera_link"><sensor name="overview" type="camera">
            <always_on>true</always_on><update_rate>5</update_rate>
            <topic>/mission/overview/image_raw</topic>
            <camera><horizontal_fov>1.1</horizontal_fov>
              <image><width>960</width><height>600</height><format>R8G8B8</format></image>
              <clip><near>0.1</near><far>80</far></clip>
            </camera>
          </sensor></link></model>'''))
    if cfg.get('arena'):
        from osp_mission.space_geometry import write_floor_mesh
        mesh=write_floor_mesh(cfg['arena'],out)
        geometry=scene.find("model[@name='floor']/link/visual/geometry")
        geometry.clear()
        ET.SubElement(ET.SubElement(geometry,'mesh'),'uri').text=str(mesh)
        # Mesh vertices are world coordinates. Cancel the enclosing floor
        # model's centre translation and -0.06 m vertical offset.
        floor_visual=scene.find("model[@name='floor']/link/visual")
        ET.SubElement(floor_visual,'pose').text=f'{-w/2} {-h/2} 0.06 0 0 0'

        observer=scene.find("model[@name='mission_observer']/pose")
        if observer is not None:
            w,h=cfg['arena']['width_m'],cfg['arena']['height_m'];span=max(w,h)
            observer.text=f'{w/2} {h/2-.58*span} {.95*span} 0 1.022 1.57079633'
    paint=ET.SubElement(scene,'model',name='floor_markings')
    ET.SubElement(paint,'static').text='true'
    link=ET.SubElement(paint,'link',name='link')
    for model in list(scene.findall('model')):
        if model.attrib['name'].startswith('paint'):
            visual=model.find('link/visual')
            visual.set('name',model.attrib['name'])
            ET.SubElement(visual,'pose').text=model.find('pose').text
            link.append(visual); scene.remove(model)
    world.write_text(ET.tostring(root,encoding='unicode'))
    (out/'ground_truth.yaml').write_text(yaml.safe_dump({'start':cfg['start'],'targets':targets,'obstacles':cfg.get('obstacles',[])}))
    return world,cfg['start']


def main():
    p=argparse.ArgumentParser()
    p.add_argument('--scenario',required=True); p.add_argument('--output',required=True)
    a=p.parse_args(); print(generate(a.scenario,a.output)[0])
