#!/usr/bin/env python3
"""Render measured YOLO pose outputs as reproducible SVG/PNG README panels.

Run from the repository root. Add --model /path/to/yolov8n-pose.pt to rerun
CPU inference; otherwise render the stored measurements without PyTorch.
Requires PyGObject/Rsvg and pycairo for PNG export.
"""
import argparse
import base64
import hashlib
import html
import json
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
ASSETS = ROOT / 'docs/assets/readme'
OUT = ASSETS / 'research'
NAMES = ['human-tracking-rosbag-start-gesture.png', 'human-tracking-rosbag-later-position.png']
SETTINGS = dict(device='cpu', imgsz=960, conf=0.35, iou=0.5)
EDGES = [(5,6),(5,7),(7,9),(6,8),(8,10),(5,11),(6,12),(11,12),(11,13),(13,15),(12,14),(14,16)]


def text(x, y, value, size=20, colour='#17364a', weight='normal'):
    return f'<text x="{x}" y="{y}" font-family="DejaVu Sans, sans-serif" font-size="{size}" font-weight="{weight}" fill="{colour}">{html.escape(value)}</text>'


def render(name, data, index):
    import cairo
    import gi
    gi.require_version('Rsvg', '2.0')
    from gi.repository import Rsvg
    boxes, poses = data['boxes'], data['keypoints']
    primary = max(range(len(boxes)), key=lambda k: (boxes[k][2]-boxes[k][0])*(boxes[k][3]-boxes[k][1]))
    kp = poses[primary]
    confident = all(kp[k][2] >= 0.35 for k in [5,6,9,10])
    raised = confident and kp[9][1] < kp[5][1] and kp[10][1] < kp[6][1]
    lowered = confident and kp[9][1] > kp[5][1] and kp[10][1] > kp[6][1]
    cue = 'Both wrists raised' if raised else 'Both wrists lowered' if lowered else 'Mixed / incomplete pose'
    image = base64.b64encode((ASSETS/name).read_bytes()).decode()
    title = 'Gesture input: raised hands' if index == 0 else 'Person detection: later frame'
    svg = ['<svg xmlns="http://www.w3.org/2000/svg" xmlns:xlink="http://www.w3.org/1999/xlink" width="1280" height="650" viewBox="0 0 1280 650">',
           '<rect width="1280" height="650" fill="white"/>',
           '<rect width="1280" height="64" fill="#17364a"/>',
           text(24,42,title,29,'white','bold'),
           f'<image x="0" y="64" width="960" height="540" xlink:href="data:image/png;base64,{image}"/>',
           '<rect x="960" y="64" width="320" height="540" fill="#f0f5f8"/>']
    for j, box in enumerate(boxes):
        x1,y1,x2,y2,confidence,cls = box
        colour = '#00e1bd' if j == primary else '#d4dce5'
        svg.append(f'<rect x="{x1}" y="{y1+64}" width="{x2-x1}" height="{y2-y1}" fill="none" stroke="{colour}" stroke-width="3"/>')
        label=f'PERSON {confidence:.2f}'
        # Put labels just below the box, keeping the face and wrists visible.
        label_y=min(y2+64,575)
        svg.append(f'<rect x="{x1}" y="{label_y}" width="154" height="27" fill="#17364a"/>')
        svg.append(text(x1+7,label_y+20,label,18,colour,'bold'))
    # Skeleton belongs to the largest detected person, not an asserted track ID.
    for a,b in EDGES:
        if kp[a][2] >= .35 and kp[b][2] >= .35:
            svg.append(f'<line x1="{kp[a][0]}" y1="{kp[a][1]+64}" x2="{kp[b][0]}" y2="{kp[b][1]+64}" stroke="#00e1bd" stroke-width="3"/>')
    for j in range(5,17):
        if kp[j][2] >= .35:
            colour = '#ffce56' if j in (9,10) else '#00e1bd'
            radius = 7 if j in (9,10) else 4
            svg.append(f'<circle cx="{kp[j][0]}" cy="{kp[j][1]+64}" r="{radius}" fill="{colour}" stroke="#17364a" stroke-width="1.5"/>')
    svg.extend([text(984,107,'MODEL OBSERVATIONS',18,weight='bold'),
                text(984,151,'Foreground person',21,weight='bold'),
                text(984,183,f'Detection score: {boxes[primary][4]:.2f}',19),
                text(984,213,f'People detected: {len(boxes)}',19),
                '<line x1="984" y1="238" x2="1256" y2="238" stroke="#b8c8d4"/>',
                text(984,276,'POSE CUE',18,weight='bold'),
                text(984,315,cue,20,weight='bold'),
                text(984,345,'Relative to the shoulders',17),
                text(984,426,'Teal: person box + pose',18),
                text(984,456,'Gold: wrist keypoints',18),
                text(984,520,'YOLOv8n-pose',21,weight='bold'),
                text(984,550,'Keypoint score >= 0.35',17),
                text(24,634,'Offline pose inference on the original recorded RGB frame',20),'</svg>'])
    stem = 'human-start-pose-v1' if index == 0 else 'human-later-pose-v1'
    svg_path=OUT/(stem+'.svg');svg_path.write_text('\n'.join(svg))
    handle=Rsvg.Handle.new_from_file(str(svg_path))
    surface=cairo.ImageSurface(cairo.FORMAT_ARGB32,1280,650)
    handle.render_cairo(cairo.Context(surface))
    surface.write_to_png(str(OUT/(stem+'.png')))


def main():
    parser=argparse.ArgumentParser();parser.add_argument('--model',type=Path);args=parser.parse_args()
    path=OUT/'human-pose-measurements.json'
    if args.model:
        import torch
        from ultralytics import YOLO
        torch.set_num_threads(4)
        model=YOLO(str(args.model));results={}
        for name in NAMES:
            r=model.predict(source=str(ASSETS/name),verbose=False,**SETTINGS)[0]
            results[name]={'boxes':r.boxes.data.cpu().tolist(),'keypoints':r.keypoints.data.cpu().tolist()}
        path.write_text(json.dumps(results,indent=2)+'\n')
    data=json.loads(path.read_text())
    for index,name in enumerate(NAMES):render(name,data[name],index)

if __name__ == '__main__':main()
