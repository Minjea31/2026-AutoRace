#!/usr/bin/env python3
"""Extract the actual CAD map paths, excluding dimensions, from the supplied PDF.

Build-time tool only: requires pdftocairo, numpy, OpenCV and Pillow. Runtime uses the
checked-in PNG/JSON. Coordinates come from the 10,000 x 6,000 mm drawing frame,
not from the photographed/pixel map. Cubic curves are sampled at <= 1 mm.
"""
import hashlib
import json
import math
from pathlib import Path
import re
import subprocess
import tempfile
import xml.etree.ElementTree as ET

import numpy as np
import cv2
from PIL import Image, ImageDraw, ImageFont

ROOT = Path(__file__).resolve().parents[2]
OUT = ROOT / 'simul/src/limo_parking_sim/assets'
RESAMPLE = getattr(Image,'Resampling',Image)


def extract():
    source = ROOT / 'assets/2026Track 도면.pdf'
    with tempfile.TemporaryDirectory() as tmp:
        svg = Path(tmp) / 'track.svg'
        subprocess.run(['pdftocairo', '-svg', str(source), str(svg)], check=True)
        root = ET.parse(svg).getroot()
    # The CAD exporter uses 24-unit strokes for track entities, 12 for
    # annotations. The first track path is the known 10 x 6 m bounding rectangle.
    paths = [e for e in root.iter() if e.tag.endswith('path')
             and 'stroke-width:24;' in e.get('style', '')]
    if len(paths) != 277:
        raise ValueError('PDF changed; inspect layer selection before extracting')
    number = r'[-+]?(?:\d*\.\d+|\d+)(?:[eE][-+]?\d+)?'

    def points(element):
        a, b, c, d, tx, ty = map(float, re.findall(number, element.get('transform')))
        tokens = re.findall(r'[MLCZmlcz]|' + number, element.get('d'))
        i, result, start = 0, [], None
        while i < len(tokens):
            op = tokens[i]
            i += 1
            if op in ('M', 'L'):
                p = np.array(list(map(float, tokens[i:i+2])))
                i += 2
                result.append(p)
                if op == 'M':
                    start = p
            elif op == 'C':
                p0 = result[-1]
                p1, p2, p3 = np.array(list(map(float, tokens[i:i+6]))).reshape(3, 2)
                i += 6
                n = max(8, math.ceil(sum(np.linalg.norm(v) for v in
                    (p1-p0, p2-p1, p3-p2)) / 1.5))
                for t in np.linspace(0, 1, n+1)[1:]:
                    result.append((1-t)**3*p0+3*(1-t)**2*t*p1+3*(1-t)*t*t*p2+t**3*p3)
            elif op in ('Z', 'z'):
                result.append(start)
            else:
                raise ValueError(f'Unsupported SVG operation: {op}')
        return np.array([[a*x+c*y+tx, b*x+d*y+ty] for x, y in result])

    boundary = points(paths[0])
    lo, hi = boundary.min(axis=0), boundary.max(axis=0)
    scale = (hi-lo) / [10, 6]
    if abs(scale[0]/scale[1]-1) > 0.0001:
        raise ValueError('CAD frame aspect ratio changed')
    polylines = []
    for element in paths[1:]:
        xy = (points(element) - lo) / scale
        xy[:, 1] = 6 - xy[:, 1]
        polylines.append(xy)
    OUT.mkdir(parents=True, exist_ok=True)
    # Work at 400 pixels/m for a compact texture (2.5 mm/texel).
    ppm = 400
    image = Image.new('RGB', (10*ppm, 6*ppm), (51, 50, 44))
    draw = ImageDraw.Draw(image)
    for xy in polylines:
        uv = [(x*ppm, (6-y)*ppm) for x, y in xy]
        area = abs(np.dot(xy[:, 0], np.roll(xy[:, 1], 1))
                   - np.dot(xy[:, 1], np.roll(xy[:, 0], 1))) / 2
        if np.linalg.norm(xy[0]-xy[-1]) < 0.002 and area < 0.2:
            draw.polygon(uv, fill=(239, 237, 218))
        draw.line(uv, fill=(239, 237, 218), width=1)
    # CAD tape is represented by two outlines. Fill only narrow enclosed
    # components, leaving road surfaces and the parking interiors untouched.
    pixels = np.array(image)
    ink = (pixels[:, :, 0] > 150).astype(np.uint8)
    # Close the gaps between paired 20/40 mm tape outlines, including strips
    # whose CAD entities have open ends at a junction. This does not add walls.
    ink = cv2.morphologyEx(ink, cv2.MORPH_CLOSE,
                          cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (17, 17)))
    dark = 1-ink
    count, labels = cv2.connectedComponents(dark)
    distance = cv2.distanceTransform(dark, cv2.DIST_L2, 3)
    radii = np.zeros(count)
    np.maximum.at(radii, labels.ravel(), distance.ravel())
    narrow = radii < 0.023*ppm
    narrow[np.unique(np.concatenate((labels[0], labels[-1], labels[:, 0], labels[:, -1])))] = False
    pixels[(ink > 0) | narrow[labels]] = (239, 237, 218)
    image = Image.fromarray(pixels).resize((8000,4800),RESAMPLE.NEAREST)
    # Morphological filling is appropriate for paired lane edges, but merges
    # closely spaced zebra bars and fills holes in Korean glyphs/diamonds.
    # Restore semantic markings separately at 800 pixels/m, preserving CAD
    # bounds and stripe coordinates. Runtime needs no Korean font installation.
    marking_ppm = 800
    draw = ImageDraw.Draw(image)
    road,paint = (51,50,44),(239,237,218)
    def uv(x,y):
        return (round(x*marking_ppm),round((6-y)*marking_ppm))
    def clear(bounds,pad=0):
        x0,x1,y0,y1=bounds
        draw.rectangle((uv(x0-pad,y1+pad),uv(x1+pad,y0-pad)),fill=road)
    def polygon(index,colour=paint):
        draw.polygon([uv(x,y) for x,y in polylines[index]],fill=colour)
    # Two staggered zebra crossings: copy actual CAD stripe polygons, not a
    # single rectangle or an aggressively closed binary mask.
    clear((3.20,3.78,4.590,5.090))
    for i in (216,217,232,233,234,235,236,249,250,251):
        polygon(i)
    clear((9.330,9.830,1.940,2.510))
    for i in range(199,209):
        polygon(i)
    # Restore the hollow diamonds.
    for inner,outer in ((255,256),(269,270),(271,272)):
        points=polylines[outer]
        clear((points[:,0].min(),points[:,0].max(),points[:,1].min(),points[:,1].max()),.004)
        polygon(outer)
        polygon(inner,road)
    fonts=[Path('/usr/share/fonts/truetype/nanum/NanumGothicBold.ttf'),
           Path('/usr/share/fonts/opentype/noto/NotoSansCJK-Bold.ttc')]
    font_path=next((p for p in fonts if p.exists()),None)
    if font_path is None:
        raise RuntimeError('Regenerating the texture requires a Korean Nanum/Noto font; runtime uses the saved PNG.')
    font=ImageFont.truetype(str(font_path),180)
    def label(text,bounds,angle):
        clear(bounds,.004)
        x0,x1,y0,y1=bounds
        bbox=font.getbbox(text)
        tile=Image.new('RGB',(bbox[2]-bbox[0]+8,bbox[3]-bbox[1]+8),road)
        ImageDraw.Draw(tile).text((4-bbox[0],4-bbox[1]),text,font=font,fill=paint,stroke_width=1)
        tile=tile.rotate(angle,expand=True,fillcolor=road)
        tile=tile.resize((round((x1-x0)*marking_ppm),round((y1-y0)*marking_ppm)),RESAMPLE.LANCZOS)
        image.paste(tile,uv(x0,y1))
    label('천천히',(3.19,3.34,5.355,5.807),-90)
    label('천천히',(2.95,3.10,4.615,5.070),-90)
    label('천천히',(9.358,9.811,3.201,3.349),180)
    label('A',(3.656,3.856,.326,.514),90)
    label('B',(3.656,3.856,1.385,1.535),90)
    # Restore a real checker start line, rather than the filled CAD outlines.
    clear((.950,1.070,5.330,5.830))
    for column in range(2):
        for row in range(10):
            if (column+row)%2==0:
                draw.rectangle((uv(.950+column*.06,5.830-row*.05),
                                uv(.950+(column+1)*.06,5.830-(row+1)*.05)),fill=paint)
    image.save(OUT / 'track.png')
    data = {'source': str(source.relative_to(ROOT)),
            'sha256': hashlib.sha256(source.read_bytes()).hexdigest(),
            'size_m': [10.0, 6.0], 'origin': 'bottom-left, x right, y up',
            'polylines_m': [np.round(xy, 6).tolist() for xy in polylines],
            'texture_size_px':[8000,4800],
            'markings':{'font':font_path.name,'zebra_stripes':16,'zebra_border_strips':4,'slow_text':'천천히',
                        'hollow_diamonds':3,'start_checker':True}}
    (OUT / 'track_geometry.json').write_text(json.dumps(data, separators=(',', ':'))+'\n')
    print(f'Extracted {len(polylines)} CAD paths -> {OUT}')


if __name__ == '__main__':
    extract()
