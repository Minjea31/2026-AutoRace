#!/usr/bin/env python3
"""Crop source artwork to the printed symbol for the 80 mm sign insert.

Uses the existing reference images and Pillow only. Plate geometry supplies the
90 mm outer boundary; source-image page margins must not shrink the symbol.
"""
from pathlib import Path
import numpy as np
from PIL import Image

root=Path(__file__).resolve().parents[2]
target=root/'simul/src/limo_parking_sim/assets'
resample=getattr(Image,'Resampling',Image)
for name in ('parking','slow','fork-left-a','fork-right-b','speed-bump','pedestrian'):
    source=root/'assets/signs'/(name+'.png')
    im=Image.open(source).convert('RGB')
    pixels=np.array(im)
    y,x=np.nonzero(np.min(pixels,axis=2)<220)
    if not len(x):
        raise ValueError(f'No artwork in {source}')
    bounds=(max(0,int(x.min())-1),max(0,int(y.min())-1),
            min(im.width,int(x.max())+2),min(im.height,int(y.max())+2))
    im.crop(bounds).resize((512,512),resample.LANCZOS).save(target/(name+'-print.png'))
    print(name,bounds)
