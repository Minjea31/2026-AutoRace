"""Live vehicle placement with an independent Tk map window.

The controller has no GUI dependencies; edits update the same physical obstacles
used by RGB/depth, lidar and collision. A sidecar YAML preserves the source map.
"""
import copy
import math
import os
from pathlib import Path
import tempfile

import numpy as np
import yaml

from .config import share_directory
from .model import Box, make_obstacles, overlaps


def read_layout(path, cfg):
    with open(path, encoding='utf-8') as stream:
        data = yaml.safe_load(stream)
    if not isinstance(data, dict) or data.get('version') != 1 or not isinstance(data.get('obstacles'), list):
        raise ValueError(f'Invalid obstacle layout: {path}')
    names = set()
    for item in data['obstacles']:
        if not isinstance(item, dict) or not isinstance(item.get('name'), str) or not item['name']:
            raise ValueError('Every obstacle requires a name')
        if item['name'] in names:
            raise ValueError('Obstacle names must be unique')
        names.add(item['name'])
        pose = item.get('pose')
        if (not isinstance(pose, list) or len(pose) != 3
                or not all(isinstance(v, (int, float)) and math.isfinite(v) for v in pose)):
            raise ValueError('Obstacle poses must contain finite x, y, yaw values')
        v = cfg['vehicle']
        box = Box(item['name'], *pose, v['length'], v['width'], 0, v['height'])
        if np.any(box.corners() < 0) or np.any(box.corners() > cfg['world']['size']):
            raise ValueError(f'Obstacle outside map: {item["name"]}')
    return copy.deepcopy(data['obstacles'])


class Placement:
    def __init__(self, cfg, model, changed=lambda: None, path=None):
        self.cfg, self.model, self.changed = cfg, model, changed
        self.path = Path(path or cfg['_obstacle_layout_path'])
        self.selected = None
        self.history = []
        self.message = '왼쪽: 설치 · 차량 선택 후 빈 곳: 이동 · 오른쪽: 삭제'
        self.saved = True

    @property
    def obstacles(self):
        return self.cfg['world']['obstacles']

    def grid_cell(self, x, y):
        layout = self.cfg['world'].get('avoidance_layout')
        if not layout or not layout['x_min'] <= x <= layout['x_max']:
            return None
        rows = layout['row_y']
        spacing = min(abs(a-b) for a in rows.values() for b in rows.values() if a != b)
        row = min(rows, key=lambda r: abs(rows[r]-y))
        if abs(rows[row]-y) > spacing/2:
            return None
        width = (layout['x_max']-layout['x_min'])/layout['columns']
        column = min(layout['columns'], int((x-layout['x_min'])/width)+1)
        return row, column, layout['x_min']+(column-.5)*width, rows[row]

    def hit(self, x, y):
        v = self.cfg['vehicle']
        for item in reversed(self.obstacles):
            px, py, yaw = item['pose']
            dx, dy = x-px, y-py
            c, s = math.cos(yaw), math.sin(yaw)
            if abs(c*dx+s*dy) <= v['length']/2+.025 and abs(-s*dx+c*dy) <= v['width']/2+.025:
                return item
        return None

    def click(self, x, y, button=1):
        yaw = math.pi
        if not all(math.isfinite(v) for v in (x, y, yaw)):
            return False
        w, h = self.cfg['world']['size']
        if not 0 <= x <= w or not 0 <= y <= h:
            return False
        cell = self.grid_cell(x, y)
        if cell is None:
            self.message = '3개 차로의 1~8번 칸 안에서만 편집할 수 있습니다.'
            return False
        target = self.hit(x, y)
        if target is None and cell:
            target = self.hit(cell[2], cell[3])
        if button == 3:
            if target is None:
                self.message = '이 위치에는 삭제할 차량이 없습니다.'
                return False
            items = [o for o in self.obstacles if o['name'] != target['name']]
            self._commit(items, f'{target["name"]} 삭제됨')
            self.selected = None
            return True
        if target is not None:
            self.selected = target['name']
            self.message = f'{self.selected} 선택됨 · 이동할 빈 위치를 클릭하세요. Esc: 선택 해제'
            return False
        x, y = cell[2], cell[3]
        v = self.cfg['vehicle']
        candidate = Box('candidate', x, y, yaw, v['length'], v['width'], v['ground_clearance'], v['height'])
        contact = [name for name in self.model.collisions_at(x, y, yaw) if name != self.selected]
        if overlaps(candidate, self.model.body()):
            contact.append('주행 차량')
        if contact:
            self.message = '배치 불가: '+', '.join(contact)
            return False
        items = copy.deepcopy(self.obstacles)
        moving = next((o for o in items if o['name'] == self.selected), None)
        if moving is None:
            index = 1
            names = {o['name'] for o in items}
            while f'obstacle_{index:03d}' in names:
                index += 1
            moving = {'name': f'obstacle_{index:03d}'}
            items.append(moving)
        moving['pose'] = [float(x), float(y), float(yaw)]
        for key in ('grid_row', 'grid_column'):
            moving.pop(key, None)
        if cell:
            moving.update(grid_row=cell[0], grid_column=cell[1])
        action = '이동' if self.selected else '설치'
        self._commit(items, f'{moving["name"]} {action} · ({x:.2f}, {y:.2f}) m')
        self.selected = None
        return True

    def _commit(self, items, message, remember=True):
        if remember:
            self.history.append(copy.deepcopy(self.obstacles))
            self.history = self.history[-100:]
        self.cfg['world']['obstacles'] = items
        self.model.obstacles = make_obstacles(self.cfg)
        self.changed()
        self.message = message
        self.save()

    def save(self):
        temporary = None
        try:
            self.path.parent.mkdir(parents=True, exist_ok=True)
            with tempfile.NamedTemporaryFile(mode='w', dir=self.path.parent, encoding='utf-8',
                                             prefix=self.path.name+'.', delete=False) as stream:
                temporary = stream.name
                yaml.safe_dump({'version': 1, 'obstacles': self.obstacles}, stream,
                               allow_unicode=True, sort_keys=False)
            os.replace(temporary, self.path)
            self.saved = True
        except OSError as exc:
            self.saved = False
            self.message += f' · 저장 실패: {exc}'
        finally:
            if temporary and os.path.exists(temporary):
                os.unlink(temporary)

    def undo(self):
        if self.history:
            self.selected = None
            self._commit(self.history.pop(), '이전 배치로 되돌렸습니다.', remember=False)


class ObstacleEditor:
    def __init__(self, cfg, model, changed=lambda: None):
        import tkinter as tk
        from tkinter import ttk
        from PIL import Image, ImageDraw, ImageFont, ImageTk
        self.tk, self.Image, self.ImageTk = tk, Image, ImageTk
        self.ImageDraw, self.ImageFont = ImageDraw, ImageFont
        self.cfg, self.model = cfg, model
        self.closed = False
        try:
            self.root = tk.Tk()
        except tk.TclError as exc:
            raise RuntimeError(str(exc)) from exc
        self.root.title('AutoRace | 장애물 편집')
        self.root.geometry('1120x650')
        self.root.minsize(820, 480)
        self.root.configure(bg='#141c27')
        self.root.protocol('WM_DELETE_WINDOW', self.close)
        self.placement = Placement(cfg, model, changed)
        style = ttk.Style(self.root)
        style.theme_use('clam')
        style.configure('TFrame', background='#141c27')
        style.configure('TLabel', background='#141c27', foreground='#dce5ef', font=('Noto Sans CJK KR', 11))
        style.configure('TButton', font=('Noto Sans CJK KR', 10), padding=7)
        toolbar = ttk.Frame(self.root, padding=10)
        toolbar.pack(fill='x')
        self.label(toolbar, '장애물 편집', 23).pack(side='left', padx=(0, 18))
        self.label(toolbar, '3개 차로 · 1~8번 칸 중앙에 배치').pack(side='left')
        for text, command in (('새 장애물 (Esc)', self.deselect), ('되돌리기', self.undo)):
            button = ttk.Button(toolbar, command=command)
            button.image = self.text_image(text, 15, '#dcdad5', '#141c27')
            button.configure(image=button.image)
            button.pack(side='right', padx=3)
        self.label(self.root, '왼쪽 클릭: 설치   |   차량 선택 → 빈 곳 클릭: 이동   |   오른쪽 클릭: 삭제').pack(
            anchor='w', padx=14, pady=(0,8))
        self.canvas = tk.Canvas(self.root, background='#202934', highlightthickness=0)
        self.canvas.pack(fill='both', expand=True, padx=12)
        footer = ttk.Frame(self.root, padding=12)
        footer.pack(fill='x')
        self.status_label = self.label(footer, '')
        self.status_label.pack(anchor='w')
        self.details_label = self.label(footer, '', 13)
        self.details_label.pack(anchor='w', pady=(5,0))
        self.texture = Image.open(share_directory()/'assets'/cfg['world']['texture']).convert('RGB')
        self.canvas.bind('<Configure>', lambda event: self.redraw())
        self.canvas.bind('<Button-1>', lambda event: self.on_click(event, 1))
        self.canvas.bind('<Button-3>', lambda event: self.on_click(event, 3))
        self.root.bind('<Escape>', lambda event: self.deselect())
        self.root.bind('<Control-z>', lambda event: self.undo())
        self.root.bind('<Control-s>', lambda event: self.save())
        self.transform = None
        self.last_robot = None
        self.redraw()

    def text_image(self, text, size=16, background='#141c27', color='#dce5ef'):
        # Some Conda Tk builds use X11 bitmap fonts without Xft, which makes
        # large/small Korean text unreadable. Pillow uses the installed TTF/OTF.
        try:
            font = self.ImageFont.truetype('NotoSansCJK-Regular.ttc', size)
        except OSError:
            try:
                font = self.ImageFont.truetype('/usr/share/fonts/truetype/nanum/NanumGothic.ttf', size)
            except OSError:
                font = self.ImageFont.truetype('DejaVuSans.ttf', size)
        bbox = font.getbbox(text or ' ')
        image = self.Image.new('RGB', (max(1,bbox[2]-bbox[0]+4), size+10), background)
        self.ImageDraw.Draw(image).text((2,2-bbox[1]), text, font=font, fill=color)
        return self.ImageTk.PhotoImage(image, master=self.root)

    def label(self, parent, text, size=16):
        from tkinter import ttk
        label = ttk.Label(parent)
        self.label_text(label, text, size)
        return label

    def label_text(self, label, text, size=16):
        label.image = self.text_image(text, size)
        label.configure(image=label.image)

    def bounds(self):
        layout = self.cfg['world']['avoidance_layout']
        rows = sorted(layout['row_y'].values())
        half = min(np.diff(rows))/2
        return layout['x_min']-.15, layout['x_max']+.15, rows[0]-half-.06, rows[-1]+half+.06

    def screen(self, x, y):
        scale, ox, oy, x0, y1 = self.transform
        return ox+(x-x0)*scale, oy+(y1-y)*scale

    def world(self, px, py):
        scale, ox, oy, x0, y1 = self.transform
        return x0+(px-ox)/scale, y1-(py-oy)/scale

    def on_click(self, event, button):
        if self.transform is None:
            return
        x, y = self.world(event.x, event.y)
        x0, x1, y0, y1 = self.bounds()
        if not x0 <= x <= x1 or not y0 <= y <= y1:
            return
        self.placement.click(x, y, button)
        self.redraw()

    def deselect(self):
        self.placement.selected = None
        self.placement.message = '새 장애물을 설치할 위치를 클릭하세요.'
        self.redraw()

    def undo(self):
        self.placement.undo()
        self.redraw()

    def save(self):
        self.placement.message = '배치를 저장했습니다.'
        self.placement.save()
        self.redraw()

    def draw_vehicle(self, pose, color, selected=False, label=''):
        x, y, yaw = pose
        v = self.cfg['vehicle']
        box = Box('display', x, y, yaw, v['length'], v['width'], 0, v['height'])
        points = [c for point in box.corners() for c in self.screen(*point)]
        self.canvas.create_polygon(points, fill=color, outline='#ffcf5e' if selected else '#111923',
                                   width=3 if selected else 1)
        c, s = math.cos(yaw), math.sin(yaw)
        a = self.screen(x+c*v['length']*.35, y+s*v['length']*.35)
        b = self.screen(x-c*v['length']*.10, y-s*v['length']*.10)
        self.canvas.create_line(*b, *a, fill='#172230', width=2, arrow='last')
        if label:
            px, py = self.screen(x, y+v['width']/2+.055)
            self.canvas.create_text(px, py, text=label, fill='#f3f7fb', font=('Noto Sans CJK KR', 9))

    def redraw(self):
        if self.closed:
            return
        self.canvas.delete('all')
        width, height = max(100, self.canvas.winfo_width()), max(100, self.canvas.winfo_height())
        x0, x1, y0, y1 = self.bounds()
        scale = min((width-28)/(x1-x0), (height-28)/(y1-y0))
        iw, ih = max(1, round((x1-x0)*scale)), max(1, round((y1-y0)*scale))
        ox, oy = (width-iw)/2, (height-ih)/2
        self.transform = scale, ox, oy, x0, y1
        w, h = self.cfg['world']['size']
        tw, th = self.texture.size
        crop = self.texture.crop((x0/w*tw, (h-y1)/h*th, x1/w*tw, (h-y0)/h*th))
        self.background = self.ImageTk.PhotoImage(crop.resize((iw, ih), self.Image.Resampling.LANCZOS), master=self.root)
        self.canvas.create_image(ox, oy, image=self.background, anchor='nw')
        layout = self.cfg['world'].get('avoidance_layout')
        if layout:
            cell_width = (layout['x_max']-layout['x_min'])/layout['columns']
            row_ys = sorted(layout['row_y'].values())
            half = min(np.diff(row_ys))/2
            for row, color in (('upper', '#ff6868'), ('middle', '#59a8ff'), ('lower', '#76df87')):
                y = layout['row_y'][row]
                for col in range(1, layout['columns']+1):
                    left = layout['x_min']+(col-1)*cell_width
                    self.canvas.create_rectangle(*self.screen(left, y+half-.025),
                                                 *self.screen(left+cell_width, y-half+.025),
                                                 outline=color, dash=(5, 4), width=2)
                    px, py = self.screen(left+cell_width/2, y+half-.085)
                    self.canvas.create_text(px, py, text=str(col), fill=color, font=('Noto Sans CJK KR', 13, 'bold'))
        # Fixed scene objects provide context; only vehicle obstacles are editable.
        for cone in self.model.cones:
            px, py = self.screen(cone.x, cone.y)
            radius = max(2, cone.radius*scale)
            self.canvas.create_oval(px-radius, py-radius, px+radius, py+radius, fill='#ff8c32', outline='')
        for item in self.placement.obstacles:
            self.draw_vehicle(item['pose'], '#e3edf5', item['name'] == self.placement.selected)
        self._robot()
        self.label_text(self.status_label, self.placement.message)
        state = '자동 저장됨' if self.placement.saved else '저장 필요 (Ctrl+S)'
        count = sum(self.placement.grid_cell(*o['pose'][:2]) is not None for o in self.placement.obstacles)
        self.label_text(self.details_label,
                        f'차로 안 차량 장애물 {count}대 · {state} · {self.placement.path.name}', 13)

    def _robot(self):
        self.canvas.delete('robot')
        # Tag only the ego marker so it can refresh without resampling the map.
        before = set(self.canvas.find_all())
        self.draw_vehicle((self.model.x, self.model.y, self.model.yaw), '#56e3cf', label='주행 차량')
        for item in set(self.canvas.find_all())-before:
            self.canvas.addtag_withtag('robot', item)
        self.last_robot = (self.model.x, self.model.y, self.model.yaw)

    def poll(self):
        if self.closed:
            return
        try:
            self.root.update_idletasks()
            self.root.update()
            if not self.closed and self.last_robot != (self.model.x, self.model.y, self.model.yaw):
                self._robot()
        except self.tk.TclError:
            self.close()

    def close(self):
        if not self.closed:
            self.closed = True
            self.root.destroy()
