"""Live vehicle and cone placement with an independent Tk map window.

The controller has no GUI dependencies; edits update the same physical obstacles
used by RGB/depth, lidar and collision. A sidecar YAML preserves the source map.
"""
import copy
import math
import os
from pathlib import Path
import tempfile
from dataclasses import asdict

import numpy as np
import yaml

from .config import share_directory
from .model import Box, cone_overlaps_box, make_obstacles, overlaps
from .world import Cone, cones as make_cones


def read_scene_layout(path, cfg):
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
    if 'cones' in data:
        if not isinstance(data['cones'], list):
            raise ValueError('Cones must be a list')
        for item in data['cones']:
            if not isinstance(item, dict) or not isinstance(item.get('name'), str) or not item['name']:
                raise ValueError('Every cone requires a name')
            if item['name'] in names:
                raise ValueError('Scene object names must be unique')
            names.add(item['name'])
            try:
                cone = Cone(**item)
            except (TypeError, ValueError) as exc:
                raise ValueError(f'Invalid cone: {item["name"]}') from exc
            half = max(cone.base_size/2, cone.radius)
            w, h = cfg['world']['size']
            if not half <= cone.x <= w-half or not half <= cone.y <= h-half:
                raise ValueError(f'Cone outside map: {cone.name}')
    return copy.deepcopy({key: data[key] for key in ('obstacles', 'cones') if key in data})


def read_layout(path, cfg):
    """Compatibility accessor for vehicle-only callers and old sidecars."""
    return read_scene_layout(path, cfg)['obstacles']


REGIONS = {'cone': '콘', 'parking': '주차', 'obstacle': '장애물'}


def region_bounds(cfg, region):
    world = cfg['world']
    if region == 'obstacle':
        layout = world['avoidance_layout']
        rows = sorted(layout['row_y'].values())
        half = min(np.diff(rows))/2
        bounds = layout['x_min']-.15, layout['x_max']+.15, rows[0]-half-.06, rows[-1]+half+.06
    elif region == 'parking':
        parking = world['parking']
        x0, x1, y0, y1 = parking['goal']
        step_x = parking['bay_width']+parking['line_width']
        step_y = parking['bay_depth']+parking['line_width']
        extra = (parking['columns']-1)*step_x/2
        bounds = x0-extra-.35, x1+extra+.35, y0-(parking['rows']-1)*step_y-.30, y1+.55
    elif region == 'cone':
        course = world.get('cone_course')
        if course:
            half = abs(course['amplitude'])+course['half_width']+.22
            bounds = (course['center_x']-half, course['center_x']+half,
                      course['start_line_y']-.30, course['end_line_y']+.30)
        else:
            bounds = 0., 1.6, 1.5, 4.3
    else:
        raise ValueError(f'Unknown editor region: {region}')
    w, h = world['size']
    x0, x1, y0, y1 = bounds
    return max(0., x0), min(w, x1), max(0., y0), min(h, y1)


class Placement:
    def __init__(self, cfg, model, changed=lambda: None, path=None):
        self.cfg, self.model, self.changed = cfg, model, changed
        self.path = Path(path or cfg['_obstacle_layout_path'])
        self.selected = None
        self.kind = 'vehicle'
        self.history = []
        self.message = '왼쪽: 설치 · 차량 선택 후 빈 곳: 이동 · 오른쪽: 삭제'
        self.saved = True

    @property
    def obstacles(self):
        return self.cfg['world']['obstacles']

    @property
    def cones(self):
        # Generated course positions are NumPy scalars; sidecars use plain YAML.
        return [{key: value if key == 'name' else float(value)
                 for key, value in asdict(cone).items()} for cone in self.model.cones]

    def set_kind(self, kind):
        if kind not in ('vehicle', 'cone'):
            raise ValueError(f'Unknown placement type: {kind}')
        self.kind, self.selected = kind, None
        self.message = ('콘' if kind == 'cone' else '차량')+'을 설치할 위치를 클릭하세요.'

    def parking_cell(self, x, y):
        parking = self.cfg['world']['parking']
        x0, x1, y0, y1 = parking['goal']
        dx = parking['bay_width']+parking['line_width']
        dy = parking['bay_depth']+parking['line_width']
        cells = [(row+1, col+1, (x0+x1)/2+(col-(parking['columns']-1)/2)*dx,
                  (y0+y1)/2-row*dy)
                 for row in range(parking['rows']) for col in range(parking['columns'])]
        return next((cell for cell in cells if abs(x-cell[2]) <= dx/2 and abs(y-cell[3]) <= dy/2), None)

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
        if self.kind == 'cone':
            return next((item for item in reversed(self.cones)
                         if math.hypot(x-item['x'], y-item['y']) <= max(item['base_size']/2, item['radius'])+.025), None)
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
        cell = None if self.kind == 'cone' else self.grid_cell(x, y) or self.parking_cell(x, y)
        target = self.hit(x, y)
        if target is None and cell:
            target = self.hit(cell[2], cell[3])
        if button == 3:
            if target is None:
                self.message = '이 위치에는 선택한 종류의 물체가 없습니다.'
                return False
            if self.kind == 'cone':
                self._commit(self.obstacles, f'{target["name"]} 삭제됨',
                             cone_items=[o for o in self.cones if o['name'] != target['name']])
            else:
                self._commit([o for o in self.obstacles if o['name'] != target['name']],
                             f'{target["name"]} 삭제됨')
            self.selected = None
            return True
        if target is not None:
            self.selected = target['name']
            self.message = f'{self.selected} 선택됨 · 이동할 빈 위치를 클릭하세요. Esc: 선택 해제'
            return False
        if self.kind == 'cone':
            return self.place_cone(x, y)
        if cell:
            x, y = cell[2], cell[3]
            yaw = math.pi if isinstance(cell[0], str) else math.pi/2
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
            names = {o['name'] for o in items+self.cones}
            while f'obstacle_{index:03d}' in names:
                index += 1
            moving = {'name': f'obstacle_{index:03d}'}
            items.append(moving)
        moving['pose'] = [float(x), float(y), float(yaw)]
        for key in ('grid_row', 'grid_column', 'parking_row', 'parking_column'):
            moving.pop(key, None)
        if cell:
            prefix = 'grid' if isinstance(cell[0], str) else 'parking'
            moving.update({prefix+'_row': cell[0], prefix+'_column': cell[1]})
        action = '이동' if self.selected else '설치'
        self._commit(items, f'{moving["name"]} {action} · ({x:.2f}, {y:.2f}) m')
        self.selected = None
        return True

    def place_cone(self, x, y):
        items = self.cones
        moving = next((o for o in items if o['name'] == self.selected), None)
        if moving is None:
            index, names = 1, {o['name'] for o in self.obstacles+items}
            while f'cone_custom_{index:03d}' in names:
                index += 1
            spec = self.cfg['world'].get('cone_course', {}).get('cone', {})
            moving = asdict(Cone(f'cone_custom_{index:03d}', x, y, **spec))
            items.append(moving)
        moving.update(x=float(x), y=float(y))
        cone = Cone(**moving)
        half = max(cone.base_size/2, cone.radius)
        w, h = self.cfg['world']['size']
        if not half <= x <= w-half or not half <= y <= h-half:
            self.message = '배치 불가: 콘 전체가 맵 안에 있어야 합니다.'
            return False
        contact = [box.name for box in self.model.obstacles if cone_overlaps_box(cone, box)]
        contact += [other.name for other in self.model.cones if other.name != cone.name
                    and math.hypot(x-other.x, y-other.y) <= cone.radius_at(0)+other.radius_at(0)]
        if cone_overlaps_box(cone, self.model.body()):
            contact.append('주행 차량')
        if contact:
            self.message = '배치 불가: '+', '.join(contact)
            return False
        action = '이동' if self.selected else '설치'
        self._commit(self.obstacles, f'{cone.name} {action} · ({x:.2f}, {y:.2f}) m', cone_items=items)
        self.selected = None
        return True

    def _commit(self, items, message, remember=True, cone_items=None):
        if remember:
            self.history.append(copy.deepcopy({'obstacles': self.obstacles, 'cones': self.cones}))
            self.history = self.history[-100:]
        world = self.cfg['world']
        world['obstacles'] = copy.deepcopy(items)
        world['cones'] = copy.deepcopy(self.cones if cone_items is None else cone_items)
        if world.get('cone_course'):
            world['cone_course']['enabled'] = False
        self.model.obstacles = make_obstacles(self.cfg)
        self.model.cones = make_cones(world)
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
                yaml.safe_dump({'version': 1, 'obstacles': self.obstacles, 'cones': self.cones}, stream,
                               allow_unicode=True, sort_keys=False)
            os.replace(temporary, self.path)
            self.saved = True
        except (OSError, yaml.YAMLError) as exc:
            self.saved = False
            self.message += f' · 저장 실패: {exc}'
        finally:
            if temporary and os.path.exists(temporary):
                os.unlink(temporary)

    def undo(self):
        if self.history:
            self.selected = None
            previous = self.history.pop()
            self._commit(previous['obstacles'], '이전 배치로 되돌렸습니다.',
                         remember=False, cone_items=previous['cones'])


class ObstacleEditor:
    def __init__(self, cfg, model, changed=lambda: None, region_changed=lambda region: None):
        import tkinter as tk
        from tkinter import ttk
        from PIL import Image, ImageDraw, ImageFont, ImageTk
        self.tk, self.Image, self.ImageTk = tk, Image, ImageTk
        self.ImageDraw, self.ImageFont = ImageDraw, ImageFont
        self.cfg, self.model = cfg, model
        self.region, self.region_changed = 'obstacle', region_changed
        self.closed = False
        try:
            self.root = tk.Tk()
        except tk.TclError as exc:
            raise RuntimeError(str(exc)) from exc
        self.root.title('AutoRace | 콘 · 주차 · 장애물 배치')
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
        self.label(toolbar, '콘 · 주차 · 장애물 배치', 23).pack(side='left', padx=(0, 18))
        for text, command in (('새로 설치 (Esc)', self.deselect), ('되돌리기', self.undo)):
            button = ttk.Button(toolbar, command=command)
            button.image = self.text_image(text, 15, '#dcdad5', '#141c27')
            button.configure(image=button.image)
            button.pack(side='right', padx=3)
        controls = ttk.Frame(self.root, padding=(12, 0, 12, 10))
        controls.pack(fill='x')
        self.label(controls, '화면').pack(side='left', padx=(0, 8))
        self.region_buttons = {}
        for region, title in REGIONS.items():
            button = ttk.Button(controls, command=lambda key=region: self.select_region(key))
            button.pack(side='left', padx=3)
            self.region_buttons[region] = (button, title)
        self.label(controls, '설치 대상').pack(side='left', padx=(22, 8))
        self.kind_buttons = {}
        for kind, title in (('cone', '콘'), ('vehicle', '차량')):
            button = ttk.Button(controls, command=lambda key=kind: self.select_kind(key))
            button.pack(side='left', padx=3)
            self.kind_buttons[kind] = (button, title)
        self.label(self.root, '왼쪽: 설치 / 선택 후 이동   |   오른쪽: 삭제   |   콘은 자유 배치 · 차량은 칸 중앙 맞춤').pack(
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
        self.update_controls()
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
        return region_bounds(self.cfg, self.region)

    def update_controls(self):
        for buttons, selected in ((self.region_buttons, self.region),
                                  (self.kind_buttons, self.placement.kind)):
            for key, (button, title) in buttons.items():
                active = key == selected
                button.image = self.text_image(title, 16, '#2563d9' if active else '#dcdad5',
                                               '#ffffff' if active else '#141c27')
                button.configure(image=button.image)

    def select_region(self, region):
        if region not in REGIONS:
            raise ValueError(f'Unknown editor region: {region}')
        self.region = region
        # Preserve a selected object when changing views to move it elsewhere.
        if self.placement.selected is None:
            self.placement.set_kind('cone' if region == 'cone' else 'vehicle')
        self.update_controls()
        self.region_changed(region)
        self.redraw()

    def select_kind(self, kind):
        self.placement.set_kind(kind)
        self.update_controls()
        self.redraw()

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
        self.placement.set_kind(self.placement.kind)
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
        if layout and self.region == 'obstacle':
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
        # Both object types remain visible regardless of the selected tool.
        for cone in self.model.cones:
            if not x0 <= cone.x <= x1 or not y0 <= cone.y <= y1:
                continue
            px, py = self.screen(cone.x, cone.y)
            radius = max(3, max(cone.radius, cone.base_size/2)*scale)
            selected = cone.name == self.placement.selected
            self.canvas.create_rectangle(px-radius, py-radius, px+radius, py+radius,
                                         fill='#ff8c32', outline='#ffcf5e' if selected else '#653817',
                                         width=3 if selected else 1)
            self.canvas.create_oval(px-radius*.45, py-radius*.45, px+radius*.45, py+radius*.45,
                                    fill='#fff5df', outline='')
        for item in self.placement.obstacles:
            if x0 <= item['pose'][0] <= x1 and y0 <= item['pose'][1] <= y1:
                self.draw_vehicle(item['pose'], '#e3edf5', item['name'] == self.placement.selected)
        self._robot()
        self.label_text(self.status_label, self.placement.message)
        state = '자동 저장됨' if self.placement.saved else '저장 필요 (Ctrl+S)'
        count = sum(x0 <= o['pose'][0] <= x1 and y0 <= o['pose'][1] <= y1 for o in self.placement.obstacles)
        cone_count = sum(x0 <= c.x <= x1 and y0 <= c.y <= y1 for c in self.model.cones)
        self.label_text(self.details_label,
                        f'{REGIONS[self.region]} 화면 · 차량 {count}대 / 콘 {cone_count}개 · {state} · {self.placement.path.name}', 13)

    def _robot(self):
        self.canvas.delete('robot')
        # Tag only the ego marker so it can refresh without resampling the map.
        before = set(self.canvas.find_all())
        x0, x1, y0, y1 = self.bounds()
        if x0 <= self.model.x <= x1 and y0 <= self.model.y <= y1:
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
