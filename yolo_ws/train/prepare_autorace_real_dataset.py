#!/usr/bin/env python3
"""Prepare the Roboflow YOLO export with a deterministic 70/20/10 split."""

import argparse
import collections
import csv
import hashlib
import io
import json
import math
from pathlib import Path, PurePosixPath
import random
import zipfile

from PIL import Image
import yaml


CLASS_NAMES = ['slow', 'bump', 'crosswalk', 'left', 'right', 'parking', 'red', 'green']


def prepare(source, destination, seed):
    if destination.exists():
        raise FileExistsError(f'Output already exists: {destination}')
    records = []
    with zipfile.ZipFile(source) as archive:
        # Some exports contain repeated identical data.yaml entries.
        configs = [yaml.safe_load(archive.read(info)) for info in archive.infolist()
                   if PurePosixPath(info.filename).name == 'data.yaml']
        if not configs or any(config != configs[0] for config in configs):
            raise ValueError('Missing or inconsistent class maps in the export')
        config = configs[0]
        original_names = config['names']
        if isinstance(original_names, dict):
            original_names = [original_names[i] for i in range(len(original_names))]
        if len(set(original_names)) != len(original_names):
            raise ValueError('Duplicate source class names')
        if not set(original_names).issubset(CLASS_NAMES):
            raise ValueError(f'Unexpected source classes: {original_names}')
        remap = {i: CLASS_NAMES.index(name) for i, name in enumerate(original_names)}
        images = sorted((info for info in archive.infolist()
                         if '/images/' in info.filename
                         and PurePosixPath(info.filename).suffix.lower()
                         in {'.jpg', '.jpeg', '.png', '.webp'}),
                        key=lambda info: info.filename)
        if len({PurePosixPath(info.filename).stem for info in images}) != len(images):
            raise ValueError('Image filenames collide across the input splits')
        digests = set()
        for info in images:
            image_data = archive.read(info)
            digest = hashlib.sha256(image_data).hexdigest()
            if digest in digests:
                raise ValueError('Duplicate image content found; prevent cross-split duplicates before splitting')
            digests.add(digest)
            label_name = str(PurePosixPath(info.filename.replace('/images/', '/labels/')).with_suffix('.txt'))
            original_label = archive.read(label_name).decode('utf-8-sig')
            rewritten = []
            counts = collections.Counter()
            for line in original_label.splitlines():
                if not line.strip():
                    continue
                fields = line.split()
                if len(fields) != 5:
                    raise ValueError(f'Expected YOLO detection labels: {label_name}')
                class_id = int(fields[0])
                if class_id not in remap:
                    raise ValueError(f'Unknown class ID in {label_name}')
                x, y, width, height = map(float, fields[1:])
                if not all(math.isfinite(v) for v in (x, y, width, height)):
                    raise ValueError(f'Non-finite box in {label_name}')
                if not (0 <= x <= 1 and 0 <= y <= 1 and 0 < width <= 1 and 0 < height <= 1):
                    raise ValueError(f'Invalid normalized box in {label_name}')
                new_class_id = remap[class_id]
                rewritten.append(' '.join([str(new_class_id)] + fields[1:]))
                counts[CLASS_NAMES[new_class_id]] += 1
            with Image.open(io.BytesIO(image_data)) as image:
                image.load()
            records.append({
                'source': info.filename, 'filename': PurePosixPath(info.filename).name,
                'sha256': digest, 'image_data': image_data,
                'label_data': '\n'.join(rewritten) + ('\n' if rewritten else ''),
                'counts': counts,
            })

    random.Random(seed).shuffle(records)
    train_count = len(records) * 7 // 10
    val_count = len(records) * 2 // 10
    groups = {'train': records[:train_count],
              'val': records[train_count:train_count + val_count],
              'test': records[train_count + val_count:]}
    present = {name for record in records for name in record['counts']}
    for split, items in groups.items():
        represented = {name for record in items for name in record['counts']}
        if not present.issubset(represented):
            raise ValueError(f'Seed {seed} omits classes from {split}; choose another seed')

    destination.mkdir(parents=True)
    summary = {
        'source_zip': str(source), 'output': str(destination), 'seed': seed,
        'split_method': 'deterministic shuffle of unique image files',
        'temporal_grouping': False,
        'requested_ratio': [7, 2, 1], 'total_images': len(records),
        'original_names': original_names, 'names': CLASS_NAMES,
        'class_id_mapping': {str(k): v for k, v in remap.items()},
        'splits': {},
    }
    manifest = []
    for split, items in groups.items():
        for folder in ['images', 'labels']:
            (destination / split / folder).mkdir(parents=True)
        counts = collections.Counter()
        for record in items:
            image_path = destination / split / 'images' / record['filename']
            label_path = destination / split / 'labels' / Path(record['filename']).with_suffix('.txt')
            image_path.write_bytes(record['image_data'])
            label_path.write_text(record['label_data'])
            counts.update(record['counts'])
            manifest.append({'split': split, 'filename': record['filename'],
                             'source_zip_member': record['source'],
                             'sha256': record['sha256']})
        summary['splits'][split] = {
            'images': len(items), 'labels': len(items),
            'empty_labels': sum(not r['counts'] for r in items),
            'boxes_by_class': {name: counts[name] for name in CLASS_NAMES},
        }
    dataset_yaml = {'path': str(destination), 'train': 'train/images',
                    'val': 'val/images', 'test': 'test/images',
                    'nc': len(CLASS_NAMES), 'names': CLASS_NAMES}
    if config.get('roboflow'):
        dataset_yaml['roboflow'] = config['roboflow']
    (destination / 'data.yaml').write_text(yaml.safe_dump(dataset_yaml, sort_keys=False))
    (destination / 'classes.csv').write_text('\n'.join(CLASS_NAMES) + '\n')
    with (destination / 'split_manifest.csv').open('w', newline='') as handle:
        writer = csv.DictWriter(handle, fieldnames=['split', 'filename', 'source_zip_member', 'sha256'])
        writer.writeheader()
        writer.writerows(manifest)
    (destination / 'conversion_summary.json').write_text(json.dumps(summary, indent=2) + '\n')
    print(json.dumps(summary, indent=2))


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--zip', type=Path, default=Path.home() / 'Downloads/autorace-real-20261117-04-06.v1i.yolo26.zip')
    parser.add_argument('--output', type=Path, default=Path(__file__).resolve().parent / 'dataset_autorace_real_20261117_04_06_721')
    parser.add_argument('--seed', type=int, default=31)
    args = parser.parse_args()
    prepare(args.zip.resolve(), args.output.resolve(), args.seed)
