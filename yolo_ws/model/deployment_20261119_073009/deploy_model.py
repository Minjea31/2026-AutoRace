#!/usr/bin/env python3
"""Install an already validated engine, preserving each replaced artifact."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import shutil


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest() if path.exists() else None


def main():
    parser=argparse.ArgumentParser();parser.add_argument('config',type=Path)
    args=parser.parse_args();cfg=json.loads(args.config.read_text())
    workspace=Path(cfg['remote_workspace']);job=Path(cfg['remote_job'])
    validation=json.loads((job/'validation.json').read_text())
    assert validation['passed'] and validation['source_sha256']==cfg['source_sha256']
    source=Path(cfg['remote_model']);engine=Path(cfg['remote_engine'])
    assert digest(source)==cfg['source_sha256']
    assert digest(engine)==validation['engine_sha256']
    assets={'best.pt':source,'best.engine':engine,
            'best.onnx':engine.with_suffix('.onnx'),
            'best.optimization.json':engine.with_suffix('.optimization.json')}
    assert all(p.is_file() for p in assets.values())
    directories=[workspace/'model',workspace/'install/yolo_detect/share/yolo_detect/model']
    assert all(p.is_dir() for p in directories)
    backup=job/'backups';backup.mkdir(exist_ok=False)
    plan=[];committed=[]
    try:
        for directory in directories:
            for name,asset in assets.items():
                target=directory/name
                if target.is_symlink():
                    raise RuntimeError(f'Unexpected symlink: {target}')
                saved=backup/target.relative_to(workspace)
                old_hash=digest(target)
                if target.exists():
                    saved.parent.mkdir(parents=True,exist_ok=True)
                    shutil.copy2(target,saved)
                    assert digest(saved)==old_hash
                staged=target.with_name('.'+target.name+'.deploy_'+cfg['tag'])
                if staged.exists():
                    raise FileExistsError(staged)
                shutil.copy2(asset,staged)
                new_hash=digest(asset);assert digest(staged)==new_hash
                plan.append(dict(source=str(asset),target=str(target),backup=str(saved) if old_hash else None,
                                 previous_sha256=old_hash,new_sha256=new_hash,staged=str(staged)))
        report=dict(status='staged',source_sha256=cfg['source_sha256'],backup_directory=str(backup),files=plan)
        (job/'deployment.json').write_text(json.dumps(report,indent=2)+'\n')
        for item in plan:
            target=Path(item['target'])
            assert digest(target)==item['previous_sha256'],f'Concurrent file change: {target}'
            os.replace(item['staged'],target)
            committed.append(item)
            assert digest(target)==item['new_sha256']
        report['status']='deployed'
        (job/'deployment.json').write_text(json.dumps(report,indent=2)+'\n')
        print(json.dumps(report,indent=2),flush=True)
    except BaseException:
        for item in reversed(committed):
            target=Path(item['target'])
            if item['backup']:
                rollback=target.with_name('.'+target.name+'.rollback_'+cfg['tag'])
                shutil.copy2(item['backup'],rollback);os.replace(rollback,target)
            else:
                target.unlink()
        raise
    finally:
        for item in plan:
            Path(item['staged']).unlink(missing_ok=True)


if __name__=='__main__':
    main()
