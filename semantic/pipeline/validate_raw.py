#!/usr/bin/env python3
"""Validate a fixed snapshot of saved RGB/semantic pairs, including during collection."""
import argparse
from collections import Counter
import json
from pathlib import Path
import time
import cv2
import numpy as np
from camera_config import CAMERA_PAIRS, CAMERA_GROUPS, supervised_class_ids
from semantic_common import atomic_write_json, atomic_write_bytes, load_class_config, build_palette_lut, semantic_bgr_to_mask


def snapshot_records(path):
    rows, errors = [], []
    with path.open('rb') as stream:
        boundary = path.stat().st_size
        while stream.tell() < boundary:
            line = stream.readline(boundary-stream.tell())
            if not line.endswith(b'\n'):
                break
            try:
                row = json.loads(line)
                if not isinstance(row, dict): raise ValueError('record is not an object')
                rows.append(row)
            except (ValueError, UnicodeDecodeError) as error:
                errors.append(f'line {len(rows)+1}: {error}')
    return rows, boundary, errors


def inside(root, value):
    path = (root / value).resolve()
    if not path.is_relative_to(root.resolve()):
        raise ValueError('image path escapes session')
    return path


def validate(root, full=False, limit=120):
    root = Path(root).resolve()
    manifest = root/'semantic_pairs.jsonl'
    if not manifest.is_file(): manifest = root/'manifest.jsonl'
    rows, boundary, errors = snapshot_records(manifest)
    all_errors = len(errors)
    warnings, counts, times, seen = [], Counter(), {}, set()
    valid_rows = []
    def error(message):
        nonlocal all_errors
        all_errors += 1
        if len(errors)<30: errors.append(message)
    for i,row in enumerate(rows):
        try:
            view = row['view']
            if view not in CAMERA_PAIRS: raise ValueError('unknown camera')
            counts[view] += 1
            key=(view,row['sequence'])
            if key in seen: raise ValueError('duplicate camera/sequence')
            seen.add(key)
            for kind in ('rgb','semantic'):
                if not inside(root,row[kind]).is_file(): raise ValueError(f'missing {kind} file')
            rgb_stamp, sem_stamp = row.get('rgb_timestamp_ns'),row.get('semantic_timestamp_ns')
            if rgb_stamp is None or sem_stamp is None: raise ValueError('missing timestamps')
            if abs(rgb_stamp-sem_stamp)>60_000_000: raise ValueError('timestamp difference exceeds 60 ms')
            times.setdefault(view,[]).append(rgb_stamp)
            supervised_class_ids(row)
            valid_rows.append((i,row))
        except (KeyError, ValueError, TypeError, OSError) as exc:
            error(f'row {i+1}: {exc}')
    if not rows: error('아직 저장된 완전한 RGB·정답 쌍이 없습니다.')
    selected = valid_rows
    if not full and len(selected)>limit:
        # Spread the sample across cameras and the complete captured timeline.
        groups = {view:[item for item in valid_rows if item[1]['view']==view] for view in counts}
        selected=[]
        per_view=max(1,limit//max(1,len(groups)))
        for items in groups.values():
            selected.extend(items[int(i)] for i in np.linspace(0,len(items)-1,min(len(items),per_view)))
        selected.sort(key=lambda item:item[0])
    config, classes, palette, ids=load_class_config()
    lut,distance=build_palette_lut(palette,ids,max_distance=90)
    valid_lut=(distance<=90).astype(np.uint8)
    pixels=np.zeros(6,np.int64);positive=Counter();unknown=total=checked=0;deltas=[];samples=[]
    colors=np.array([c['color'][::-1] for c in classes],np.uint8)
    preview_rows=set()
    report_dir=root/'validation';report_dir.mkdir(exist_ok=True)
    for index,row in selected:
        try:
            rgb=cv2.imread(str(inside(root,row['rgb'])))
            semantic=cv2.imread(str(inside(root,row['semantic'])))
            if rgb is None or semantic is None: raise ValueError('unreadable image')
            if rgb.shape!=semantic.shape: raise ValueError('RGB/semantic shape mismatch')
            if row.get('width',rgb.shape[1])!=rgb.shape[1] or row.get('height',rgb.shape[0])!=rgb.shape[0]:
                raise ValueError('manifest dimensions differ from image')
            mask=semantic_bgr_to_mask(semantic,lut)
            valid=semantic_bgr_to_mask(semantic,valid_lut)
            active=supervised_class_ids(row);mask[~np.isin(mask,active)]=255
            total+=mask.size;unknown+=int((valid==0).sum())
            for class_id in active:
                count=int((mask==class_id).sum());pixels[class_id]+=count
                if count:positive[class_id]+=1
            deltas.append(abs(row['rgb_timestamp_ns']-row['semantic_timestamp_ns'])/1e6)
            checked+=1
            if row['view'] not in preview_rows:
                preview_rows.add(row['view'])
                painted=np.zeros_like(rgb);present=mask<6;painted[present]=colors[mask[present]]
                shape=(320,max(1,round(rgb.shape[0]*320/rgb.shape[1])))
                tiles=[cv2.resize(picture,shape,interpolation=cv2.INTER_NEAREST if i else cv2.INTER_AREA) for i,picture in enumerate((rgb,semantic,painted))]
                preview=np.hstack(tiles)
                atomic_write_bytes(report_dir/f"{row['view']}.jpg",cv2.imencode('.jpg',preview,[cv2.IMWRITE_JPEG_QUALITY,85])[1].tobytes())
                samples.append({'view':row['view'],'sequence':row['sequence'],'row':index+1})
        except (ValueError,TypeError,KeyError,OSError,cv2.error) as exc:error(f'row {index+1}: {exc}')
        if checked and checked%100==0: print(f'checked={checked}/{len(selected)} errors={all_errors}',flush=True)
    durations={view:(max(stamps)-min(stamps))/1e9 for view,stamps in times.items() if stamps}
    active_ids=sorted({c for view in counts for c in CAMERA_PAIRS[view]['supervised_class_ids']})
    missing=[classes[c]['name'] for c in active_ids if pixels[c]==0]
    if missing:warnings.append('검사한 영상에 없는 클래스: '+', '.join(missing))
    if unknown and total and unknown/total>.05:warnings.append('알 수 없는 시맨틱 색상 픽셀이 5%를 넘습니다. 정답 색상표를 확인하세요.')
    if not full and len(selected)<len(valid_rows):warnings.append('영상 내용은 표본 검사입니다. 전체 검사 전에는 모든 이미지가 정상이라고 판단하지 마세요.')
    expected_views = set(counts)
    metadata_path = root/'session.json'
    if metadata_path.is_file():
        try:
            metadata = json.loads(metadata_path.read_text())
            expected_views = set(metadata.get('camera_pairs') or CAMERA_GROUPS.get(metadata.get('group'), counts))
        except (ValueError, TypeError, AttributeError, OSError) as exc:
            error(f'session.json: {exc}')
    missing_views = sorted(expected_views-set(counts))
    if missing_views:warnings.append('아직 저장되지 않은 카메라: '+', '.join(missing_views))
    report={'time':time.time(),'mode':'full' if full else 'quick','session':root.name,'manifest':manifest.name,
            'snapshot_bytes':boundary,'total_pairs':len(rows),'checked_images':checked,'error_count':all_errors,
            'errors':errors[:30],'warnings':warnings,'ok':all_errors==0,'counts':dict(counts),'duration_seconds':durations,
            'sync_max_ms':max(deltas) if deltas else None,'unknown_pixel_fraction':unknown/total if total else None,
            'class_pixels':pixels.tolist(),'positive_frames':{str(c):positive[c] for c in range(6)},'samples':samples,
            'quick_training_candidate':bool(expected_views) and all(counts[v]>=30 and durations.get(v,0)>=30 for v in expected_views) and all_errors==0,
            'model_quality_verified':False}
    atomic_write_json(root/'raw_validation.json',report)
    print(json.dumps(report,ensure_ascii=False),flush=True)
    return report


def main():
    parser=argparse.ArgumentParser();parser.add_argument('--input',type=Path,required=True);parser.add_argument('--full',action='store_true')
    args=parser.parse_args();report=validate(args.input,args.full)
    return 0 if report['ok'] else 1

if __name__=='__main__':raise SystemExit(main())
