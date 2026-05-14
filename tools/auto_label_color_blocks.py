#!/usr/bin/env python3
import argparse
import csv
import shutil
from pathlib import Path

import cv2
import numpy as np

CLASSES = ['red', 'green', 'blue']
CLASS_ID = {name: idx for idx, name in enumerate(CLASSES)}
IMAGE_EXTS = {'.jpg', '.jpeg', '.png', '.bmp'}


def parse_args():
    parser = argparse.ArgumentParser(description='Auto-label single-color block images with OpenCV color segmentation.')
    parser.add_argument('--raw', default='datasets/color_block_capture/raw', help='Raw capture directory.')
    parser.add_argument('--out', default='datasets/color_block_capture/auto_labeled', help='Output labeled directory.')
    parser.add_argument('--preview', default='datasets/color_block_capture/quality/auto_label_preview.jpg', help='Preview contact sheet path.')
    parser.add_argument('--samples-per-class', type=int, default=18, help='Preview samples per class.')
    parser.add_argument('--min-area-ratio', type=float, default=0.0008, help='Smallest accepted contour area ratio.')
    parser.add_argument('--max-area-ratio', type=float, default=0.10, help='Largest accepted contour area ratio.')
    parser.add_argument('--padding', type=int, default=8, help='Bounding box padding in pixels.')
    return parser.parse_args()


def mask_for_class(image, class_name):
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    b, g, r = cv2.split(image)
    if class_name == 'red':
        hue_mask = ((hsv[:, :, 0] <= 12) | (hsv[:, :, 0] >= 168))
        chroma_mask = (
            (hsv[:, :, 1] >= 120)
            & (hsv[:, :, 2] >= 50)
            & (r >= 100)
            & (r.astype(np.int16) >= g.astype(np.int16) + 55)
            & (r.astype(np.int16) >= b.astype(np.int16) + 55)
        )
        return ((hue_mask & chroma_mask).astype(np.uint8) * 255)
    if class_name == 'green':
        hue_mask = (hsv[:, :, 0] >= 35) & (hsv[:, :, 0] <= 100)
        chroma_mask = (
            (hsv[:, :, 1] >= 35)
            & (hsv[:, :, 2] >= 25)
            & (g >= 45)
            & (g.astype(np.int16) >= r.astype(np.int16) - 5)
        )
        return ((hue_mask & chroma_mask).astype(np.uint8) * 255)
    if class_name == 'blue':
        hue_mask = (hsv[:, :, 0] >= 85) & (hsv[:, :, 0] <= 125)
        chroma_mask = (
            (hsv[:, :, 1] >= 55)
            & (hsv[:, :, 2] >= 50)
            & (b >= 70)
            & (b.astype(np.int16) >= r.astype(np.int16) + 20)
            & (g.astype(np.int16) >= r.astype(np.int16) + 10)
        )
        return ((hue_mask & chroma_mask).astype(np.uint8) * 255)
    raise ValueError(f'unsupported class: {class_name}')


def find_bbox(image, class_name, min_area_ratio, max_area_ratio, padding):
    h, w = image.shape[:2]
    mask = mask_for_class(image, class_name)

    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=1)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=2)

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    area_total = float(h * w)
    candidates = []
    for contour in contours:
        area = cv2.contourArea(contour)
        ratio = area / area_total
        if ratio < min_area_ratio or ratio > max_area_ratio:
            continue
        x, y, bw, bh = cv2.boundingRect(contour)
        if bw < 8 or bh < 8:
            continue
        fill = area / max(1, bw * bh)
        aspect = bw / max(1, bh)
        if fill < 0.18 or aspect < 0.25 or aspect > 4.0:
            continue
        score = area * (0.75 + min(fill, 1.0))
        candidates.append((score, x, y, bw, bh, area, fill))

    if not candidates:
        return None, mask, 'no_contour'
    candidates.sort(reverse=True)
    _, x, y, bw, bh, area, fill = candidates[0]

    x1 = max(0, x - padding)
    y1 = max(0, y - padding)
    x2 = min(w - 1, x + bw + padding)
    y2 = min(h - 1, y + bh + padding)
    if x2 <= x1 or y2 <= y1:
        return None, mask, 'bad_bbox'
    return (x1, y1, x2, y2, area / area_total, fill), mask, 'ok'


def yolo_line(class_name, bbox, width, height):
    x1, y1, x2, y2, *_ = bbox
    cx = ((x1 + x2) / 2.0) / width
    cy = ((y1 + y2) / 2.0) / height
    bw = (x2 - x1) / width
    bh = (y2 - y1) / height
    return f'{CLASS_ID[class_name]} {cx:.6f} {cy:.6f} {bw:.6f} {bh:.6f}\n'


def iter_images(raw_dir):
    for class_name in CLASSES:
        class_dir = raw_dir / class_name
        if not class_dir.exists():
            continue
        for image_path in sorted(p for p in class_dir.rglob('*') if p.suffix.lower() in IMAGE_EXTS):
            yield class_name, image_path


def draw_preview_tile(image, class_name, rel_path, bbox, status):
    tile = image.copy()
    h, w = tile.shape[:2]
    if bbox:
        x1, y1, x2, y2, area_ratio, fill = bbox
        color = (0, 255, 255)
        cv2.rectangle(tile, (x1, y1), (x2, y2), color, 3)
        status_text = f'{class_name} area={area_ratio:.3f}'
    else:
        status_text = f'{class_name} {status}'
    cv2.rectangle(tile, (0, 0), (w, 46), (0, 0, 0), -1)
    cv2.putText(tile, status_text, (8, 18), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 1, cv2.LINE_AA)
    cv2.putText(tile, rel_path[-56:], (8, 38), cv2.FONT_HERSHEY_SIMPLEX, 0.42, (230, 230, 230), 1, cv2.LINE_AA)
    return cv2.resize(tile, (256, 160), interpolation=cv2.INTER_AREA)


def make_preview(preview_tiles, preview_path, cols=6):
    if not preview_tiles:
        return
    rows = int(np.ceil(len(preview_tiles) / cols))
    blank = np.full_like(preview_tiles[0], 245)
    while len(preview_tiles) < rows * cols:
        preview_tiles.append(blank.copy())
    grid_rows = []
    for r in range(rows):
        grid_rows.append(np.hstack(preview_tiles[r * cols:(r + 1) * cols]))
    sheet = np.vstack(grid_rows)
    preview_path.parent.mkdir(parents=True, exist_ok=True)
    cv2.imwrite(str(preview_path), sheet)


def main():
    args = parse_args()
    raw_dir = Path(args.raw)
    out_dir = Path(args.out)
    image_out = out_dir / 'images'
    label_out = out_dir / 'labels'
    quality_dir = out_dir / 'quality'

    if out_dir.exists():
        shutil.rmtree(out_dir)
    image_out.mkdir(parents=True, exist_ok=True)
    label_out.mkdir(parents=True, exist_ok=True)
    quality_dir.mkdir(parents=True, exist_ok=True)

    stats = {cls: {'ok': 0, 'failed': 0} for cls in CLASSES}
    failures = []
    preview_tiles = []
    preview_seen = {cls: 0 for cls in CLASSES}
    manifest_rows = []

    for class_name, image_path in iter_images(raw_dir):
        image = cv2.imread(str(image_path))
        if image is None:
            failures.append((str(image_path), class_name, 'read_failed'))
            stats[class_name]['failed'] += 1
            continue
        bbox, _mask, status = find_bbox(
            image,
            class_name,
            args.min_area_ratio,
            args.max_area_ratio,
            args.padding,
        )
        if bbox is None:
            failures.append((str(image_path), class_name, status))
            stats[class_name]['failed'] += 1
        else:
            h, w = image.shape[:2]
            target_name = f'{class_name}_{image_path.stem}{image_path.suffix.lower()}'
            target_image = image_out / target_name
            target_label = label_out / f'{Path(target_name).stem}.txt'
            shutil.copy2(image_path, target_image)
            target_label.write_text(yolo_line(class_name, bbox, w, h), encoding='utf-8')
            stats[class_name]['ok'] += 1
            manifest_rows.append({
                'image': target_name,
                'label': target_label.name,
                'class': class_name,
                'source': image_path.as_posix(),
                'bbox_xyxy': ' '.join(str(int(v)) for v in bbox[:4]),
                'area_ratio': f'{bbox[4]:.6f}',
                'fill': f'{bbox[5]:.6f}',
            })

        if preview_seen[class_name] < args.samples_per_class:
            rel_path = image_path.relative_to(raw_dir).as_posix()
            preview_tiles.append(draw_preview_tile(image, class_name, rel_path, bbox, status))
            preview_seen[class_name] += 1

    with (out_dir / 'auto_labels_manifest.csv').open('w', newline='', encoding='utf-8') as f:
        fieldnames = ['image', 'label', 'class', 'source', 'bbox_xyxy', 'area_ratio', 'fill']
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(manifest_rows)

    with (quality_dir / 'failures.csv').open('w', newline='', encoding='utf-8') as f:
        writer = csv.writer(f)
        writer.writerow(['image', 'class', 'reason'])
        writer.writerows(failures)

    make_preview(preview_tiles, Path(args.preview))
    print('Auto-label summary')
    for cls in CLASSES:
        print(f'  {cls}: ok={stats[cls]["ok"]} failed={stats[cls]["failed"]}')
    print(f'output: {out_dir.resolve()}')
    print(f'preview: {Path(args.preview).resolve()}')
    if failures:
        print(f'failures: {len(failures)} -> {(quality_dir / "failures.csv").resolve()}')


if __name__ == '__main__':
    main()
