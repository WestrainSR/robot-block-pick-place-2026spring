#!/usr/bin/env python3
import argparse
import csv
import html
from pathlib import Path

import cv2
import numpy as np
from ultralytics import YOLO

IMAGE_EXTS = {'.jpg', '.jpeg', '.png', '.bmp'}
COLORS = {
    'red': (50, 50, 240),
    'green': (60, 190, 80),
    'blue': (230, 140, 40),
}


def parse_args():
    parser = argparse.ArgumentParser(description='Visualize color block detector predictions.')
    parser.add_argument('--model', default='runs/block_detection/competition_blocks_yolo11n/weights/best.pt')
    parser.add_argument('--source', default='datasets/color_block_capture/yolo/images/test')
    parser.add_argument('--labels', default='datasets/color_block_capture/yolo/labels/test')
    parser.add_argument('--out', default='reports/color_block_model_visualization')
    parser.add_argument('--conf', type=float, default=0.70)
    parser.add_argument('--imgsz', type=int, default=640)
    return parser.parse_args()


def read_gt(label_path, names):
    if not label_path.exists():
        return []
    labels = []
    for line in label_path.read_text(encoding='utf-8').splitlines():
        parts = line.split()
        if len(parts) != 5:
            continue
        cls_id = int(float(parts[0]))
        labels.append(names.get(cls_id, str(cls_id)))
    return labels


def draw_label(image, x, y, text, color):
    font = cv2.FONT_HERSHEY_SIMPLEX
    scale = 0.52
    thickness = 1
    (tw, th), baseline = cv2.getTextSize(text, font, scale, thickness)
    y = max(y, th + 8)
    cv2.rectangle(image, (x, y - th - 8), (x + tw + 8, y + baseline + 2), color, -1)
    cv2.putText(image, text, (x + 4, y - 5), font, scale, (255, 255, 255), thickness, cv2.LINE_AA)


def visualize_one(model, image_path, out_image_path, names, conf, imgsz):
    image = cv2.imread(str(image_path))
    if image is None:
        raise RuntimeError(f'failed to read {image_path}')

    result = model.predict(str(image_path), imgsz=imgsz, conf=conf, verbose=False)[0]
    predictions = []
    for box in result.boxes:
        cls_id = int(box.cls.item())
        class_name = names.get(cls_id, str(cls_id))
        score = float(box.conf.item())
        x1, y1, x2, y2 = [int(round(v)) for v in box.xyxy[0].tolist()]
        color = COLORS.get(class_name, (0, 215, 255))
        cv2.rectangle(image, (x1, y1), (x2, y2), color, 3)
        draw_label(image, max(0, x1), max(0, y1), f'{class_name} {score:.2f}', color)
        predictions.append({
            'class': class_name,
            'confidence': score,
            'box_xyxy': [x1, y1, x2, y2],
        })

    out_image_path.parent.mkdir(parents=True, exist_ok=True)
    cv2.imwrite(str(out_image_path), image)
    return predictions


def make_contact_sheet(tiles, out_path, cols=4):
    if not tiles:
        return
    rows = int(np.ceil(len(tiles) / cols))
    blank = np.full_like(tiles[0], 245)
    while len(tiles) < rows * cols:
        tiles.append(blank.copy())
    grid = []
    for row in range(rows):
        grid.append(np.hstack(tiles[row * cols:(row + 1) * cols]))
    out_path.parent.mkdir(parents=True, exist_ok=True)
    cv2.imwrite(str(out_path), np.vstack(grid))


def make_tile(image_path, gt, preds):
    image = cv2.imread(str(image_path))
    image = cv2.resize(image, (320, 240), interpolation=cv2.INTER_AREA)
    bar = np.zeros((54, 320, 3), dtype=np.uint8)
    pred_text = ', '.join(f'{p["class"]}:{p["confidence"]:.2f}' for p in preds) or 'none'
    gt_text = ', '.join(gt) or 'none'
    cv2.putText(bar, f'GT: {gt_text}', (8, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.48, (255, 255, 255), 1, cv2.LINE_AA)
    cv2.putText(bar, f'Pred: {pred_text}', (8, 43), cv2.FONT_HERSHEY_SIMPLEX, 0.44, (230, 230, 230), 1, cv2.LINE_AA)
    return np.vstack([bar, image])


def write_html(out_dir, model_path, source_dir, conf, rows):
    cards = []
    for row in rows:
        rel = Path('images') / row['visualization']
        pred = html.escape(row['pred_classes'] or 'none')
        gt = html.escape(row['gt_classes'] or 'none')
        ok = 'ok' if row['match'] == 'yes' else 'warn'
        cards.append(
            f'<article class="card {ok}">'
            f'<img src="{html.escape(rel.as_posix())}" alt="{html.escape(row["image"])}">'
            f'<div><strong>{html.escape(row["image"])}</strong></div>'
            f'<div>GT: {gt}</div><div>Pred: {pred}</div>'
            f'</article>'
        )
    matched = sum(1 for row in rows if row['match'] == 'yes')
    total = len(rows)
    doc = f'''<!doctype html>
<html lang="zh-CN">
<head>
  <meta charset="utf-8">
  <title>Color Block Model Visualization</title>
  <style>
    body {{ margin: 0; font-family: system-ui, -apple-system, BlinkMacSystemFont, "Segoe UI", sans-serif; color: #1f2933; background: #f6f7f9; }}
    header {{ padding: 24px 28px 12px; background: #ffffff; border-bottom: 1px solid #dde2e7; }}
    h1 {{ margin: 0 0 10px; font-size: 24px; }}
    .meta {{ color: #52606d; line-height: 1.6; }}
    .grid {{ display: grid; grid-template-columns: repeat(auto-fill, minmax(320px, 1fr)); gap: 14px; padding: 18px 28px 32px; }}
    .card {{ background: #ffffff; border: 1px solid #d9e2ec; border-radius: 8px; overflow: hidden; box-shadow: 0 1px 2px rgba(15, 23, 42, 0.04); }}
    .card.warn {{ border-color: #f59e0b; }}
    .card img {{ display: block; width: 100%; height: auto; }}
    .card div {{ padding: 0 10px 7px; font-size: 13px; }}
    .card div:first-of-type {{ padding-top: 10px; }}
  </style>
</head>
<body>
  <header>
    <h1>Color Block Detector</h1>
    <div class="meta">Model: {html.escape(str(model_path))}</div>
    <div class="meta">Source: {html.escape(str(source_dir))}</div>
    <div class="meta">Confidence threshold: {conf:.2f}; images matched by class: {matched}/{total}</div>
    <div class="meta">Function: detects red, green, and blue blocks and outputs class name, confidence, and bounding box.</div>
  </header>
  <main class="grid">
    {''.join(cards)}
  </main>
</body>
</html>
'''
    (out_dir / 'index.html').write_text(doc, encoding='utf-8')


def main():
    args = parse_args()
    model_path = Path(args.model).resolve()
    source_dir = Path(args.source).resolve()
    label_dir = Path(args.labels).resolve()
    out_dir = Path(args.out).resolve()
    image_out = out_dir / 'images'
    image_out.mkdir(parents=True, exist_ok=True)

    model = YOLO(str(model_path))
    names = {int(k): str(v) for k, v in model.names.items()}
    rows = []
    tiles = []

    for image_path in sorted(p for p in source_dir.iterdir() if p.suffix.lower() in IMAGE_EXTS):
        out_image_path = image_out / image_path.name
        preds = visualize_one(model, image_path, out_image_path, names, args.conf, args.imgsz)
        gt = read_gt(label_dir / f'{image_path.stem}.txt', names)
        pred_classes = [pred['class'] for pred in preds]
        match = 'yes' if set(gt).issubset(set(pred_classes)) and gt else 'no'
        row = {
            'image': image_path.name,
            'gt_classes': ','.join(gt),
            'pred_classes': ','.join(f'{p["class"]}:{p["confidence"]:.3f}' for p in preds),
            'match': match,
            'visualization': image_path.name,
        }
        rows.append(row)
        tiles.append(make_tile(out_image_path, gt, preds))

    with (out_dir / 'predictions.csv').open('w', newline='', encoding='utf-8') as f:
        writer = csv.DictWriter(f, fieldnames=['image', 'gt_classes', 'pred_classes', 'match', 'visualization'])
        writer.writeheader()
        writer.writerows(rows)

    make_contact_sheet(tiles, out_dir / 'contact_sheet.jpg')
    write_html(out_dir, model_path, source_dir, args.conf, rows)
    matched = sum(1 for row in rows if row['match'] == 'yes')
    print(f'visualized {len(rows)} images')
    print(f'class matches: {matched}/{len(rows)}')
    print(f'html: {(out_dir / "index.html").resolve()}')
    print(f'contact sheet: {(out_dir / "contact_sheet.jpg").resolve()}')


if __name__ == '__main__':
    main()
