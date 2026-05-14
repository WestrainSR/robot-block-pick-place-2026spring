#!/usr/bin/env python3
import argparse
import random
import shutil
from pathlib import Path

CLASSES = ['redstone', 'glass', 'glowstone', 'grass']
IMAGE_EXTS = {'.jpg', '.jpeg', '.png', '.bmp'}


def parse_args():
    parser = argparse.ArgumentParser(description='Split labeled YOLO images into train/val/test folders.')
    parser.add_argument('--images', required=True, help='Directory containing labeled images.')
    parser.add_argument('--labels', required=True, help='Directory containing YOLO .txt label files.')
    parser.add_argument('--out', required=True, help='Output YOLO dataset directory.')
    parser.add_argument('--train', type=float, default=0.8, help='Train split ratio.')
    parser.add_argument('--val', type=float, default=0.1, help='Validation split ratio.')
    parser.add_argument('--test', type=float, default=0.1, help='Test split ratio.')
    parser.add_argument('--seed', type=int, default=2026, help='Random seed.')
    parser.add_argument('--copy-empty', action='store_true', help='Keep images without labels as background samples.')
    return parser.parse_args()


def validate_ratios(train, val, test):
    total = train + val + test
    if abs(total - 1.0) > 1e-6:
        raise SystemExit(f'split ratios must sum to 1.0, got {total}')


def read_label(label_path: Path):
    rows = []
    for lineno, line in enumerate(label_path.read_text(encoding='utf-8').splitlines(), 1):
        line = line.strip()
        if not line:
            continue
        parts = line.split()
        if len(parts) != 5:
            raise SystemExit(f'{label_path}:{lineno}: expected 5 columns, got {len(parts)}')
        class_id = int(parts[0])
        if class_id < 0 or class_id >= len(CLASSES):
            raise SystemExit(f'{label_path}:{lineno}: class id {class_id} outside 0..{len(CLASSES)-1}')
        coords = [float(v) for v in parts[1:]]
        if any(v < 0.0 or v > 1.0 for v in coords):
            raise SystemExit(f'{label_path}:{lineno}: bbox values must be normalized to [0, 1]')
        rows.append(line)
    return rows


def main():
    args = parse_args()
    validate_ratios(args.train, args.val, args.test)

    image_dir = Path(args.images)
    label_dir = Path(args.labels)
    out_dir = Path(args.out)
    images = sorted(p for p in image_dir.iterdir() if p.suffix.lower() in IMAGE_EXTS)
    if not images:
        raise SystemExit(f'no images found in {image_dir}')

    samples = []
    missing = []
    empty = []
    for image in images:
        label = label_dir / f'{image.stem}.txt'
        if not label.exists():
            missing.append(image.name)
            if args.copy_empty:
                samples.append((image, None))
            continue
        rows = read_label(label)
        if rows:
            samples.append((image, label))
        else:
            empty.append(image.name)
            if args.copy_empty:
                samples.append((image, label))

    if not samples:
        raise SystemExit('no usable samples found')

    random.seed(args.seed)
    random.shuffle(samples)
    n = len(samples)
    n_train = int(n * args.train)
    n_val = int(n * args.val)
    splits = {
        'train': samples[:n_train],
        'val': samples[n_train:n_train + n_val],
        'test': samples[n_train + n_val:],
    }

    for split, split_samples in splits.items():
        (out_dir / 'images' / split).mkdir(parents=True, exist_ok=True)
        (out_dir / 'labels' / split).mkdir(parents=True, exist_ok=True)
        for image, label in split_samples:
            shutil.copy2(image, out_dir / 'images' / split / image.name)
            target_label = out_dir / 'labels' / split / f'{image.stem}.txt'
            if label is None:
                target_label.write_text('', encoding='utf-8')
            else:
                shutil.copy2(label, target_label)

    data_yaml = out_dir / 'data.yaml'
    data_yaml.write_text(
        'path: ' + str(out_dir.resolve()).replace('\\', '/') + '\n'
        'train: images/train\n'
        'val: images/val\n'
        'test: images/test\n'
        'names:\n'
        + ''.join(f'  {i}: {name}\n' for i, name in enumerate(CLASSES)),
        encoding='utf-8',
    )

    print(f'wrote dataset: {out_dir.resolve()}')
    print(f'train={len(splits["train"])} val={len(splits["val"])} test={len(splits["test"])}')
    if missing:
        print(f'warning: {len(missing)} images had no label file')
    if empty:
        print(f'warning: {len(empty)} label files were empty')


if __name__ == '__main__':
    main()
