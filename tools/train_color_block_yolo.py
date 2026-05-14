#!/usr/bin/env python3
import argparse
from pathlib import Path

import torch
from ultralytics import YOLO


def parse_args():
    parser = argparse.ArgumentParser(description='Train the red/green/blue block detector with Ultralytics YOLO.')
    parser.add_argument('--data', default='datasets/color_block_capture/yolo/data.yaml', help='YOLO data.yaml path.')
    parser.add_argument('--model', default='yolo11n.pt', help='Base model checkpoint.')
    parser.add_argument('--epochs', type=int, default=80, help='Training epochs.')
    parser.add_argument('--imgsz', type=int, default=640, help='Training image size.')
    parser.add_argument('--batch', type=int, default=8, help='Batch size.')
    parser.add_argument('--patience', type=int, default=20, help='Early-stop patience.')
    parser.add_argument('--project', default='runs/block_detection', help='Ultralytics output project directory.')
    parser.add_argument('--name', default='competition_blocks_yolo11n', help='Ultralytics run name.')
    parser.add_argument('--device', default=None, help='Training device, e.g. cpu or 0. Auto-detect by default.')
    parser.add_argument('--workers', type=int, default=0, help='DataLoader workers. Keep 0 on Windows.')
    parser.add_argument('--exist-ok', action='store_true', help='Reuse an existing run directory.')
    parser.add_argument('--export-openvino', action='store_true', help='Export the best checkpoint to OpenVINO after training.')
    return parser.parse_args()


def main():
    args = parse_args()
    data_path = Path(args.data)
    if not data_path.exists():
        raise SystemExit(f'data.yaml not found: {data_path}')

    device = args.device
    if device is None:
        device = 0 if torch.cuda.is_available() else 'cpu'

    project_dir = Path(args.project).resolve()
    model = YOLO(args.model)
    results = model.train(
        data=str(data_path),
        epochs=args.epochs,
        imgsz=args.imgsz,
        batch=args.batch,
        patience=args.patience,
        project=str(project_dir),
        name=args.name,
        device=device,
        workers=args.workers,
        cache=False,
        exist_ok=args.exist_ok,
    )

    save_dir = Path(results.save_dir)
    best = save_dir / 'weights' / 'best.pt'
    print(f'training run: {save_dir.resolve()}')
    print(f'best checkpoint: {best.resolve()}')

    if args.export_openvino:
        YOLO(str(best)).export(format='openvino', imgsz=args.imgsz)


if __name__ == '__main__':
    main()
