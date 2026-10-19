"""Explicit, one-time pretrained perception download; never starts ROS/motors."""
import argparse
import os
from pathlib import Path
from .configuration import load_config,resolve_path


def main():
    parser=argparse.ArgumentParser(description="Download YOLO11n and metric indoor depth to local model caches")
    parser.add_argument("--config",required=True)
    args=parser.parse_args()
    c=load_config(args.config)
    weights=resolve_path(c,c["perception"]["detector_weights"])
    weights.parent.mkdir(parents=True,exist_ok=True)
    from ultralytics import YOLO
    from transformers import AutoImageProcessor,AutoModelForDepthEstimation
    previous=Path.cwd()
    try:
        os.chdir(weights.parent)
        YOLO(weights.name)
    finally:
        os.chdir(previous)
    if not weights.is_file():
        raise SystemExit("YOLO downloader did not create the expected local weight file")
    model_id=c["perception"]["depth_model_id"]
    revision=c["perception"].get("depth_model_revision")
    AutoImageProcessor.from_pretrained(model_id,revision=revision)
    model=AutoModelForDepthEstimation.from_pretrained(model_id,revision=revision)
    if getattr(model.config,"depth_estimation_type",None)!="metric":
        raise SystemExit("Requested depth checkpoint is not metric")
    print("Person detector:",weights)
    print("Metric depth cached:",model_id,"revision:",revision or "default (not pinned)")
    print("No world-model checkpoint was trained; camera/motor calibration remains required.")


if __name__=="__main__":
    main()
