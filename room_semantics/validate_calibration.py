"""Check a calibration file's schema; this does not measure the hardware."""
import argparse
import json
from .calibration import load_calibration


def main():
    parser=argparse.ArgumentParser(__doc__); parser.add_argument("calibration"); parser.add_argument("--require-verified",action="store_true")
    args=parser.parse_args(); value=load_calibration(args.calibration,args.require_verified)
    print(json.dumps({"schema_version":1,"camera_frame":value.camera_frame,"lidar_frame":value.lidar_frame,
                      "verified":value.verified,"image_is_rectified":value.image_is_rectified,
                      "camera_from_lidar":value.camera_from_lidar.tolist(),"hardware_measurement_performed":False},sort_keys=True))


if __name__=="__main__": main()
