"""Actual pretrained-weight smoke: python -m room_semantics.detect_image image.jpg."""
import argparse
import json
from pathlib import Path
import time
from PIL import Image,ImageDraw
from .detector import GroundingDinoDetector,DEFAULT_MODEL,DEFAULT_LABELS,DEFAULT_REVISION


def main():
    p=argparse.ArgumentParser(__doc__)
    p.add_argument("image"); p.add_argument("--out",required=True)
    p.add_argument("--annotated"); p.add_argument("--model",default=DEFAULT_MODEL)
    p.add_argument("--revision",default=DEFAULT_REVISION); p.add_argument("--device",default="auto")
    p.add_argument("--labels",nargs="+",default=list(DEFAULT_LABELS)); p.add_argument("--local-files-only",action="store_true")
    p.add_argument("--threshold",type=float,default=.35); p.add_argument("--text-threshold",type=float,default=.25)
    args=p.parse_args()
    image=Image.open(args.image).convert("RGB")
    detector=GroundingDinoDetector(args.model,args.device,args.threshold,args.text_threshold,args.labels,args.revision,args.local_files_only)
    start=time.perf_counter(); detections=detector.detect(image); elapsed=time.perf_counter()-start
    result={"schema_version":1,"source":"pretrained_grounding_dino","image":str(Path(args.image).resolve()),
            "model_id":detector.model_id,"requested_revision":detector.revision,"resolved_revision":detector.resolved_revision,
            "inference_seconds":elapsed,"image_size":[image.width,image.height],"detections":detections,
            "metric_localization":False,"note":"An empty result is valid; this smoke is not a detection-quality benchmark."}
    out=Path(args.out); out.parent.mkdir(parents=True,exist_ok=True); out.write_text(json.dumps(result,ensure_ascii=False,indent=2)+"\n",encoding="utf-8")
    if args.annotated:
        draw=ImageDraw.Draw(image)
        for d in detections:
            draw.rectangle(d["bbox_xyxy"],outline="red",width=2)
            draw.text(d["bbox_xyxy"][:2],f'{d["label"]} {d["score"]:.2f}',fill="red")
        target=Path(args.annotated); target.parent.mkdir(parents=True,exist_ok=True); image.save(target)
    print(json.dumps(result,ensure_ascii=False))


if __name__=="__main__": main()
