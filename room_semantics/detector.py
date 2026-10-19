"""Optional real GroundingDINO adapter. Heavy libraries/weights load on demand."""
import importlib.metadata
import math
import re

TRANSFORMERS_VERSION="4.52.3"
DEFAULT_MODEL="IDEA-Research/grounding-dino-tiny"
DEFAULT_REVISION="a2bb814dd30d776dcf7e30523b00659f4f141c71"
DEFAULT_LABELS=("person","chair","table","door")


def canonical_label(text,allowed=DEFAULT_LABELS):
    words=set(re.findall(r"[a-z]+",str(text).lower()))
    names=[label for label in allowed if set(label.split())<=words]
    # A phrase containing two prompted classes is semantically ambiguous.
    return names[0] if len(names)==1 else None


def postprocess_results(result,width,height,allowed=DEFAULT_LABELS):
    labels=result.get("text_labels")
    if labels is None:
        labels=result.get("labels")  # v4.52.3 may retain deprecated string alias.
    if labels is None: raise ValueError("Detector output has no textual labels")
    def as_list(value):
        return value.detach().cpu().tolist() if hasattr(value,"detach") else list(value)
    boxes=as_list(result["boxes"]); scores=as_list(result["scores"])
    if len(boxes)!=len(scores) or len(labels)!=len(boxes): raise ValueError("Detector output row count mismatch")
    detections=[]
    for box,score,text in sorted(zip(boxes,scores,labels),key=lambda row:-float(row[1])):
        label=canonical_label(text,allowed)
        if label is None or len(box)!=4 or not all(math.isfinite(float(v)) for v in [*box,score]) or not 0<=float(score)<=1: continue
        x1,y1,x2,y2=map(float,box)
        x1=max(0.,min(x1,width)); x2=max(0.,min(x2,width))
        y1=max(0.,min(y1,height)); y2=max(0.,min(y2,height))
        if x2<=x1 or y2<=y1: continue
        detection={"label":label,"score":float(score),"bbox_xyxy":[x1,y1,x2,y2]}
        # Suppress almost-identical same-label phrases, not competing classes.
        if any(old["label"]==label and _iou(old["bbox_xyxy"],detection["bbox_xyxy"])>.7 for old in detections): continue
        detections.append(detection)
    return sorted(detections,key=lambda d:(d["label"],d["bbox_xyxy"][0],d["bbox_xyxy"][1],-d["score"]))


def _iou(a,b):
    intersection=max(0.,min(a[2],b[2])-max(a[0],b[0]))*max(0.,min(a[3],b[3])-max(a[1],b[1]))
    return intersection/max(1e-9,(a[2]-a[0])*(a[3]-a[1])+(b[2]-b[0])*(b[3]-b[1])-intersection)


class GroundingDinoDetector:
    def __init__(self,model_id=DEFAULT_MODEL,device="auto",threshold=.35,text_threshold=.25,
                 labels=DEFAULT_LABELS,revision=DEFAULT_REVISION,local_files_only=False):
        # ROS-free geometry/tests remain usable without Transformers or torch.
        installed=importlib.metadata.version("transformers")
        if installed!=TRANSFORMERS_VERSION:
            raise RuntimeError(f"Install transformers=={TRANSFORMERS_VERSION}; found {installed}")
        import torch
        from transformers import AutoProcessor,AutoModelForZeroShotObjectDetection
        self.torch=torch; self.labels=tuple(labels)
        if not self.labels or any(not re.fullmatch(r"[a-z]+(?: [a-z]+)*",s) for s in self.labels):
            raise ValueError("Use canonical lowercase English class prompts")
        if not 0<threshold<1 or not 0<text_threshold<1: raise ValueError("Detector thresholds must be in (0,1)")
        self.threshold=threshold; self.text_threshold=text_threshold
        self.device="cuda" if device=="auto" and torch.cuda.is_available() else ("cpu" if device=="auto" else device)
        self.processor=AutoProcessor.from_pretrained(model_id,revision=revision,local_files_only=local_files_only)
        self.model=AutoModelForZeroShotObjectDetection.from_pretrained(model_id,revision=revision,local_files_only=local_files_only).to(self.device).eval()
        self.model_id=model_id; self.revision=revision
        self.resolved_revision=getattr(self.model.config,"_commit_hash",None)
        self.prompt=" . ".join(self.labels)+" ."

    def detect(self,image):
        """PIL RGB image -> pixel boxes using official 4.52.3 postprocessor API."""
        inputs=self.processor(images=image,text=self.prompt,return_tensors="pt").to(self.device)
        with self.torch.inference_mode(): outputs=self.model(**inputs)
        # In Transformers 4.52.3 this is threshold=, not box_threshold=.
        results=self.processor.post_process_grounded_object_detection(
            outputs,inputs["input_ids"],threshold=self.threshold,
            text_threshold=self.text_threshold,target_sizes=[(image.height,image.width)])
        return postprocess_results(results[0],image.width,image.height,self.labels)
