import contextlib
import subprocess
import sys
import types
from PIL import Image
from room_semantics.detector import canonical_label,postprocess_results,GroundingDinoDetector


def test_import_is_lazy_and_does_not_require_transformers_or_torch():
    subprocess.run([sys.executable,"-c","import sys; import room_semantics.detector; assert 'transformers' not in sys.modules; assert 'torch' not in sys.modules"],check=True)


def test_canonical_labels_and_ambiguous_class_phrase():
    assert canonical_label("a person.")=="person"
    assert canonical_label("a dining table")=="table"
    assert canonical_label("person chair") is None
    assert canonical_label("unknown") is None


def test_text_labels_are_used_without_treating_class_indices_as_names():
    result={"boxes":[[-5,10,80,120],[10,10,40,40]],"scores":[.8,.6],"text_labels":["person","unknown"]}
    assert postprocess_results(result,100,100)==[{"label":"person","score":.8,"bbox_xyxy":[0.,10.,80.,100.]}]


def test_official_transformers_4523_postprocess_signature(monkeypatch):
    calls={}
    class Batch(dict):
        def to(self,device): return self
    class Processor:
        @classmethod
        def from_pretrained(cls,*args,**kwargs): return cls()
        def __call__(self,**kwargs): return Batch(input_ids=[[1,2]])
        def post_process_grounded_object_detection(self,outputs,input_ids,*,threshold,text_threshold,target_sizes):
            calls.update(threshold=threshold,text_threshold=text_threshold,target_sizes=target_sizes)
            return [{"boxes":[[10,20,50,80]],"scores":[.9],"text_labels":["person"]}]
    class Model:
        config=types.SimpleNamespace(_commit_hash="mock-api-test")
        @classmethod
        def from_pretrained(cls,*args,**kwargs): return cls()
        def to(self,*args): return self
        def eval(self): return self
        def __call__(self,**kwargs): return object()
    fake_torch=types.SimpleNamespace(cuda=types.SimpleNamespace(is_available=lambda:False),inference_mode=contextlib.nullcontext)
    monkeypatch.setitem(sys.modules,"torch",fake_torch)
    monkeypatch.setitem(sys.modules,"transformers",types.SimpleNamespace(AutoProcessor=Processor,AutoModelForZeroShotObjectDetection=Model))
    monkeypatch.setattr("room_semantics.detector.importlib.metadata.version",lambda _:"4.52.3")
    detector=GroundingDinoDetector()
    result=detector.detect(Image.new("RGB",(100,120)))
    assert result[0]["label"]=="person"
    assert calls==dict(threshold=.35,text_threshold=.25,target_sizes=[(120,100)])
    # This tests API wiring, not a neural detector or downloaded weights.
