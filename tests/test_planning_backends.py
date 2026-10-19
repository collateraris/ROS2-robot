import json
import pytest
from room_planning import backends
from room_planning.core import PlanRejected,parse_scene


def scene():
    return parse_scene(json.dumps({"schema_version":1,"stamp_ns":10,"image_stamp_ns":10,"scan_stamp_ns":10,
        "frame_id":"map","objects":[{"id":"chair_0001","label":"chair","score":.9,"position":{"x":1,"y":2,"z":0},
        "uncertainty_m":.1,"source":"camera_lidar","last_seen_ns":10}]}))


def test_rules_is_actual_bounded_multistep_baseline():
    assert backends.rules_plan("к стулу; подожди 2 секунды; домой",scene(),{"home"}).document()=={
        "steps":[{"action":"goto_object","object_id":"chair_0001"},{"action":"wait","seconds":2.0},
                 {"action":"goto_waypoint","name":"home"}]}
    with pytest.raises(PlanRejected):backends.rules_plan("к неизвестному объекту",scene(),set())
    with pytest.raises(PlanRejected):backends.rules_plan("подожди 999 секунд",scene(),set())


class Response:
    def __init__(self,data):self.data=data
    def __enter__(self):return self
    def __exit__(self,*args):pass
    def read(self,limit):return self.data[:limit]


def test_ollama_real_api_contract_and_strict_response(monkeypatch):
    requests=[]
    def opening(request,timeout):
        requests.append((request,timeout))
        return Response(json.dumps({"done":True,"message":{"content":'{"steps":[{"action":"goto_object","object_id":"chair_0001"}]}'}}).encode())
    monkeypatch.setattr(backends,"urlopen",opening)
    result=backends.ollama_plan("к стулу",scene(),{"home"},image_jpeg=b"JPEGfake")
    assert result.steps[0].object_id=="chair_0001"
    req,timeout=requests[0];body=json.loads(req.data)
    assert req.full_url=="http://127.0.0.1:11434/api/chat" and timeout==25
    assert body["model"]=="qwen2.5vl:7b" and body["stream"] is False
    assert body["format"]["additionalProperties"] is False
    assert body["messages"][1]["images"]==["SlBFR2Zha2U="]
    # The LLM receives identifiers and canonical labels, not privileged map XY.
    assert "position" not in body["messages"][1]["content"]


@pytest.mark.parametrize("endpoint",["https://example.org","http://192.168.1.4:11434","http://user:secret@localhost:11434","http://localhost:11434/other"])
def test_local_model_cannot_send_scene_to_arbitrary_endpoint(endpoint):
    with pytest.raises(PlanRejected):backends.ollama_plan("к стулу",scene(),set(),endpoint=endpoint)


@pytest.mark.parametrize("content",[
    '{"steps":[{"action":"goto_object","object_id":"invented"}]}',
    '{"steps":[{"action":"goto_waypoint","name":"home","x":2}]}',
    'Model reasoning: {"steps":[{"action":"stop"}]}',
])
def test_ollama_output_cannot_bypass_local_parser(monkeypatch,content):
    monkeypatch.setattr(backends,"urlopen",lambda *a,**k:Response(json.dumps({"done":True,"message":{"content":content}}).encode()))
    with pytest.raises(PlanRejected):backends.ollama_plan("к стулу",scene(),{"home"})
