"""Small deterministic label/distance tracker, no appearance/person identity claim."""
import math


class ObjectTracker:
    def __init__(self,max_distance_m=.60,ttl_s=.8,ambiguity_margin_m=.15):
        if max_distance_m<=0 or not 0<ttl_s<=1 or ambiguity_margin_m<0:
            raise ValueError("Invalid tracker limits")
        self.max_distance_m=max_distance_m; self.ttl_ns=int(ttl_s*1e9)
        self.ambiguity_margin_m=ambiguity_margin_m; self.tracks={}; self.next_id=1
        self.image_stamp_ns=0; self.scan_stamp_ns=0
        self.last_batch_scan_ns=0

    def expire(self,now_ns):
        self.tracks={key:value for key,value in self.tracks.items() if 0<=now_ns-value["last_seen_ns"]<=self.ttl_ns}

    def update(self,observations,now_ns):
        self.expire(now_ns); available=set(self.tracks)
        if not observations: return
        batch_scan=max(o["last_seen_ns"] for o in observations)
        # Repeated inference on one scan cannot extend TTL or fabricate IDs.
        if batch_scan<=self.last_batch_scan_ns: return
        self.last_batch_scan_ns=batch_scan
        observations=sorted(observations,key=lambda o:(o["label"],o["position"]["x"],o["position"]["y"],-o["score"]))
        for observation in observations:
            if not 0<=now_ns-observation["last_seen_ns"]<=self.ttl_ns: continue
            p=observation["position"]
            candidates=[]
            for key in sorted(available):
                old=self.tracks[key]
                if old["label"]!=observation["label"] or observation["last_seen_ns"]<=old["last_seen_ns"]: continue
                q=old["position"]; d=math.hypot(p["x"]-q["x"],p["y"]-q["y"])
                if d<=self.max_distance_m: candidates.append((d,key))
            candidates.sort()
            # Crossing same-class tracks can be ambiguous. Allocate a new ID;
            # retain the old tracks only until TTL so callers see ambiguity.
            if candidates and (len(candidates)==1 or candidates[1][0]-candidates[0][0]>=self.ambiguity_margin_m):
                identifier=candidates[0][1]; available.remove(identifier)
            else:
                identifier=f'{observation["label"]}_{self.next_id:04d}'; self.next_id+=1
            item=dict(observation,id=identifier)
            self.tracks[identifier]=item
            self.image_stamp_ns=max(self.image_stamp_ns,item["image_stamp_ns"])
            self.scan_stamp_ns=max(self.scan_stamp_ns,item["scan_stamp_ns"])

    def scene(self,now_ns,frame_id="map",status="ok",rejections=None):
        self.expire(now_ns)
        stamp=min(self.image_stamp_ns,self.scan_stamp_ns)
        return {"schema_version":1,"stamp_ns":stamp,"published_ns":int(now_ns),
                "image_stamp_ns":self.image_stamp_ns,"scan_stamp_ns":self.scan_stamp_ns,
                "frame_id":frame_id,"status":status if self.tracks else "no_grounded_objects",
                "objects":[self.tracks[key] for key in sorted(self.tracks)],
                "rejections":dict(sorted((rejections or {}).items()))}
