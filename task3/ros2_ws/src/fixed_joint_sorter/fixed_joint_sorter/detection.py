import json
from collections import Counter


def parse_message(payload):
    data = json.loads(payload)
    if isinstance(data, dict):
        data = data.get("detections", data.get("objects", []))
    if not isinstance(data, list):
        raise ValueError("检测消息必须是数组")
    result = []
    for item in data:
        box = item.get("bbox", {})
        x1, y1, x2, y2 = [float(box[key]) for key in ("x1", "y1", "x2", "y2")]
        result.append({
            "class_name": str(item.get("class_name", "")),
            "class_id": int(item.get("class_id", -1)),
            "confidence": float(item.get("confidence", 0.0)),
            "center": ((x1 + x2) / 2.0, (y1 + y2) / 2.0),
            "area": max(0.0, x2 - x1) * max(0.0, y2 - y1),
        })
    return result


def layout_frame(frame, allowed_classes, min_confidence):
    """Map one frame to P1..P6: left top-down, then right top-down."""
    candidates = [d for d in frame if d["class_name"] in allowed_classes and d["confidence"] >= min_confidence]
    if len(candidates) < 6:
        return None
    # Ignore weaker duplicate boxes if YOLO emits more than six valid detections.
    candidates = sorted(candidates, key=lambda d: (d["confidence"], d["area"]), reverse=True)[:6]
    by_x = sorted(candidates, key=lambda d: d["center"][0])
    left = sorted(by_x[:3], key=lambda d: d["center"][1])
    right = sorted(by_x[3:], key=lambda d: d["center"][1])
    ordered = left + right
    return {"P{}".format(index + 1): detection["class_name"] for index, detection in enumerate(ordered)}


def classify_initial_layout(frames, allowed_classes, min_confidence, required_votes):
    """Lock each station class from repeated full-table frames."""
    votes = {"P{}".format(index): [] for index in range(1, 7)}
    for frame in frames:
        mapped = layout_frame(frame, allowed_classes, min_confidence)
        if mapped is None:
            continue
        for station, class_name in mapped.items():
            votes[station].append(class_name)
    result = {}
    for station, station_votes in votes.items():
        if station_votes:
            class_name, count = Counter(station_votes).most_common(1)[0]
            if count >= required_votes:
                result[station] = class_name
    return result
