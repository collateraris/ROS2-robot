"""Pure command/parameter helpers, usable without ROS."""
from copy import deepcopy
from math import isfinite

STOP_WORDS = {'стоп', 'остановись', 'stop', 'отмена', 'cancel'}
PREFIXES = ('подъедь к ', 'езжай к ', 'поезжай к ', 'иди к ', 'к ',
            'подъедь в ', 'езжай в ', 'поезжай в ', 'go to ', 'goto ')


def normalize(text):
    return ' '.join(str(text).casefold().replace('ё', 'е').strip().split()).rstrip('.!')


def parse_command(text, waypoints):
    """Return (stop|waypoint|unknown, canonical_name|None). Exact alias grammar."""
    command = normalize(text)
    if command in STOP_WORDS:
        return 'stop', None
    target = command
    for prefix in PREFIXES:
        if target.startswith(prefix):
            target = target[len(prefix):]
            break
    for name, spec in waypoints.items():
        aliases = [name] + spec.get('aliases', [])
        if target in {normalize(alias) for alias in aliases}:
            return 'waypoint', name
    return 'unknown', None


def validate_waypoints(document):
    result = document.get('waypoints', {})
    if not isinstance(result, dict) or not result:
        raise ValueError('waypoints must be a nonempty dictionary')
    seen = set()
    for name, spec in result.items():
        if not isinstance(name, str) or not isinstance(spec, dict):
            raise ValueError('invalid waypoint')
        pose = spec.get('pose')
        if not isinstance(pose, list) or len(pose) != 3:
            raise ValueError(f'{name}: pose must be [x_m, y_m, yaw_rad]')
        if not all(isinstance(v, (float, int)) and not isinstance(v, bool) and isfinite(v)
                   for v in pose):
            raise ValueError(f'{name}: nonfinite or invalid pose')
        if not isinstance(spec.get('aliases', []), list):
            raise ValueError(f'{name}: aliases must be a list')
        for alias in [name] + spec.get('aliases', []):
            if not isinstance(alias, str) or not normalize(alias):
                raise ValueError(f'{name}: invalid alias')
            normalized = normalize(alias)
            if normalized in seen or normalized in STOP_WORDS:
                raise ValueError(f'duplicate/reserved waypoint alias: {alias}')
            seen.add(normalized)
    return result


def deep_merge(base, override):
    result = deepcopy(base)
    for key, value in override.items():
        if isinstance(value, dict) and isinstance(result.get(key), dict):
            result[key] = deep_merge(result[key], value)
        else:
            result[key] = deepcopy(value)
    return result
