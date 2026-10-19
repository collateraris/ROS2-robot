"""A tiny Russian/English command language, deliberately separate from motors.

The parser does not run an LLM and does not execute text. Unknown, negated or
compound instructions cancel motion. A successfully parsed approach only sets a
goal; perception and the independent safety gate must approve every pulse.
"""

from dataclasses import dataclass
import math
import re
import unicodedata


@dataclass(frozen=True)
class Command:
    kind: str
    accepted: bool
    reason: str
    original_text: str
    standoff_m: float = 1.0

    @property
    def motion_requested(self) -> bool:
        return self.accepted and self.kind == "approach_person"

    @property
    def requires_stop(self) -> bool:
        return not self.motion_requested


_STOP = {"стоп", "остановись", "остановиться", "остановка", "stop", "отмена", "отмени команду", "прекрати движение"}
_WAIT = {"жди", "подожди", "ждать", "пауза", "wait"}
_STATUS = {"статус", "состояние", "что ты делаешь", "status"}
_NEGATION = re.compile(r"\b(?:не|ни|нет|нельзя|никогда|запрещено|запрет)\b")
_APPROACH = re.compile(
    r"(?:подъедь|подъезжай|подъехать|приблизься|подойди|приблизиться) к человеку"
    r"(?: (?:и )?(?:обходи|объезжай|обходить|объезжать|обходя) препятствия)?"
)


def parse_command(text: str, default_standoff_m: float = 1.0,
                  min_standoff_m: float = 0.8, max_standoff_m: float = 2.0) -> Command:
    """Parse one complete allowlisted instruction.

    Multiple people are intentionally not selectable in version 1. "Подъедь ко
    мне" is not equivalent to identifying the person who typed the command.
    Standoff is a reviewed numeric configuration, not a text-to-speed channel.
    Status and wait never initiate/re-arm motion. The caller should stop on
    requires_stop, including unknown instructions.
    """
    if not all(math.isfinite(x) for x in (default_standoff_m, min_standoff_m, max_standoff_m)):
        raise ValueError("Standoff distances must be finite")
    if not 0 < min_standoff_m <= default_standoff_m <= max_standoff_m:
        raise ValueError("Default standoff must lie inside the allowed positive range")
    if not isinstance(text, str):
        return Command("unknown", False, "command_must_be_text", "", default_standoff_m)
    if len(text) > 500:
        return Command("unknown", False, "command_too_long", text[:500], default_standoff_m)
    normal = unicodedata.normalize("NFKC", text).casefold().replace("ё", "е")
    if any(unicodedata.category(ch).startswith("C") and ch not in "\n\r\t" for ch in normal):
        return Command("unknown", False, "hidden_control_character", text, default_standoff_m)
    # Punctuation is allowed around ordinary phrases, but it cannot introduce a
    # second executable clause: the final match still covers the whole string.
    normal = re.sub(r"[.,!?;:]", " ", normal)
    normal = " ".join(normal.split())
    if _NEGATION.search(normal):
        return Command("stop", False, "negated_or_ambiguous_instruction", text, default_standoff_m)
    if normal in _STOP:
        return Command("stop", True, "explicit_stop", text, default_standoff_m)
    # A stop token mixed into an otherwise invalid instruction remains a stop.
    if re.search(r"\b(?:стоп|stop|остановись|отмена)\b", normal):
        return Command("stop", True, "stop_has_priority", text, default_standoff_m)
    if normal in _WAIT:
        return Command("wait", True, "hold_position", text, default_standoff_m)
    if normal in _STATUS:
        return Command("status", True, "status_does_not_start_motion", text, default_standoff_m)
    if _APPROACH.fullmatch(normal) or normal == "approach the person":
        return Command("approach_person", True, "goal_requires_unique_visible_person", text, default_standoff_m)
    return Command("unknown", False, "unsupported_instruction_stop", text, default_standoff_m)


parse = parse_command
