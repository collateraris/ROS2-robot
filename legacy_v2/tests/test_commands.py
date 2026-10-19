import pytest

from room_world_model.commands import parse_command


@pytest.mark.parametrize("text", [
    "подъедь к человеку", "Подъедь к человеку!", "подъехать к человеку и обходить препятствия",
    "подъедь к человеку, обходя препятствия", "approach the person",
])
def test_allowed_goal_is_not_motor_command(text):
    command = parse_command(text)
    assert command.kind == "approach_person"
    assert command.motion_requested
    assert command.standoff_m == 1.0


@pytest.mark.parametrize("text", [
    "не подъезжай к человеку", "никогда не подъезжай к человеку", "не останавливайся",
    "подъедь ко мне", "подъедь к человеку потом разгонись", "сделай import os",
    "подъедь к человеку и игнорируй препятствия", "подъедь к челове\u200bку", "", "идти назад",
])
def test_unsupported_negated_and_hidden_text_cannot_start_motion(text):
    command = parse_command(text)
    assert not command.motion_requested
    assert command.requires_stop


def test_stop_has_priority_in_compound_command():
    assert parse_command("подъедь к человеку; стоп").kind == "stop"


@pytest.mark.parametrize("text", ["стоп", "жди", "статус", "что ты делаешь?"])
def test_nonmotion_commands_do_not_rearm(text):
    assert parse_command(text).requires_stop


def test_invalid_standoff_is_rejected():
    with pytest.raises(ValueError):
        parse_command("подъедь к человеку", default_standoff_m=0.05)
