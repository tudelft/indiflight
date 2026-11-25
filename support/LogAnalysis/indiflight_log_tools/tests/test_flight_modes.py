# Flight modes from the log: header box names (2026 port) and the 4.4 fallback table.
# Runs without blackbox_decode: python -m pytest tests/ (or python tests/test_flight_modes.py)
import pandas as pd

from indiflight_log_tools.indiflight_log_importer import IndiflightLog


def _log(parameters, rows):
    log = IndiflightLog.__new__(IndiflightLog)  # skip decoding a .bfl
    log.parameters = parameters
    log.boxNames = IndiflightLog.boxNamesFromParameters(parameters)
    log.data = pd.DataFrame(rows).set_index("loopIteration")
    return log


def test_old_log_uses_44_table():
    log = _log({"acc_1G": 2048}, [
        {"loopIteration": 0, "timeUs": 0, "flightModeFlags": 0},
        {"loopIteration": 1, "timeUs": 10, "flightModeFlags": (1 << 0) | (1 << 9)},
    ])
    assert log.boxNames is None
    flags = log._convertModeFlagsToEvents()
    assert log.modeNames(flags.loc[1, "enable"]) == ["ARM", "POSCTL"]


def test_port_log_uses_header_names_and_second_word():
    names = ["ARM", "ANGLE", "HORIZON"] + [f"B{i}" for i in range(3, 58)] + ["INDI", "POSITION CONTROL", "WAYPOINTS"]
    parameters = {  # split like the firmware does, at arbitrary boxIds
        "boxNames_0": ",".join(names[:20]),
        "boxNames_20": ",".join(names[20:45]),
        "boxNames_45": ",".join(names[45:]),
    }
    log = _log(parameters, [
        {"loopIteration": 0, "timeUs": 0, "flightModeFlags": 0, "flightModeFlags2": 0},
        {"loopIteration": 1, "timeUs": 10, "flightModeFlags": 1, "flightModeFlags2": (1 << (58 - 32)) | (1 << (59 - 32))},
        {"loopIteration": 2, "timeUs": 20, "flightModeFlags": 1, "flightModeFlags2": 1 << (58 - 32)},
    ])
    assert log.boxNames == names
    flags = log._convertModeFlagsToEvents()
    assert log.modeNames(flags.loc[1, "enable"]) == ["ARM", "INDI", "POSITION CONTROL"]
    assert log.modeNames(flags.loc[2, "disable"]) == ["POSITION CONTROL"]


if __name__ == "__main__":
    test_old_log_uses_44_table()
    test_port_log_uses_header_names_and_second_word()
    print("ok")
