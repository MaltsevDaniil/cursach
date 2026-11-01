"""Validated JSON defaults converted to tokens, so CLI overrides use argparse."""
import argparse
import json
import sys


def parse_with_config(parser, argv=None):
    argv = list(sys.argv[1:] if argv is None else argv)
    probe = argparse.ArgumentParser(add_help=False)
    probe.add_argument("--config-json")
    known, _ = probe.parse_known_args(argv)
    tokens = []
    if known.config_json:
        with open(known.config_json, encoding="utf8") as stream:
            data = json.load(stream)
        if not isinstance(data, dict):
            parser.error("config must be a JSON object")
        actions = {a.dest: a for a in parser._actions if a.dest not in ("help", "config_json")}
        for key, value in data.items():
            if key not in actions:
                parser.error(f"unknown config key: {key}")
            action = actions[key]
            option = action.option_strings[0]
            if isinstance(action, argparse.BooleanOptionalAction):
                if type(value) is not bool:
                    parser.error(f"{key} must be boolean")
                tokens.append(option if value else "--no-"+option[2:])
            else:
                values = value if isinstance(value, list) else [value]
                if action.nargs == "+" and (not isinstance(value, list) or not value):
                    parser.error(f"{key} must be a nonempty array")
                if action.nargs is None and isinstance(value, list):
                    parser.error(f"{key} must be scalar")
                for item in values:
                    if action.type is int and type(item) is not int:
                        parser.error(f"{key} must contain integers")
                    if action.type is float and (isinstance(item, bool) or not isinstance(item, (int,float))):
                        parser.error(f"{key} must be numeric")
                    if action.type is None and not isinstance(item, str):
                        parser.error(f"{key} must contain strings")
                tokens += [option, *map(str, values)]
    return parser.parse_args(tokens+argv)
