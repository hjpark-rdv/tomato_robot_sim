#!/usr/bin/env python3
"""Persistent bounded newline-JSON OMPL service; stdout is protocol only."""
import argparse
import json
from pathlib import Path
import sys
import traceback

from newton_ompl import Planner


def emit(value):
    print(json.dumps(value, allow_nan=False, separators=(',', ':')), flush=True)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('assets', help='Selected up/down asset directory containing inputs/')
    args = parser.parse_args()
    try:
        assets = Path(args.assets)
        planner = Planner.from_inputs(assets / 'inputs')
    except Exception as error:
        emit(dict(kind='error', error=str(error)))
        traceback.print_exc(file=sys.stderr)
        return 1
    emit(planner.ready())
    while True:
        line = sys.stdin.buffer.readline(1024 * 1024 + 1)
        if not line:
            break
        if len(line) > 1024 * 1024:
            emit(dict(id=None, success=False, error='Request exceeds 1MiB'))
            return 1
        request = None
        try:
            request = json.loads(line)
            if not isinstance(request, dict):
                raise ValueError('Object request required')
            emit(planner.plan(request))
        except Exception as error:
            emit(dict(id=request.get('id') if isinstance(request, dict) else None,
                      success=False, trajectory=None, error=str(error)))
            traceback.print_exc(file=sys.stderr)
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
