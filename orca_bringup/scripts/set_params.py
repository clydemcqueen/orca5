#!/usr/bin/env python3

"""
Connect to an ArduSub instance (e.g. in BlueOS or SITL) and ensure parameters match a .parm file.

Usage:
  ./scripts/set_params.py
  ./scripts/set_params.py --file config/hw.parm --device udpin:0.0.0.0:14550
  ./scripts/set_params.py --dry-run
"""

import argparse
import math
import os
import sys
import time

import pymavlink.dialects.v20.ardupilotmega as apm
import pymavlink.mavutil as mavutil

# ANSI color codes for readable terminal output
COLOR_GREEN = '\033[92m'
COLOR_YELLOW = '\033[93m'
COLOR_RED = '\033[91m'
COLOR_CYAN = '\033[96m'
COLOR_BOLD = '\033[1m'
COLOR_RESET = '\033[0m'


def get_default_parm_path() -> str:
    """Find default hw.parm file path relative to this script or installed package."""
    # Check relative to script directory
    script_dir = os.path.dirname(os.path.abspath(__file__))
    candidate = os.path.normpath(os.path.join(script_dir, '..', 'config', 'hw.parm'))
    if os.path.isfile(candidate):
        return candidate

    # Check ament package share directory if available
    try:
        from ament_index_python.packages import get_package_share_directory

        share_dir = get_package_share_directory('orca_bringup')
        candidate = os.path.join(share_dir, 'config', 'hw.parm')
        if os.path.isfile(candidate):
            return candidate
    except Exception:
        pass

    return 'hw.parm'


def parse_parm_file(filepath: str) -> list[tuple[str, float]]:
    """
    Parse an ArduPilot .parm file into a list of (param_name, param_value) tuples.

    Ignores comments (# or //) and empty lines.
    """
    if not os.path.isfile(filepath):
        raise FileNotFoundError(f'Parameter file not found: {filepath}')

    params = []
    with open(filepath, 'r') as f:
        for line_num, raw_line in enumerate(f, start=1):
            line = raw_line.strip()
            if not line or line.startswith('#') or line.startswith('//'):
                continue

            # Strip inline comments
            for comment_char in ('#', '//'):
                if comment_char in line:
                    line = line.split(comment_char)[0].strip()

            if not line:
                continue

            # Handle space, tab, comma, or equal-sign delimiters
            tokens = line.replace(',', ' ').replace('=', ' ').split()
            if len(tokens) < 2:
                print(
                    f'{COLOR_YELLOW}Warning: Skipping invalid line {line_num} in {filepath}: {raw_line.strip()}{COLOR_RESET}'
                )
                continue

            param_name = tokens[0].strip()
            try:
                param_value = float(tokens[1].strip())
            except ValueError:
                print(
                    f'{COLOR_YELLOW}Warning: Invalid numeric value on line {line_num} in {filepath}: {tokens[1]}{COLOR_RESET}'
                )
                continue

            params.append((param_name, param_value))

    return params


def format_val(val: float) -> str:
    """Format float cleanly as integer if it represents an integer."""
    if val.is_integer():
        return str(int(val))
    return f'{val:.4f}'.rstrip('0').rstrip('.')


def is_close(val1: float, val2: float, tolerance: float = 1e-4) -> bool:
    """Check if two parameter values are equal within tolerance."""
    return math.isclose(val1, val2, rel_tol=tolerance, abs_tol=tolerance)


class ArduSubParamClient:
    """Handles MAVLink connection and parameter operations with ArduSub."""

    def __init__(self, device: str, timeout: float = 5.0, verbose: bool = False):
        self.device = device
        self.timeout = timeout
        self.verbose = verbose
        self.conn = None
        self.target_system = 1
        self.target_component = apm.MAV_COMP_ID_AUTOPILOT1

    def connect(self) -> bool:
        """Establish MAVLink connection and wait for heartbeat."""
        print(f'Connecting to MAVLink endpoint: {COLOR_CYAN}{self.device}{COLOR_RESET}...')
        self.conn = mavutil.mavlink_connection(self.device, source_system=255, source_component=0)

        print('Waiting for heartbeat...')
        hb = self.conn.wait_heartbeat(timeout=self.timeout)
        if hb is None:
            print(f'{COLOR_RED}Error: Timed out waiting for heartbeat from {self.device}{COLOR_RESET}')
            return False

        self.target_system = hb.get_srcSystem()
        # Always target the autopilot component
        self.target_component = apm.MAV_COMP_ID_AUTOPILOT1

        autopilot_type = hb.autopilot
        type_name = 'ArduSub' if hb.type == apm.MAV_TYPE_SUBMARINE else f'Type {hb.type}'
        print(
            f'{COLOR_GREEN}Connected to {type_name} (System {self.target_system}, Component {self.target_component}, Autopilot {autopilot_type}){COLOR_RESET}'
        )
        return True

    def fetch_all_params(self) -> dict[str, tuple[float, int]]:
        """Fetch all parameters from the vehicle into a dictionary {name: (value, type)}."""
        params: dict[str, tuple[float, int]] = {}
        if self.verbose:
            print('Requesting full parameter list from vehicle...')

        self.conn.mav.param_request_list_send(self.target_system, self.target_component)
        start_time = time.time()
        param_count = None

        while time.time() - start_time < self.timeout:
            msg = self.conn.recv_match(type='PARAM_VALUE', blocking=True, timeout=0.5)
            if msg:
                p_name = msg.param_id
                params[p_name] = (msg.param_value, msg.param_type)
                param_count = msg.param_count
                start_time = time.time()  # Reset timeout while receiving packets
                if param_count and len(params) >= param_count:
                    break

        if self.verbose:
            print(f'Fetched {len(params)} parameters from vehicle.')
        return params

    def get_param(self, name: str, timeout: float = 1.0) -> tuple[float, int] | None:
        """Query a single parameter from the vehicle."""
        # Drain any residual PARAM_VALUE messages
        while self.conn.recv_match(type='PARAM_VALUE', blocking=False) is not None:
            pass

        self.conn.mav.param_request_read_send(self.target_system, self.target_component, name.encode('utf-8'), -1)
        start_time = time.time()
        while time.time() - start_time < timeout:
            msg = self.conn.recv_match(type='PARAM_VALUE', blocking=True, timeout=0.1)
            if msg and msg.param_id == name:
                return msg.param_value, msg.param_type
        return None

    def set_param(
        self,
        name: str,
        value: float,
        param_type: int = mavutil.mavlink.MAV_PARAM_TYPE_REAL32,
        retries: int = 3,
        timeout: float = 1.0,
    ) -> tuple[bool, float | None]:
        """
        Send PARAM_SET and wait for PARAM_VALUE confirmation.

        Returns (success, confirmed_value).
        """
        for attempt in range(1, retries + 1):
            # Drain any old PARAM_VALUE messages
            while self.conn.recv_match(type='PARAM_VALUE', blocking=False) is not None:
                pass

            if self.verbose:
                print(f'  Sending PARAM_SET {name}={value} (attempt {attempt}/{retries})...')

            self.conn.mav.param_set_send(
                self.target_system,
                self.target_component,
                name.encode('utf-8'),
                float(value),
                param_type,
            )

            start_time = time.time()
            while time.time() - start_time < timeout:
                msg = self.conn.recv_match(type='PARAM_VALUE', blocking=True, timeout=0.1)
                if msg and msg.param_id == name:
                    if is_close(msg.param_value, value):
                        return True, msg.param_value
                    # Received a response, but value doesn't match yet
                    if self.verbose:
                        print(f'  PARAM_VALUE for {name} returned {msg.param_value}, expected {value}')

        return False, None


def sync_parameters(
    client: ArduSubParamClient,
    target_params: list[tuple[str, float]],
    dry_run: bool = False,
    retries: int = 3,
    timeout: float = 1.0,
    quiet: bool = False,
) -> bool:
    """
    Ensure all parameters in target_params match on the vehicle.

    Returns True if all parameters match or were successfully set.
    """
    # Fetch initial parameter table from vehicle
    current_params = client.fetch_all_params()

    count_ok = 0
    count_set = 0
    count_error = 0
    count_missing = 0

    print(f'\nEvaluating {len(target_params)} parameters against vehicle state{" (DRY RUN)" if dry_run else ""}:\n')

    for name, target_val in target_params:
        if name not in current_params:
            # Try a direct query in case it was missed during batch fetch
            direct_res = client.get_param(name, timeout=timeout)
            if direct_res is not None:
                current_params[name] = direct_res

        if name not in current_params:
            print(
                f'  {COLOR_RED}[MISSING]{COLOR_RESET} {name:<16} Target: {format_val(target_val)} (Not found on vehicle)'
            )
            count_missing += 1
            continue

        curr_val, ptype = current_params[name]

        if is_close(curr_val, target_val):
            count_ok += 1
            if not quiet:
                print(f'  {COLOR_GREEN}[OK]{COLOR_RESET}      {name:<16} = {format_val(curr_val)}')
        else:
            if dry_run:
                count_set += 1
                print(
                    f'  {COLOR_YELLOW}[PLAN]{COLOR_RESET}    {name:<16} {format_val(curr_val)} -> {format_val(target_val)}'
                )
            else:
                success, confirmed_val = client.set_param(
                    name, target_val, param_type=ptype, retries=retries, timeout=timeout
                )
                if success and confirmed_val is not None:
                    count_set += 1
                    print(
                        f'  {COLOR_GREEN}[SET]{COLOR_RESET}     {name:<16} {format_val(curr_val)} -> {format_val(confirmed_val)}'
                    )
                else:
                    count_error += 1
                    print(
                        f'  {COLOR_RED}[ERROR]{COLOR_RESET}   {name:<16} Failed to set to {format_val(target_val)} (current: {format_val(curr_val)})'
                    )

    # Print summary table
    print('\n' + '=' * 45)
    print(f'{COLOR_BOLD}Parameter Synchronization Summary:{COLOR_RESET}')
    print(f'  Total parameters: {len(target_params)}')
    print(f'  Already matching: {COLOR_GREEN}{count_ok}{COLOR_RESET}')
    if dry_run:
        print(f'  To be modified:   {COLOR_YELLOW}{count_set}{COLOR_RESET}')
    else:
        print(f'  Modified / Set:   {COLOR_GREEN if count_set > 0 else COLOR_RESET}{count_set}{COLOR_RESET}')
    if count_missing > 0:
        print(f'  Missing on sub:   {COLOR_RED}{count_missing}{COLOR_RESET}')
    if count_error > 0:
        print(f'  Failed to set:    {COLOR_RED}{count_error}{COLOR_RESET}')
    print('=' * 45 + '\n')

    return (count_missing == 0) and (count_error == 0)


def main():
    default_parm_path = get_default_parm_path()

    parser = argparse.ArgumentParser(
        description='Connect to ArduSub in BlueOS / SITL and configure parameters from a .parm file.'
    )
    parser.add_argument(
        '-f',
        '--file',
        default=default_parm_path,
        help=f'Path to parameter file (default: {default_parm_path})',
    )
    parser.add_argument(
        '-d',
        '--device',
        default='udpin:0.0.0.0:14550',
        help='MAVLink device connection string (default: udpin:0.0.0.0:14550)',
    )
    parser.add_argument(
        '--dry-run',
        '--check',
        action='store_true',
        help='Compare parameters without modifying vehicle state',
    )
    parser.add_argument(
        '-t',
        '--timeout',
        type=float,
        default=5.0,
        help='MAVLink response timeout in seconds (default: 5.0)',
    )
    parser.add_argument(
        '-r',
        '--retries',
        type=int,
        default=3,
        help='Number of retries for setting a parameter (default: 3)',
    )
    parser.add_argument(
        '-q',
        '--quiet',
        action='store_true',
        help='Quiet mode: show only modifications and errors',
    )
    parser.add_argument(
        '-v',
        '--verbose',
        action='store_true',
        help='Verbose MAVLink output',
    )

    args = parser.parse_args()

    # Parse target parameter file
    try:
        target_params = parse_parm_file(args.file)
        print(f'Loaded {len(target_params)} parameters from {COLOR_CYAN}{args.file}{COLOR_RESET}')
    except Exception as e:
        print(f'{COLOR_RED}Error loading parameter file: {e}{COLOR_RESET}', file=sys.stderr)
        sys.exit(1)

    if not target_params:
        print(f'{COLOR_YELLOW}No parameters found to sync in {args.file}. Exiting.{COLOR_RESET}')
        sys.exit(0)

    # Initialize client and connect
    client = ArduSubParamClient(device=args.device, timeout=args.timeout, verbose=args.verbose)
    if not client.connect():
        sys.exit(1)

    # Run parameter sync
    success = sync_parameters(
        client=client,
        target_params=target_params,
        dry_run=args.dry_run,
        retries=args.retries,
        timeout=args.timeout,
        quiet=args.quiet,
    )

    sys.exit(0 if success else 1)


if __name__ == '__main__':
    main()
