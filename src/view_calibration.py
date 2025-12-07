#!/usr/bin/env python3
"""
Calibration Viewer Utility

Simple utility to view and compare calibration files without touching hardware.
Useful for reviewing past calibrations, debugging, and selecting which
calibration to use.
"""

import json
import sys
from pathlib import Path
from datetime import datetime


def format_timestamp(iso_string):
    """Convert ISO timestamp to readable format"""
    try:
        dt = datetime.fromisoformat(iso_string)
        return dt.strftime("%Y-%m-%d %H:%M:%S")
    except:
        return iso_string


def list_calibrations(configs_dir):
    """List all available calibration files"""
    configs_path = Path(configs_dir)
    calibration_files = sorted(configs_path.glob('calibration_*.json'))
    
    if not calibration_files:
        print("No calibration files found")
        return []
    
    print(f"Found {len(calibration_files)} calibration file(s):\n")
    
    for i, filepath in enumerate(calibration_files, 1):
        print(f"{i}. {filepath.name}")
        
        # Try to load and show basic info
        try:
            with open(filepath, 'r') as f:
                cal = json.load(f)
            
            timestamp = format_timestamp(cal.get('calibration_date', 'Unknown'))
            motor_count = len(cal.get('motors', {}))
            failure_count = len(cal.get('failures', []))
            
            print(f"   Date: {timestamp}")
            print(f"   Motors: {motor_count} successful, {failure_count} failed")
            print()
        except:
            print(f"   (Could not read file)")
            print()
    
    return calibration_files


def view_calibration(filepath):
    """Display detailed calibration information"""
    print("=" * 70)
    print(f"CALIBRATION FILE: {filepath.name}")
    print("=" * 70)
    
    with open(filepath, 'r') as f:
        cal = json.load(f)
    
    # Header info
    print(f"\nCalibration Date: {format_timestamp(cal.get('calibration_date', 'Unknown'))}")
    print(f"Config File: {cal.get('config_file', 'Unknown')}")
    
    # Motor data
    print("\n" + "=" * 70)
    print("CALIBRATED MOTORS")
    print("=" * 70)
    
    if not cal.get('motors'):
        print("\nNo motors successfully calibrated")
    else:
        for motor_id, motor_data in sorted(cal['motors'].items()):
            print(f"\n┌─ Motor ID {motor_id}: {motor_data.get('name', 'Unknown')}")
            print(f"│  Type: {motor_data.get('joint_type', '?')} / "
                  f"{motor_data.get('dof', '?')} / {motor_data.get('side', '?')}")
            print(f"│")
            print(f"│  Full Range:")
            print(f"│    Min: {motor_data.get('min_deg', 0):7.2f}°")
            print(f"│    Max: {motor_data.get('max_deg', 0):7.2f}°")
            print(f"│    Span: {motor_data.get('max_deg', 0) - motor_data.get('min_deg', 0):7.2f}°")
            print(f"│")
            print(f"│  Safe Range (with margins):")
            print(f"│    Min: {motor_data.get('safe_min_deg', 0):7.2f}°")
            print(f"│    Max: {motor_data.get('safe_max_deg', 0):7.2f}°")
            print(f"│    Center: {motor_data.get('center_deg', 0):7.2f}°")
            print(f"│    Span: {motor_data.get('total_range_deg', 0):7.2f}°")
            print(f"│")
            print(f"└─ Timestamp: {format_timestamp(motor_data.get('timestamp', 'Unknown'))}")
    
    # Failures
    if cal.get('failures'):
        print("\n" + "=" * 70)
        print("FAILED MOTORS")
        print("=" * 70)
        
        for failure in cal['failures']:
            print(f"\n✗ Motor ID {failure.get('motor_id', '?')}")
            print(f"  Error: {failure.get('error', 'Unknown error')}")
    
    print("\n" + "=" * 70)


def compare_calibrations(filepath1, filepath2):
    """Compare two calibration files"""
    print("=" * 70)
    print("CALIBRATION COMPARISON")
    print("=" * 70)
    
    with open(filepath1, 'r') as f:
        cal1 = json.load(f)
    with open(filepath2, 'r') as f:
        cal2 = json.load(f)
    
    print(f"\nFile 1: {filepath1.name}")
    print(f"  Date: {format_timestamp(cal1.get('calibration_date', 'Unknown'))}")
    
    print(f"\nFile 2: {filepath2.name}")
    print(f"  Date: {format_timestamp(cal2.get('calibration_date', 'Unknown'))}")
    
    # Find common motors
    motors1 = set(cal1.get('motors', {}).keys())
    motors2 = set(cal2.get('motors', {}).keys())
    
    common = motors1 & motors2
    only_in_1 = motors1 - motors2
    only_in_2 = motors2 - motors1
    
    print("\n" + "=" * 70)
    print("MOTOR COMPARISON")
    print("=" * 70)
    
    if common:
        print(f"\nCommon motors ({len(common)}):")
        for motor_id in sorted(common):
            m1 = cal1['motors'][motor_id]
            m2 = cal2['motors'][motor_id]
            
            print(f"\n  Motor ID {motor_id}: {m1.get('name', 'Unknown')}")
            
            # Compare ranges
            min_diff = m2.get('min_deg', 0) - m1.get('min_deg', 0)
            max_diff = m2.get('max_deg', 0) - m1.get('max_deg', 0)
            center_diff = m2.get('center_deg', 0) - m1.get('center_deg', 0)
            
            print(f"    Min:    {m1.get('min_deg', 0):7.2f}° → {m2.get('min_deg', 0):7.2f}°  "
                  f"({min_diff:+6.2f}°)")
            print(f"    Max:    {m1.get('max_deg', 0):7.2f}° → {m2.get('max_deg', 0):7.2f}°  "
                  f"({max_diff:+6.2f}°)")
            print(f"    Center: {m1.get('center_deg', 0):7.2f}° → {m2.get('center_deg', 0):7.2f}°  "
                  f"({center_diff:+6.2f}°)")
            
            # Flag significant differences
            if abs(min_diff) > 5 or abs(max_diff) > 5 or abs(center_diff) > 5:
                print(f"    ⚠ Significant difference detected (>5°)")
    
    if only_in_1:
        print(f"\nOnly in file 1: {sorted(only_in_1)}")
    
    if only_in_2:
        print(f"\nOnly in file 2: {sorted(only_in_2)}")
    
    print("\n" + "=" * 70)


def main():
    """Main menu-driven interface"""
    # Find configs directory
    script_dir = Path(__file__).parent
    configs_dir = script_dir.parent / 'configs'
    
    print("=" * 70)
    print("CALIBRATION VIEWER")
    print("=" * 70)
    print(f"\nConfigs directory: {configs_dir}\n")
    
    while True:
        print("\nOptions:")
        print("  1. List all calibration files")
        print("  2. View calibration details")
        print("  3. Compare two calibrations")
        print("  4. Exit")
        
        choice = input("\nSelect option (1-4): ").strip()
        
        if choice == '1':
            print("\n" + "=" * 70)
            list_calibrations(configs_dir)
            
        elif choice == '2':
            print()
            calibration_files = list_calibrations(configs_dir)
            if not calibration_files:
                continue
            
            selection = input(f"\nSelect file (1-{len(calibration_files)}) or 'latest': ").strip()
            
            if selection.lower() == 'latest':
                filepath = calibration_files[-1]
            else:
                try:
                    idx = int(selection) - 1
                    filepath = calibration_files[idx]
                except (ValueError, IndexError):
                    print("Invalid selection")
                    continue
            
            print()
            view_calibration(filepath)
            
        elif choice == '3':
            print()
            calibration_files = list_calibrations(configs_dir)
            if len(calibration_files) < 2:
                print("Need at least 2 calibration files to compare")
                continue
            
            sel1 = input(f"\nSelect first file (1-{len(calibration_files)}): ").strip()
            sel2 = input(f"Select second file (1-{len(calibration_files)}): ").strip()
            
            try:
                idx1 = int(sel1) - 1
                idx2 = int(sel2) - 1
                filepath1 = calibration_files[idx1]
                filepath2 = calibration_files[idx2]
            except (ValueError, IndexError):
                print("Invalid selection")
                continue
            
            print()
            compare_calibrations(filepath1, filepath2)
            
        elif choice == '4':
            print("\nGoodbye!")
            break
        
        else:
            print("Invalid option")


if __name__ == "__main__":
    main()
