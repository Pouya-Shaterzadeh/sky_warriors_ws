#!/usr/bin/env python3
import argparse
import pytest
import sys

def main():
    parser = argparse.ArgumentParser(description="MAVSDK Failsafe Test Runner CLI")
    parser.add_argument("--all", action="store_true", help="Run all failsafe tests.")
    parser.add_argument("--test", type=str, help="Run a specific test by string matching (e.g., 'gps_loss').")
    parser.add_argument("--list", action="store_true", help="List available tests.")
    
    args = parser.parse_args()
    
    test_file = "failsafe_tests.py"

    if args.list:
        print(f"Listing available tests in {test_file}:\n")
        pytest.main(["--co", test_file])
        sys.exit(0)
    
    pytest_args = ["-v", "--tb=short", test_file]
    
    if args.test:
        print(f"\n--- Running specifically matched test: {args.test} ---\n")
        pytest_args.extend(["-k", args.test])
    elif args.all:
         print(f"\n--- Running ALL failsafe tests ---\n")
    else:
         print("\nPlease specify a test to run using --test <name> or --all\n")
         parser.print_help()
         sys.exit(1)
         
    exit_code = pytest.main(pytest_args)
    sys.exit(exit_code)

if __name__ == "__main__":
    main()
