# TODO: This will be a Qt-based GUI app
# This is not part of the BatteryLab package when using pip install

import argparse

from BatteryLab.robots.ZaberRail import zaber_cli_app
from BatteryLab.robots.SuctionPump import suction_cli_app

def main():
    help_str = 'Welcome to BatteryLab CLI program!'
    print(help_str)
    parser = argparse.ArgumentParser(
        description="[TODO] BatteryLab CLI program parameters explanations!",
        formatter_class=argparse.RawDescriptionHelpFormatter
    )

    parser.add_argument(
        "-m",
        "--mode",
        type=str,
        default="suction",
        help="Choose which app to use. Options: 1. suction, 2. zaber_rail"
    )

    args = parser.parse_args()
    print("args.mode: ", args.mode)
    if args.mode == 'suction':
        suction_cli_app()
    elif args.mode == 'zaber':
        zaber_cli_app()
    else:
        print("Your options are not valid! Program will exit!")

if __name__ == '__main__':
    main()