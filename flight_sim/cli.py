import argparse
import processor
import sender 

def main():
    args = parse_args()
    args.func(args)

def parse_args():
    parser = argparse.ArgumentParser()

    commands = parser.add_subparsers(dest='command')
    commands.required = True

    process = commands.add_parser('process')
    process.set_defaults(func=process_data)
    process.add_argument('--imu', nargs='+', help='At least one IMU CSV file must be provided', required=True)
    process.add_argument('--baro', nargs='+', help='At least one Baro CSV file must be provided', required=True)
    process.add_argument('--acc', nargs='*', required=False)
    process.add_argument('--out', required=False)

    send = commands.add_parser('send') 
    send.set_defaults(func=send_data)
    send.add_argument('--port', required=True)
    send.add_argument('--file', required=True)

    return parser.parse_args()

def process_data(args):
    print("Processing data")
    processor.process_data(args)

def send_data(args):
    sender.send(args)

if __name__ == '__main__':
    main()