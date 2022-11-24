import asyncio
import websockets
import time


def reading_log_files(filename):
    with open(filename, "r") as f:
        data = f.read().splitlines()
    return data


def log_generator(filename, period=0.1):
    data = reading_log_files(filename)
    while True:
        time.sleep(period)
        new_data = reading_log_files(filename)
        yield new_data[len(data):]
        data = new_data


async def handler(websocket):
    x = log_generator("transfer_data.txt")
    for lines in x:
        # msg = await websocket.recv()
        # print(f"Receive: {msg}")
        if len(lines) > 0:
            print(f"Send: {lines[-1]}")
            await websocket.send(lines[-1] + '\n')


async def main():
    async with websockets.serve(handler, "0.0.0.0", 8000):
        await asyncio.Future()  # run forever


asyncio.run(main())