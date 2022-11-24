import asyncio
import websockets
import time


async def handler(websocket):
    msgs = ['r 30\n', 'v 24.52 24.34\n']
    while True:
        for msg in msgs:
            await websocket.send(msg)
            time.sleep(0.1)


async def main():
    async with websockets.serve(handler, "0.0.0.0", 8000):
        await asyncio.Future()  # run forever


asyncio.run(main())
