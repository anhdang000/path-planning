import asyncio
import websockets
import time


async def handler(websocket):
    msgs = ['25 30\n', '24 45\n', '15 45\n']
    while True:
        for msg in msgs:
            await websocket.send(msg)
            time.sleep(0.8)


async def main():
    async with websockets.serve(handler, "0.0.0.0", 8000):
        await asyncio.Future()  # run forever


asyncio.run(main())
