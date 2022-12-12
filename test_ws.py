import asyncio
import websockets
import time


async def handler(websocket):
    msgs = ['v 0 0', '<90,90,90,90,2500,2500,2500,2500>']
    # msgs = ['r 45\n']
    while True:
        for msg in msgs:
            print(f'Send: {msg}')
            await websocket.send(msg)
            time.sleep(0.5)


async def main():
    async with websockets.serve(handler, "0.0.0.0", 8000):
        await asyncio.Future()  # run forever


asyncio.run(main())
