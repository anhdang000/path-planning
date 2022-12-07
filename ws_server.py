import asyncio
import websockets
from utils import log_generator


async def handler(websocket):
    x = log_generator("transfer_data/send.txt")
    for lines in x:
        if len(lines) > 0:
            print(f'Send: {lines[-1]}')
            await websocket.send(lines[-1] + '\n')
        try:
            msg = await asyncio.wait_for(websocket.recv(), timeout=0.00001)
            print(f'Receive: {msg}')
            with open('transfer_data/receive.txt', 'w') as f:
                f.write(msg + '\n')
        except:
            pass


asyncio.get_event_loop().run_until_complete(
    websockets.serve(handler, '0.0.0.0', 8000))

asyncio.get_event_loop().run_forever()
