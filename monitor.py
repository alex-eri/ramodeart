import asyncio
from aioconsole import ainput
from functools import reduce
import serial_asyncio
import operator
import time


async def main():
    r,w = await serial_asyncio.open_serial_connection(url='/dev/ttyUSB1', baudrate=250000)

    async def read(r):
        while True:
            d = await r.readexactly(512)
            if b'\xff' in d:
                print("\033[2J\033[1;1H", int(time.time()), d.hex())
            
    asyncio.create_task(read(r))

    while True:
        hd = await ainput(">")

asyncio.run(main())
