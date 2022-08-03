import asyncio
from aioconsole import ainput
from functools import reduce
import serial_asyncio
import operator

async def main():
    r,w = await serial_asyncio.open_serial_connection(url='/dev/ttyUSB2', baudrate=250000)

    async def read(r):
        while True:
            d = await r.readexactly(512)
            print(d.hex(), end='')
            
    asyncio.create_task(read(r))

    while True:
        hd = await ainput(">")

asyncio.run(main())
