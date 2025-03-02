from pybricks.hubs import InventorHub
from pybricks.tools import wait


async def show(hub,start):
    print("show")
    i=start
    while i>0:
        hub.display.number(i)
        await wait(1000)
        i -= 1
    print("show end")
