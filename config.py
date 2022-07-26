import socket
import struct

sock = socket.socket(socket.AF_INET,socket.SOCK_DGRAM)

while True:
    ip = input('IP: ')
    universe = input('Universe (начиная с 1): ') or '1'
    address = input('Address (начиная с 1): ') or '1'
    nodeName = input('Name: ') or 'ESPDimmer1'
    LongName = input('LongName: ') or 'ESPDimmer by Ramode'
    wifiSSID = input('SSID: ') or "RamodeART"
    wifiPass = input('PSK: ') or "45342523442"
    runMode = input('MODE: ') or 0
    write = input('Write? (y/n): ')
    if write != 'y' : continue

    # ip="192.168.117.46"
    # universe=1
    # address=2
    # nodeName="ESPLight1"
    # LongName="whatever"
    # wifiSSID ="RamodeART"
    # wifiPass ="45342523442"
    # break

    address = (int(address)-1).to_bytes(2, 'little')
    universe = (int(universe)-1).to_bytes(2, 'little')

    nodeName = (nodeName.encode()+b'\0'*18)[:18]
    LongName = (LongName.encode()+b'\0'*64)[:64]
    wifiSSID = (wifiSSID.encode()+b'\0'*40)[:40]
    wifiPass = (wifiPass.encode()+b'\0'*40)[:40]

    runMode = int(runMode)

    data = b'Art-Net\0\x17\x60'+(14).to_bytes(2, 'little') +b'ER0\0'+universe+address+nodeName+LongName+wifiSSID+wifiPass+runMode.to_bytes(1,'little')

    sock.sendto(data , (ip , 6454))
