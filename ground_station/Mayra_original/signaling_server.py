import asyncio
import websockets
import json

connected = set()

async def handler(websocket):
    connected.add(websocket)
    print("🔵 Client connected")

    try:
        async for message in websocket:
            data = json.loads(message)

            # OFFER / ANSWER / ICE
            if "offer" in data or "answer" in data or "ice" in data:
                for client in connected:
                    if client != websocket:
                        await client.send(json.dumps(data))

            # COMMANDS (zoom, record, stop)
            elif "command" in data:
                print("Forwarding command:", data["command"])
                for client in connected:
                    if client != websocket:
                        await client.send(json.dumps({"command": data["command"]}))

    except:
        pass

    finally:
        connected.remove(websocket)
        print("🔴 Client disconnected")

async def main():
    async with websockets.serve(handler, "0.0.0.0", 8765):
        print("🌐 Signaling server running on ws://0.0.0.0:8765")
        await asyncio.Future()

asyncio.run(main())
