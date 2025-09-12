import asyncio
from typing import Set
from fastapi import FastAPI, WebSocket, WebSocketDisconnect, Depends, HTTPException
from fastapi.security import OAuth2PasswordRequestForm
from pydantic import BaseModel

from .mqtt_client import MQTTClient
from .auth import (
    Token,
    authenticate_user,
    create_token,
    decode_token,
    get_current_user,
    require_role,
    User,
)

broker = "broker.emqx.io"
port = 1883
topic_from_laptop = "raspberry/laptop_to_pi"
topic_to_laptop = "raspberry/pi_to_laptop"

app = FastAPI()
mqtt_client = MQTTClient(broker, port, topic_from_laptop, topic_to_laptop)
subscribers: Set[WebSocket] = set()
_event_loop: asyncio.AbstractEventLoop | None = None


class PublishRequest(BaseModel):
    message: str


@app.on_event("startup")
async def startup_event() -> None:
    global _event_loop
    _event_loop = asyncio.get_event_loop()
    mqtt_client.set_message_callback(handle_mqtt_message)
    mqtt_client.start()


@app.on_event("shutdown")
async def shutdown_event() -> None:
    mqtt_client.stop()


def handle_mqtt_message(message: str) -> None:
    if _event_loop is None:
        return
    for ws in list(subscribers):
        try:
            asyncio.run_coroutine_threadsafe(ws.send_text(message), _event_loop)
        except Exception:
            pass


@app.post("/token", response_model=Token)
async def login(form_data: OAuth2PasswordRequestForm = Depends()) -> Token:
    user = authenticate_user(form_data.username, form_data.password)
    if not user:
        raise HTTPException(status_code=400, detail="Incorrect username or password")
    return Token(access_token=create_token(user))


@app.post("/publish")
async def publish(req: PublishRequest, user: User = Depends(require_role("operator"))):
    mqtt_client.publish(req.message)
    return {"status": "sent"}


@app.websocket("/ws")
async def websocket_endpoint(websocket: WebSocket, token: str):
    user = decode_token(token)
    if user is None:
        await websocket.close(code=1008)
        return
    await websocket.accept()
    subscribers.add(websocket)
    try:
        while True:
            await websocket.receive_text()
    except WebSocketDisconnect:
        subscribers.remove(websocket)
