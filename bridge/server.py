import asyncio
from typing import List

from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.middleware.cors import CORSMiddleware
from fastapi.websockets import WebSocketState
from pydantic import BaseModel

from sim.loop import Simulation


telemetry_queue: "asyncio.Queue[dict]" = asyncio.Queue()


class MissionPolygon(BaseModel):
    vertices: List[tuple]


app = FastAPI()

app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)


@app.websocket("/ws/telemetry")
async def websocket_endpoint(ws: WebSocket) -> None:
    await ws.accept()
    try:
        while True:
            telemetry = await telemetry_queue.get()
            if ws.application_state == WebSocketState.CONNECTED:
                await ws.send_json(telemetry)
    except WebSocketDisconnect:
        ...


async def telemetry_sink(telem: dict) -> None:
    await telemetry_queue.put(telem)


async def run_simulation() -> None:
    sim = Simulation(lambda d: asyncio.create_task(telemetry_sink(d)))
    await sim.run()


def start() -> None:
    loop = asyncio.get_event_loop()
    loop.create_task(run_simulation())

