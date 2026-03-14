## WSV Robot – Project Root

This folder is the logical root for the **Autonomous Water Surface Cleaning Robot (AWSCR)** software stack.

Current layout:

- `CMakeLists.txt` – entry point that builds the core C++ library from `../core` and its Python bindings.
- Core “Brain” library sources are in `../core` (C++20, hardware-agnostic).
- Python SIL simulation lives in `../sim`.
- FastAPI/WebSocket telemetry bridge lives in `../bridge`.
- Python bindings for the core library live in `../bindings`.

For the purposes of your thesis and experiments, open this `wsv_robot` folder as the project root; all additional files from now on will be created under it.

