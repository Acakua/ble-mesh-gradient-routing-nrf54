@echo off
REM Gateway launcher - uses nRF Connect SDK toolchain Python 3.12
REM This Python already has pyserial, fastapi, uvicorn, networkx installed
REM Run this from the nRF terminal: .\run_gateway.bat

set NRF_PYTHON=C:\ncs\toolchains\0b393f9e1b\opt\bin\python.exe

echo [Gateway] Starting with nRF toolchain Python 3.12...
"%NRF_PYTHON%" "%~dp0Gateway.py" %*
