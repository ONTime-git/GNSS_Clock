# UBX Protocol Parser - Implementation

This folder contains the implementation file for the UBX protocol parser, responsible for parsing UBX messages from GNSS modules.

## Contents

- **ubx_parser.c** – Implements the UBX message parser, providing functions to process incoming UBX data and extract message contents.

## Features

- Parse UBX messages from GNSS modules (e.g., u-blox DAN-F10N).
- Provide structured access to message data.
- Standardized error and result handling via `ubx_error.h`.

## Notes

- Inspired by **ubxlib**, with some code adapted and heavily modified for this project.
- Designed for STM32 microcontrollers and embedded platforms.
