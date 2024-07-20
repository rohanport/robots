#!/bin/bash
# Basic entrypoint for RxInfer Docker containers

export JULIA_PROJECT="/rxinfer"

# Execute the command passed into this entrypoint
exec "$@"
