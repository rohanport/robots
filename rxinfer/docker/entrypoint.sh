#!/bin/bash
# Basic entrypoint for RxInfer Docker containers

export JULIA_PROJECT="/workspace/rxinfer"

# Execute the command passed into this entrypoint
exec "$@"
