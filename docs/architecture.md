# Architecture

This document will describe how the modules of the simulator fit together. It
will name each layer, state what the layer owns, and state which way the
dependencies point. It will record the fixed step rate of the physics loop and
the frame budget of the render loop. It will not hold physics equations, which
belong in `docs/flight-model.md`. Read `docs/CONVENTIONS.md` section 4 first,
because the separation rule shapes every choice here.

## Module map

## Layer rules and allowed imports

## The main loop

## Fixed step physics and variable rate render

## Time, step size, and catch up behavior

## Data flow from input to render

## State ownership

## Coordinate frame conversion at the render edge

## Startup and shutdown

## The Node flight test harness

## Open questions
