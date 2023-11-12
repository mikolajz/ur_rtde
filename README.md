# RTDE library

A Rust library to communicate with UR robots using the RTDE protocol.

See https://www.universal-robots.com/articles/ur/interface-communication/real-time-data-exchange-rtde-guide/
for specification.

## For reviewers - design decision/missing features

* Some things might have been easier to implement using an async framework
(e.g. cancelling a wait for a new packet from the robot when we want to send
something through the socket), but since I've heard learning async is more
complex than regular Rust, so I used traditional multi-threading, at the cost
of some complex code using `mio`.
* The error handling is far from perfect - based on `String`s and doesn't
handle things like `RTDEThread` restarting the connection if something got
stuck. BTW, if there are some good example of creates doing error handling
right, I will be glad for pointers.
* I have a `Drop` trait in `RTDEThread` that tries to close the connection and
panics if it failed. However, if the drop is called due to a panic, I get a
"panic when handling panic" error with no more details. It's very hard to
debug. Are there some best practices how to avoid it? Should I just ignore
errors?

## General architecture

The library is to be built of three layers:

- `raw_rtde` - communication on the level of sending/receiving specific RTDE packets.
- `rtde_thread` - launches a thread that handles RTDE communication. Can be asked to
send a set of RTDE variables or wait until a set of variables got received. Handles
setup commands etc.
- `rtde_mux` (in progress) - The most user-friendly layer. Allows to:
  - use typed struct (with `RTDE(Read|Write)VariableSet` trait) instead of vectors of
  RTDE variables.
  - multiplex a connection, so that multiple "virtual connections" with separate
  `RTDE(Read|Write)VariableSet` can be used over a single TCP connection. This is useful
  because the number of RTDE connections is limited, so a single binary maintaining one
  connection may have different modules that want to work independently.

## Things I don't like:
* To avoid dragging <T> parameter to `RTDEWriter`, I moved some methods of `RTDEThread`
to `RTDEThreadTrait` to use `dyn RTDEThreadTrait`. However, now people need to import both
of then to use `RTDEThread`. Is there a way around it?
