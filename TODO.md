# Roadmap
- Support ROS drop-in replacement
- Support UDP/TCP/HTTP connection
- Support data recording and replay with HDF5
- Inplememnt input modes (Queue can be implemented with `boost.circular_buffer`)
- Make OpenICV compatible with `find_package(XXX COMPONENTS YYY)`
- Add connection mergers
- Add doxygen documentation
- Add spinlock for ThreadedNode
- Add `EventLoop` Node ()
- Add `Async Node` (Use language features like async/await. `boost.asio` and `boost.fiber` can be a choice, what's its importance compared with thread nodes?)
- Improve type check for data objects using `boost.conversion`
- Find where the memory leaks (should happen between data transfer)
- Shrink dependencies between components
- Add support for Joystick input
- Utilize thread/process priority of Linux System
- Add events for node operations (connected, started, stopped, etc.)
- Add openicv binary installation using CPack

# Bindings support
- Python can be included as an engine or just a function (icvPythonFunction)
- Other languages that cannot be embedded in could act as an engine and should be completely implemented

# Data types
- icvStringTensor?
- Use protobuf of protobuf-like serializer based on msgpack to support message definition. It should be implemented as simple struct and wrapped with icvPrimitiveData.

# Misc improvements
- Add function preconditions like VTK_EXPECTS
