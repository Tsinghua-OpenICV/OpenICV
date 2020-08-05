# Code style for OpenICV


- All classes are named in `icv` + Pascal style class name
- All public classes are directly under `icv` namespace
- Header files are guarded by `icv` + Pascal style class name + `_h`
- Use space to indent
- Brackets are always put at a new line
- Do not use `using` in header files to prevent it mess up users' code environment
- If an object has a lifetime associated with another object and you are able to manage its lifetime, then use raw pointers. Otherwise use `boost::shared_ptr`.
  - Depend on `shared_ptr` too much will make you unable to find where the memory leaks happened and influence performance.
- Use `boost` modules for compiler compatibility while use `std` modules for less dependencies and binary size
- Test cases are constructed for every function, test suites are constructed for every classes and test modules are constructed for every module (main folders).
- Test files are named as `<class>.test.cxx`