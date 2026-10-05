# C/C++ Coding Standard

**Version:** 1.0
**Date:** 2026-10-04
**Scope:** C and C++ code in this repository. Third-party code (for example
the SLAMTEC SDK under `third_party/`) keeps its own style.

## Table of Contents

1. [Overview](#overview)
2. [File Organization](#file-organization)
3. [Naming Conventions](#naming-conventions)
4. [Formatting and Style](#formatting-and-style)
5. [Language Features](#language-features)
6. [Design Patterns](#design-patterns)
7. [Error Handling](#error-handling)
8. [Threading and Concurrency](#threading-and-concurrency)
9. [Memory Management](#memory-management)
10. [Documentation](#documentation)
11. [Standard Library First](#standard-library-first)
12. [Performance Guidelines](#performance-guidelines)
13. [Linux Kernel Driver Development](#linux-kernel-driver-development)

---

## Overview

This document defines the coding standards for C and C++ development in this project. They are designed to maintain consistency, readability, and maintainability across all components, and to rely on the C++ standard library. It is expected that all C/C++ code developed for this project follows this coding standard.

### Goals
- Maintain consistency with existing codebase patterns
- Ensure code readability and maintainability
- Facilitate code reviews and collaboration
- Reduce common programming errors
- Prefer the C++ standard library over custom foundation code

---

## File Organization

### Directory Structure
```
CMakeLists.txt            # Top-level build; see docs/cpp-build.md
cpp/
├── CMakeLists.txt
├── include/heron/        # Public headers, one subdirectory per subsystem
│   └── lidar/
├── src/                  # Implementation, one subdirectory per subsystem
│   └── lidar/            #   private headers live beside their .cpp
├── probes/               # On-target diagnostics
├── tools/                # Host-facing programs
└── tests/                # Unit tests
third_party/              # Vendor code as pinned submodules; keeps its own style
```

### File Naming
- **Header files:** Use `.h` extension
- **Implementation files:** Use `.cpp` extension
- **File names:** PascalCase matching primary class name
  - `ScanBuilder.h`, `ScanBuilder.cpp`

### Include Organization
Organize includes in the following order with blank lines between groups:

```cpp
// System/standard library includes
#include <algorithm>
#include <iostream>
#include <vector>

// Third-party library includes
#include "external_library.h"

// Project-specific includes
#include "ComponentSpecificHeader.h"

// Local includes (same directory)
#include "LocalHeader.h"
```
- **Angle brackets (<>):** for standard library headers and external dependencies (e.g., #include <vector>, #include <boost/algorithm.hpp>)
- **Double quotes (""):**  for project-specific headers within the same codebase (e.g., #include "my_module.h").
---

## Naming Conventions

### Classes and Structures
- **Classes:** PascalCase
  ```cpp
  class ScanBuilder;
  class MotorLink;
  ```

- **Structures:** PascalCase (same as classes)
  ```cpp
  struct Point;
  struct WheelOdometry;
  ```

### Functions and Methods
- **Functions/Methods:** camelCase
  ```cpp
  void startMotor();
  int32_t parseResponseHeader();
  bool isConnectionValid();
  ```

- **Regular getters:** Prefix with `get` or use property name
  ```cpp
  const std::string& getName() const;
  bool isEnabled() const;
  ```

- **Regular setters:** Prefix with `set`
  ```cpp
  void setName(const std::string& name);
  void setEnabled(bool enabled);
  ```

### Variables
- **Member variables:** Prefix with `m_` + camelCase
  ```cpp
  std::string m_deviceName;
  std::shared_ptr<ScanSource> m_scanSource;
  bool m_isConnected;
  ```

- **Static members:** Prefix with `s_` + camelCase
  ```cpp
  static std::map<uintptr_t, Worker*> s_workerRegistry;
  static RequestHandler* s_singleton;
  ```

- **Local variables:** camelCase
  ```cpp
  std::string deviceId;
  int connectionCount;
  ```

- **Function parameters:** camelCase
  ```cpp
  void processData(const std::string& inputData, int bufferSize);
  ```

### Constants
- **Global constants:** Prefix with `k` + PascalCase
  ```cpp
  const int kMaxBufferSize = 1024;
  const std::string kDefaultDeviceName = "Unknown";
  ```

- **Enum values:** Prefix with enum class or component abbreviation + `_`
  ```cpp
  enum TokenType {
    TT_NUMBER,
    TT_MULTIPLY,
    TT_DIVIDE
  };
  ```

### Namespaces
- **Namespaces:** PascalCase
  ```cpp
  namespace Heron { ... }
  namespace Heron::Lidar { ... }
  ```

### Macros
- **Macros:** SCREAMING_SNAKE_CASE
  ```cpp
  #define MAX_BUFFER_SIZE 1024
  #define HERON_ASSERT(condition) ...
  ```

---

## Formatting and Style

### Indentation
- **Standard:** 2 spaces for all indentation
- **No tabs:** Never use tab characters for indentation
- **Continuation:** Align with opening parenthesis or use 4 spaces
  ```cpp
  functionCall(parameter1,
               parameter2,
               parameter3);

  // OR
  functionCall(
      parameter1,
      parameter2,
      parameter3);

  // Align with opening parenthesis or beginning boolean expression
  if (m_motorRunning &&
      m_scan.isComplete() &&
      m_scan.pointCount() > kMinPoints &&
      (now - m_lastScanTime) < kScanTimeout) {
  }
  ```
- **Access Modifiers:** Indented with 2 spaces, and 2 spaces for its contents
  ```cpp
  class ResourceManager {
    public:
      ResourceManager() : m_resource(acquireResource()) {}
      ~ResourceManager() { releaseResource(m_resource); }
    
    private:
      Resource m_resource;
  };
  ```



**Note:** Configure your editor to show whitespace and convert tabs to spaces to maintain consistency.

### Braces
- **All constructs:** K&R style (opening brace on same line)
  ```cpp
  void functionName() {
    // function body
  }

  class ClassName {
    public:
      // class members
  };

  if (condition1) {
    // body
  } else if (condition2) {
    // else if body
  } else {
    // else body
  }

  // Either cover all cases or provide default
  // Braces should always be provided in cases that declare variables.
  // Add return no break
  // Add fallthrough (C++), in pure C use // FALLTHROUGH
  switch (condition) {
    case CASE_1:
      break;
    case CASE_2:
      break;
    default:
      break;
  }


  while (condition) {
    // body
  }

  for (int i = 0; i < count; ++i) {
    // body
  }
  ```

### Line Length
- **Preferred maximum:** 100 characters
- **Hard maximum:** 120 characters
- Break long lines at logical points

### Spacing
- **Around operators:** Space before and after binary operators
  ```cpp
  result = operand1 + operand2;
  if (value == expectedValue) { ... }
  ```

- **Function calls:** No space between function name and parentheses
  ```cpp
  functionCall(parameters);
  if (condition) { ... }  // Exception: control structures have space
  ```

- **Pointer/reference declarations:** Attach to name
  ```cpp
  std::string *stringPtr;
  const std::string &stringRef;
  std::string s1, &s2;
  ```

### Blank Lines
- **Between functions:** 1 blank line
- **Between logical sections:** 1 blank line
- **After includes:** 1 blank line
- **Class sections:** 1 blank line between public/private/protected
- **No Trailing Whitespaces**
---

## Language Features

Use modern C++ features (the project builds with C++20) when they are available and improve clarity or safety.

### Header Guards
  ```cpp
  // Do not use underscores to start or end #define
  #ifndef COMPONENT_HEADER_NAME_H
  #define COMPONENT_HEADER_NAME_H

  // header content

  #endif // COMPONENT_HEADER_NAME_H
  ```


### Const Correctness
- **Methods:** Mark as const when they don't modify object state
  ```cpp
  const std::string& getName() const;
  bool isValid() const;
  ```

- **Parameters:** Use const references for non-primitive types
  ```cpp
  void processData(const std::string& data);
  void setConfiguration(const Configuration& config);
  ```

- **Member variables:** Use const for immutable data
  ```cpp
  const std::string m_componentName;
  mutable std::mutex m_mutex; // OK to lock in const methods
  ```

### Member Initialization

Initialize every member before the constructor body runs. Prefer, in order:

1. **An in-class initializer** on the declaration, for a value that is the same for every constructor
2. **The constructor's initializer list**, for a value that depends on constructor parameters
3. **The constructor body**, only when the value requires real work — a loop, a syscall, a `memcpy`, a branch

Assignment in the body is not initialization: the member is default-initialized first and
then overwritten. For a scalar with no in-class initializer that means it briefly holds an
indeterminate value, and if some other constructor forgets the assignment it keeps it.
Stating the default once on the declaration makes it a property of the class rather than of
whichever constructor happened to run. (Static analysers flag the pattern, for example
SonarQube rule `cpp:S3230`.)

- **Non-compliant** — the two constructors have to agree by inspection, and the second one forgets `m_retries`
  ```cpp
  class Connection {
    public:
      Connection(void) { m_port = 0; m_retries = 3; m_connected = false; }
      explicit Connection(uint16_t port) : m_port(port) { m_connected = false; }
    private:
      uint16_t m_port;
      uint8_t  m_retries;   // indeterminate when built from a port
      bool     m_connected;
  };
  ```

- **Compliant** — the defaults are stated once, and the parameterised constructor overrides only what it needs
  ```cpp
  class Connection {
    public:
      Connection(void) = default;
      explicit Connection(uint16_t port) : m_port(port) {}
    private:
      uint16_t m_port{0};
      uint8_t  m_retries{3};
      bool     m_connected{false};
  };
  ```


- **Only constructor parameters force a member into the initializer list.** An in-class
  initializer cannot see constructor parameters, so any value derived from one stays in the
  list. Constness is not the deciding factor: `const` members, reference members, and members
  initialized from `this` can all take in-class initializers.
  ```cpp
  const std::string m_devicePath{"/dev/rplidar"}; // fine: same for every constructor
  Widget *m_self{this};                           // legal, but see the caution below
  explicit Sink(Port &port) : m_port(port) {}     // must stay in the list: from a parameter
  ```
  Using `this` in an in-class initializer is legal but the object is still under
  construction, so store the pointer and nothing more — do not call virtual functions or
  read members declared after it.

- **Declaration order is what runs.** Initializer list entries execute in declaration order,
  not the order written, so a body assignment that read a member assigned above it can break
  when moved. Build with `-Wreorder` and treat its warnings as errors in review.

- **Container `(count, value)` constructors must keep function-call form.** Braces mean an
  initializer list, not a count.
  ```cpp
  std::vector<uint32_t> m_status = std::vector<uint32_t>(4, 0);   // four zeros
  std::vector<uint32_t> m_status{4, 0};                           // WRONG: two elements, 4 and 0
  ```
  The same trap in reverse: `m_values(0)` on a `std::vector` requests an *empty* vector, which
  default construction already gives. Delete the entry rather than converting it.

- **Divergent constructors keep their explicit entries.** The in-class initializer is the
  common default; a constructor that genuinely needs a different value still says so.

- **Empty parentheses are only removable when something already covers the member.** On a
  class type with a real default constructor, `m_x()` is redundant. On a POD struct or a raw
  array, `m_x()` *is* what zeroes it — deleting it leaves the member indeterminate. Give those
  an explicit `{}` on the declaration instead.
  ```cpp
  MotorSettings m_settings{};   // POD: braces still required
  ```

- **Preprocessor-guarded entries need the identical guard on the declaration**, or the two
  configurations disagree about which members exist.

- **In-class initializers survive value-initialization, but do not count on the zero step.**
  Whenever a constructor runs it applies the in-class initializers, so `T()` produces those
  values regardless of how the default constructor came about. Moving values onto
  declarations is therefore safe for a type used as a default argument.
  ```cpp
  struct Config { uint8_t mode{4}; };
  Config c = Config();   // mode == 4, NOT 0
  ```
  The zero-initialization step, however, is *not* universal, and this is the part worth
  getting right. Value-initialization zero-initializes the object first only when the default
  constructor is **not user-provided** — that is, implicit, or defaulted on its first
  declaration. If the class has a user-provided default constructor, there is no zero step at
  all, and any member without an in-class initializer is left indeterminate. That is exactly
  the failure this section exists to prevent, so do not let `T()` at the call site stand in for
  initializing the member.
  ```cpp
  struct UserProvided { int a; UserProvided() {} };       // a is INDETERMINATE after T()
  struct Defaulted    { int a; Defaulted() = default; };  // a is zeroed by T()
  ```
  Measured on this project's toolchain (g++ 14.2, aarch64, the trixie build container) by
  constructing each with placement-new over storage pre-filled with `0xA5`: at `-O0`
  `UserProvided` reads back `0xA5A5A5A5` and `Defaulted` reads back `0`. At `-O2`
  `UserProvided` reads back `0` as well — the compiler discards the fill as a dead store.
  Indeterminate does not mean visibly wrong: a missing initializer can pass every test in
  an optimized build and still be undefined behaviour.

  Adding `= default` does not protect in-class initializers, and removing it does not zero
  them. What it changes is aggregate status, and therefore whether positional brace
  initialization (`Config{4}`) is available to callers — an API-shape decision. That does
  differ by standard, but not because "user-declared" means anything different: the
  *aggregate definition* changed. C++17 disqualifies a class only for a user-provided,
  `explicit`, or inherited constructor, so `Config() = default;` leaves it an aggregate;
  C++20 disqualifies any user-*declared* constructor, so the same class is not one. This repo
  builds `-std=c++20`; any `static_assert` on `std::is_aggregate` is standard-dependent and
  should say so.

### Type Usage

#### Fixed-Width Integer Types
- **Required:** Use fixed-width integer types from `<cstdint>` instead of built-in types
  ```cpp
  #include <cstdint>

  // Good - explicit size and signedness
  int32_t count = 0;
  uint16_t port = 8080;
  int64_t timestamp = getCurrentTime();
  uint8_t buffer[256];

  // Avoid - platform-dependent sizes
  int count = 0;        // Could be 16, 32, or 64 bits
  long timestamp = 0;   // Could be 32 or 64 bits
  unsigned port = 8080; // Size not explicit
  ```

- **Specific type guidelines:**
  - `int8_t`/`uint8_t` for byte-sized values
  - `int16_t`/`uint16_t` for 16-bit values (ports, small counters)
  - `int32_t`/`uint32_t` for general-purpose integers
  - `int64_t`/`uint64_t` for large values (timestamps, file sizes)
  - `size_t` for array indices and memory sizes (STL compatibility)
  - `bool` for boolean values

- **Exceptions:** Platform-specific APIs that require built-in types
  ```cpp
  // OK when required by external API
  int result = some_c_api_function(param);
  ```

#### Auto Keyword
- **Use judiciously:** For complex iterator types or to not repeat types. 
  ```cpp
  // Good
  auto it = complexContainer.begin();

  // Avoid - type is not obvious
  auto result = someFunction(); // What type is returned?
  ```

#### Range-Based For Loops
- **Prefer over traditional loops:**
  ```cpp
  for (const auto& item : container) {
    processItem(item);
  }
  ```

### Virtual Functions
- **Destructors:** Always virtual in base classes
  ```cpp
  class BaseClass {
    public:
      virtual ~BaseClass() = default;
  };
  ```

- **Override keyword:** Use for overridden functions
  ```cpp
  class DerivedClass : public BaseClass {
    public:
      void virtualMethod() override;
  };
  ```

### Type Casting
- **Preferred:** Use C++-style casts instead of C-style casts for improved type safety and clarity
  ```cpp
  // Good - explicit intent and compile-time checking
  static_cast<int32_t>(floatValue);                    // Numeric conversions
  const_cast<char*>(constString);                      // Remove const/volatile
  reinterpret_cast<uint8_t*>(rawPointer);             // Low-level pointer conversion
  dynamic_cast<DerivedClass*>(basePtr);                // Runtime polymorphic casting

  // Avoid - unclear intent and potential runtime errors
  (int32_t)floatValue;                                 // C-style cast
  (char*)constString;                                  // Hides const removal
  (uint8_t*)rawPointer;                               // Unsafe pointer conversion
  ```

- **Cast selection guidelines:**
  - **`static_cast`:** For well-defined conversions (numeric, inheritance hierarchy)
  - **`const_cast`:** Only to remove const/volatile qualifiers when necessary
  - **`reinterpret_cast`:** For low-level bit pattern reinterpretation (use sparingly)
  - **`dynamic_cast`:** For safe downcasting with runtime type checking

- **Best practices:**
  ```cpp
  // Numeric conversions - use static_cast
  double value = 3.14159;
  int32_t truncated = static_cast<int32_t>(value);

  // Inheritance hierarchy - prefer static_cast when type is known
  BaseClass* base = getDerivedObject();
  DerivedClass* derived = static_cast<DerivedClass*>(base); // Known safe

  // Polymorphic downcasting - use dynamic_cast for safety
  BaseClass* unknown = getUnknownObject();
  DerivedClass* derived = dynamic_cast<DerivedClass*>(unknown);
  if (derived != nullptr) {
    // Safe to use derived
  }

  // Const removal - minimize usage, document necessity
  const char* constData = getConstString();
  char* mutableData = const_cast<char*>(constData); // Only when API requires
  ```

---

## Design Patterns


### Observer/Event Pattern
Use standard callables. A subject holds `std::function` callbacks; a consumer
that may be destroyed first unregisters, or holds a `std::weak_ptr` the
subject checks before calling.

```cpp
class ScanSource {
  public:
    using Listener = std::function<void(const Scan &)>;

    void addListener(Listener listener) {
      std::lock_guard<std::mutex> lock(m_mutex);
      m_listeners.push_back(std::move(listener));
    }

  private:
    std::mutex m_mutex;
    std::vector<Listener> m_listeners;
};
```

### Factory Pattern
Use static factory methods where appropriate:

```cpp
class EventFactory {
public:
  static std::unique_ptr<Event> createEvent(EventType type);
};
```

---

## Error Handling

### Exception Handling
- **Primary method:** Use the standard exception hierarchy from `<stdexcept>`.
  `std::runtime_error` for failures detected at run time (I/O, a device that
  does not answer), `std::invalid_argument` / `std::out_of_range` for bad
  inputs, `std::logic_error` for broken invariants. Derive a project exception
  from one of these only when callers need to catch it separately.
  ```cpp
  try {
    riskyOperation();
  } catch (const std::runtime_error& e) {
    // Log
    throw; // Re-throw if cannot handle
  }
  ```

- **Throw and catch semantics:** Throw exceptions by value and catch them by const reference; never throw or catch by pointer
  ```cpp
  throw std::runtime_error("Heron::Lidar::Driver::open() : no answer from /dev/rplidar");
  // ...
  catch (const std::runtime_error& e)
  ```

- **Message format:** Begin every message with its context, `"<namespace>::<class>::<method>() : description"`. A free function uses `"<namespace>::<function>() : description"`.
  ```cpp
  throw std::runtime_error("Heron::Lidar::RprawWriter::RprawWriter() : cannot create " + path);
  ```

- **C-style APIs:** Code that cannot throw across a boundary (a C callback, a
  vendor interface returning status codes) reports through return values, and
  checks every status it receives.

### Input Validation
- **Validate parameters:** At public API boundaries
  ```cpp
  void setBufferSize(int32_t size) {
    if (size <= 0 || size > kMaxBufferSize) {
      throw std::invalid_argument("Heron::Component::setBufferSize() : invalid buffer size");
    }
    m_bufferSize = size;
  }
  ```

---

## Threading and Concurrency

### Thread Creation
- **Use `std::jthread`** (C++20), owned by the object whose work it does. It
  joins on destruction and carries a `std::stop_token` for cooperative
  cancellation, so a thread cannot outlive its owner by accident.
  ```cpp
  class Worker {
    public:
      Worker() : m_thread([this](std::stop_token stop) { run(stop); }) {}

    private:
      void run(std::stop_token stop) {
        while (!stop.stop_requested()) {
          processData();
        }
      }

      std::jthread m_thread;  // declared last: joined first, before members it uses
  };
  ```
- **Thread names:** where a name helps debugging, set it from inside the thread
  with `pthread_setname_np()` (Linux; at most 15 characters).
- **Priority and affinity:** use `pthread_setschedparam()` and
  `pthread_setaffinity_np()` on `m_thread.native_handle()`, and check their
  return codes; a real-time policy needs privileges the process may not have.

### Synchronization
- **Locking:** `std::mutex` with RAII guards. `std::lock_guard` for one mutex,
  `std::scoped_lock` for several, `std::unique_lock` with
  `std::condition_variable`.
  ```cpp
  class ThreadSafeClass {
    public:
      void criticalSection() const {
        std::lock_guard<std::mutex> lock(m_mutex);
        // Thread-safe operations
      }

    private:
      mutable std::mutex m_mutex;
  };
  ```
- **Counting and signalling:** `std::counting_semaphore` / `std::binary_semaphore`
  (C++20) or a `std::condition_variable`; never a sleep-and-poll loop.

### Atomic Operations
- **Use std::atomic:** For simple shared variables
  ```cpp
  std::atomic<bool> m_stopRequested{false};
  std::atomic<int> m_connectionCount{0};
  ```

---

## Memory Management

### RAII Principles
- **Resource management:** Acquire in constructor, release in destructor
  ```cpp
  class ResourceManager {
    public:
      ResourceManager() : m_resource(acquireResource()) {}
      ~ResourceManager() { releaseResource(m_resource); }

    private:
      Resource m_resource;
  };
  ```

### Smart Pointers
- **Preferred approach:** Use modern C++ smart pointers for all dynamic memory management
- **Ownership:** Use appropriate smart pointer types
  ```cpp
  std::unique_ptr<Component> m_ownedComponent;
  std::shared_ptr<SharedResource> m_sharedResource;
  std::weak_ptr<Observable> m_observedObject;
  ```

- **Factory functions:** Use `std::make_unique` and `std::make_shared`
  ```cpp
  // Good - exception safe
  auto component = std::make_unique<Component>(params);
  auto shared = std::make_shared<Resource>();

  // Avoid - potential memory leak if constructor throws
  std::unique_ptr<Component> component(new Component(params));
  ```

### Raw Pointers and Manual Memory Management
- **Limited use:** Only for non-owning references or C APIs
  ```cpp
  // OK - non-owning reference
  void processObject(const Object* obj);

  // OK - C API requirement
  void callCApi(void* buffer, size_t size);
  ```

- **Avoid new/delete:** Use smart pointers instead for ownership
  ```cpp
  // Avoid - manual memory management
  Object* obj = new Object();
  // ... code that might throw
  delete obj; // May never be reached

  // Good - automatic cleanup
  auto obj = std::make_unique<Object>();
  // Automatically cleaned up when obj goes out of scope
  ```

- **Arrays:** Prefer containers over raw arrays
  ```cpp
  // Avoid
  int* array = new int[size];
  delete[] array;

  // Good
  std::vector<int> array(size);
  std::array<int, 10> fixedArray; // For compile-time known sizes
  ```

---

## Documentation

### File Headers
Use consistent file header format:

```cpp
// File        : ComponentName.h
// Author      : Author Name
// Description : Brief description of the file's purpose.
//
// Copyright (c) 2026 Chris Pane
// SPDX-License-Identifier: MIT
// Licensed under the MIT License; see LICENSE in the repository root.
```

The same block heads every `.h` and `.cpp` file, with `File` naming the file
itself. The full license text lives once, in `LICENSE`; the SPDX line is the
machine-readable form that tools such as REUSE and license scanners look for.

### Function Documentation
Document public APIs using multi-line comments:

```cpp
/**
 * Brief description of what the function does.
 *
 * Detailed description if necessary, including usage notes,
 * side effects, or important behavior.
 *
 * @param paramName Description of the parameter
 * @param anotherParam Description of another parameter
 * @return Description of return value
 * @throws ExceptionType Description of when exception is thrown
 */
void publicFunction(int paramName, const std::string& anotherParam);
```

### Inline Comments
- **Purpose:** Explain why, not what
- **Style:** Use `//` for single-line comments
  ```cpp
  // One revolution at the maximum sample rate must fit without reallocating
  int32_t bufferSize = kMaxSampleRate / kMinRotationHz;

  if (bufferSize > kMaxBufferSize) {
    // Fall back to dropping scans rather than growing without bound
    enableScanDropping();
  }
  ```

### Class Documentation
Document classes with their purpose and usage:

```cpp
/**
 * Assembles measurement nodes from the LiDAR into complete scans.
 *
 * Nodes arrive one at a time from the driver; a new scan begins at
 * each start-of-revolution flag. It handles out-of-range samples and
 * reports a scan only once a full revolution has been seen.
 *
 * Thread Safety: Not thread-safe. External synchronization required.
 *
 * Usage:
 * @code
 * ScanAccumulator accumulator;
 * accumulator.addNode(node);
 * if (accumulator.hasScan()) { publish(accumulator.takeScan()); }
 * @endcode
 */
class ScanAccumulator {
  // class implementation
};
```

---
## Standard Library First
This project has no foundation class library. Use the C++ standard library
(C++20), and POSIX only where the standard has no equivalent:

| Need | Use |
|---|---|
| Threads | `std::jthread`, owned by the object whose work it does |
| Locks and signalling | `std::mutex`, `std::lock_guard` / `std::scoped_lock`, `std::condition_variable`, `std::counting_semaphore` |
| Shared flags and counters | `std::atomic` |
| Ownership | `std::unique_ptr`, `std::shared_ptr`, `std::weak_ptr` |
| Errors | the `<stdexcept>` hierarchy |
| Time | `std::chrono::steady_clock` for intervals and timestamps; `system_clock` only for wall-clock display |
| Thread naming, priority, affinity | POSIX `pthread_*` on `native_handle()` |

Before adding a helper library of your own, check that the standard does not
already provide it; when in doubt, discuss it in review.

## Performance Guidelines

### General Principles
1. **Avoid premature optimization** - Write clear, correct code first
2. **Profile before optimizing** - Measure actual performance bottlenecks
3. **Consider algorithmic complexity** - Choose appropriate algorithms and data structures

### Specific Guidelines

#### String Handling
- **Pass by const reference:** For function parameters
  ```cpp
  void processString(const std::string& str); // Good
  void processString(std::string str);        // Avoid - unnecessary copy
  ```

- **Reserve capacity:** For strings with known approximate size
  ```cpp
  std::string result;
  result.reserve(estimatedSize);
  ```

#### Container Usage
- **Choose appropriate containers:**
  - `std::vector` for sequential access
  - `std::map` for ordered key-value pairs
  - `std::unordered_map` for fast key-value lookup
  - `std::list` only when frequent insertion/deletion in middle

- **Prefer range-based operations:**
  ```cpp
  // Good
  std::copy(source.begin(), source.end(), std::back_inserter(dest));

  // Avoid
  for (size_t i = 0; i < source.size(); ++i) {
    dest.push_back(source[i]);
  }
  ```

#### Function Calls
- **Minimize virtual function overhead:** In performance-critical loops
- **Consider inlining:** For small, frequently-called functions
  ```cpp
  inline bool isValid() const { return m_isValid; }
  ```

#### Memory Access
- **Prefer stack allocation:** For small, short-lived objects
- **Cache locality:** Group related data together in structures
- **Avoid unnecessary allocations:** In tight loops

---

## Enforcement and Tools

### Code Reviews
All code changes should be reviewed for compliance with these standards. Reviewers should check for:
- Naming convention adherence
- Formatting consistency
- Appropriate design pattern usage
- Error handling completeness
- Documentation quality

### Automated Tools
Consider using these tools to enforce standards:
- **Clang-format:** For automatic formatting
- **Clang-tidy:** For static analysis
- **Cppcheck:** For additional static analysis

### Exceptions
Deviations from these standards must be:
1. Documented with rationale
2. Approved in code review
3. Limited in scope
4. Consistent within the local context

---

## Migration Guide

### For Existing Code
When modifying existing code:
1. **Maintain local consistency** - Don't mix styles within a file
2. **Gradual improvement** - Apply standards to modified sections
3. **Document deviations** - Note where existing patterns are preserved

### For New Code
All new code must fully comply with these standards from initial implementation.

---

## Linux Kernel Driver Development

When developing Linux kernel mode drivers within this project, **follow the official Linux kernel coding standards** rather than the C++ standards defined in this document. Kernel code has different constraints and conventions that must be respected for proper integration with the kernel subsystem.

### Official Linux Kernel Coding Standard
- **Primary Reference:** [Linux Kernel Coding Style](https://www.kernel.org/doc/html/latest/process/coding-style.html)
- **Documentation:** [Linux Kernel Documentation](https://www.kernel.org/doc/html/latest/)

### Key Differences from This Standard

#### Indentation
- **Kernel standard:** 8-character tabs (not spaces)
- **Rationale:** Deep nesting indicates code that needs refactoring

#### Braces
- **Kernel standard:** K&R style with specific exceptions for functions
  ```c
  // Functions - opening brace on new line
  int function(int x)
  { 
  	body of function
  }

  // Control structures - K&R style
  if (condition) {
  	action();
  }
  ```

#### Line Length
- **Kernel standard:** 80 characters (strictly enforced)
- **Exception:** Some maintainers accept 100 characters for readability

#### Naming Conventions
- **Variables/functions:** lowercase with underscores
  ```c
  int variable_name;
  void function_name(void);
  ```
- **Macros/constants:** ALL_CAPS with underscores
  ```c
  #define MAX_BUFFER_SIZE 1024
  ```

#### Memory Management
- **Use kernel APIs:** `kmalloc()`, `kfree()`, `vmalloc()`, etc.
- **No standard library:** Cannot use `malloc()`, `free()`, or C++ features
- **GFP flags:** Always specify appropriate allocation flags
  ```c
  ptr = kmalloc(size, GFP_KERNEL);  // Can sleep
  ptr = kmalloc(size, GFP_ATOMIC);  // Atomic context
  ```

#### Data Types
- **Kernel types:** Use kernel-specific types when available
  ```c
  u8, u16, u32, u64          // Unsigned types
  s8, s16, s32, s64          // Signed types
  size_t, ssize_t            // Size types
  ```

### Kernel Driver Guidelines

#### File Organization
- **Location:** Place kernel drivers in their own directory, separate from user-space code
- **Naming:** Follow existing patterns in the codebase
- **Headers:** Use kernel-style file headers with appropriate copyright

#### Integration with User Space
- **Character devices:** Use standard Linux device interface patterns
- **IOCTL commands:** Define in shared headers between kernel and user space
- **Error handling:** Use standard Linux error codes (`-EINVAL`, `-ENOMEM`, etc.)

#### Testing and Validation
- **Kernel compliance:** Use kernel build system warnings and static analysis
- **Testing:** Test on target hardware and validate with kernel debugging tools
- **Documentation:** Follow kernel documentation standards for module parameters and interfaces

### When in Doubt
- **Consult kernel documentation** first
- **Follow existing patterns** in the Linux kernel source
- **Use kernel development tools:** `checkpatch.pl`, sparse, etc.
- **Test thoroughly** on target hardware

**Remember:** Kernel code operates in a constrained environment with different rules than user-space C++ code. Always prioritize kernel standards and best practices for driver development.

*This document is a living standard that will evolve with the codebase and development practices.*
