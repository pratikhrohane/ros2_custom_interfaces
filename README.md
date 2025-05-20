# ROS 2 Custom Messages and Services: training_interfaces & training Packages
🔗 Jump to testing directly → [Testing Steps](https://github.com/pratikhrohane/ros2_custom_interfaces?tab=readme-ov-file#running-and-testing)
---

## Overview

This repository demonstrates how to create and use **custom ROS 2 messages and services** in two packages:

- `training_interfaces`: Defines custom messages and services.
- `training`: Contains ROS 2 nodes that **publish/subscribe** to custom messages and **call/provide** custom services.

---

## Naming Conventions

- **Message files (`.msg`) and Service files (`.srv`) use CamelCase** format.  
  For example:
  - Message: `Person.msg`  
  - Service: `Value.srv`  

- C++ header files generated from these follow lowercase convention in include paths:
  - `training_interfaces/msg/person.hpp`
  - `training_interfaces/srv/value.hpp`

---

## Package Structure

### 1. `training_interfaces` (Message and Service Definitions)

- Contains:
  - `msg/Person.msg`
  - `srv/Value.srv`

- Messages are simple **data structures**; services have **Request** and **Response** parts.

#### Message: `Person.msg`

```plaintext
string name
uint8 age
````

* Fields:

  * `name` (string)
  * `age` (unsigned 8-bit integer)

#### Service: `Value.srv`

```plaintext
int64 a
int64 b
---
int64 sum
```

* Input (Request): `a`, `b` (integers)
* Output (Response): `sum` (integer)

---

## How to Build

### Step 1: Build `training_interfaces`

This package uses the ROS 2 **interface generation system**:

* In `CMakeLists.txt`, we use:

```cmake
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Person.msg"
  "srv/Value.srv"
  DEPENDENCIES std_msgs
)
```

* **Why?**

  * `rosidl_generate_interfaces` triggers generation of C++ (and other language) headers for messages and services.
  * `DEPENDENCIES std_msgs` allows you to use standard messages inside your custom messages if needed.

* In `package.xml`:

```xml
<build_depend>rosidl_default_generators</build_depend>
<exec_depend>rosidl_default_runtime</exec_depend>
<depend>std_msgs</depend>
```

* **Why?**

  * `rosidl_default_generators` is needed at build time to generate message code.
  * `rosidl_default_runtime` is needed at runtime to use the generated code.

* Build command:

```bash
colcon build --packages-select training_interfaces
```

### Step 2: Build `training` package

* Depends on `training_interfaces` for message and service headers.

* In `CMakeLists.txt`:

```cmake
find_package(rclcpp REQUIRED)
find_package(training_interfaces REQUIRED)

add_executable(person_pub_node src/person_pub_node.cpp)
ament_target_dependencies(person_pub_node rclcpp training_interfaces)
```

* **Why?**

  * `find_package(training_interfaces REQUIRED)` lets you include generated message/service headers.
  * `ament_target_dependencies` links your executable against the ROS 2 client library and generated interfaces.

* In `package.xml`:

```xml
<depend>training_interfaces</depend>
<depend>rclcpp</depend>
```

* Build command:

```bash
colcon build --packages-select training
```

---

## Running and Testing

### Source workspace:

```bash
source install/setup.bash
```

### Launch publisher and subscriber nodes

In separate terminals:

```bash
ros2 run training person_pub_node
ros2 run training person_sub_node
```

### Launch service server and client nodes

In separate terminals:

```bash
ros2 run training add_server
ros2 run training add_client
```

---

## Message & Service Syntax Reference

### Message (`Person.msg`):

```plaintext
string name
uint8 age
```

* **Basic syntax**: `type field_name` per line.
* Common types: `int32`, `float64`, `string`, `bool`, etc.

### Service (`Value.srv`):

```plaintext
int64 a
int64 b
---
int64 sum
```

* **Two parts separated by `---`**

  * Top: Request fields
  * Bottom: Response fields

---

## Example Code Snippets

### Include generated headers in C++:

```cpp
#include "training_interfaces/msg/person.hpp"
#include "training_interfaces/srv/value.hpp"
```

### Using time literals (e.g., `1s`):

```cpp
#include <chrono>
using namespace std::chrono_literals;
```

### Example of waiting for a service:

```cpp
while (!client->wait_for_service(1s)) {
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Waiting for service...");
}
```

---

## Troubleshooting

* If you get errors like `No such file or directory` for message headers:

  * Make sure to **build `training_interfaces` first**.
  * Run `source install/setup.bash` after build.
  * Use correct include paths in lowercase.

* If you get errors about `operator""s` for `1s` literal:

  * Add `#include <chrono>` and `using namespace std::chrono_literals;` in your source files.
  * Make sure `CMakeLists.txt` sets `CMAKE_CXX_STANDARD` to at least 14.

---
✅ Done!
