cmake_minimum_required(VERSION 3.5)

project(synthesis_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(action_msgs REQUIRED)
find_package(flexbe_msgs REQUIRED)
find_package(std_msgs REQUIRED)

set(msg_files
    "msg/SynthesisErrorCodes.msg"
    "msg/BehaviorSynthesisRequest.msg"
    "msg/SynthesisOptions.msg"
    "msg/LTLSpecification.msg"
    "msg/AutomatonState.msg"
    "msg/FSAutomaton.msg"
)

set(service_files
    "srv/GenerateLTLSpecification.srv"
    "srv/SynthesizeAutomaton.srv"
    "srv/GenerateFlexBESM.srv"
)

set(action_files
    "action/BehaviorSynthesis.action"
)

rosidl_generate_interfaces(${PROJECT_NAME}
    ${action_files}
    ${msg_files}
    ${service_files}
    ADD_LINTER_TESTS
    DEPENDENCIES builtin_interfaces action_msgs flexbe_msgs std_msgs
    )

ament_export_dependencies(rosidl_default_runtime)

ament_package()
