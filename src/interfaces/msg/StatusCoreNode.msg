# StatusCoreNode.msg is a message that should be included in every node status message as its base bloc.
std_msgs/Header header # Header of the message, there should always be one to get a timestamp.
# Fields related to the chronometer parameters of the node.
builtin_interfaces/Time date_instanciated # The date at which the node was instanciated.
builtin_interfaces/Duration uptime # The duration between now and the moment the node was created.
bool is_node_enabled # Whether or not the node itself is enabled.
builtin_interfaces/Time date_enabled # The date at which the node was enabled. Empty if disabled.
builtin_interfaces/Duration runtime # The duration between now and the last time the node was enabled. Empty if disabled.
builtin_interfaces/Duration execution_time # The duration of each iteration of the node's main loop.
# Fields related to other aspects of the node.
bool is_debug_enabled # Whether or not the debug mode of the mode is enabled.