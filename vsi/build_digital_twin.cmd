# build_digital_twin.cmd
# Clean, consistent CAN mapping for three components.

# Add components and their ports
add component -type Python -name simulator
add port -componentName simulator -name simulator -gateway python2DtCan

add component -type Python -name controller
add port -componentName controller -name controller -gateway python2DtCan

add component -type Python -name visualizer
add port -componentName visualizer -name visualizer -gateway python2DtCan

# Define Signals (Strictly match what is published/received by components)
# Simulator Publishes: x, y, theta, t
# Simulator Listens: v, omega
define componentSignals -componentName simulator -signals "[x:double:output,y:double:output,theta:double:output,t:double:output,v:double:input,omega:double:input]"

# Controller Listens: x, y, theta
# Controller Publishes: v, omega
define componentSignals -componentName controller -signals "[x:double:input,y:double:input,theta:double:input,v:double:output,omega:double:output]"

# Visualizer Listens: x, y
define componentSignals -componentName visualizer -signals "[x:double:input,y:double:input]"

# Define Frames (Map precisely to CAN IDs defined in python files)
# x -> 12
define frame -componentName simulator -portName simulator -id 12 -signals [x:0:64]
define frame -componentName controller -portName controller -id 12 -signals [x:0:64]
define frame -componentName visualizer -portName visualizer -id 12 -signals [x:0:64]

# y -> 13
define frame -componentName simulator -portName simulator -id 13 -signals [y:0:64]
define frame -componentName controller -portName controller -id 13 -signals [y:0:64]
define frame -componentName visualizer -portName visualizer -id 13 -signals [y:0:64]

# theta -> 14
define frame -componentName simulator -portName simulator -id 14 -signals [theta:0:64]
define frame -componentName controller -portName controller -id 14 -signals [theta:0:64]

# t -> 15
define frame -componentName simulator -portName simulator -id 15 -signals [t:0:64]

# v -> 16
define frame -componentName controller -portName controller -id 16 -signals [v:0:64]
define frame -componentName simulator -portName simulator -id 16 -signals [v:0:64]

# omega -> 17
define frame -componentName controller -portName controller -id 17 -signals [omega:0:64]
define frame -componentName simulator -portName simulator -id 17 -signals [omega:0:64]

# Step size & Time
set totalSimTime 40 s
set simulationStep 0.02 s

set workspaceDir .
set digitalTwinName LineFollowingDT

generate
