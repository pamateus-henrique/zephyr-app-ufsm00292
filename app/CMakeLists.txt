cmake_minimum_required(VERSION 3.20.0)

# Tell Zephyr where to find our devicetree bindings
list(APPEND DTS_ROOT ${CMAKE_CURRENT_SOURCE_DIR})

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(ksz8851_app)

# Your application sources
target_sources(app PRIVATE
    src/main.c
)

# Add the driver sources directly to the app target
# This ensures they're properly linked
zephyr_library_sources_ifdef(CONFIG_ETH_KSZ8851SNL
    drivers/ethernet/eth_ksz8851snl.c
)
