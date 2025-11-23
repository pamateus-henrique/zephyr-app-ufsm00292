zephyr_library()

zephyr_library_sources_ifdef(CONFIG_ETH_KSZ8851SNL eth_ksz8851snl.c)

# Undef any rogue BIT macro before Zephyr headers
zephyr_library_compile_options_ifdef(CONFIG_ETH_KSZ8851SNL -include ${CMAKE_CURRENT_LIST_DIR}/bit_sanitizer.h)

# Ensure offsets.h is generated before compiling this driver
if(CONFIG_ETH_KSZ8851SNL)
  add_dependencies(${ZEPHYR_CURRENT_LIBRARY} offsets_h)
endif()
