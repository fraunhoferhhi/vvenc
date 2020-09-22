cmake_host_system_information( RESULT num_cores QUERY NUMBER_OF_LOGICAL_CORES )
message( STATUS "${num_cores}" )
