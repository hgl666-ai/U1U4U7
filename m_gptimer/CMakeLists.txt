idf_component_register(SRCS "m_gptimer.c"
                    INCLUDE_DIRS "." 
                    REQUIRES esp_driver_gptimer m_gpio m_error)
