idf_component_register(SRCS "gatts_server.c"
                            "main.c"
                            "time.c"
                            "wifi.c"
                            "spiffs.c"
                            "http_client.c"
                            "epd.c"
                            "ncolor_v2.c"
                            "moding_deep_sleep.c"
                            "display_pic.c"
                    INCLUDE_DIRS "."
                    INCLUDE_DIRS "include"
                    )