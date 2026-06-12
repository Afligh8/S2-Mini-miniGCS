idf_component_register(SRCS "webctrl.c"
                      INCLUDE_DIRS "include"
                      EMBED_TXTFILES "web/index.html"
                      REQUIRES crazyflie platform config esp_wifi nvs_flash esp_http_server json)
