/***************
CTAG TBD >>to be determined<< is an open source eurorack synthesizer module.

A project conceived within the Creative Technologies Arbeitsgruppe of
Kiel University of Applied Sciences: https://www.creative-technologies.de

(c) 2020 by Robert Manzke. All rights reserved.

The CTAG TBD software is licensed under the GNU General Public License
(GPL 3.0), available here: https://www.gnu.org/licenses/gpl-3.0.txt

***************/

#include "NetRest.hpp"
#include "DspProxy.hpp"
#include "LinkClient.hpp"
#include "ctag/link/protocol.hpp"
#include "esp_log.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "nvs_flash.h"
#include "esp_http_server.h"
#include <cstring>
#include <cstdio>
#include <string>

static const char *TAG = "NetRest";

namespace CTAG {
    namespace CTRL {

        namespace {
            constexpr const char *kApSsid = "TBD-Panel";

            // --- tiny query helpers (URIs are the WebUI's, all GET-style) ---
            // httpd's URL-value helper is config-dependent; parse the raw
            // query string ourselves (the WebUI URIs are simple key=value).
            bool queryVal(httpd_req_t *req, const char *key, char *out, size_t cap) {
                size_t qlen = httpd_req_get_url_query_len(req) + 1;
                if (qlen > 256) return false;
                char q[256] = {0};
                if (httpd_req_get_url_query_str(req, q, sizeof(q)) != ESP_OK)
                    return false;
                size_t klen = strlen(key);
                char *p = q;
                while (*p) {
                    if (strncmp(p, key, klen) == 0 && p[klen] == '=') {
                        char *v = p + klen + 1;
                        char *end = strchr(v, '&');
                        size_t n = end ? (size_t) (end - v) : strlen(v);
                        if (n >= cap) n = cap - 1;
                        memcpy(out, v, n);
                        out[n] = '\0';
                        // URL-decode %xx minimally for names
                        for (char *w = out; *w; ++w)
                            if (*w == '+' ) *w = ' ';
                        return true;
                    }
                    char *amp = strchr(p, '&');
                    if (!amp) break;
                    p = amp + 1;
                }
                return false;
            }

            bool queryInt(httpd_req_t *req, const char *key, int &out) {
                char val[16] = {0};
                if (!queryVal(req, key, val, sizeof(val))) return false;
                out = atoi(val);
                return true;
            }

            bool queryStr(httpd_req_t *req, const char *key, char *out, size_t cap) {
                return queryVal(req, key, out, cap);
            }

            void replyJson(httpd_req_t *req, const std::string &json) {
                httpd_resp_set_type(req, "application/json");
                httpd_resp_send(req, json.c_str(), (ssize_t) json.size());
            }

            void sendLink(const char *fmt, ...) {
                char buf[512];
                va_list ap;
                va_start(ap, fmt);
                vsnprintf(buf, sizeof(buf), fmt, ap);
                va_end(ap);
                LinkClient::SendCommand(buf);
            }

            // --- GET handlers: serve straight from the DspProxy cache ---
            esp_err_t hGetPlugins(httpd_req_t *req) {
                replyJson(req, DspProxy::GetCStrJSONSoundProcessors());
                return ESP_OK;
            }

            esp_err_t hGetActivePlugin(httpd_req_t *req) {
                std::string s = "{\"active0\":\"";
                s += DspProxy::GetStringID(0);
                s += "\",\"active1\":\"";
                s += DspProxy::GetStringID(1);
                s += "\",\"st0\":";
                s += DspProxy::IsPluginStereo(DspProxy::GetStringID(0)) ? "1" : "0";
                s += ",\"st1\":";
                s += DspProxy::IsPluginStereo(DspProxy::GetStringID(1)) ? "1" : "0";
                s += "}";
                replyJson(req, s);
                return ESP_OK;
            }

            esp_err_t hGetPluginParams(httpd_req_t *req) {
                int ch = 0;
                queryInt(req, "ch", ch);
                if (ch < 0 || ch > 1) ch = 0;
                replyJson(req, DspProxy::GetCStrJSONActivePluginParams(ch));
                return ESP_OK;
            }

            esp_err_t hGetPresets(httpd_req_t *req) {
                int ch = 0;
                queryInt(req, "ch", ch);
                if (ch < 0 || ch > 1) ch = 0;
                replyJson(req, DspProxy::GetCStrJSONGetPresets(ch));
                return ESP_OK;
            }

            esp_err_t hGetConfiguration(httpd_req_t *req) {
                replyJson(req, DspProxy::GetCStrJSONConfiguration());
                return ESP_OK;
            }

            esp_err_t hGetIOCaps(httpd_req_t *req) {
                replyJson(req, std::string("{\"sd\":") +
                               (DspProxy::IsSDMounted() ? "1" : "0") + "}");
                return ESP_OK;
            }

            // --- Mutation handlers: forward as link commands ---
            esp_err_t hSetActivePlugin(httpd_req_t *req) {
                int ch = 0;
                char id[64] = {0};
                queryInt(req, "ch", ch);
                queryStr(req, "id", id, sizeof(id));
                if (id[0] == '\0') {
                    httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "missing id");
                    return ESP_OK;
                }
                sendLink("{\"cmd\":\"/debug/setActivePlugin\",\"ch\":%d,\"id\":\"%s\"}",
                         ch, id);
                replyJson(req, "{\"ok\":1}");
                return ESP_OK;
            }

            esp_err_t hSetPluginParam(httpd_req_t *req) {
                int ch = 0, value = 0;
                char id[64] = {0};
                queryInt(req, "ch", ch);
                queryInt(req, "value", value);
                queryStr(req, "id", id, sizeof(id));
                if (id[0] == '\0') {
                    httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "missing id");
                    return ESP_OK;
                }
                sendLink("{\"cmd\":\"/link/paramSet\",\"ch\":%d,\"id\":\"%s\","
                         "\"key\":\"current\",\"value\":%d}", ch, id, value);
                replyJson(req, "{\"ok\":1}");
                return ESP_OK;
            }

            esp_err_t hSavePreset(httpd_req_t *req) {
                int ch = 0, number = 0;
                char name[64] = {0};
                queryInt(req, "ch", ch);
                queryInt(req, "number", number);
                queryStr(req, "name", name, sizeof(name));
                if (name[0] == '\0') {
                    httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "missing name");
                    return ESP_OK;
                }
                sendLink("{\"cmd\":\"/link/presetSave\",\"ch\":%d,\"name\":\"%s\","
                         "\"number\":%d}", ch, name, number);
                replyJson(req, "{\"ok\":1}");
                return ESP_OK;
            }

            esp_err_t hLoadPreset(httpd_req_t *req) {
                int ch = 0, number = 0;
                queryInt(req, "ch", ch);
                queryInt(req, "number", number);
                sendLink("{\"cmd\":\"/link/presetLoad\",\"ch\":%d,\"number\":%d}",
                         ch, number);
                replyJson(req, "{\"ok\":1}");
                return ESP_OK;
            }

            esp_err_t hSetConfiguration(httpd_req_t *req) {
                char body[2048] = {0};
                size_t total = 0;
                int got;
                while (total < sizeof(body) - 1 &&
                       (got = httpd_req_recv(req, &body[total],
                                             sizeof(body) - 1 - total)) > 0)
                    total += (size_t) got;
                body[total] = '\0';
                if (total == 0) {
                    httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "empty body");
                    return ESP_OK;
                }
                // configSet takes the config JSON as an escaped string.
                char framed[2304];
                snprintf(framed, sizeof(framed),
                         "{\"cmd\":\"/link/configSet\",\"config\":\"%s\"}", body);
                LinkClient::SendCommand(framed);
                replyJson(req, "{\"ok\":1}");
                return ESP_OK;
            }

            void registerGet(httpd_handle_t server, const char *uri,
                             esp_err_t (*handler)(httpd_req_t *)) {
                httpd_uri_t h = {};
                h.uri = uri;
                h.method = HTTP_GET;
                h.handler = handler;
                httpd_register_uri_handler(server, &h);
            }
        }

        void NetRest::Start() {
            esp_err_t ret = nvs_flash_init();
            if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
                ESP_ERROR_CHECK(nvs_flash_erase());
                ESP_ERROR_CHECK(nvs_flash_init());
            }
            esp_err_t err = esp_netif_init();
            if (err != ESP_ERR_INVALID_STATE) ESP_ERROR_CHECK(err);
            err = esp_event_loop_create_default();
            if (err != ESP_ERR_INVALID_STATE) ESP_ERROR_CHECK(err);

            esp_netif_create_default_wifi_ap();
            wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
            ESP_ERROR_CHECK(esp_wifi_init(&cfg));
            ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE));

            wifi_config_t wifi_config;
            memset((void *) &wifi_config, 0, sizeof(wifi_config_t));
            strcpy((char *) wifi_config.ap.ssid, kApSsid);
            wifi_config.ap.ssid_len = strlen(kApSsid);
            wifi_config.ap.max_connection = 5;
            wifi_config.ap.authmode = WIFI_AUTH_OPEN;
            ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
            ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &wifi_config));
            ESP_ERROR_CHECK(esp_wifi_start());
            ESP_LOGI(TAG, "softAP '%s' up", kApSsid);

            httpd_handle_t server = nullptr;
            httpd_config_t hc = HTTPD_DEFAULT_CONFIG();
            hc.max_uri_handlers = 12; // default 8; we register 11 routes
            hc.stack_size = 3584;     // RAM is tight alongside NimBLE
            hc.lru_purge_enable = true;
            if (httpd_start(&server, &hc) == ESP_OK) {
                registerGet(server, "/api/v1/getPlugins", hGetPlugins);
                registerGet(server, "/api/v1/getActivePlugin", hGetActivePlugin);
                registerGet(server, "/api/v1/getPluginParams", hGetPluginParams);
                registerGet(server, "/api/v1/getPresets", hGetPresets);
                registerGet(server, "/api/v1/getConfiguration", hGetConfiguration);
                registerGet(server, "/api/v1/getIOCaps", hGetIOCaps);
                registerGet(server, "/api/v1/setActivePlugin", hSetActivePlugin);
                registerGet(server, "/api/v1/setPluginParam", hSetPluginParam);
                registerGet(server, "/api/v1/savePreset", hSavePreset);
                registerGet(server, "/api/v1/loadPreset", hLoadPreset);
                static const httpd_uri_t setConfig = {
                    "/api/v1/setConfiguration", HTTP_POST, hSetConfiguration, nullptr};
                httpd_register_uri_handler(server, &setConfig);
                ESP_LOGI(TAG, "REST bridge up (%d routes)", 11);
            } else {
                ESP_LOGW(TAG, "httpd_start failed");
            }
        }
    }
}
