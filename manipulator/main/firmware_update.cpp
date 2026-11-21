// SPDX-FileCopyrightText: 2026 MattEdm
// SPDX-License-Identifier: GPL-3.0-or-later

#include "firmware_update.h"
#include "link_task.h"
#include "omachine_update_protocol.h"

#include <stdio.h>
#include <string.h>
#include <ctype.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/event_groups.h"
#include "esp_app_desc.h"
#include "esp_app_format.h"
#include "esp_crt_bundle.h"
#include "esp_event.h"
#include "esp_http_client.h"
#include "esp_http_server.h"
#include "esp_https_ota.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_netif.h"
#include "lwip/sockets.h"
#include "esp_now.h"
#include "esp_ota_ops.h"
#include "esp_partition.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "nvs.h"

static const char *TAG = "fw-update";

#define LINK_CHANNEL         1        // ESP-NOW's channel (ESPNOW_CHANNEL in link_task.cpp)
#define WIFI_NAMESPACE       "wifi"
#define WIFI_JOIN_TIMEOUT_MS 20000
#define CONTROLLER_PROJECT   "omachine-controller"
#define HELLO_TRIES          10       // 2 s to find the controller
#define REPLY_TIMEOUT_MS     200
#define DATA_TRIES           20
#define BEGIN_TIMEOUT_MS     3000     // BEGIN puts the ODrive in IDLE before it answers
#define END_TIMEOUT_MS       5000     // END checks the whole image before it answers
#define RESTART_WAIT_MS      30000    // for the controller to come back on its new firmware
#define FLASH_SECTOR         4096     // erase size

static const uint8_t BROADCAST_MAC[ESP_NOW_ETH_ALEN] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

volatile WifiSetupState wifiSetupState = WIFI_SETUP_OFF;
volatile UpdateState updateState = UPDATE_IDLE;
volatile bool updateInProgress = false;
volatile int updateProgress = -1;
volatile uint32_t updateStepCount = 0;
static char s_step[48];
static char s_resultTitle[32];
static char s_resultLine[48];

const char *updateStep() { return s_step; }
const char *updateResultTitle() { return s_resultTitle; }
const char *updateResultLine() { return s_resultLine; }

static void setStep(const char *step, int progress = -1)
{
  snprintf(s_step, sizeof(s_step), "%s", step);
  updateProgress = progress;
  updateStepCount = updateStepCount + 1;
  ESP_LOGI(TAG, "%s", step);
}

// ---- Update frames from the controller ------------------------------------------------------

typedef struct {
  uint8_t mac[ESP_NOW_ETH_ALEN];
  om_update_header_t header;
  om_update_hello_t hello;   // HELLO_ACK only
} UpdateReply;

static QueueHandle_t s_replies;

void firmwareUpdateOnRecv(const uint8_t *mac, const uint8_t *data, int len)
{
  if (!s_replies || len < (int)sizeof(om_update_header_t)) {
    return;
  }
  UpdateReply reply = {};
  memcpy(reply.mac, mac, ESP_NOW_ETH_ALEN);
  memcpy(&reply.header, data, sizeof(reply.header));
  if (reply.header.type == OM_UPD_HELLO_ACK && len >= (int)(sizeof(om_update_header_t) + sizeof(om_update_hello_t))) {
    memcpy(&reply.hello, data + sizeof(om_update_header_t), sizeof(reply.hello));
    reply.hello.project[sizeof(reply.hello.project) - 1] = 0;
    reply.hello.version[sizeof(reply.hello.version) - 1] = 0;
  }
  xQueueSend(s_replies, &reply, 0);
}

static void addPeer(const uint8_t *mac)
{
  esp_now_peer_info_t peer = {};
  memcpy(peer.peer_addr, mac, ESP_NOW_ETH_ALEN);
  peer.channel = 0;   // the current channel
  peer.ifidx = WIFI_IF_STA;
  if (esp_now_is_peer_exist(mac)) {
    return;
  }
  esp_err_t err = esp_now_add_peer(&peer);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "esp_now_add_peer: %s", esp_err_to_name(err));
  }
}

// Sends 'frame' and waits for a reply of type 'expect' from 'from' (any sender if null), resending
// up to 'tries' times
static bool exchange(const uint8_t *to, const void *frame, size_t len, uint8_t expect, const uint8_t *from,
                     UpdateReply *reply, int timeoutMs, int tries)
{
  for (int t = 0; t < tries; t++) {
    xQueueReset(s_replies);
    esp_now_send(to, (const uint8_t *)frame, len);
    TickType_t until = xTaskGetTickCount() + pdMS_TO_TICKS(timeoutMs);
    for (;;) {
      TickType_t now = xTaskGetTickCount();
      if (now >= until || xQueueReceive(s_replies, reply, until - now) != pdTRUE) {
        break;
      }
      if (reply->header.magic == OM_UPDATE_MAGIC && reply->header.type == expect &&
          (!from || memcmp(reply->mac, from, ESP_NOW_ETH_ALEN) == 0)) {
        return true;
      }
    }
  }
  return false;
}

static bool helloController(UpdateReply *reply, int tries)
{
  addPeer(BROADCAST_MAC);
  om_update_header_t hello = {};
  hello.magic = OM_UPDATE_MAGIC;
  hello.type = OM_UPD_HELLO;
  return exchange(BROADCAST_MAC, &hello, sizeof(hello), OM_UPD_HELLO_ACK, nullptr, reply, REPLY_TIMEOUT_MS, tries);
}

// ---- Wi-Fi ----------------------------------------------------------------------------------

#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAILED_BIT    BIT1
static EventGroupHandle_t s_wifiEvents;
static volatile bool s_joining = false;
static int s_joinRetries = 0;

static void onWifiEvent(void *arg, esp_event_base_t base, int32_t id, void *data)
{
  if (base == WIFI_EVENT && id == WIFI_EVENT_STA_DISCONNECTED) {
    xEventGroupClearBits(s_wifiEvents, WIFI_CONNECTED_BIT);
    if (s_joining && ++s_joinRetries <= 3) {
      esp_wifi_connect();
    } else if (s_joining) {
      xEventGroupSetBits(s_wifiEvents, WIFI_FAILED_BIT);
    }
  } else if (base == IP_EVENT && id == IP_EVENT_STA_GOT_IP) {
    xEventGroupSetBits(s_wifiEvents, WIFI_CONNECTED_BIT);
  }
}

static bool loadWifi(char *ssid, size_t ssidLen, char *pass, size_t passLen)
{
  nvs_handle_t h;
  if (nvs_open(WIFI_NAMESPACE, NVS_READONLY, &h) != ESP_OK) {
    return false;
  }
  bool ok = nvs_get_str(h, "ssid", ssid, &ssidLen) == ESP_OK && ssid[0] != 0;
  if (ok && pass) {
    if (nvs_get_str(h, "pass", pass, &passLen) != ESP_OK) {
      pass[0] = 0;
    }
  }
  nvs_close(h);
  return ok;
}

bool wifiStoredSsid(char *ssid, size_t len)
{
  return loadWifi(ssid, len, nullptr, 0);
}

static bool joinWifi()
{
  char ssid[33] = {}, pass[65] = {};
  if (!loadWifi(ssid, sizeof(ssid), pass, sizeof(pass))) {
    return false;
  }
  char step[48];
  snprintf(step, sizeof(step), "Joining %s", ssid);
  setStep(step);

  wifi_config_t cfg = {};
  memcpy(cfg.sta.ssid, ssid, strlen(ssid));
  memcpy(cfg.sta.password, pass, strlen(pass));
  cfg.sta.threshold.authmode = pass[0] ? WIFI_AUTH_WPA_PSK : WIFI_AUTH_OPEN;
  esp_wifi_set_config(WIFI_IF_STA, &cfg);
  xEventGroupClearBits(s_wifiEvents, WIFI_CONNECTED_BIT | WIFI_FAILED_BIT);
  s_joinRetries = 0;
  s_joining = true;
  esp_wifi_connect();
  EventBits_t bits = xEventGroupWaitBits(s_wifiEvents, WIFI_CONNECTED_BIT | WIFI_FAILED_BIT, pdFALSE, pdFALSE,
                                         pdMS_TO_TICKS(WIFI_JOIN_TIMEOUT_MS));
  if (!(bits & WIFI_CONNECTED_BIT)) {
    s_joining = false;
    esp_wifi_disconnect();
    return false;
  }
  return true;
}

// Back to the link's channel, so ESP-NOW reaches the controller again
static void leaveWifi()
{
  s_joining = false;
  esp_wifi_disconnect();
  vTaskDelay(pdMS_TO_TICKS(200));
  linkRadioRestore();
}

// ---- Downloads ------------------------------------------------------------------------------

static void imageUrl(char *url, size_t len, const char *file)
{
  snprintf(url, len, "%s/%s", CONFIG_OM_UPDATE_URL, file);
}

static void httpConfig(esp_http_client_config_t *cfg, const char *url)
{
  memset(cfg, 0, sizeof(*cfg));
  cfg->url = url;
  cfg->crt_bundle_attach = esp_crt_bundle_attach;
  cfg->buffer_size = 4096;      // GitHub's redirect carries a long Location header
  cfg->buffer_size_tx = 2048;
  cfg->timeout_ms = 15000;
}

// Opens 'url', following redirects (GitHub's release links redirect to its download host).
// Returns the content length, or -1 with the client cleaned up.
static int64_t openDownload(esp_http_client_handle_t client)
{
  for (int redirects = 0; redirects < 5; redirects++) {
    if (esp_http_client_open(client, 0) != ESP_OK) {
      return -1;
    }
    int64_t length = esp_http_client_fetch_headers(client);
    int status = esp_http_client_get_status_code(client);
    if (status == 301 || status == 302 || status == 303 || status == 307 || status == 308) {
      esp_http_client_flush_response(client, nullptr);
      esp_http_client_set_redirection(client);
      continue;
    }
    if (status != 200) {
      ESP_LOGE(TAG, "HTTP %d", status);
      return -status;
    }
    return length;
  }
  return -1;
}

static bool readFully(esp_http_client_handle_t client, uint8_t *buf, int len)
{
  int got = 0;
  while (got < len) {
    int n = esp_http_client_read(client, (char *)buf + got, len - got);
    if (n <= 0) {
      return false;
    }
    got += n;
  }
  return true;
}

// Where an image keeps its esp_app_desc_t
#define APP_DESC_OFFSET (sizeof(esp_image_header_t) + sizeof(esp_image_segment_header_t))

// Fetches just the version of an image (its first few hundred bytes)
static bool fetchImageDesc(const char *file, esp_app_desc_t *desc, int *httpStatus)
{
  char url[256];
  imageUrl(url, sizeof(url), file);
  esp_http_client_config_t cfg;
  httpConfig(&cfg, url);
  esp_http_client_handle_t client = esp_http_client_init(&cfg);
  int64_t length = openDownload(client);
  *httpStatus = length < 0 ? (int)-length : 200;
  uint8_t head[APP_DESC_OFFSET + sizeof(esp_app_desc_t)];
  bool ok = length >= (int64_t)sizeof(head) && readFully(client, head, sizeof(head));
  esp_http_client_close(client);
  esp_http_client_cleanup(client);
  if (!ok) {
    return false;
  }
  memcpy(desc, head + APP_DESC_OFFSET, sizeof(*desc));
  return head[0] == ESP_IMAGE_HEADER_MAGIC && desc->magic_word == ESP_APP_DESC_MAGIC_WORD;
}

// Downloads controller.bin into this board's spare app slot (just as storage: it's never booted
// here). Returns its size, or 0.
static uint32_t downloadControllerImage(const esp_partition_t *slot)
{
  char url[256];
  imageUrl(url, sizeof(url), "controller.bin");
  esp_http_client_config_t cfg;
  httpConfig(&cfg, url);
  esp_http_client_handle_t client = esp_http_client_init(&cfg);
  int64_t length = openDownload(client);
  uint32_t written = 0;
  if (length > 0 && length <= (int64_t)slot->size) {
    static uint8_t buf[4096];
    while (written < (uint32_t)length) {
      int want = (int)((uint32_t)length - written < sizeof(buf) ? (uint32_t)length - written : sizeof(buf));
      if (!readFully(client, buf, want)) {
        break;
      }
      if ((written % FLASH_SECTOR) == 0 &&
          esp_partition_erase_range(slot, written, FLASH_SECTOR) != ESP_OK) {
        break;
      }
      if (esp_partition_write(slot, written, buf, want) != ESP_OK) {
        break;
      }
      written += want;
      updateProgress = (int)(written * 100ULL / (uint64_t)length);
    }
  }
  esp_http_client_close(client);
  esp_http_client_cleanup(client);
  return written == (uint32_t)length ? written : 0;
}

// ---- Relay to the controller ----------------------------------------------------------------

static bool sendToController(const uint8_t *mac, const esp_partition_t *slot, uint32_t size, char *why, size_t whyLen)
{
  addPeer(mac);
  UpdateReply reply;
  om_update_header_t begin = {};
  begin.magic = OM_UPDATE_MAGIC;
  begin.type = OM_UPD_BEGIN;
  begin.length = size;
  if (!exchange(mac, &begin, sizeof(begin), OM_UPD_ACK, mac, &reply, BEGIN_TIMEOUT_MS, 3) ||
      reply.header.status != OM_UPD_OK) {
    snprintf(why, whyLen, "It didn't start");
    return false;
  }

  static uint8_t frame[sizeof(om_update_header_t) + OM_UPDATE_CHUNK];
  uint32_t offset = 0;
  while (offset < size) {
    uint32_t n = size - offset < OM_UPDATE_CHUNK ? size - offset : OM_UPDATE_CHUNK;
    om_update_header_t data = {};
    data.magic = OM_UPDATE_MAGIC;
    data.type = OM_UPD_DATA;
    data.offset = offset;
    data.length = n;
    memcpy(frame, &data, sizeof(data));
    if (esp_partition_read(slot, offset, frame + sizeof(data), n) != ESP_OK) {
      snprintf(why, whyLen, "Couldn't read the image");
      return false;
    }
    if (!exchange(mac, frame, sizeof(data) + n, OM_UPD_ACK, mac, &reply, REPLY_TIMEOUT_MS, DATA_TRIES)) {
      snprintf(why, whyLen, "It stopped answering");
      return false;
    }
    if (reply.header.status != OM_UPD_OK) {
      snprintf(why, whyLen, "It couldn't write it");
      return false;
    }
    offset = reply.header.offset;   // where the controller wants to go on from
    updateProgress = (int)(offset * 100ULL / size);
  }

  om_update_header_t end = {};
  end.magic = OM_UPDATE_MAGIC;
  end.type = OM_UPD_END;
  end.length = size;
  if (!exchange(mac, &end, sizeof(end), OM_UPD_ACK, mac, &reply, END_TIMEOUT_MS, 3)) {
    snprintf(why, whyLen, "No answer at the end");
    return false;
  }
  if (reply.header.status != OM_UPD_OK) {
    snprintf(why, whyLen, "It rejected the image");
    return false;
  }
  return true;
}

static void abortController(const uint8_t *mac)
{
  om_update_header_t abort = {};
  abort.magic = OM_UPDATE_MAGIC;
  abort.type = OM_UPD_ABORT;
  UpdateReply reply;
  exchange(mac, &abort, sizeof(abort), OM_UPD_ACK, mac, &reply, REPLY_TIMEOUT_MS, 5);
}

// ---- The update -----------------------------------------------------------------------------

static void finish(bool ok, const char *title, const char *line)
{
  snprintf(s_resultTitle, sizeof(s_resultTitle), "%s", title);
  snprintf(s_resultLine, sizeof(s_resultLine), "%s", line ? line : "");
  if (ok) ESP_LOGI(TAG, "%s: %s", title, s_resultLine);
  else ESP_LOGE(TAG, "%s: %s", title, s_resultLine);
  updateInProgress = false;
  updateState = ok ? UPDATE_DONE : UPDATE_FAILED;
}

static bool installOwnImage(char *why, size_t whyLen)
{
  char url[256];
  imageUrl(url, sizeof(url), "manipulator.bin");
  esp_http_client_config_t http;
  httpConfig(&http, url);
  esp_https_ota_config_t ota = {};
  ota.http_config = &http;
  esp_https_ota_handle_t handle = nullptr;
  if (esp_https_ota_begin(&ota, &handle) != ESP_OK) {
    snprintf(why, whyLen, "Couldn't download it");
    return false;
  }
  esp_app_desc_t desc;
  if (esp_https_ota_get_img_desc(handle, &desc) != ESP_OK ||
      strcmp(desc.project_name, esp_app_get_description()->project_name) != 0) {
    esp_https_ota_abort(handle);
    snprintf(why, whyLen, "Not a manipulator image");
    return false;
  }
  esp_err_t err;
  int size = esp_https_ota_get_image_size(handle);
  while ((err = esp_https_ota_perform(handle)) == ESP_ERR_HTTPS_OTA_IN_PROGRESS) {
    if (size > 0) updateProgress = (int)(esp_https_ota_get_image_len_read(handle) * 100LL / size);
  }
  if (err != ESP_OK || !esp_https_ota_is_complete_data_received(handle)) {
    esp_https_ota_abort(handle);
    snprintf(why, whyLen, "The download broke off");
    return false;
  }
  if (esp_https_ota_finish(handle) != ESP_OK) {   // checks the image and makes it the boot image
    snprintf(why, whyLen, "The image didn't check out");
    return false;
  }
  return true;
}

static void updateTask(void *arg)
{
  char why[48] = {};
  char ssid[33];
  const char *volatile baseUrl = CONFIG_OM_UPDATE_URL;   // volatile: checked at run time, not folded away
  if (strstr(baseUrl, "OWNER/REPO")) {
    finish(false, "No update address", "Set OM_UPDATE_URL first");
    vTaskDelete(nullptr);
  }
  if (!wifiStoredSsid(ssid, sizeof(ssid))) {
    finish(false, "No Wi-Fi set up", "Use Wi-Fi setup first");
    vTaskDelete(nullptr);
  }
  uint8_t state = elements[OM_ELEMENT_STATE_INDEX];
  if (linkUp && (state == OM_STATE_AUTO || state == OM_STATE_PREAUTO)) {
    finish(false, "Not now", "Stop the machine first");
    vTaskDelete(nullptr);
  }

  setStep("Finding the controller");
  UpdateReply hello;
  if (!helloController(&hello, HELLO_TRIES) || strcmp(hello.hello.project, CONTROLLER_PROJECT) != 0) {
    finish(false, "Controller not found", "Is it on?");
    vTaskDelete(nullptr);
  }
  uint8_t controllerMac[ESP_NOW_ETH_ALEN];
  memcpy(controllerMac, hello.mac, ESP_NOW_ETH_ALEN);
  char controllerVersion[32];
  snprintf(controllerVersion, sizeof(controllerVersion), "%s", hello.hello.version);
  ESP_LOGI(TAG, "Controller " MACSTR " runs %s; this board runs %s", MAC2STR(controllerMac), controllerVersion,
           esp_app_get_description()->version);

  updateInProgress = true;   // the motor goes off: the radio is about to leave the link's channel
  if (!joinWifi()) {
    leaveWifi();
    finish(false, "Can't join Wi-Fi", ssid);
    vTaskDelete(nullptr);
  }

  // Which images are new
  setStep("Checking for updates");
  esp_app_desc_t controllerDesc, ownDesc;
  int status;
  if (!fetchImageDesc("controller.bin", &controllerDesc, &status) ||
      strcmp(controllerDesc.project_name, CONTROLLER_PROJECT) != 0) {
    leaveWifi();
    snprintf(why, sizeof(why), status == 404 ? "No release found" : status == 200 ? "Bad controller.bin" : "HTTP error %d", status);
    finish(false, "Update not found", why);
    vTaskDelete(nullptr);
  }
  if (!fetchImageDesc("manipulator.bin", &ownDesc, &status) ||
      strcmp(ownDesc.project_name, esp_app_get_description()->project_name) != 0) {
    leaveWifi();
    snprintf(why, sizeof(why), status == 404 ? "No manipulator.bin" : status == 200 ? "Bad manipulator.bin" : "HTTP error %d", status);
    finish(false, "Update not found", why);
    vTaskDelete(nullptr);
  }
  bool controllerNew = strcmp(controllerDesc.version, controllerVersion) != 0;
  bool ownNew = strcmp(ownDesc.version, esp_app_get_description()->version) != 0;
  ESP_LOGI(TAG, "Release: controller %s%s, manipulator %s%s", controllerDesc.version, controllerNew ? " (new)" : "",
           ownDesc.version, ownNew ? " (new)" : "");
  if (!controllerNew && !ownNew) {
    leaveWifi();
    finish(true, "Up to date", ownDesc.version);
    vTaskDelete(nullptr);
  }

  if (controllerNew) {
    const esp_partition_t *slot = esp_ota_get_next_update_partition(nullptr);
    setStep("Downloading (controller)", 0);
    uint32_t size = slot ? downloadControllerImage(slot) : 0;
    leaveWifi();
    if (size == 0) {
      finish(false, "Download failed", "controller.bin");
      vTaskDelete(nullptr);
    }
    setStep("Sending to the controller", 0);
    if (!sendToController(controllerMac, slot, size, why, sizeof(why))) {
      abortController(controllerMac);
      finish(false, "Controller not updated", why);
      vTaskDelete(nullptr);
    }
    setStep("Controller restarting");
    bool back = false;
    TickType_t until = xTaskGetTickCount() + pdMS_TO_TICKS(RESTART_WAIT_MS);
    while (xTaskGetTickCount() < until) {
      if (helloController(&hello, 1) && strcmp(hello.hello.version, controllerDesc.version) == 0) {
        back = true;
        break;
      }
      vTaskDelay(pdMS_TO_TICKS(300));
    }
    if (!back) {
      finish(false, "Controller not updated", "It went back to the old one");
      vTaskDelete(nullptr);
    }
  }

  if (ownNew) {
    if (!joinWifi()) {
      leaveWifi();
      finish(false, "Can't join Wi-Fi", ssid);
      vTaskDelete(nullptr);
    }
    setStep("Downloading (manipulator)", 0);
    bool ok = installOwnImage(why, sizeof(why));
    leaveWifi();
    if (!ok) {
      finish(false, "Manipulator not updated", why);
      vTaskDelete(nullptr);
    }
    finish(true, "Updated", "Restarting...");
    vTaskDelay(pdMS_TO_TICKS(3000));
    esp_restart();
  }
  finish(true, "Controller updated", controllerDesc.version);
  vTaskDelete(nullptr);
}

void firmwareUpdateStart()
{
  if (updateState == UPDATE_RUNNING) {
    return;
  }
  updateState = UPDATE_RUNNING;
  updateProgress = -1;
  s_step[0] = 0;
  xTaskCreatePinnedToCore(updateTask, "FwUpdate", 8192, nullptr, 2, nullptr, 0);
}

// ---- Wi-Fi setup page -----------------------------------------------------------------------

static httpd_handle_t s_server = nullptr;

static const char SETUP_PAGE[] =
  "<!doctype html><html><head><meta charset=utf-8><meta name=viewport content='width=device-width,initial-scale=1'>"
  "<title>omachine Wi-Fi</title><style>body{font-family:sans-serif;margin:24px;max-width:420px}"
  "input{width:100%;padding:10px;margin:6px 0 16px;font-size:16px;box-sizing:border-box}"
  "button{padding:12px 20px;font-size:16px}</style></head><body><h2>omachine Wi-Fi</h2>"
  "<p>The home network the manipulator joins to download firmware updates.</p>"
  "<form method=post action=/save><label>Network name<input name=ssid maxlength=32 required></label>"
  "<label>Password<input name=pass type=password maxlength=64></label><button>Save</button></form></body></html>";

static esp_err_t onSetupConnection(httpd_handle_t hd, int sockfd)
{
  ESP_LOGI(TAG, "Setup page: a browser connected");
  return ESP_OK;
}

static esp_err_t onSetupPage(httpd_req_t *req)
{
  ESP_LOGI(TAG, "Setup page requested (%s)", req->uri);
  httpd_resp_set_type(req, "text/html");
  return httpd_resp_send(req, SETUP_PAGE, HTTPD_RESP_USE_STRLEN);
}

// Decodes one form field (application/x-www-form-urlencoded) into 'out'
static bool formField(const char *body, const char *name, char *out, size_t outLen)
{
  size_t nameLen = strlen(name);
  const char *p = body;
  while (p && *p) {
    if (strncmp(p, name, nameLen) == 0 && p[nameLen] == '=') {
      p += nameLen + 1;
      size_t n = 0;
      while (*p && *p != '&') {
        char c = *p++;
        if (c == '+') {
          c = ' ';
        } else if (c == '%' && isxdigit((unsigned char)p[0]) && isxdigit((unsigned char)p[1])) {
          char hex[3] = {p[0], p[1], 0};
          c = (char)strtol(hex, nullptr, 16);
          p += 2;
        }
        if (n + 1 >= outLen) {
          return false;   // too long
        }
        out[n++] = c;
      }
      out[n] = 0;
      return true;
    }
    p = strchr(p, '&');
    if (p) p++;
  }
  return false;
}

// Every other address (a phone checking for internet access, e.g. /generate_204): send it to the
// setup page. A phone that sees this redirect shows "Sign in to network", which opens the page in
// a window that goes over this network even when a VPN carries everything else.
static esp_err_t onSetupRedirect(httpd_req_t *req)
{
  ESP_LOGI(TAG, "Setup page: redirecting %s", req->uri);
  httpd_resp_set_status(req, "302 Found");
  httpd_resp_set_hdr(req, "Location", "http://" WIFI_SETUP_ADDRESS "/");
  return httpd_resp_send(req, nullptr, 0);
}

// The other half of the sign-in page: a DNS server that answers every name with this board, so
// the phone's internet check comes here and gets redirected
static volatile bool s_dnsRun = false;
static int s_dnsSocket = -1;

static void dnsTask(void *arg)
{
  int sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
  struct sockaddr_in addr = {};
  addr.sin_family = AF_INET;
  addr.sin_port = htons(53);
  addr.sin_addr.s_addr = htonl(INADDR_ANY);
  if (sock < 0 || bind(sock, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
    ESP_LOGE(TAG, "Setup DNS server didn't start");
    if (sock >= 0) close(sock);
    vTaskDelete(nullptr);
  }
  struct timeval timeout = {.tv_sec = 1, .tv_usec = 0};
  setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));
  s_dnsSocket = sock;
  uint8_t buf[512];
  while (s_dnsRun) {
    struct sockaddr_in from;
    socklen_t fromLen = sizeof(from);
    int len = recvfrom(sock, buf, sizeof(buf) - 16, 0, (struct sockaddr *)&from, &fromLen);
    if (len < 12) {
      continue;   // timeout, or not a DNS query
    }
    // Find the end of the (first) question: its name, then type and class
    int pos = 12;
    while (pos < len && buf[pos] != 0) pos += buf[pos] + 1;
    pos += 5;
    if (pos > len) {
      continue;
    }
    bool typeA = buf[pos - 4] == 0 && buf[pos - 3] == 1;
    buf[2] = 0x81;   // a response, recursion desired
    buf[3] = 0x80;   // recursion available, no error
    buf[4] = 0; buf[5] = 1;                  // one question
    buf[6] = 0; buf[7] = typeA ? 1 : 0;      // one answer for an A query, none for others (AAAA etc.)
    buf[8] = buf[9] = buf[10] = buf[11] = 0;
    len = pos;
    if (typeA) {
      const uint8_t answer[] = {0xC0, 0x0C, 0, 1, 0, 1, 0, 0, 0, 60, 0, 4, 192, 168, 4, 1};
      memcpy(buf + len, answer, sizeof(answer));
      len += sizeof(answer);
    }
    sendto(sock, buf, len, 0, (struct sockaddr *)&from, fromLen);
  }
  close(sock);
  s_dnsSocket = -1;
  vTaskDelete(nullptr);
}

static esp_err_t onSetupSave(httpd_req_t *req)
{
  char body[512];
  int len = req->content_len < sizeof(body) - 1 ? req->content_len : sizeof(body) - 1;
  int got = 0;
  while (got < len) {
    int n = httpd_req_recv(req, body + got, len - got);
    if (n <= 0) {
      return ESP_FAIL;
    }
    got += n;
  }
  body[got] = 0;
  char ssid[33] = {}, pass[65] = {};
  if (!formField(body, "ssid", ssid, sizeof(ssid)) || ssid[0] == 0) {
    httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Network name missing or too long");
    return ESP_OK;
  }
  if (!formField(body, "pass", pass, sizeof(pass))) {
    pass[0] = 0;
  }
  nvs_handle_t h;
  esp_err_t err = nvs_open(WIFI_NAMESPACE, NVS_READWRITE, &h);
  if (err == ESP_OK) {
    err = nvs_set_str(h, "ssid", ssid);
    if (err == ESP_OK) err = nvs_set_str(h, "pass", pass);
    if (err == ESP_OK) err = nvs_commit(h);
    nvs_close(h);
  }
  if (err != ESP_OK) {
    httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Couldn't save it");
    return ESP_OK;
  }
  ESP_LOGI(TAG, "Wi-Fi network saved: %s", ssid);
  httpd_resp_set_type(req, "text/html");
  httpd_resp_sendstr(req, "<!doctype html><html><head><meta charset=utf-8><meta name=viewport "
                          "content='width=device-width,initial-scale=1'></head><body style='font-family:sans-serif;margin:24px'>"
                          "<h2>Saved</h2><p>You can close this page and leave the omachine-setup network.</p></body></html>");
  wifiSetupState = WIFI_SETUP_SAVED;
  return ESP_OK;
}

void wifiSetupStart()
{
  if (wifiSetupState == WIFI_SETUP_RUNNING) {
    return;
  }
  esp_wifi_set_mode(WIFI_MODE_APSTA);
  linkRadioRestore();   // the mode change can drop Long Range, which the link needs
  wifi_config_t ap = {};
  memcpy(ap.ap.ssid, WIFI_SETUP_SSID, strlen(WIFI_SETUP_SSID));
  ap.ap.ssid_len = strlen(WIFI_SETUP_SSID);
  ap.ap.channel = LINK_CHANNEL;   // the link's channel, so the link keeps working meanwhile
  ap.ap.authmode = WIFI_AUTH_OPEN;
  ap.ap.max_connection = 2;
  esp_wifi_set_config(WIFI_IF_AP, &ap);

  // Hand out this board as the network's DNS server (see dnsTask)
  esp_netif_t *apNetif = esp_netif_get_handle_from_ifkey("WIFI_AP_DEF");
  if (apNetif) {
    esp_netif_dns_info_t dns = {};
    dns.ip.type = ESP_IPADDR_TYPE_V4;
    dns.ip.u_addr.ip4.addr = ESP_IP4TOADDR(192, 168, 4, 1);
    uint8_t offerDns = 0x02;   // OFFER_DNS (lwIP dhcpserver.h)
    esp_netif_dhcps_stop(apNetif);
    esp_netif_dhcps_option(apNetif, ESP_NETIF_OP_SET, ESP_NETIF_DOMAIN_NAME_SERVER, &offerDns, sizeof(offerDns));
    esp_netif_set_dns_info(apNetif, ESP_NETIF_DNS_MAIN, &dns);
    esp_netif_dhcps_start(apNetif);
  }
  s_dnsRun = true;
  xTaskCreate(dnsTask, "SetupDns", 3072, nullptr, 3, nullptr);

  httpd_config_t config = HTTPD_DEFAULT_CONFIG();
  config.lru_purge_enable = true;
  config.open_fn = onSetupConnection;
  config.uri_match_fn = httpd_uri_match_wildcard;
  esp_err_t err = httpd_start(&s_server, &config);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Setup page server didn't start: %s", esp_err_to_name(err));
  } else {
    ESP_LOGI(TAG, "Setup page server listening on port %d", config.server_port);
    httpd_uri_t page = {};
    page.uri = "/";
    page.method = HTTP_GET;
    page.handler = onSetupPage;
    httpd_register_uri_handler(s_server, &page);
    httpd_uri_t save = {};
    save.uri = "/save";
    save.method = HTTP_POST;
    save.handler = onSetupSave;
    httpd_register_uri_handler(s_server, &save);
    httpd_uri_t other = {};
    other.uri = "/*";   // after "/" and "/save", so it only catches the rest
    other.method = HTTP_GET;
    other.handler = onSetupRedirect;
    httpd_register_uri_handler(s_server, &other);
  }
  wifiSetupState = WIFI_SETUP_RUNNING;
  ESP_LOGI(TAG, "Wi-Fi setup: join %s and open http://%s", WIFI_SETUP_SSID, WIFI_SETUP_ADDRESS);
}

void wifiSetupStop()
{
  s_dnsRun = false;   // dnsTask notices within a second and closes its socket
  if (s_server) {
    httpd_stop(s_server);
    s_server = nullptr;
  }
  esp_wifi_set_mode(WIFI_MODE_STA);
  linkRadioRestore();
  wifiSetupState = WIFI_SETUP_OFF;
}

// ---- Setup ----------------------------------------------------------------------------------

void firmwareUpdateInit()
{
  s_replies = xQueueCreate(4, sizeof(UpdateReply));
  s_wifiEvents = xEventGroupCreate();
  esp_event_handler_register(WIFI_EVENT, WIFI_EVENT_STA_DISCONNECTED, onWifiEvent, nullptr);
  esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, onWifiEvent, nullptr);
}
