#define FW_VERSION "ESP32-S3-CC1101-ERT-1.0.0"

#include <Arduino.h>
#include <Preferences.h>
#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <ArduinoOTA.h>
#include <SPI.h>
#include <rtl_433_ESP.h>
#include <WiFiClient.h>
#include <PubSubClient.h>
#include "Credentials.h"

extern CC1101 radio;

#define PIN_GDO0  4
#define PIN_CS   10
#define PIN_SCK   9
#define PIN_MOSI 11
#define PIN_MISO 13

static float    rxFreq      = 914.224f;
static int      rssiGate    = -95;
static uint8_t  rxBwIdx     = 0;
static const uint8_t BW_NIBBLES[6] = {0x00, 0x40, 0x60, 0x80, 0xA0, 0xC0};
static int      rssiThresh   = 3;
static uint8_t  agcCtrl2     = 0x07;
static uint8_t  agcCtrl1     = 0x40;
static uint8_t  agcCtrl0     = 0xB3;
static uint32_t goodDecodes = 0;
static uint32_t lastAgcMs   = 0;
static uint32_t lastStatusMs = 0;
static bool     radioOk      = false;
static char     radioErrMsg[80] = "not initialized";

#define MAX_PKT 20
struct Pkt {
    char     json[320];
    uint32_t ts;
};
static Pkt  pkts[MAX_PKT];
static int  pktHead  = 0;
static int  pktCount = 0;
static SemaphoreHandle_t pktMutex;

#define DBG_LINES 32
#define DBG_LINE_LEN 80
static char     dbgBuf[DBG_LINES][DBG_LINE_LEN];
static int      dbgHead  = 0;
static int      dbgCount = 0;
static SemaphoreHandle_t dbgMutex;

extern "C" void ert_dbg_log(const char* line) {
    if (!dbgMutex) return;
    if (xSemaphoreTake(dbgMutex, 0) == pdTRUE) {
        strncpy(dbgBuf[dbgHead], line, DBG_LINE_LEN - 1);
        dbgBuf[dbgHead][DBG_LINE_LEN - 1] = '\0';
        dbgHead = (dbgHead + 1) % DBG_LINES;
        if (dbgCount < DBG_LINES) dbgCount++;
        xSemaphoreGive(dbgMutex);
    }
}

static WiFiClient   wifiClient;
static PubSubClient mqtt(wifiClient);

static uint32_t mqttIds[16];
static char     mqttLastJson[16][320];
static int      mqttIdCount = 0;
static SemaphoreHandle_t mqttMutex;

#define MQTT_PUB_QUEUE 4
struct MqttPubMsg { char topic[80]; char payload[320]; };
static MqttPubMsg        mqttPubQ[MQTT_PUB_QUEUE];
static volatile int      mqttPubHead = 0;
static volatile int      mqttPubTail = 0;
static SemaphoreHandle_t mqttPubMutex;
static bool              mqttDiscoveryDone    = false;
static volatile bool     mqttNeedsDiscovery   = false;
static volatile bool     mqttJsonDirty        = false;
static char              espName[32]          = "ESP32 X";
static char              deviceNames[16][32];
static char              deviceUnits[16][16];
static float             deviceScales[16];
static uint32_t          lastSeenMs[16];
static time_t            lastSeenTime[16];
static volatile bool     lastSeenDirty[16];
static uint32_t          deviceDecodeCounts[16];
static volatile bool     countDirty = false;

#define MAX_SEEN 50
static uint32_t      seenIds[MAX_SEEN];
static char          seenLastJson[MAX_SEEN][200];
static time_t        seenLastTime[MAX_SEEN];
static uint32_t      seenDecodeCount[MAX_SEEN];
static int           seenIdCount = 0;
static volatile bool seenDirty   = false;
static bool          seenSlotDirty[MAX_SEEN] = {};

static Preferences prefs;

static bool mqttHasId(uint32_t id) {
    for (int i = 0; i < mqttIdCount; i++) if (mqttIds[i] == id) return true;
    return false;
}
static void nvsSave() {
    prefs.begin("ert", false);
    prefs.putInt("count", mqttIdCount);
    for (int i = 0; i < mqttIdCount; i++) {
        char k[8]; snprintf(k, sizeof(k), "id%d", i);
        prefs.putULong(k, mqttIds[i]);
        char dn[9]; snprintf(dn, sizeof(dn), "dname%d", i);
        prefs.putString(dn, deviceNames[i]);
        char du[8]; snprintf(du, sizeof(du), "unit%d", i);
        prefs.putString(du, deviceUnits[i]);
        char ds[9]; snprintf(ds, sizeof(ds), "scale%d", i);
        prefs.putFloat(ds, deviceScales[i]);
    }
    prefs.putString("espname", espName);
    prefs.putUChar("bw", rxBwIdx);
    prefs.putInt("rssiThr", rssiThresh);
    prefs.putUChar("agc2", agcCtrl2);
    prefs.putUChar("agc1", agcCtrl1);
    prefs.putUChar("agc0", agcCtrl0);
    prefs.end();
}
static void nvsLoad() {
    prefs.begin("ert", true);
    mqttIdCount = prefs.getInt("count", 0);
    if (mqttIdCount > 16) mqttIdCount = 0;
    for (int i = 0; i < mqttIdCount; i++) {
        char k[8]; snprintf(k, sizeof(k), "id%d", i);
        mqttIds[i] = (uint32_t)prefs.getULong(k, 0);
        char kj[8]; snprintf(kj, sizeof(kj), "json%d", i);
        String js = prefs.getString(kj, "");
        strncpy(mqttLastJson[i], js.c_str(), sizeof(mqttLastJson[i]) - 1);
        mqttLastJson[i][sizeof(mqttLastJson[i]) - 1] = '\0';
        char dn[9]; snprintf(dn, sizeof(dn), "dname%d", i);
        String nm = prefs.getString(dn, "");
        strncpy(deviceNames[i], nm.c_str(), sizeof(deviceNames[i]) - 1);
        deviceNames[i][sizeof(deviceNames[i]) - 1] = '\0';
        char du[8]; snprintf(du, sizeof(du), "unit%d", i);
        String un = prefs.getString(du, "");
        strncpy(deviceUnits[i], un.c_str(), sizeof(deviceUnits[i]) - 1);
        deviceUnits[i][sizeof(deviceUnits[i]) - 1] = '\0';
        char ds[9]; snprintf(ds, sizeof(ds), "scale%d", i);
        deviceScales[i] = prefs.getFloat(ds, 1.0f);
        char lt[10]; snprintf(lt, sizeof(lt), "ltime%d", i);
        lastSeenTime[i] = (time_t)prefs.getULong(lt, 0);
        char ck[8]; snprintf(ck, sizeof(ck), "cnt%d", i);
        deviceDecodeCounts[i] = prefs.getULong(ck, 0);
    }
    String en = prefs.getString("espname", "ESP32 X");
    strncpy(espName, en.c_str(), sizeof(espName) - 1);  espName[sizeof(espName) - 1] = '\0';
    rxBwIdx    = prefs.getUChar("bw",     0);    if (rxBwIdx >= 6) rxBwIdx = 0;
    rssiThresh = prefs.getInt("rssiThr",  3);
    agcCtrl2   = prefs.getUChar("agc2",   0x07);
    agcCtrl1   = prefs.getUChar("agc1",   0x40);
    agcCtrl0   = prefs.getUChar("agc0",   0xB3);
    prefs.end();
    printf("[NVS] loaded %d device(s), esp='%s'\n", mqttIdCount, espName);
}
static void nvsSaveLastJson() {
    static char snapJson[16][320];
    static bool snapValid[16];
    int cnt;
    xSemaphoreTake(mqttMutex, portMAX_DELAY);
    cnt = mqttIdCount;
    for (int i = 0; i < cnt; i++) {
        snapValid[i] = mqttLastJson[i][0] != '\0';
        if (snapValid[i]) memcpy(snapJson[i], mqttLastJson[i], 320);
    }
    mqttJsonDirty = false;
    xSemaphoreGive(mqttMutex);
    prefs.begin("ert", false);
    for (int i = 0; i < cnt; i++) {
        if (snapValid[i]) {
            char kj[8]; snprintf(kj, sizeof(kj), "json%d", i);
            prefs.putString(kj, snapJson[i]);
        }
    }
    prefs.end();
}
static void nvsSaveLastSeen() {
    static time_t   snapTime[16];
    static uint32_t snapCnt[16];
    static bool     snapTDirty[16];
    bool snapCDirty; int cnt;
    xSemaphoreTake(mqttMutex, portMAX_DELAY);
    bool anyDirty = countDirty;
    if (!anyDirty) for (int i = 0; i < mqttIdCount; i++) if (lastSeenDirty[i]) { anyDirty = true; break; }
    if (!anyDirty) { xSemaphoreGive(mqttMutex); return; }
    cnt = mqttIdCount;
    for (int i = 0; i < cnt; i++) { snapTDirty[i] = lastSeenDirty[i]; lastSeenDirty[i] = false; snapTime[i] = lastSeenTime[i]; }
    snapCDirty = countDirty; countDirty = false;
    for (int i = 0; i < cnt; i++) snapCnt[i] = deviceDecodeCounts[i];
    xSemaphoreGive(mqttMutex);
    prefs.begin("ert", false);
    for (int i = 0; i < cnt; i++) {
        if (snapTDirty[i]) { char lt[10]; snprintf(lt, sizeof(lt), "ltime%d", i); prefs.putULong(lt, (unsigned long)snapTime[i]); }
    }
    if (snapCDirty) {
        for (int i = 0; i < cnt; i++) { char ck[8]; snprintf(ck, sizeof(ck), "cnt%d", i); prefs.putULong(ck, snapCnt[i]); }
    }
    prefs.end();
}

static void nvsLoadSeen() {
    prefs.begin("ertseen", true);
    seenIdCount = prefs.getInt("scount", 0);
    if (seenIdCount > MAX_SEEN) seenIdCount = 0;
    for (int i = 0; i < seenIdCount; i++) {
        char k[8]; snprintf(k, sizeof(k), "sid%d", i);
        seenIds[i] = (uint32_t)prefs.getULong(k, 0);
        char jk[12], tk[12], ck[12];
        snprintf(jk, sizeof(jk), "sj%08lX", (unsigned long)seenIds[i]);
        snprintf(tk, sizeof(tk), "st%08lX", (unsigned long)seenIds[i]);
        snprintf(ck, sizeof(ck), "sc%08lX", (unsigned long)seenIds[i]);
        String js = prefs.getString(jk, "");
        strncpy(seenLastJson[i], js.c_str(), 199); seenLastJson[i][199] = '\0';
        seenLastTime[i] = (time_t)prefs.getULong(tk, 0);
        seenDecodeCount[i] = prefs.getULong(ck, 0);
    }
    prefs.end();
    printf("[NVS] seen %d other device(s)\n", seenIdCount);
}
static void nvsSaveSeen() {
    static uint32_t snapIds[MAX_SEEN];
    int cnt;
    if (xSemaphoreTake(mqttMutex, pdMS_TO_TICKS(10)) != pdTRUE) return;
    cnt = seenIdCount;
    memcpy(snapIds, seenIds, cnt * sizeof(uint32_t));
    seenDirty = false;
    xSemaphoreGive(mqttMutex);
    prefs.begin("ertseen", false);
    prefs.putInt("scount", cnt);
    for (int i = 0; i < cnt; i++) {
        char k[8]; snprintf(k, sizeof(k), "sid%d", i);
        prefs.putULong(k, snapIds[i]);
    }
    prefs.end();
}
static void nvsSaveSeenDirty() {
    static uint32_t snapIds[MAX_SEEN];
    static char     snapJson[MAX_SEEN][200];
    static time_t   snapTime[MAX_SEEN];
    static uint32_t snapCnt[MAX_SEEN];
    static bool     snapDirty[MAX_SEEN];
    int cnt;
    if (xSemaphoreTake(mqttMutex, pdMS_TO_TICKS(10)) != pdTRUE) return;
    cnt = seenIdCount;
    memcpy(snapIds,  seenIds,        cnt * sizeof(uint32_t));
    for (int i = 0; i < cnt; i++) memcpy(snapJson[i], seenLastJson[i], 200);
    memcpy(snapTime, seenLastTime,   cnt * sizeof(time_t));
    memcpy(snapCnt,  seenDecodeCount,cnt * sizeof(uint32_t));
    memcpy(snapDirty,seenSlotDirty,  cnt * sizeof(bool));
    memset(seenSlotDirty, 0, sizeof(seenSlotDirty));
    xSemaphoreGive(mqttMutex);
    bool any = false;
    for (int i = 0; i < cnt; i++) if (snapDirty[i]) { any = true; break; }
    if (!any) return;
    prefs.begin("ertseen", false);
    for (int i = 0; i < cnt; i++) {
        if (!snapDirty[i]) continue;
        char jk[12], tk[12], ck[12];
        snprintf(jk, sizeof(jk), "sj%08lX", (unsigned long)snapIds[i]);
        snprintf(tk, sizeof(tk), "st%08lX", (unsigned long)snapIds[i]);
        snprintf(ck, sizeof(ck), "sc%08lX", (unsigned long)snapIds[i]);
        prefs.putString(jk, snapJson[i]);
        prefs.putULong(tk, (unsigned long)snapTime[i]);
        prefs.putULong(ck, snapCnt[i]);
    }
    prefs.end();
}
static void seenAddId(uint32_t id, const char* json = nullptr, time_t ts = 0) {
    if (mqttHasId(id)) return;
    for (int i = 0; i < seenIdCount; i++) {
        if (seenIds[i] == id) {
            uint32_t tid = seenIds[i];
            char tj[200]; memcpy(tj, seenLastJson[i], 200);
            time_t tt = seenLastTime[i];
            uint32_t tc = seenDecodeCount[i];
            for (int j = i; j > 0; j--) {
                seenIds[j] = seenIds[j-1];
                memcpy(seenLastJson[j], seenLastJson[j-1], 200);
                seenLastTime[j] = seenLastTime[j-1];
                seenDecodeCount[j] = seenDecodeCount[j-1];
            }
            seenIds[0] = tid;
            memcpy(seenLastJson[0], tj, 200);
            seenLastTime[0] = tt;
            seenDecodeCount[0] = tc;
            if (json) { strncpy(seenLastJson[0], json, 199); seenLastJson[0][199] = '\0'; seenDecodeCount[0]++; seenSlotDirty[0] = true; }
            if (ts > 0) seenLastTime[0] = ts;
            return;
        }
    }
    int end = (seenIdCount < MAX_SEEN) ? seenIdCount : MAX_SEEN - 1;
    for (int i = end; i > 0; i--) {
        seenIds[i] = seenIds[i-1];
        memcpy(seenLastJson[i], seenLastJson[i-1], 200);
        seenLastTime[i] = seenLastTime[i-1];
        seenDecodeCount[i] = seenDecodeCount[i-1];
    }
    seenIds[0] = id;
    seenLastJson[0][0] = '\0';
    if (json) { strncpy(seenLastJson[0], json, 199); seenLastJson[0][199] = '\0'; seenSlotDirty[0] = true; }
    seenLastTime[0] = ts;
    seenDecodeCount[0] = (json ? 1 : 0);
    if (seenIdCount < MAX_SEEN) seenIdCount++;
}
static void seenRemoveId(uint32_t id) {
    for (int i = 0; i < seenIdCount; i++) {
        if (seenIds[i] == id) {
            seenIdCount--;
            for (int j = i; j < seenIdCount; j++) {
                seenIds[j] = seenIds[j+1];
                memcpy(seenLastJson[j], seenLastJson[j+1], 200);
                seenLastTime[j] = seenLastTime[j+1];
                seenDecodeCount[j] = seenDecodeCount[j+1];
            }
            seenDirty = true; return;
        }
    }
}

static void mqttAddId(uint32_t id) {
    if (!mqttHasId(id) && mqttIdCount < 16) {
        int slot = mqttIdCount;
        mqttLastJson[slot][0] = '\0';
        lastSeenTime[slot]       = 0;
        deviceDecodeCounts[slot] = 0;
        for (int i = 0; i < seenIdCount; i++) {
            if (seenIds[i] == id) {
                if (seenLastJson[i][0]) { strncpy(mqttLastJson[slot], seenLastJson[i], sizeof(mqttLastJson[slot])-1); mqttLastJson[slot][sizeof(mqttLastJson[slot])-1]='\0'; }
                lastSeenTime[slot]       = seenLastTime[i];
                deviceDecodeCounts[slot] = seenDecodeCount[i];
                break;
            }
        }
        seenRemoveId(id);
        deviceNames[slot][0] = '\0';
        deviceUnits[slot][0] = '\0';
        deviceScales[slot]   = 1.0f;
        mqttIds[slot]        = id;
        mqttIdCount++;
        nvsSave();
        if (mqttLastJson[slot][0]) { mqttJsonDirty = true; lastSeenDirty[slot] = true; countDirty = true; }
        mqttNeedsDiscovery = true;
    }
}
static void mqttRemoveId(uint32_t id) {
    for (int i = 0; i < mqttIdCount; i++) {
        if (mqttIds[i] == id) {
            seenAddId(id);
            seenDirty = true;
            --mqttIdCount;
            mqttIds[i] = mqttIds[mqttIdCount];
            strncpy(mqttLastJson[i], mqttLastJson[mqttIdCount], 319);
            strncpy(deviceNames[i],  deviceNames[mqttIdCount],  31);
            strncpy(deviceUnits[i],  deviceUnits[mqttIdCount],  15);
            deviceScales[i]              = deviceScales[mqttIdCount];
            mqttLastJson[mqttIdCount][0] = '\0';
            deviceNames[mqttIdCount][0]  = '\0';
            deviceUnits[mqttIdCount][0]  = '\0';
            deviceScales[mqttIdCount]    = 1.0f;
            nvsSave(); return;
        }
    }
}

static void mqttEnqueue(const char* topic, const char* payload) {
    if (xSemaphoreTake(mqttPubMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
        int next = (mqttPubHead + 1) % MQTT_PUB_QUEUE;
        if (next != mqttPubTail) {
            strncpy(mqttPubQ[mqttPubHead].topic,   topic,   79);
            strncpy(mqttPubQ[mqttPubHead].payload, payload, 319);
            mqttPubHead = next;
        }
        xSemaphoreGive(mqttPubMutex);
    }
}

static void mqttDrainQueue() {
    while (mqtt.connected() && mqttPubTail != mqttPubHead) {
        char topic[80], payload[320];
        if (xSemaphoreTake(mqttPubMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
            strncpy(topic,   mqttPubQ[mqttPubTail].topic,   79);
            strncpy(payload, mqttPubQ[mqttPubTail].payload, 319);
            mqttPubTail = (mqttPubTail + 1) % MQTT_PUB_QUEUE;
            xSemaphoreGive(mqttPubMutex);
            mqtt.publish(topic, payload);
            printf("[MQTT] → %s\n", topic);
        }
    }
}

static void slugify(const char* src, char* dst, size_t dstLen) {
    size_t j = 0;
    for (size_t i = 0; src[i] && j < dstLen - 1; i++) {
        char c = (char)tolower((unsigned char)src[i]);
        if (isalnum((unsigned char)c)) { dst[j++] = c; }
        else if (j > 0 && dst[j-1] != '_') { dst[j++] = '_'; }
    }
    while (j > 0 && dst[j-1] == '_') j--;
    dst[j] = '\0';
}

static void sanitizeName(char* s) {
    for (char* p = s; *p; p++) if (*p=='"'||*p=='\\'||*p=='\n'||*p=='\r') *p=' ';
}

static void getEspLabel(char* out, size_t n) {
    if (espName[0]) strncpy(out, espName, n-1);
    else            strncpy(out, "ESP32 X", n-1);
    out[n-1] = '\0';
}

static void getDevLabel(int idx, char* out, size_t n) {
    if (deviceNames[idx][0]) strncpy(out, deviceNames[idx], n-1);
    else snprintf(out, n, "Device ID %lu", (unsigned long)mqttIds[idx]);
    out[n-1] = '\0';
}

static void getStateTopic(int idx, char* out, size_t n) {
    char espLabel[32], devLabel[48], espSlug[34], devSlug[50];
    getEspLabel(espLabel, sizeof(espLabel));
    getDevLabel(idx, devLabel, sizeof(devLabel));
    slugify(espLabel, espSlug, sizeof(espSlug));
    slugify(devLabel, devSlug, sizeof(devSlug));
    snprintf(out, n, "%s/%s", espSlug, devSlug);
}

static void publishDiscovery(uint32_t id, int idx) {
    if (!mqtt.connected()) return;
    char topic[128], payload[700], stateTopic[80], espLabel[32], devLabel[48];
    char espSlug[34], devSlug[50];
    getEspLabel(espLabel, sizeof(espLabel));
    getDevLabel(idx, devLabel, sizeof(devLabel));
    slugify(espLabel, espSlug, sizeof(espSlug));
    slugify(devLabel, devSlug, sizeof(devSlug));
    snprintf(stateTopic, sizeof(stateTopic), "%s/%s", espSlug, devSlug);

    char oldTopic[96];
    snprintf(oldTopic, sizeof(oldTopic), "homeassistant/sensor/ert_scm_%lu/config",      (unsigned long)id);
    mqtt.publish(oldTopic, "", true);
    snprintf(oldTopic, sizeof(oldTopic), "homeassistant/sensor/ert_scm_%lu_rssi/config", (unsigned long)id);
    mqtt.publish(oldTopic, "", true);

    char valTmpl[96];
    float sc = deviceScales[idx];
    if (sc == 1.0f) {
        strncpy(valTmpl, "{{value_json.consumption_data}}", sizeof(valTmpl));
    } else {
        snprintf(valTmpl, sizeof(valTmpl), "{{value_json.consumption_data|float*%g}}", sc);
    }

    snprintf(topic, sizeof(topic), "homeassistant/sensor/%s/%s/config", espSlug, devSlug);
    const char* unit = deviceUnits[idx];
    if (unit[0]) {
        snprintf(payload, sizeof(payload),
            "{\"name\":\"%s\","
            "\"state_topic\":\"%s\","
            "\"value_template\":\"%s\","
            "\"unit_of_measurement\":\"%s\","
            "\"device_class\":\"gas\","
            "\"state_class\":\"total_increasing\","
            "\"unique_id\":\"ert_scm_%lu\","
            "\"device\":{\"identifiers\":[\"esp32_%s\"],"
            "\"name\":\"%s\","
            "\"model\":\"ERT Decoder\",\"manufacturer\":\"Itron\"}}",
            devLabel, stateTopic, valTmpl, unit,
            (unsigned long)id, espSlug, espLabel);
    } else {
        snprintf(payload, sizeof(payload),
            "{\"name\":\"%s\","
            "\"state_topic\":\"%s\","
            "\"value_template\":\"%s\","
            "\"device_class\":\"gas\","
            "\"state_class\":\"total_increasing\","
            "\"unique_id\":\"ert_scm_%lu\","
            "\"device\":{\"identifiers\":[\"esp32_%s\"],"
            "\"name\":\"%s\","
            "\"model\":\"ERT Decoder\",\"manufacturer\":\"Itron\"}}",
            devLabel, stateTopic, valTmpl,
            (unsigned long)id, espSlug, espLabel);
    }
    mqtt.publish(topic, payload, true);

    snprintf(topic, sizeof(topic), "homeassistant/sensor/%s/%s_rssi/config", espSlug, devSlug);
    snprintf(payload, sizeof(payload),
        "{\"name\":\"%s RSSI\","
        "\"state_topic\":\"%s\","
        "\"value_template\":\"{{value_json.rssi}}\","
        "\"unit_of_measurement\":\"dBm\","
        "\"device_class\":\"signal_strength\","
        "\"entity_category\":\"diagnostic\","
        "\"unique_id\":\"ert_scm_%lu_rssi\","
        "\"device\":{\"identifiers\":[\"esp32_%s\"],"
        "\"name\":\"%s\"}}",
        devLabel, stateTopic,
        (unsigned long)id, espSlug, espLabel);
    mqtt.publish(topic, payload, true);
    printf("[MQTT] Discovery id=%lu topic=homeassistant/sensor/%s/%s/config\n",
           (unsigned long)id, espSlug, devSlug);
}
static void mqttEnsureConnected() {
    if (!mqtt.connected() && WiFi.status() == WL_CONNECTED) {
        mqtt.connect(MQTT_CLIENT_ID, MQTT_USER, MQTT_PASSWORD);
    }
}

static AsyncWebServer  server(80);
static AsyncEventSource events("/events");

static rtl_433_ESP rf;
static char        rfMsgBuf[512];

void rtl433Callback(char* message) {
    if (strstr(message, "\"model\":\"status\"")) return;

    xSemaphoreTake(pktMutex, portMAX_DELAY);
    strncpy(pkts[pktHead].json, message, sizeof(pkts[pktHead].json) - 1);
    pkts[pktHead].json[sizeof(pkts[pktHead].json) - 1] = '\0';
    pkts[pktHead].ts = millis();
    pktHead = (pktHead + 1) % MAX_PKT;
    if (pktCount < MAX_PKT) pktCount++;
    goodDecodes++;
    xSemaphoreGive(pktMutex);

    printf("[ERT] %s\n", message);

    const char* idPos = strstr(message, "\"id\":");
    if (idPos) {
        uint32_t pktId = (uint32_t)strtoul(idPos + 5, nullptr, 10);
        if (pktId > 0) {
            events.send(message, "decode", millis());
            char pubTopic[80]; pubTopic[0] = '\0';
            xSemaphoreTake(mqttMutex, portMAX_DELAY);
            bool selected = false;
            for (int i = 0; i < mqttIdCount; i++) {
                if (mqttIds[i] == pktId) {
                    selected = true;
                    strncpy(mqttLastJson[i], message, 319);
                    mqttJsonDirty = true;
                    lastSeenMs[i] = millis();
                    deviceDecodeCounts[i]++;
                    countDirty = true;
                    time_t _now; time(&_now);
                    if (_now > 1000000000L) { lastSeenTime[i] = _now; lastSeenDirty[i] = true; }
                    getStateTopic(i, pubTopic, sizeof(pubTopic));
                    break;
                }
            }
            if (!selected) {
                time_t _t; time(&_t);
                seenAddId(pktId, message, (_t > 1000000000L) ? _t : 0);
                seenDirty = true;
            }
            xSemaphoreGive(mqttMutex);
            if (selected) {
                mqttEnqueue(pubTopic, message);
            }
        }
    }
}

static const char PAGE[] PROGMEM = R"HTML(
<!DOCTYPE html><html lang="en"><head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>ERT Receiver v4.0</title>
<style>
*{box-sizing:border-box;margin:0;padding:0}
body{font-family:'Courier New',monospace;background:#060d14;color:#00e5ff;min-height:100vh}
.bar{display:flex;align-items:center;gap:12px;padding:10px 18px;background:#0a1a24;border-bottom:2px solid #00bcd4;flex-wrap:wrap}
.bar h1{font-size:13px;color:#fff;text-transform:uppercase;letter-spacing:2px}
.badge{padding:3px 10px;border-radius:10px;border:1px solid #00bcd4;font-size:11px;color:#00e5ff;background:#041824}
.main{padding:14px 18px;display:flex;flex-direction:column;gap:12px}
.panel{background:#07121a;border:1px solid #0a3040;border-radius:6px;padding:12px}
.pt{font-size:10px;color:#fff;text-transform:uppercase;letter-spacing:2px;margin-bottom:10px;border-bottom:1px solid #0a3040;padding-bottom:5px}
.row{display:flex;align-items:center;gap:10px;margin-bottom:8px;flex-wrap:wrap}
.lbl{font-size:10px;color:#005f7a;text-transform:uppercase;min-width:120px}
input[type=number],input[type=range],input[type=text]{background:#000;color:#00e5ff;border:1px solid #0a3040;padding:4px 7px;font-family:inherit;font-size:12px;border-radius:3px;outline:none}
input[type=number]{width:110px}input[type=range]{width:160px}input[type=text]{width:90px}
input:focus{border-color:#00bcd4}
.btn{padding:6px 14px;background:#041824;border:1px solid #00bcd4;color:#00e5ff;font-family:inherit;font-size:11px;font-weight:bold;cursor:pointer;border-radius:3px}
.btn:hover{filter:brightness(1.4)}
.btn-apply{border-color:#00ff88;color:#00ff88}
.btn-refresh{border-color:#ffaa00;color:#ffaa00}
.stat{display:flex;gap:20px;flex-wrap:wrap;margin-top:4px}
.si{display:flex;flex-direction:column;gap:2px}
.sl{font-size:9px;color:#005f7a;text-transform:uppercase;letter-spacing:1px}
.sv{font-size:13px;font-weight:bold}
table{width:100%;border-collapse:collapse;font-size:11px}
th{background:#0a1a24;padding:6px 8px;text-align:left;color:#fff;border-bottom:1px solid #0a3040}
td{padding:5px 8px;border-bottom:1px solid #061018;color:#00e5ff}
tr:hover td{background:#0a1a24}
.ok{color:#00ff88}.no{color:#ff4444}
input[type=checkbox].mqttcb,input[type=checkbox].selcb{accent-color:#00ff88;width:15px;height:15px;cursor:pointer;vertical-align:middle}
.ssedot{width:8px;height:8px;border-radius:50%;background:#555;display:inline-block;margin-right:6px}
.ssedot.on{background:#00ff88}
#log{background:#000;padding:8px;border-radius:4px;border:1px solid #061018;max-height:160px;overflow-y:auto;font-size:10px;color:#00bcd4;line-height:1.6}
</style></head><body>
<div class="bar">
  <h1>&#x2B22; ESP32+CC1101 915MHz HA Decoder</h1>
  <span class="badge" id="fwBadge">—</span>
  <span class="badge" id="sseBadge"><span class="ssedot" id="sseDot"></span>SSE</span>
  <span class="badge" id="mqttBadge"><span class="ssedot" id="mqttDot"></span>MQTT</span>
  <span class="badge" id="rssiLive">RSSI —</span>
  <span class="badge" id="nvsBadge">NVS —</span>
  <span class="badge" id="uptimeBadge">Up —</span>
  <span class="badge" id="healthBadge">SYS <span style="display:inline-block;width:8px;height:8px;border-radius:50%;background:#555;vertical-align:middle"></span></span>
</div>
<div class="main">

  <div class="panel" id="errorsPanel" style="display:none;border-color:#ff4444">
    <div class="pt" style="color:#ff4444">&#x26A0; System Errors</div>
    <div id="errorsContent" style="font-size:12px;line-height:2.4"></div>
  </div>

  <div class="panel">
    <div class="pt">Settings</div>
    <div class="row">
      <span class="lbl">ESP32 Name</span>
      <input type="text" id="espNameIn" value="" maxlength="31" placeholder="ESP32 X" style="width:160px;color:#888" readonly>
      <button class="btn" id="espNameBtn" style="border-color:#00bcd4;color:#00e5ff" onclick="espNameToggle()">&#9998; Edit</button>
      <span id="espNameNote" style="font-size:10px;color:#005f7a;margin-left:8px"></span>
    </div>
    <div class="row">
      <span class="lbl">Frequency (MHz)</span>
      <input type="number" id="freqIn" value="914.224" step="0.001" min="300" max="928">
      <button class="btn btn-apply" onclick="applyFreq()">Apply</button>
    </div>
    <div class="row">
      <span class="lbl">RX Bandwidth</span>
      <select id="bwSel" style="background:#000;color:#00e5ff;border:1px solid #0a3040;padding:4px 7px;font-family:inherit;font-size:12px;border-radius:3px;outline:none">
        <option value="0">812 kHz</option>
        <option value="1">406 kHz</option>
        <option value="2">270 kHz</option>
        <option value="3">203 kHz</option>
        <option value="4">135 kHz</option>
        <option value="5">102 kHz</option>
      </select>
      <button class="btn btn-apply" onclick="applyBw()">Apply</button>
    </div>
    <div class="row">
      <span class="lbl">RSSI Gate (dBm)</span>
      <input type="range" id="gateSlider" min="-120" max="-10" step="1" value="-95">
      <span id="gateDisp" style="font-size:12px;min-width:60px">-95 dBm</span>
      <button class="btn btn-apply" onclick="applyGate()">Apply</button>
    </div>
    <div class="row">
      <span class="lbl">HW RSSI Threshold</span>
      <input type="number" id="rssiThreshIn" value="3" min="-20" max="20" step="1" style="width:58px">
      <span style="font-size:10px;color:#888;margin-left:6px">dB above noise floor</span>
      <button class="btn btn-apply" onclick="applyRssiThresh()">Apply</button>
    </div>
    <div class="row" style="margin-top:6px">
      <button class="btn btn-refresh" id="regToggleBtn" onclick="toggleRegisters()">&#9654; Show Registers</button>
      <button class="btn" id="resetBtn" style="border-color:#ff4444;color:#ff4444;margin-left:8px" onclick="resetDefaults()">&#x21BA; Reset Defaults</button>
      <span id="refreshNote" style="font-size:10px;color:#005f7a;margin-left:8px"></span>
    </div>
    <div class="stat" id="statBar">
      <div class="si"><div class="sl">RSSI</div><div class="sv" id="sRssi">—</div></div>
      <div class="si"><div class="sl">Freq</div><div class="sv" id="sFreq">—</div></div>
      <div class="si"><div class="sl">Gate</div><div class="sv" id="sGate">—</div></div>
      <div class="si"><div class="sl">HW Thr</div><div class="sv" id="sRssiThr">—</div></div>
      <div class="si"><div class="sl">RX BW</div><div class="sv" id="sBw">—</div></div>
      <div class="si"><div class="sl">Good Decodes</div><div class="sv" id="sGood">—</div></div>
    </div>
  </div>

  <div class="panel" id="regPanel" style="display:none">
    <div class="pt">CC1101 Registers</div>
    <table id="regTable">
      <thead><tr>
        <th>Register</th><th>Addr</th><th>Value</th><th>Description</th><th style="width:130px">Write</th>
      </tr></thead>
      <tbody id="regBody"><tr><td colspan="5" style="color:#005f7a;text-align:center">Loading…</td></tr></tbody>
    </table>
  </div>

  <div class="panel">
    <div class="pt">&#x2714; Selected Devices — MQTT Active</div>
    <table>
      <thead><tr>
        <th>MQTT</th><th>Custom Name</th><th>Unit</th><th>Scale</th><th></th><th>Last Seen</th><th>Model</th><th>ID</th><th>Consumption</th><th>RSSI</th><th>Msgs</th>
      </tr></thead>
      <tbody id="selBody"><tr><td colspan="11" style="color:#005f7a;text-align:center">No devices selected — check &#9744; a device below to enable MQTT</td></tr></tbody>
    </table>
  </div>

  <div class="panel">
    <div class="pt" style="display:flex;align-items:center;justify-content:space-between;gap:6px"><span>Other Detected Devices</span><span style="display:flex;gap:6px"><button class="btn" style="border-color:#00ff88;color:#00ff88;padding:3px 10px;font-size:10px" onclick="othAdd()">+ Add</button><button class="btn" style="border-color:#ff4444;color:#ff4444;padding:3px 10px;font-size:10px" onclick="othDelete()">&#x2715; Delete</button></span></div>
    <table>
      <thead><tr>
        <th>Sel</th><th>Last Seen</th><th>Model</th><th>ID</th><th>Messages</th>
      </tr></thead>
      <tbody id="othBody"><tr><td colspan="5" style="color:#005f7a;text-align:center">Waiting for decode events…</td></tr></tbody>
    </table>
  </div>

  <div class="panel">
    <div class="pt">Raw Decode Log</div>
    <div id="log"></div>
  </div>

</div>
<script>
// ── MQTT selection + device tracking ──────────────────────────────────────
let mqttSelected    = new Set();
let selectedDevices = {}; // id  → {model,id,consumption,rssi,mic,lastSeen,count}
let otherDevices    = {}; // model|id → {model,id,lastSeen,count}
let deviceNamesMap  = {}; // id → custom name string
let deviceUnitsMap  = {}; // id → unit of measurement string
let deviceScalesMap = {}; // id → scale factor string (default "1")
let devEditState    = {}; // id → true while row is in edit mode

function toggleMqtt(cb) {
    const id = cb.dataset.id;
    fetch('/mqtt-toggle?id='+id+'&on='+(cb.checked?'1':'0'))
        .then(r=>r.json()).then(res=>{
            if (res.ok) {
                if (cb.checked) {
                    mqttSelected.add(id);
                    const k = Object.keys(otherDevices).find(k=>otherDevices[k].id===id);
                    if (k) {
                        const dev = otherDevices[k];
                        selectedDevices[id] = {model:dev.model,id:id,consumption:'—',rssi:'—',mic:'?',lastSeen:dev.lastSeen,count:dev.count};
                        delete otherDevices[k];
                    }
                } else {
                    mqttSelected.delete(id);
                    if (selectedDevices[id]) {
                        const dev = selectedDevices[id];
                        otherDevices[dev.model+'|'+id] = {model:dev.model,id:id,lastSeen:dev.lastSeen,count:dev.count};
                        delete selectedDevices[id];
                    }
                }
                renderTables();
            } else cb.checked = !cb.checked;
        }).catch(()=>{ cb.checked = !cb.checked; });
}

fetch('/mqtt-selected').then(r=>r.json()).then(d=>{
    if(d.ids) d.ids.forEach((id, i)=>{
        const sid = String(id);
        mqttSelected.add(sid);
        if(d.names  && d.names[i]  !== undefined) deviceNamesMap[sid]  = d.names[i];
        if(d.units  && d.units[i]  !== undefined) deviceUnitsMap[sid]  = d.units[i];
        if(d.scales && d.scales[i] !== undefined) deviceScalesMap[sid] = String(d.scales[i]);
        const ageSec = (d.ages && d.ages[i] !== undefined) ? d.ages[i] : -1;
        const lastSeen = ageSec >= 0 ? Date.now() - ageSec * 1000 : 0;
        const initCount = (d.counts && d.counts[i] !== undefined) ? d.counts[i] : 0;
        if (!selectedDevices[sid]) {
            selectedDevices[sid] = {model:'ERT-SCM', id:sid, consumption:'—', rssi:'—', mic:'?', lastSeen:lastSeen, count:initCount};
        } else {
            selectedDevices[sid].lastSeen = lastSeen;
        }
    });
    if(d.espname !== undefined) document.getElementById('espNameIn').value = d.espname;
    if(d.seen) d.seen.forEach((id,i) => {
        const sid = String(id);
        const mdl = (d.seenModels && d.seenModels[i]) ? d.seenModels[i] : 'ERT';
        const k = mdl+'|'+sid;
        if (!mqttSelected.has(sid) && !otherDevices[k]) {
            const age = (d.seenAges && d.seenAges[i] !== undefined) ? d.seenAges[i] : -1;
            const cnt = (d.seenCounts && d.seenCounts[i] !== undefined) ? d.seenCounts[i] : 0;
            const lastSeen = age >= 0 ? Date.now() - age * 1000 : 0;
            otherDevices[k] = {model:mdl, id:sid, lastSeen, count:cnt};
        }
    });
    renderTables(); // show saved devices immediately, before any packets arrive
    if(d.last) d.last.forEach(item=>{ if(item) addPacket(JSON.stringify(item), true); });
    return fetch('/decoded');
}).then(r=>r.json()).then(d=>{
    if (d.packets && d.packets.length) {
        d.packets.slice().reverse().forEach(p => addPacket(p.json, true));
        document.getElementById('sGood').textContent = d.good;
    }
}).catch(()=>{});

function updateMqttBadge() {
    fetch('/mqtt-status').then(r=>r.json()).then(d=>{
        document.getElementById('mqttDot').className = 'ssedot'+(d.connected?' on':'');
    }).catch(()=>{ document.getElementById('mqttDot').className='ssedot'; });
}
updateMqttBadge();
setInterval(updateMqttBadge, 8000);
setInterval(renderTables, 30000);

// ── SSE connection ─────────────────────────────────────────────────────────
const sse = new EventSource('/events');
const dot = document.getElementById('sseDot');
const badge = document.getElementById('sseBadge');

sse.onopen = () => { dot.className = 'ssedot on'; };
sse.onerror = () => { dot.className = 'ssedot'; };

sse.addEventListener('decode', e => {
    addPacket(e.data);
});

function timeAgo(ms) {
    if (!ms) return 'prev session';
    const s = Math.floor((Date.now() - ms) / 1000);
    if (s < 60)   return s + 's ago';
    if (s < 3600) return Math.floor(s/60) + ' min ago';
    if (s < 86400) return Math.floor(s/3600) + ' hr ago';
    return Math.floor(s/86400) + 'd ago';
}
function fmtCount(n) {
    if (n >= 1000000) return (n/1000000).toFixed(1).replace(/\.0$/,'') + 'M';
    if (n >= 10000)   return Math.round(n/1000) + 'k';
    if (n >= 1000)    return (n/1000).toFixed(1).replace(/\.0$/,'') + 'k';
    return n;
}

function addPacket(raw, isReplay) {
    let d = {};
    try { d = JSON.parse(raw); } catch(_) {}

    const log = document.getElementById('log');
    log.textContent = raw + '\n' + log.textContent;
    if (log.textContent.length > 4000) log.textContent = log.textContent.slice(0, 4000);

    const devId = (d.id !== undefined && d.id !== null) ? String(d.id) : '';
    const cons  = d.consumption || d.Consumption || d.consumption_data || '—';
    const now   = isReplay ? 0 : Date.now();

    if (devId && mqttSelected.has(devId)) {
        if (!selectedDevices[devId]) selectedDevices[devId] = {model:d.model||'—',id:devId,consumption:'—',rssi:'—',mic:'?',lastSeen:now,count:0};
        const dev = selectedDevices[devId];
        if (now) dev.lastSeen = now; dev.count++;
        if (cons!=='—') dev.consumption=cons;
        if (d.rssi) dev.rssi=d.rssi;
        if (d.mic)  dev.mic=d.mic;
    } else {
        const k=(d.model||'?')+'|'+devId;
        if (!otherDevices[k]) otherDevices[k]={model:d.model||'—',id:devId,lastSeen:now,count:0};
        if (now) otherDevices[k].lastSeen = now; otherDevices[k].count++;
    }
    renderTables();
    if (d.rssi) document.getElementById('rssiLive').textContent='RSSI '+d.rssi+' dBm';
}

function renderTables() {
    // Preserve any unsaved values for rows currently in edit mode before re-render
    Object.keys(devEditState).forEach(id => {
        if (devEditState[id]) {
            const ne = document.getElementById('dname_'+id);
            const ue = document.getElementById('dunit_'+id);
            const se = document.getElementById('dscale_'+id);
            if (ne) deviceNamesMap[id]  = ne.value;
            if (ue) deviceUnitsMap[id]  = ue.value;
            if (se) deviceScalesMap[id] = se.value;
        }
    });
    const selBody = document.getElementById('selBody');
    selBody.innerHTML = '';
    const selKeys = Object.keys(selectedDevices);
    if (!selKeys.length) {
        selBody.innerHTML = '<tr><td colspan="11" style="color:#005f7a;text-align:center">No devices selected — check &#9744; a device below to enable MQTT</td></tr>';
    } else {
        selKeys.forEach(id => {
            const dev = selectedDevices[id];
            const curName  = deviceNamesMap[id]  || '';
            const curUnit  = deviceUnitsMap[id]  || '';
            const curScale = deviceScalesMap[id] !== undefined ? deviceScalesMap[id] : '1';
            const editing = devEditState[id] || false;
            const inC  = editing ? '#ffaa00' : '#888';
            const inB  = 'background:#000;border:1px solid #0a3040;padding:3px 6px;font-size:11px;border-radius:3px;color:'+inC;
            const rdonly = editing ? '' : 'readonly';
            const btnTxt  = editing ? '&#10004; Apply' : '&#9998; Edit';
            const btnClr  = editing ? 'border-color:#00ff88;color:#00ff88' : 'border-color:#00bcd4;color:#00e5ff';
            const tr = document.createElement('tr');
            tr.innerHTML =
                '<td style="text-align:center"><input type="checkbox" class="mqttcb" data-id="'+id+'" onchange="toggleMqtt(this)" checked></td>'+
                '<td><input id="dname_'+id+'" type="text" '+rdonly+' style="width:130px;'+inB+'" placeholder="Device ID '+id+'" value="'+curName+'"></td>'+
                '<td><input id="dunit_'+id+'" type="text" '+rdonly+' style="width:60px;'+inB+'" placeholder="CCF" value="'+curUnit+'"></td>'+
                '<td><input id="dscale_'+id+'" type="text" '+rdonly+' style="width:70px;'+inB+'" placeholder="1" value="'+curScale+'"></td>'+
                '<td style="white-space:nowrap"><button id="dedit_'+id+'" class="btn" style="'+btnClr+';padding:3px 8px;font-size:10px" onclick="devEdit(\''+id+'\')">'+btnTxt+'</button>'+
                ' <button id="ppub_'+id+'" class="btn" style="border-color:#ffaa00;color:#ffaa00;padding:3px 8px;font-size:10px" onclick="pubDevice(\''+id+'\')">&#8679; Pub</button></td>'+
                '<td>'+timeAgo(dev.lastSeen)+'</td><td>'+dev.model+'</td>'+
                '<td style="color:#00ff88;font-weight:bold">'+dev.id+'</td>'+
                '<td style="color:#00ff88">'+dev.consumption+'</td>'+
                '<td>'+dev.rssi+'</td>'+
                '<td style="color:#00ff88;font-weight:bold">'+fmtCount(dev.count)+'</td>';
            selBody.appendChild(tr);
        });
    }
    const othBody = document.getElementById('othBody');
    othBody.innerHTML = '';
    const othKeys = Object.keys(otherDevices);
    if (!othKeys.length) {
        othBody.innerHTML = '<tr><td colspan="5" style="color:#005f7a;text-align:center">Waiting for decode events…</td></tr>';
    } else {
        othKeys.sort((a,b)=>(otherDevices[b].lastSeen||0)-(otherDevices[a].lastSeen||0));
        othKeys.forEach(k => {
            const dev = otherDevices[k];
            const cb = dev.id ? '<input type="checkbox" class="selcb" data-id="'+dev.id+'">' : '';
            const tr = document.createElement('tr');
            tr.innerHTML =
                '<td style="text-align:center">'+cb+'</td>'+
                '<td>'+timeAgo(dev.lastSeen)+'</td><td>'+dev.model+'</td>'+
                '<td>'+(dev.id||'—')+'</td>'+
                '<td style="color:#555">'+fmtCount(dev.count)+'</td>';
            othBody.appendChild(tr);
        });
    }
}

function othAdd(){const cbs=document.querySelectorAll('#othBody .selcb:checked');if(!cbs.length)return;const promises=[];cbs.forEach(cb=>{const id=cb.dataset.id;promises.push(fetch('/mqtt-toggle?id='+id+'&on=1').then(r=>r.json()).then(res=>{if(res.ok){mqttSelected.add(id);const k=Object.keys(otherDevices).find(k=>otherDevices[k].id===id);if(k){const dev=otherDevices[k];selectedDevices[id]={model:dev.model,id,consumption:'—',rssi:'—',mic:'?',lastSeen:dev.lastSeen,count:dev.count};delete otherDevices[k];}}}));});Promise.all(promises).then(()=>{renderTables();fetch('/mqtt-selected').then(r=>r.json()).then(d=>{if(d.last)d.last.forEach(item=>{if(item)addPacket(JSON.stringify(item),true);});renderTables();}).catch(()=>{});});}
function othDelete(){const cbs=document.querySelectorAll('#othBody .selcb:checked');if(!cbs.length)return;const promises=[];cbs.forEach(cb=>{const id=cb.dataset.id;promises.push(fetch('/seen-delete?id='+id).then(r=>r.json()).then(res=>{if(res.ok){const k=Object.keys(otherDevices).find(k=>otherDevices[k].id===id);if(k)delete otherDevices[k];}}));});Promise.all(promises).then(()=>renderTables());}
// ── Registers — show/hide toggle, editable AGCCTRL rows ───────────────────
var REG_META = {
    IOCFG2:  {addr:'0x00', note:'GDO2 output signal select (0x29=CHIP_RDY)'},
    IOCFG1:  {addr:'0x01', note:'GDO1 output signal select (SDO in SPI mode)'},
    IOCFG0:  {addr:'0x02', note:'GDO0 output — 0x0D=async OOK data ✓  0x0E=carrier sense ✗'},
    PKTCTRL1:{addr:'0x07', note:'Packet control 1 — address check, CRC auto-flush'},
    PKTCTRL0:{addr:'0x08', note:'Data format — bits[5:4]: 00=normal, 03=async OOK ✓'},
    MDMCFG4: {addr:'0x10', note:'RX bandwidth [7:4] + data rate exponent [3:0]; 0x0B=812 kHz'},
    MDMCFG3: {addr:'0x11', note:'Data rate mantissa; 0x4A=65.46 kBaud (2× ERT chip rate)'},
    MDMCFG2: {addr:'0x12', note:'Modulation: 0x30=ASK/OOK, 0x00=2-FSK'},
    MDMCFG1: {addr:'0x13', note:'FEC enable + preamble byte count'},
    MDMCFG0: {addr:'0x14', note:'Channel spacing mantissa'},
    MCSM2:   {addr:'0x16', note:'RX time-out after sync word'},
    MCSM1:   {addr:'0x17', note:'CCA mode + next state after RX/TX end'},
    MCSM0:   {addr:'0x18', note:'Auto-calibrate timing + POR timeout'},
    AGCCTRL2:{addr:'0x1B', note:'[7:6] Max DVGA gain  [5:3] Max LNA  [2:0] magnitude target', editable:true, options:[
        {val:'0x07', desc:'0x07 — All DVGA steps, max LNA, 42 dB target (working ✓)'},
        {val:'0xC7', desc:'0xC7 — 3 DVGA steps disabled, max LNA, 42 dB (lib default, kills OOK)'},
        {val:'0x47', desc:'0x47 — 1 DVGA step disabled, max LNA, 42 dB'},
        {val:'0x87', desc:'0x87 — 2 DVGA steps disabled, max LNA, 42 dB'},
        {val:'0x04', desc:'0x04 — All DVGA steps, max LNA, 36 dB target'},
        {val:'0x03', desc:'0x03 — All DVGA steps, max LNA, 33 dB target'},
    ]},
    AGCCTRL1:{addr:'0x1C', note:'[6] sample rate  [5:4] CS relative threshold  [3:0] CS absolute threshold', editable:true, options:[
        {val:'0x40', desc:'0x40 — CS=-14 dB, chip rate samples (lib default)'},
        {val:'0x60', desc:'0x60 — CS=-6 dB, chip rate samples'},
        {val:'0x50', desc:'0x50 — CS=-10 dB, chip rate samples'},
        {val:'0x00', desc:'0x00 — No carrier sense threshold'},
    ]},
    AGCCTRL0:{addr:'0x1D', note:'[7:6] hysteresis  [5:4] filter/wait length  [3:2] freeze  [1:0] OOK averaging window', editable:true, options:[
        {val:'0xB3', desc:'0xB3 — 64-chip window, 64-sample wait, medium hyst (working ✓)'},
        {val:'0x91', desc:'0x91 — 16-chip window, 16-sample wait, medium hyst (lib default, too fast)'},
        {val:'0xF3', desc:'0xF3 — 64-chip window, 64-sample wait, large hyst'},
        {val:'0xB1', desc:'0xB1 — 16-chip window, 64-sample wait, medium hyst'},
        {val:'0xB0', desc:'0xB0 — 8-chip window,  64-sample wait, medium hyst'},
    ]},
};
function updateRegTable(regs) {
    var tbody = document.getElementById('regBody');
    tbody.innerHTML = '';
    Object.keys(REG_META).forEach(function(name) {
        var meta = REG_META[name];
        var val = regs[name];
        var hexVal = (val !== undefined) ? '0x' + val.toString(16).toUpperCase().padStart(2,'0') : '—';
        var editable = meta.editable === true;
        var writeCell = '';
        if (editable && val !== undefined && meta.options) {
            var currentHex = hexVal.toLowerCase();
            var opts = meta.options.map(function(o){
                var sel = (o.val.toLowerCase()===currentHex)?' selected':'';
                return '<option value="'+o.val+'"'+sel+'>'+o.desc+'</option>';
            }).join('');
            writeCell = '<select id="reg_'+name+'" style="background:#001a28;color:#ffaa00;border:1px solid #0a3040;padding:2px 5px;font-family:inherit;font-size:11px;max-width:340px;width:100%">'+opts+'</select>'+
                        '<button class="btn btn-apply" style="padding:2px 8px;font-size:10px;margin-top:4px;display:block" onclick="applyReg(\''+name+'\',\''+meta.addr+'\')">Set</button>';
        }
        var tr = document.createElement('tr');
        tr.innerHTML =
            '<td style="'+(editable?'color:#ffaa00;font-weight:bold':'')+'">'+name+'</td>'+
            '<td style="color:#005f7a">'+meta.addr+'</td>'+
            '<td id="regval_'+name+'" style="'+(editable?'color:#ffaa00;font-weight:bold':'color:#00ff88')+'">'+hexVal+'</td>'+
            '<td style="color:#888;font-size:10px">'+meta.note+'</td>'+
            '<td style="vertical-align:top">'+writeCell+'</td>';
        tbody.appendChild(tr);
    });
}
let regPanelVisible = false;
function toggleRegisters() {
    const panel = document.getElementById('regPanel');
    const btn   = document.getElementById('regToggleBtn');
    regPanelVisible = !regPanelVisible;
    panel.style.display = regPanelVisible ? '' : 'none';
    btn.innerHTML = regPanelVisible ? '&#9660; Hide Registers' : '&#9654; Show Registers';
    if (regPanelVisible) {
        document.getElementById('refreshNote').textContent = '…';
        fetch('/status').then(r=>r.json()).then(d=>{
            document.getElementById('sRssi').textContent = d.rssi + ' dBm';
            document.getElementById('sFreq').textContent = d.freq.toFixed(3) + ' MHz';
            document.getElementById('sGate').textContent = d.gate + ' dBm';
            document.getElementById('sGood').textContent = d.good;
            document.getElementById('rssiLive').textContent = 'RSSI ' + d.rssi + ' dBm';
            document.getElementById('refreshNote').textContent = 'updated ' + new Date().toLocaleTimeString();
        }).catch(()=>{ document.getElementById('refreshNote').textContent = 'error'; });
        fetch('/registers').then(r=>r.json()).then(regs=>{ updateRegTable(regs); }).catch(()=>{});
    }
}
function applyReg(name, addr) {
    var inp = document.getElementById('reg_'+name);
    if (!inp) return;
    var hexStr = inp.value.replace(/^0x/i,'').trim();
    var val = parseInt(hexStr, 16);
    if (isNaN(val) || val < 0 || val > 255) return;
    fetch('/ctrl?reg='+addr+'&val=0x'+val.toString(16).toUpperCase()).then(r=>r.json()).then(()=>{
        document.getElementById('regval_'+name).textContent = '0x'+val.toString(16).toUpperCase().padStart(2,'0');
    }).catch(()=>{});
}
function applyRssiThresh() {
    const t = parseInt(document.getElementById('rssiThreshIn').value);
    if (isNaN(t) || t < -20 || t > 20) return;
    fetch('/ctrl?rssi_thresh=' + t).then(r=>r.json()).then(()=>{
        document.getElementById('sRssiThr').textContent = t + ' dB';
    });
}
function resetDefaults() {
    const btn = document.getElementById('resetBtn');
    btn.textContent = '…';
    fetch('/reset-defaults').then(r=>r.json()).then(d=>{
        document.getElementById('freqIn').value = '914.224';
        document.getElementById('sFreq').textContent = '914.224 MHz';
        document.getElementById('gateSlider').value = '-95';
        document.getElementById('gateDisp').textContent = '-95 dBm';
        document.getElementById('sGate').textContent = '-95 dBm';
        document.getElementById('rssiThreshIn').value = '3';
        document.getElementById('sRssiThr').textContent = '3 dB';
        const bwSel = document.getElementById('bwSel');
        bwSel.value = String(d.bw);
        const bwText = bwSel.options[d.bw] ? bwSel.options[d.bw].text : '—';
        document.getElementById('sBw').textContent = bwText;
        if (regPanelVisible) fetch('/registers').then(r=>r.json()).then(regs=>{ updateRegTable(regs); }).catch(()=>{});
        btn.textContent = '✓ Done';
        setTimeout(()=>{ btn.innerHTML = '&#x21BA; Reset Defaults'; }, 2000);
    }).catch(()=>{ btn.innerHTML = '&#x21BA; Reset Defaults'; });
}

// ── Controls ──────────────────────────────────────────────────────────────
document.getElementById('gateSlider').addEventListener('input', function() {
    document.getElementById('gateDisp').textContent = this.value + ' dBm';
});
function applyFreq() {
    const f = parseFloat(document.getElementById('freqIn').value);
    if (f < 300 || f > 928) return;
    fetch('/ctrl?f=' + f.toFixed(3)).then(r=>r.json()).then(()=>{
        document.getElementById('sFreq').textContent = f.toFixed(3) + ' MHz';
    });
}
function applyGate() {
    const g = parseInt(document.getElementById('gateSlider').value);
    fetch('/ctrl?gate=' + g).then(r=>r.json()).then(()=>{
        document.getElementById('sGate').textContent = g + ' dBm';
    });
}
function applyBw() {
    const sel = document.getElementById('bwSel');
    const idx = parseInt(sel.value);
    fetch('/ctrl?bw=' + idx).then(r=>r.json()).then(()=>{
        document.getElementById('sBw').textContent = sel.options[idx].text;
    });
}
let espNameEditing = false;
function espNameToggle() {
    const inp = document.getElementById('espNameIn');
    const btn = document.getElementById('espNameBtn');
    if (!espNameEditing) {
        espNameEditing = true;
        inp.removeAttribute('readonly');
        inp.style.color = '#ffaa00';
        btn.innerHTML = '&#10004; Apply';
        btn.style.borderColor = '#00ff88';
        btn.style.color = '#00ff88';
        inp.focus();
    } else {
        const n = inp.value.trim();
        fetch('/ctrl?espname='+encodeURIComponent(n)).then(r=>r.json()).then(()=>{
            espNameEditing = false;
            inp.setAttribute('readonly','');
            inp.style.color = '#888';
            btn.innerHTML = '&#9998; Edit';
            btn.style.borderColor = '#00bcd4';
            btn.style.color = '#00e5ff';
            document.getElementById('espNameNote').textContent = 'saved';
            setTimeout(()=>document.getElementById('espNameNote').textContent='', 3000);
        }).catch(()=>{ document.getElementById('espNameNote').textContent='error'; });
    }
}
function pubDevice(id) {
    const btn = document.getElementById('ppub_'+id);
    if (!btn) return;
    fetch('/mqtt-publish?id='+id).then(r=>r.json()).then(d=>{
        btn.innerHTML = d.ok ? '&#10003; Sent' : '&#10007; No data';
        setTimeout(()=>{ btn.innerHTML='&#8679; Pub'; }, 2000);
    }).catch(()=>{ btn.innerHTML='&#10007; Err'; setTimeout(()=>{ btn.innerHTML='&#8679; Pub'; },2000); });
}
function devEdit(id) {
    const nameEl = document.getElementById('dname_'+id);
    const unitEl = document.getElementById('dunit_'+id);
    const btnEl  = document.getElementById('dedit_'+id);
    const scaleEl = document.getElementById('dscale_'+id);
    if (!nameEl) return;
    if (!devEditState[id]) {
        devEditState[id] = true;
        nameEl.removeAttribute('readonly');  nameEl.style.color  = '#ffaa00';
        unitEl.removeAttribute('readonly');  unitEl.style.color  = '#ffaa00';
        scaleEl.removeAttribute('readonly'); scaleEl.style.color = '#ffaa00';
        btnEl.innerHTML = '&#10004; Apply';
        btnEl.style.borderColor = '#00ff88'; btnEl.style.color = '#00ff88';
        nameEl.focus();
    } else {
        devEditState[id] = false;
        const nm = nameEl.value.trim();
        const un = unitEl.value.trim();
        let   sc = parseFloat(scaleEl.value);
        if (isNaN(sc) || sc <= 0) sc = 1;
        deviceNamesMap[id]  = nm;
        deviceUnitsMap[id]  = un;
        deviceScalesMap[id] = String(sc);
        scaleEl.value = String(sc); // normalize display
        nameEl.setAttribute('readonly','');  nameEl.style.color  = '#888';
        unitEl.setAttribute('readonly','');  unitEl.style.color  = '#888';
        scaleEl.setAttribute('readonly',''); scaleEl.style.color = '#888';
        btnEl.innerHTML = '&#9998; Edit';
        btnEl.style.borderColor = '#00bcd4'; btnEl.style.color = '#00e5ff';
        fetch('/ctrl?devname='+id+'&name='+encodeURIComponent(nm)).catch(()=>{});
        fetch('/ctrl?devunit='+id+'&unit='+encodeURIComponent(un)).catch(()=>{});
        fetch('/ctrl?devscale='+id+'&scale='+sc).catch(()=>{});
    }
}
// Load current frequency and gate from ESP32 on page open (so refresh doesn't reset to HTML defaults)
fetch('/status').then(r=>r.json()).then(d=>{
    document.getElementById('freqIn').value = d.freq.toFixed(3);
    document.getElementById('gateSlider').value = d.gate;
    document.getElementById('gateDisp').textContent = d.gate + ' dBm';
    document.getElementById('sFreq').textContent = d.freq.toFixed(3) + ' MHz';
    document.getElementById('sGate').textContent = d.gate + ' dBm';
    document.getElementById('sGood').textContent = d.good;
}).catch(()=>{});

// ── System status (top bar + errors panel) ───────────────────────────────
function updateTopBarHealth(d) {
    const b=document.getElementById('healthBadge'); if(!b) return;
    const dot=c=>'<span style="display:inline-block;width:8px;height:8px;border-radius:50%;background:'+c+';margin-left:4px;vertical-align:middle"></span>';
    const errs=[];
    if(!d.radio.ok) errs.push('Radio: not detected');
    if(!d.wifi.ok)  errs.push('WiFi down');
    if(!d.mqtt.ok)  errs.push('MQTT disconnected');
    if(!errs.length){
        b.style.borderColor='#00ff88'; b.style.color='#00ff88';
        b.innerHTML='SYS'+dot('#00ff88');
    } else {
        b.style.borderColor='#ff4444'; b.style.color='#ff4444';
        b.innerHTML=dot('#ff4444')+' '+errs.join(' | ');
    }
}
function refreshHealth() {
    fetch('/health').then(r=>r.json()).then(d=>{
        const nb=document.getElementById('nvsBadge');
        const ub=document.getElementById('uptimeBadge');
        if(nb) nb.textContent=d.nvs.devices+' device'+(d.nvs.devices!==1?'s':'');
        if(ub){const m=Math.floor(d.uptime/60),s=d.uptime%60;ub.textContent='Up '+m+'m'+(s?' '+s+'s':'');}
        const fb=document.getElementById('fwBadge');if(fb&&d.fw)fb.textContent=d.fw;
        if(d.bw!==undefined){const sel=document.getElementById('bwSel');if(sel){sel.value=String(d.bw);const sb=document.getElementById('sBw');if(sb)sb.textContent=sel.options[d.bw]?sel.options[d.bw].text:'—';}}
        if(d.rssiThr!==undefined){const ti=document.getElementById('rssiThreshIn');if(ti)ti.value=String(d.rssiThr);const st=document.getElementById('sRssiThr');if(st)st.textContent=d.rssiThr+' dB';}
        const errDot='<span style="display:inline-block;width:9px;height:9px;border-radius:50%;background:#ff4444;margin-right:8px;vertical-align:middle"></span>';
        const errs=[];
        if(!d.radio.ok) errs.push(errDot+'<b>Radio CC1101:</b> <span style="color:#ff4444">'+d.radio.msg+'</span>');
        if(!d.wifi.ok)  errs.push(errDot+'<b>WiFi:</b> <span style="color:#ff4444">Not connected</span>');
        if(!d.mqtt.ok)  errs.push(errDot+'<b>MQTT:</b> <span style="color:#ff4444">Disconnected ('+d.mqtt.broker+')</span>');
        const panel=document.getElementById('errorsPanel');
        if(panel){panel.style.display=errs.length?'':'none';if(errs.length)document.getElementById('errorsContent').innerHTML=errs.join('<br>');}
        updateTopBarHealth(d);
    }).catch(()=>{
        const panel=document.getElementById('errorsPanel');
        if(panel){panel.style.display='';document.getElementById('errorsContent').innerHTML='<span style="color:#ff4444">&#x26A0; Health check failed — server may be starting up</span>';}
    });
}
refreshHealth();
setInterval(refreshHealth,10000);
</script>
</body></html>
)HTML";

static void setupServer() {

    server.on("/", HTTP_GET, [](AsyncWebServerRequest* r){
        AsyncWebServerResponse* resp = r->beginResponse_P(200, "text/html", (const uint8_t*)PAGE, sizeof(PAGE)-1);
        r->send(resp);
    });

    server.on("/status", HTTP_GET, [](AsyncWebServerRequest* r){
        int rssi = rtl_433_ESP::signalRssi;
        char buf[128];
        snprintf(buf, sizeof(buf),
            "{\"rssi\":%d,\"freq\":%.3f,\"gate\":%d,\"good\":%lu}",
            rssi, rxFreq, rssiGate, goodDecodes);
        r->send(200, "application/json", buf);
    });

    server.on("/decoded", HTTP_GET, [](AsyncWebServerRequest* r){
        xSemaphoreTake(pktMutex, portMAX_DELAY);

        static char buf[8192];
        int p = snprintf(buf, sizeof(buf), "{\"good\":%lu,\"packets\":[", goodDecodes);
        int start = (pktCount < MAX_PKT) ? 0 : pktHead;
        bool first = true;
        for (int i = pktCount - 1; i >= 0; i--) {
            int idx = ((start + i) % MAX_PKT);
            if (!first) buf[p++] = ',';
            p += snprintf(buf + p, sizeof(buf) - p,
                "{\"ts\":%lu,\"json\":", pkts[idx].ts);

            p += snprintf(buf + p, sizeof(buf) - p, "%s}", pkts[idx].json);
            first = false;
            if (p > (int)sizeof(buf) - 100) break;
        }
        snprintf(buf + p, sizeof(buf) - p, "]}");
        xSemaphoreGive(pktMutex);
        r->send(200, "application/json", buf);
    });

    server.on("/ctrl", HTTP_GET, [](AsyncWebServerRequest* r){
        if (r->hasParam("f")) {
            float f = r->getParam("f")->value().toFloat();
            if (f >= 300 && f <= 928) {
                rxFreq = f;
                rtl_433_ESP::initReceiver(PIN_GDO0, f);
                rtl_433_ESP::enableReceiver();
                printf("[CTRL] Freq -> %.3f MHz\n", f);
            }
        }
        if (r->hasParam("gate")) {
            rssiGate = r->getParam("gate")->value().toInt();
            printf("[CTRL] Gate -> %d dBm\n", rssiGate);
        }
        if (r->hasParam("bw")) {
            int idx = r->getParam("bw")->value().toInt();
            if (idx >= 0 && idx < 6) {
                rxBwIdx = (uint8_t)idx;
                if (radioOk) radio.SPIwriteRegister(0x10, 0x0B | BW_NIBBLES[rxBwIdx]);
                nvsSave();
                printf("[CTRL] BW idx=%d -> MDMCFG4=0x%02X\n", idx, 0x0B | BW_NIBBLES[rxBwIdx]);
            }
        }
        if (r->hasParam("rssi_thresh")) {
            int t = r->getParam("rssi_thresh")->value().toInt();
            if (t >= -20 && t <= 20) {
                rssiThresh = t;
                if (radioOk) rf.setRSSIThreshold(rssiThresh);
                nvsSave();
                printf("[CTRL] RSSI HW threshold -> %d\n", rssiThresh);
            }
        }
        if (r->hasParam("reg") && r->hasParam("val")) {
            int addr = (int)strtol(r->getParam("reg")->value().c_str(), nullptr, 16);
            int val  = (int)strtol(r->getParam("val")->value().c_str(), nullptr, 16);
            if (addr >= 0 && addr <= 0x3D && val >= 0 && val <= 0xFF && radioOk) {
                radio.SPIwriteRegister((uint8_t)addr, (uint8_t)val);
                if (addr == 0x1B) agcCtrl2 = (uint8_t)val;
                if (addr == 0x1C) agcCtrl1 = (uint8_t)val;
                if (addr == 0x1D) agcCtrl0 = (uint8_t)val;
                nvsSave();
                printf("[CTRL] Reg 0x%02X <- 0x%02X\n", addr, val);
            }
        }
        if (r->hasParam("devscale") && r->hasParam("scale")) {
            uint32_t devId = (uint32_t)r->getParam("devscale")->value().toInt();
            float sc = r->getParam("scale")->value().toFloat();
            if (sc <= 0.0f) sc = 1.0f;
            xSemaphoreTake(mqttMutex, portMAX_DELAY);
            for (int i = 0; i < mqttIdCount; i++) {
                if (mqttIds[i] == devId) {
                    deviceScales[i] = sc;
                    nvsSave();
                    mqttNeedsDiscovery = true;
                    break;
                }
            }
            xSemaphoreGive(mqttMutex);
            printf("[CTRL] DevScale id=%lu -> %g\n", (unsigned long)devId, sc);
        }
        if (r->hasParam("devunit") && r->hasParam("unit")) {
            uint32_t devId = (uint32_t)r->getParam("devunit")->value().toInt();
            String u = r->getParam("unit")->value();
            u.trim();
            if (u.length() < 16) {
                xSemaphoreTake(mqttMutex, portMAX_DELAY);
                for (int i = 0; i < mqttIdCount; i++) {
                    if (mqttIds[i] == devId) {
                        strncpy(deviceUnits[i], u.c_str(), sizeof(deviceUnits[i]) - 1);
                        deviceUnits[i][sizeof(deviceUnits[i]) - 1] = '\0';
                        sanitizeName(deviceUnits[i]);
                        nvsSave();
                        mqttNeedsDiscovery = true;
                        break;
                    }
                }
                xSemaphoreGive(mqttMutex);
                printf("[CTRL] DevUnit id=%lu -> '%s'\n", (unsigned long)devId, u.c_str());
            }
        }
        if (r->hasParam("espname")) {
            String nm = r->getParam("espname")->value();
            nm.trim();
            if (nm.length() < 32) {
                xSemaphoreTake(mqttMutex, portMAX_DELAY);
                strncpy(espName, nm.c_str(), sizeof(espName) - 1);
                espName[sizeof(espName) - 1] = '\0';
                sanitizeName(espName);
                nvsSave();
                mqttNeedsDiscovery = true;
                xSemaphoreGive(mqttMutex);
                printf("[CTRL] EspName -> '%s'\n", espName);
            }
        }
        if (r->hasParam("devname") && r->hasParam("name")) {
            uint32_t devId = (uint32_t)r->getParam("devname")->value().toInt();
            String nm = r->getParam("name")->value();
            nm.trim();
            if (nm.length() < 32) {
                xSemaphoreTake(mqttMutex, portMAX_DELAY);
                for (int i = 0; i < mqttIdCount; i++) {
                    if (mqttIds[i] == devId) {
                        strncpy(deviceNames[i], nm.c_str(), sizeof(deviceNames[i]) - 1);
                        deviceNames[i][sizeof(deviceNames[i]) - 1] = '\0';
                        sanitizeName(deviceNames[i]);
                        nvsSave();
                        mqttNeedsDiscovery = true;
                        break;
                    }
                }
                xSemaphoreGive(mqttMutex);
                printf("[CTRL] DevName id=%lu -> '%s'\n", (unsigned long)devId, nm.c_str());
            }
        }
        r->send(200, "application/json", "{\"ok\":1}");
    });

    server.on("/health", HTTP_GET, [](AsyncWebServerRequest* r){
        char buf[420];
        snprintf(buf, sizeof(buf),
            "{\"radio\":{\"ok\":%s,\"msg\":\"%s\"},"
            "\"wifi\":{\"ok\":%s,\"ip\":\"%s\"},"
            "\"mqtt\":{\"ok\":%s,\"broker\":\"%s\"},"
            "\"nvs\":{\"devices\":%d},"
            "\"bw\":%u,"
            "\"rssiThr\":%d,"
            "\"fw\":\"%s\","
            "\"uptime\":%lu}",
            radioOk ? "true" : "false", radioErrMsg,
            WiFi.status() == WL_CONNECTED ? "true" : "false",
            WiFi.localIP().toString().c_str(),
            mqtt.connected() ? "true" : "false", MQTT_HOST,
            mqttIdCount,
            rxBwIdx,
            rssiThresh,
            FW_VERSION,
            millis() / 1000UL);
        r->send(200, "application/json", buf);
    });

    server.on("/reset-defaults", HTTP_GET, [](AsyncWebServerRequest* r){
        rxFreq     = 914.224f;
        rssiGate   = -95;
        rxBwIdx    = 0;
        rssiThresh = 3;
        agcCtrl2   = 0x07;
        agcCtrl1   = 0x40;
        agcCtrl0   = 0xB3;
        if (radioOk) {
            rtl_433_ESP::initReceiver(PIN_GDO0, rxFreq);
            rtl_433_ESP::enableReceiver();
            rf.setRSSIThreshold(rssiThresh);
            radio.SPIwriteRegister(0x10, 0x0B);
            radio.SPIwriteRegister(0x11, 0x4A);
            radio.SPIwriteRegister(0x1B, agcCtrl2);
            radio.SPIwriteRegister(0x1C, agcCtrl1);
            radio.SPIwriteRegister(0x1D, agcCtrl0);
        }
        nvsSave();
        printf("[RESET] All settings restored to working defaults\n");
        r->send(200, "application/json", "{\"ok\":1,\"bw\":0,\"rssiThr\":3}");
    });

    server.on("/registers", HTTP_GET, [](AsyncWebServerRequest* r){
        if (!radioOk) { r->send(200, "application/json", "{}"); return; }
        static const struct { const char* name; uint8_t addr; } REGS[] = {
            {"IOCFG2",  0x00}, {"IOCFG1",  0x01}, {"IOCFG0",  0x02},
            {"PKTCTRL1",0x07}, {"PKTCTRL0",0x08},
            {"MDMCFG4", 0x10}, {"MDMCFG3", 0x11}, {"MDMCFG2", 0x12},
            {"MDMCFG1", 0x13}, {"MDMCFG0", 0x14},
            {"MCSM2",   0x16}, {"MCSM1",   0x17}, {"MCSM0",   0x18},
            {"AGCCTRL2",0x1B}, {"AGCCTRL1",0x1C}, {"AGCCTRL0",0x1D},
        };
        static char buf[512];
        int p = 0;
        buf[p++] = '{';
        for (int i = 0; i < (int)(sizeof(REGS)/sizeof(REGS[0])); i++) {
            uint8_t val = radio.SPIreadRegister(REGS[i].addr);
            if (i > 0) buf[p++] = ',';
            p += snprintf(buf+p, sizeof(buf)-p, "\"%s\":%u", REGS[i].name, val);
        }
        buf[p++] = '}';
        buf[p] = '\0';

        radio.SPIsendCommand(RADIOLIB_CC1101_CMD_RX);
        printf("[REGS] Read complete, returned to RX\n");
        r->send(200, "application/json", buf);
    });

    server.on("/dbg", HTTP_GET, [](AsyncWebServerRequest* r){
        xSemaphoreTake(dbgMutex, portMAX_DELAY);
        static char buf[DBG_LINES * DBG_LINE_LEN + 64];
        int p = 0;
        buf[p++] = '[';
        int start = (dbgCount < DBG_LINES) ? 0 : dbgHead;
        bool first = true;
        for (int i = 0; i < dbgCount; i++) {
            int idx = (start + i) % DBG_LINES;
            if (!first) buf[p++] = ',';
            p += snprintf(buf+p, sizeof(buf)-p, "\"%s\"", dbgBuf[idx]);
            first = false;
        }
        buf[p++] = ']'; buf[p] = '\0';
        xSemaphoreGive(dbgMutex);
        r->send(200, "application/json", buf);
    });

    server.on("/mqtt-status", HTTP_GET, [](AsyncWebServerRequest* r){
        r->send(200, "application/json", mqtt.connected() ? "{\"connected\":true}" : "{\"connected\":false}");
    });

    server.on("/mqtt-toggle", HTTP_GET, [](AsyncWebServerRequest* r){
        if (!r->hasParam("id") || !r->hasParam("on")) {
            r->send(400, "application/json", "{\"ok\":0}"); return;
        }
        uint32_t id = (uint32_t)r->getParam("id")->value().toInt();
        bool on = r->getParam("on")->value() == "1";
        xSemaphoreTake(mqttMutex, portMAX_DELAY);
        if (on) mqttAddId(id); else mqttRemoveId(id);
        xSemaphoreGive(mqttMutex);
        printf("[MQTT] %s id=%lu\n", on?"select":"deselect", (unsigned long)id);
        r->send(200, "application/json", "{\"ok\":1}");
    });
    server.on("/seen-delete", HTTP_GET, [](AsyncWebServerRequest* r){
        if (!r->hasParam("id")) { r->send(400, "application/json", "{\"ok\":0}"); return; }
        uint32_t delId = (uint32_t)r->getParam("id")->value().toInt();
        xSemaphoreTake(mqttMutex, portMAX_DELAY);
        seenRemoveId(delId);
        seenDirty = true;
        xSemaphoreGive(mqttMutex);
        r->send(200, "application/json", "{\"ok\":1}");
    });

    server.on("/mqtt-selected", HTTP_GET, [](AsyncWebServerRequest* r){
        xSemaphoreTake(mqttMutex, portMAX_DELAY);
        static char buf[11000];
        int p = snprintf(buf, sizeof(buf), "{\"ids\":[");
        for (int i = 0; i < mqttIdCount; i++) {
            if (i > 0) buf[p++] = ',';
            p += snprintf(buf+p, sizeof(buf)-p, "%lu", (unsigned long)mqttIds[i]);
        }
        p += snprintf(buf+p, sizeof(buf)-p, "],\"last\":[");
        for (int i = 0; i < mqttIdCount; i++) {
            if (i > 0) buf[p++] = ',';
            if (mqttLastJson[i][0])
                p += snprintf(buf+p, sizeof(buf)-p, "%s", mqttLastJson[i]);
            else
                p += snprintf(buf+p, sizeof(buf)-p, "null");
        }
        p += snprintf(buf+p, sizeof(buf)-p, "],\"names\":[");
        for (int i = 0; i < mqttIdCount; i++) {
            if (i > 0) buf[p++] = ',';
            p += snprintf(buf+p, sizeof(buf)-p, "\"%s\"", deviceNames[i]);
        }
        p += snprintf(buf+p, sizeof(buf)-p, "],\"units\":[");
        for (int i = 0; i < mqttIdCount; i++) {
            if (i > 0) buf[p++] = ',';
            p += snprintf(buf+p, sizeof(buf)-p, "\"%s\"", deviceUnits[i]);
        }
        p += snprintf(buf+p, sizeof(buf)-p, "],\"scales\":[");
        for (int i = 0; i < mqttIdCount; i++) {
            if (i > 0) buf[p++] = ',';
            p += snprintf(buf+p, sizeof(buf)-p, "%g", deviceScales[i]);
        }
        p += snprintf(buf+p, sizeof(buf)-p, "],\"ages\":[");
        time_t nowEpoch; time(&nowEpoch);
        bool ntpOk = (nowEpoch > 1000000000L);
        uint32_t nowMs = millis();
        for (int i = 0; i < mqttIdCount; i++) {
            if (i > 0) buf[p++] = ',';
            long age = -1;
            if (ntpOk && lastSeenTime[i] > 0)
                age = (long)(nowEpoch - lastSeenTime[i]);
            else if (lastSeenMs[i] > 0)
                age = (long)((nowMs - lastSeenMs[i]) / 1000);
            p += snprintf(buf+p, sizeof(buf)-p, "%ld", age);
        }
        p += snprintf(buf+p, sizeof(buf)-p, "],\"counts\":[");
        for (int i = 0; i < mqttIdCount; i++) {
            if (i > 0) buf[p++] = ',';
            p += snprintf(buf+p, sizeof(buf)-p, "%lu", (unsigned long)deviceDecodeCounts[i]);
        }
        p += snprintf(buf+p, sizeof(buf)-p, "],\"espname\":\"%s\",\"seen\":[", espName);
        for (int i = 0; i < seenIdCount; i++) {
            if (i > 0) buf[p++] = ',';
            p += snprintf(buf+p, sizeof(buf)-p, "%lu", (unsigned long)seenIds[i]);
        }
        p += snprintf(buf+p, sizeof(buf)-p, "],\"seenAges\":[");
        { time_t _ne; time(&_ne); bool _nok = (_ne > 1000000000L);
          for (int i = 0; i < seenIdCount; i++) { if (i) buf[p++] = ','; long _a = (_nok && seenLastTime[i] > 0) ? (long)(_ne - seenLastTime[i]) : -1L; p += snprintf(buf+p, sizeof(buf)-p, "%ld", _a); } }
        p += snprintf(buf+p, sizeof(buf)-p, "],\"seenCounts\":[");
        for (int i = 0; i < seenIdCount; i++) { if (i) buf[p++] = ','; p += snprintf(buf+p, sizeof(buf)-p, "%lu", (unsigned long)seenDecodeCount[i]); }
        p += snprintf(buf+p, sizeof(buf)-p, "],\"seenModels\":[");
        for (int i = 0; i < seenIdCount; i++) {
            if (i) buf[p++] = ',';
            const char* ms = strstr(seenLastJson[i], "\"model\":\"");
            if (ms) { ms += 9; const char* me = strchr(ms, '"'); if (me && (me-ms) < 32) { buf[p++]='"'; memcpy(buf+p,ms,me-ms); p+=me-ms; buf[p++]='"'; buf[p]='\0'; continue; } }
            p += snprintf(buf+p, sizeof(buf)-p, "\"ERT\"");
        }
        p += snprintf(buf+p, sizeof(buf)-p, "]}");
        xSemaphoreGive(mqttMutex);
        r->send(200, "application/json", buf);
    });

    server.on("/mqtt-publish", HTTP_GET, [](AsyncWebServerRequest* r){
        if (!r->hasParam("id")) { r->send(400, "application/json", "{\"ok\":0}"); return; }
        uint32_t devId = (uint32_t)r->getParam("id")->value().toInt();
        xSemaphoreTake(mqttMutex, portMAX_DELAY);
        bool found = false;
        for (int i = 0; i < mqttIdCount; i++) {
            if (mqttIds[i] == devId && mqttLastJson[i][0]) {
                char pubTopic[80];
                getStateTopic(i, pubTopic, sizeof(pubTopic));
                mqttEnqueue(pubTopic, mqttLastJson[i]);
                found = true;
                printf("[MQTT] Manual publish id=%lu -> %s\n", (unsigned long)devId, pubTopic);
                break;
            }
        }
        xSemaphoreGive(mqttMutex);
        r->send(200, "application/json", found ? "{\"ok\":1}" : "{\"ok\":0,\"err\":\"no data yet\"}");
    });

    events.onConnect([](AsyncEventSourceClient* c){
        printf("[SSE] Client connected, last=%u\n", c->lastId());
    });
    server.addHandler(&events);

    server.begin();
    printf("[HTTP] Server started\n");
}

static bool cc1101Present() {
    SPI.begin(PIN_SCK, PIN_MISO, PIN_MOSI);
    SPI.beginTransaction(SPISettings(500000, MSBFIRST, SPI_MODE0));
    pinMode(PIN_CS, OUTPUT);
    delay(2);
    uint8_t v1, v2;

    digitalWrite(PIN_CS, LOW); delayMicroseconds(10);
    SPI.transfer(0xF1); v1 = SPI.transfer(0x00);
    digitalWrite(PIN_CS, HIGH); delayMicroseconds(20);
    digitalWrite(PIN_CS, LOW); delayMicroseconds(10);
    SPI.transfer(0xF1); v2 = SPI.transfer(0x00);
    digitalWrite(PIN_CS, HIGH);
    SPI.endTransaction();
    SPI.end();
    printf("[CC1101] probe: VERSION=0x%02X 0x%02X (expect 0x14)\n", v1, v2);
    bool validVer = (v1 == 0x04 || v1 == 0x07 || v1 == 0x14);
    return (v1 == v2 && validVer);
}

static void nvsTask(void*);
void setup() {
    Serial.begin(115200);
    delay(500);
    printf("\n[BOOT] %s\n", FW_VERSION);

    pktMutex  = xSemaphoreCreateMutex();
    dbgMutex  = xSemaphoreCreateMutex();
    mqttMutex    = xSemaphoreCreateMutex();
    mqttPubMutex = xSemaphoreCreateMutex();
    nvsLoad();
    nvsLoadSeen();

    WiFi.setAutoReconnect(true);
    WiFi.begin(WIFI_SSID, WIFI_PASS);
    printf("[WIFI] Connecting");
    uint32_t t0 = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - t0 < 15000) {
        delay(500); printf(".");
    }
    if (WiFi.status() == WL_CONNECTED) {
        printf("\n[WIFI] %s\n", WiFi.localIP().toString().c_str());
        configTime(0, 0, "pool.ntp.org", "time.nist.gov");
        printf("[NTP] Syncing...");
        time_t t = 0; int tries = 0;
        while (t < 1000000000L && tries++ < 20) { delay(200); time(&t); }
        printf(" %s\n", t > 1000000000L ? "OK" : "failed (will retry)");
    } else {
        printf("\n[WIFI] Failed -- OTA unavailable\n");
    }

    ArduinoOTA.setHostname("ert-pulse");
    ArduinoOTA.begin();

    mqtt.setServer(MQTT_HOST, MQTT_PORT);
    mqtt.setBufferSize(600);
    mqttEnsureConnected();
    printf("[MQTT] broker=%s:%d client=%s\n", MQTT_HOST, MQTT_PORT, MQTT_CLIENT_ID);

    setupServer();
    printf("[READY] %s  http://%s\n", FW_VERSION, WiFi.localIP().toString().c_str());

    if (cc1101Present()) {
        radioOk = true;
        snprintf(radioErrMsg, sizeof(radioErrMsg), "OK — %.3f MHz", rxFreq);
        rf.initReceiver(PIN_GDO0, rxFreq);
        rf.setCallback(rtl433Callback, rfMsgBuf, sizeof(rfMsgBuf));
        rf.enableReceiver();
        rf.setRSSIThreshold(rssiThresh);
        printf("[RTL433] Receiver enabled on GDO0=GPIO%d  %.3f MHz  rssiThr=%d\n", PIN_GDO0, rxFreq, rssiThresh);
        printf("[RTL433] MINIMUM_PULSE_LENGTH=%d us  MINIMUM_SIGNAL_LENGTH=%d us\n",
               MINIMUM_PULSE_LENGTH, MINIMUM_SIGNAL_LENGTH);

        radio.SPIwriteRegister(0x10, 0x0B | BW_NIBBLES[rxBwIdx]);
        radio.SPIwriteRegister(0x11, 0x4A);
        printf("[MDM] setup: MDMCFG4=0x%02X  MDMCFG3=0x%02X  (~65.46 kBaud)\n",
            radio.SPIreadRegister(0x10), radio.SPIreadRegister(0x11));
        radio.SPIwriteRegister(0x1B, agcCtrl2);
        radio.SPIwriteRegister(0x1D, agcCtrl0);
        lastAgcMs = millis();
        printf("[AGC] setup: AGCCTRL2=0x%02X  AGCCTRL0=0x%02X\n",
            radio.SPIreadRegister(0x1B), radio.SPIreadRegister(0x1D));
    } else {
        snprintf(radioErrMsg, sizeof(radioErrMsg),
                 "CC1101 not detected — check wiring (SPI probe returned no chip)");
        printf("[CC1101] Not detected — radio disabled. Web server running normally.\n");
    }
    xTaskCreatePinnedToCore(nvsTask, "nvsTask", 4096, nullptr, 1, nullptr, 0);
}

static void nvsTask(void*) {
    uint32_t tick = 0;
    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(10000));
        if (mqttJsonDirty) nvsSaveLastJson();
        nvsSaveLastSeen();
        nvsSaveSeenDirty();
        if (++tick >= 3) { tick = 0; if (seenDirty) nvsSaveSeen(); }
    }
}

void loop() {
    ArduinoOTA.handle();
    if (radioOk) rf.loop();
    mqtt.loop();
    mqttDrainQueue();

    bool nowConnected = mqtt.connected();
    if (nowConnected && (!mqttDiscoveryDone || mqttNeedsDiscovery)) {
        xSemaphoreTake(mqttMutex, portMAX_DELAY);
        for (int i = 0; i < mqttIdCount; i++) publishDiscovery(mqttIds[i], i);
        xSemaphoreGive(mqttMutex);
        mqttDiscoveryDone  = true;
        mqttNeedsDiscovery = false;
    }
    if (!nowConnected) mqttDiscoveryDone = false;

    static uint32_t lastWifiRetry = 0;
    if (WiFi.status() != WL_CONNECTED) {
        uint32_t nowMs = millis();
        if (nowMs - lastWifiRetry > 30000) {
            lastWifiRetry = nowMs;
            printf("[WIFI] Reconnecting...\n");
            WiFi.reconnect();
        }
    }

    static uint32_t lastMqttRetry = 0;
    if (!mqtt.connected() && WiFi.status() == WL_CONNECTED) {
        uint32_t nowMs = millis();
        if (nowMs - lastMqttRetry > 10000) {
            lastMqttRetry = nowMs;
            mqttEnsureConnected();
        }
    }

    uint32_t now = millis();

    if (radioOk && now - lastStatusMs >= 60000) {
        lastStatusMs = now;
        rf.getStatus();
    }

    if (radioOk && now - lastAgcMs >= 5000) {
        lastAgcMs = now;

        uint8_t mdm4 = radio.SPIreadRegister(0x10);
        uint8_t mdm3 = radio.SPIreadRegister(0x11);
        uint8_t targetMdm4 = 0x0B | BW_NIBBLES[rxBwIdx];
        if (mdm4 != targetMdm4 || mdm3 != 0x4A) {
            radio.SPIwriteRegister(0x10, targetMdm4);
            radio.SPIwriteRegister(0x11, 0x4A);
            printf("[MDM] re-applied (was 0x%02X/0x%02X) -> 0x%02X/0x4A\n", mdm4, mdm3, targetMdm4);
        }

        uint8_t agc2 = radio.SPIreadRegister(0x1B);
        uint8_t agc0 = radio.SPIreadRegister(0x1D);
        if (agc2 != agcCtrl2 || agc0 != agcCtrl0) {
            radio.SPIwriteRegister(0x1B, agcCtrl2);
            radio.SPIwriteRegister(0x1D, agcCtrl0);
            printf("[AGC] re-applied (was 0x%02X/0x%02X) -> 0x%02X/0x%02X\n", agc2, agc0, agcCtrl2, agcCtrl0);
        }
    }
}
