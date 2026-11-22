#include "wifi_board.h"
#include "codecs/no_audio_codec.h"
#include "display/lcd_display.h"
#include "system_reset.h"
#include "application.h"
#include "button.h"
#include "config.h"
#include "mcp_server.h"
#include "led/circular_strip.h"

#include <esp_log.h>
#include <driver/i2c_master.h>
#include <driver/rtc_io.h>
#include <esp_lcd_panel_vendor.h>
#include <esp_lcd_panel_io.h>
#include <esp_lcd_panel_ops.h>
#include <driver/spi_common.h>
#include <esp_sleep.h>
#include <esp_adc/adc_oneshot.h>
#include <esp_adc/adc_cali.h>
#include <esp_adc/adc_cali_scheme.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <lvgl.h>
#include <vector>
#include <string>
#include <cstring>
#include <cstdlib>
#include <cmath>

#define TAG "KosmonavtLcdBoard"

// Отдельный пин и количество диодов для «парящей» подсветки подиума
#define PODIUM_STRIP_PIN   GPIO_NUM_38
#define PODIUM_STRIP_COUNT 12

// Заглушка для системного Led, чтобы прошивка сама не перебивала цвета шлема и подиума
class DummySystemLed : public Led {
public:
    virtual void OnStateChanged() override {}
};

// Кастомный дисплей с крупными живыми глазами (белки, зрачки, блики) и полным засыпанием
class KosmonavtEyeDisplay : public SpiLcdDisplay {
private:
    lv_obj_t* eye_layer_ = nullptr;
    lv_obj_t* left_eye_ = nullptr;
    lv_obj_t* right_eye_ = nullptr;
    lv_obj_t* left_pupil_ = nullptr;
    lv_obj_t* right_pupil_ = nullptr;
    lv_obj_t* left_glint_ = nullptr;
    lv_obj_t* right_glint_ = nullptr;
    lv_obj_t* status_bar_ = nullptr;
    lv_timer_t* blink_timer_ = nullptr;
    lv_timer_t* look_timer_ = nullptr;

    int target_h_ = 104;
    int target_r_ = 32;
    int target_y_ = 58;
    int pupil_dx_ = 0;
    int pupil_dy_ = 0;
    bool is_blinking_ = false;
    bool is_sleeping_ = false;

    void UpdatePupilPositions() {
        if (!left_pupil_ || !right_pupil_) return;
        int pw = 28;
        int ph = (target_h_ < 44) ? (target_h_ - 8) : 36;
        if (ph < 8) ph = 8;

        lv_obj_set_size(left_pupil_, pw, ph);
        lv_obj_set_size(right_pupil_, pw, ph);

        int px = (74 - pw) / 2 + pupil_dx_;
        int py = (target_h_ - ph) / 2 + pupil_dy_;

        if (px < 6) px = 6;
        if (px > 74 - pw - 6) px = 74 - pw - 6;
        if (py < 4) py = 4;
        if (py > target_h_ - ph - 4) py = target_h_ - ph - 4;

        lv_obj_set_pos(left_pupil_, px, py);
        lv_obj_set_pos(right_pupil_, px, py);
    }

    void EnsureEyeLayer() {
        if (eye_layer_) return;
        lv_obj_t* scr = lv_scr_act();
        if (!scr) return;

        eye_layer_ = lv_obj_create(scr);
        lv_obj_set_size(eye_layer_, 240, 240);
        lv_obj_set_pos(eye_layer_, 0, 0);
        lv_obj_set_style_bg_color(eye_layer_, lv_color_black(), 0);
        lv_obj_set_style_bg_opa(eye_layer_, LV_OPA_COVER, 0);
        lv_obj_set_style_border_width(eye_layer_, 0, 0);
        lv_obj_set_style_pad_all(eye_layer_, 0, 0);
        lv_obj_clear_flag(eye_layer_, LV_OBJ_FLAG_SCROLLABLE);

        left_eye_ = lv_obj_create(eye_layer_);
        lv_obj_set_size(left_eye_, 74, 104);
        lv_obj_set_pos(left_eye_, 32, 58);
        lv_obj_set_style_bg_color(left_eye_, lv_color_white(), 0);
        lv_obj_set_style_bg_opa(left_eye_, LV_OPA_COVER, 0);
        lv_obj_set_style_border_width(left_eye_, 0, 0);
        lv_obj_set_style_pad_all(left_eye_, 0, 0);
        lv_obj_set_style_radius(left_eye_, 32, 0);
        lv_obj_clear_flag(left_eye_, LV_OBJ_FLAG_SCROLLABLE);

        right_eye_ = lv_obj_create(eye_layer_);
        lv_obj_set_size(right_eye_, 74, 104);
        lv_obj_set_pos(right_eye_, 134, 58);
        lv_obj_set_style_bg_color(right_eye_, lv_color_white(), 0);
        lv_obj_set_style_bg_opa(right_eye_, LV_OPA_COVER, 0);
        lv_obj_set_style_border_width(right_eye_, 0, 0);
        lv_obj_set_style_pad_all(right_eye_, 0, 0);
        lv_obj_set_style_radius(right_eye_, 32, 0);
        lv_obj_clear_flag(right_eye_, LV_OBJ_FLAG_SCROLLABLE);

        left_pupil_ = lv_obj_create(left_eye_);
        lv_obj_set_size(left_pupil_, 28, 36);
        lv_obj_set_style_bg_color(left_pupil_, lv_color_black(), 0);
        lv_obj_set_style_bg_opa(left_pupil_, LV_OPA_COVER, 0);
        lv_obj_set_style_border_width(left_pupil_, 0, 0);
        lv_obj_set_style_pad_all(left_pupil_, 0, 0);
        lv_obj_set_style_radius(left_pupil_, 14, 0);
        lv_obj_clear_flag(left_pupil_, LV_OBJ_FLAG_SCROLLABLE);

        right_pupil_ = lv_obj_create(right_eye_);
        lv_obj_set_size(right_pupil_, 28, 36);
        lv_obj_set_style_bg_color(right_pupil_, lv_color_black(), 0);
        lv_obj_set_style_bg_opa(right_pupil_, LV_OPA_COVER, 0);
        lv_obj_set_style_border_width(right_pupil_, 0, 0);
        lv_obj_set_style_pad_all(right_pupil_, 0, 0);
        lv_obj_set_style_radius(right_pupil_, 14, 0);
        lv_obj_clear_flag(right_pupil_, LV_OBJ_FLAG_SCROLLABLE);

        left_glint_ = lv_obj_create(left_pupil_);
        lv_obj_set_size(left_glint_, 8, 8);
        lv_obj_set_pos(left_glint_, 5, 6);
        lv_obj_set_style_bg_color(left_glint_, lv_color_white(), 0);
        lv_obj_set_style_bg_opa(left_glint_, LV_OPA_COVER, 0);
        lv_obj_set_style_border_width(left_glint_, 0, 0);
        lv_obj_set_style_radius(left_glint_, 4, 0);

        right_glint_ = lv_obj_create(right_pupil_);
        lv_obj_set_size(right_glint_, 8, 8);
        lv_obj_set_pos(right_glint_, 5, 6);
        lv_obj_set_style_bg_color(right_glint_, lv_color_white(), 0);
        lv_obj_set_style_bg_opa(right_glint_, LV_OPA_COVER, 0);
        lv_obj_set_style_border_width(right_glint_, 0, 0);
        lv_obj_set_style_radius(right_glint_, 4, 0);

        UpdatePupilPositions();

        status_bar_ = lv_obj_create(eye_layer_);
        lv_obj_set_size(status_bar_, 72, 6);
        lv_obj_set_pos(status_bar_, 84, 196);
        lv_obj_set_style_bg_color(status_bar_, lv_color_make(0, 180, 255), 0);
        lv_obj_set_style_bg_opa(status_bar_, LV_OPA_COVER, 0);
        lv_obj_set_style_border_width(status_bar_, 0, 0);
        lv_obj_set_style_radius(status_bar_, 3, 0);

        blink_timer_ = lv_timer_create(BlinkTimerCb, 3400, this);
        look_timer_ = lv_timer_create(LookTimerCb, 1900, this);
    }

    static void BlinkTimerCb(lv_timer_t* timer) {
        auto* self = static_cast<KosmonavtEyeDisplay*>(lv_timer_get_user_data(timer));
        if (!self || self->is_sleeping_ || !self->left_eye_ || !self->right_eye_) return;
        lv_obj_move_foreground(self->eye_layer_);
        if (!self->is_blinking_) {
            self->is_blinking_ = true;
            lv_obj_add_flag(self->left_pupil_, LV_OBJ_FLAG_HIDDEN);
            lv_obj_add_flag(self->right_pupil_, LV_OBJ_FLAG_HIDDEN);
            lv_obj_set_height(self->left_eye_, 8);
            lv_obj_set_height(self->right_eye_, 8);
            lv_obj_set_y(self->left_eye_, self->target_y_ + self->target_h_ / 2);
            lv_obj_set_y(self->right_eye_, self->target_y_ + self->target_h_ / 2);
            lv_timer_set_period(timer, 130);
        } else {
            self->is_blinking_ = false;
            lv_obj_clear_flag(self->left_pupil_, LV_OBJ_FLAG_HIDDEN);
            lv_obj_clear_flag(self->right_pupil_, LV_OBJ_FLAG_HIDDEN);
            lv_obj_set_height(self->left_eye_, self->target_h_);
            lv_obj_set_height(self->right_eye_, self->target_h_);
            lv_obj_set_y(self->left_eye_, self->target_y_);
            lv_obj_set_y(self->right_eye_, self->target_y_);
            self->UpdatePupilPositions();
            lv_timer_set_period(timer, 3400);
        }
    }

    static void LookTimerCb(lv_timer_t* timer) {
        auto* self = static_cast<KosmonavtEyeDisplay*>(lv_timer_get_user_data(timer));
        if (!self || self->is_sleeping_ || self->is_blinking_) return;
        static const int offsets[][2] = {{0, 0}, {-10, -4}, {10, -4}, {0, 0}, {-8, 6}, {8, 6}, {0, -8}};
        int idx = rand() % 7;
        self->pupil_dx_ = offsets[idx][0];
        self->pupil_dy_ = offsets[idx][1];
        self->UpdatePupilPositions();
    }

    void ApplyEyeShape(int h, int r, int y_offset, int dx = 0, int dy = 0) {
        EnsureEyeLayer();
        if (!left_eye_ || !right_eye_) return;
        lv_obj_move_foreground(eye_layer_);
        target_h_ = h;
        target_r_ = r;
        target_y_ = y_offset;
        pupil_dx_ = dx;
        pupil_dy_ = dy;
        if (!is_blinking_) {
            lv_obj_set_height(left_eye_, target_h_);
            lv_obj_set_height(right_eye_, target_h_);
            lv_obj_set_style_radius(left_eye_, target_r_, 0);
            lv_obj_set_style_radius(right_eye_, target_r_, 0);
            lv_obj_set_y(left_eye_, target_y_);
            lv_obj_set_y(right_eye_, target_y_);
            UpdatePupilPositions();
        }
    }

    void SetScreenSleep(bool sleep) {
        EnsureEyeLayer();
        if (!eye_layer_) return;
        is_sleeping_ = sleep;
        auto* bl = Board::GetInstance().GetBacklight();
        if (sleep) {
            lv_obj_add_flag(eye_layer_, LV_OBJ_FLAG_HIDDEN);
            if (bl) bl->SetBrightness(0);
        } else {
            lv_obj_clear_flag(eye_layer_, LV_OBJ_FLAG_HIDDEN);
            lv_obj_move_foreground(eye_layer_);
            if (bl) bl->RestoreBrightness();
        }
    }

public:
    KosmonavtEyeDisplay(esp_lcd_panel_io_handle_t io, esp_lcd_panel_handle_t panel,
                        int width, int height, int offset_x, int offset_y,
                        bool mirror_x, bool mirror_y, bool swap_xy)
        : SpiLcdDisplay(io, panel, width, height, offset_x, offset_y, mirror_x, mirror_y, swap_xy) {
        DisplayLockGuard lock(this);
        EnsureEyeLayer();
    }

    virtual bool SetEmotion(const char* emotion) override {
        if (!emotion) return false;
        DisplayLockGuard lock(this);
        if (strcmp(emotion, "sleep") == 0 || strcmp(emotion, "sleepy") == 0) {
        SetScreenSleep(true);
            return true;
        }
        SetScreenSleep(false);
        if (strcmp(emotion, "happy") == 0 || strcmp(emotion, "laughing") == 0 || strcmp(emotion, "loving") == 0) {
            ApplyEyeShape(58, 28, 80, 0, -4);
        } else if (strcmp(emotion, "surprised") == 0 || strcmp(emotion, "shocked") == 0) {
            ApplyEyeShape(118, 36, 50, 0, 0);
        } else if (strcmp(emotion, "sad") == 0 || strcmp(emotion, "crying") == 0) {
            ApplyEyeShape(46, 20, 96, 0, 10);
        } else if (strcmp(emotion, "thinking") == 0 || strcmp(emotion, "confused") == 0) {
            ApplyEyeShape(72, 24, 72, 12, -14);
        } else {
            ApplyEyeShape(104, 32, 58, 0, 0);
        }
        return true;
    }

    virtual void SetStatus(const char* status) override {
        if (!status) return;
        DisplayLockGuard lock(this);
        EnsureEyeLayer();
        if (!status_bar_) return;

        if (strstr(status, "Listening") || strstr(status, "Слушаю")) {
            SetScreenSleep(false);
            lv_obj_set_style_bg_color(status_bar_, lv_color_make(0, 255, 100), 0);
            ApplyEyeShape(112, 34, 54, 0, 0);
        } else if (strstr(status, "Speaking") || strstr(status, "Говорю")) {
            SetScreenSleep(false);
            lv_obj_set_style_bg_color(status_bar_, lv_color_make(0, 180, 255), 0);
            ApplyEyeShape(96, 30, 62, 0, 0);
        } else if (strstr(status, "Connecting") || strstr(status, "Подключение") ||
                   strstr(status, "Starting") || strstr(status, "Запуск") ||
                   strstr(status, "Upgrading") || strstr(status, "Обновление")) {
            SetScreenSleep(false);
            lv_obj_set_style_bg_color(status_bar_, lv_color_make(255, 180, 0), 0);
            ApplyEyeShape(84, 28, 68, 0, -10);
        } else {
            SetScreenSleep(true);
        }
    }

    virtual void SetChatMessage(const char* role, const char* content) override {
        DisplayLockGuard lock(this);
        EnsureEyeLayer();
    }
};

// Датчик BMP280 с авто-восстановлением связи на 50 кГц
class SimpleBmp280 {
private:
    i2c_master_bus_handle_t bus_ = nullptr;
    i2c_master_dev_handle_t dev_handle_ = nullptr;
    bool initialized_ = false;
    uint16_t dig_T1_ = 0, dig_P1_ = 0;
    int16_t dig_T2_ = 0, dig_T3_ = 0, dig_P2_ = 0, dig_P3_ = 0, dig_P4_ = 0;
    int16_t dig_P5_ = 0, dig_P6_ = 0, dig_P7_ = 0, dig_P8_ = 0, dig_P9_ = 0;

    bool ReadReg(uint8_t reg, uint8_t* data, size_t len) {
        if (!dev_handle_) return false;
        return i2c_master_transmit_receive(dev_handle_, &reg, 1, data, len, 150) == ESP_OK;
    }
    bool WriteReg(uint8_t reg, uint8_t val) {
        if (!dev_handle_) return false;
        std::vector<uint8_t> buf = {reg, val};
        return i2c_master_transmit(dev_handle_, buf.data(), buf.size(), 150) == ESP_OK;
    }

public:
    bool Init(i2c_master_bus_handle_t bus) {
        if (!bus) return false;
        bus_ = bus;
        if (dev_handle_) {
            i2c_master_bus_rm_device(dev_handle_);
            dev_handle_ = nullptr;
        }
        initialized_ = false;

        std::vector<uint8_t> addrs = {0x76, 0x77};
        for (uint8_t addr : addrs) {
            i2c_device_config_t dev_cfg = {
                .dev_addr_length = I2C_ADDR_BIT_LEN_7,
                .device_address = addr,
                .scl_speed_hz = 50000, // 50 кГц — максимально надёжно на длинных проводах в подиум
            };
            if (i2c_master_bus_add_device(bus_, &dev_cfg, &dev_handle_) == ESP_OK) {
                uint8_t id = 0;
                if (ReadReg(0xD0, &id, 1) && (id == 0x58 || id == 0x60)) {
                    std::vector<uint8_t> c(24, 0);
                    if (ReadReg(0x88, c.data(), 24)) {
                        dig_T1_ = (c.at(1) << 8) | c.at(0);
                        dig_T2_ = (c.at(3) << 8) | c.at(2);
                        dig_T3_ = (c.at(5) << 8) | c.at(4);
                        dig_P1_ = (c.at(7) << 8) | c.at(6);
                        dig_P2_ = (c.at(9) << 8) | c.at(8);
                        dig_P3_ = (c.at(11) << 8) | c.at(10);
                        dig_P4_ = (c.at(13) << 8) | c.at(12);
                        dig_P5_ = (c.at(15) << 8) | c.at(14);
                        dig_P6_ = (c.at(17) << 8) | c.at(16);
                        dig_P7_ = (c.at(19) << 8) | c.at(18);
                        dig_P8_ = (c.at(21) << 8) | c.at(20);
                        dig_P9_ = (c.at(23) << 8) | c.at(22);
                        WriteReg(0xF4, 0x27);
                        WriteReg(0xF5, 0xA0);
                        initialized_ = true;
                        ESP_LOGI(TAG, "BMP280 найден по адресу 0x%02X (ID=0x%02X)", addr, id);
                        return true;
                    }
                }
                i2c_master_bus_rm_device(dev_handle_);
                dev_handle_ = nullptr;
            }
        }
        return false;
    }

    bool Read(float& temp_c, float& press_mmhg) {
        if (!initialized_) {
            if (!Init(bus_)) return false;
        }
        std::vector<uint8_t> d(6, 0);
        if (!ReadReg(0xF7, d.data(), 6)) {
            if (!Init(bus_) || !ReadReg(0xF7, d.data(), 6)) {
                return false;
            }
        }

        int32_t adc_P = ((uint32_t)d.at(0) << 12) | ((uint32_t)d.at(1) << 4) | (d.at(2) >> 4);
        int32_t adc_T = ((uint32_t)d.at(3) << 12) | ((uint32_t)d.at(4) << 4) | (d.at(5) >> 4);

        int32_t var1 = ((((adc_T >> 3) - ((int32_t)dig_T1_ << 1))) * ((int32_t)dig_T2_)) >> 11;
        int32_t var2 = (((((adc_T >> 4) - ((int32_t)dig_T1_)) * ((adc_T >> 4) - ((int32_t)dig_T1_))) >> 12) * ((int32_t)dig_T3_)) >> 14;
        int32_t t_fine = var1 + var2;
        temp_c = ((t_fine * 5 + 128) >> 8) / 100.0f;

        int64_t p_var1 = ((int64_t)t_fine) - 128000;
        int64_t p_var2 = p_var1 * p_var1 * (int64_t)dig_P6_ + ((p_var1 * (int64_t)dig_P5_) << 17) + (((int64_t)dig_P4_) << 35);
        p_var1 = ((p_var1 * p_var1 * (int64_t)dig_P3_) >> 8) + ((p_var1 * (int64_t)dig_P2_) << 12);
        p_var1 = (((((int64_t)1) << 47) + p_var1)) * ((int64_t)dig_P1_) >> 33;
        if (p_var1 == 0) return false;

        int64_t p = (((1048576LL - (int64_t)adc_P) << 31) - p_var2) * 3125 / p_var1;
        p = ((p + ((((int64_t)dig_P9_) * (p >> 13) * (p >> 13)) >> 25) + ((((int64_t)dig_P8_) * p) >> 19)) >> 8) + (((int64_t)dig_P7_) << 4);
        press_mmhg = ((float)p / 256.0f) * 0.00750062f;
        return true;
    }
};

// Контроллер независимой адресной ленты (для шлема или подиума) с эффектами и попиксельным управлением
struct StripChannel {
    CircularStrip* strip = nullptr;
    int count = 12;
    uint8_t r = 0, g = 150, b = 255;
    uint8_t brightness = 80;
    std::string effect = "solid";
    std::vector<StripColor> pixels;
    uint16_t step = 0;

    void Init(gpio_num_t pin, int led_count, uint8_t def_r, uint8_t def_g, uint8_t def_b) {
        count = led_count;
        r = def_r; g = def_g; b = def_b;
        pixels.assign(count, {0, 0, 0});
        strip = new CircularStrip(pin, count);
        ApplySolid();
    }

    StripColor Scale(uint8_t cr, uint8_t cg, uint8_t cb) const {
        return {
            (uint8_t)((cr * brightness) / 100),
            (uint8_t)((cg * brightness) / 100),
            (uint8_t)((cb * brightness) / 100)
        };
    }

    void ApplySolid() {
        if (!strip) return;
        StripColor c = Scale(r, g, b);
        for (int i = 0; i < count; i++) pixels[i] = c;
        strip->SetAllColor(c);
    }

    void SetPixel(int index_1_based, uint8_t pr, uint8_t pg, uint8_t pb) {
        if (!strip || index_1_based < 1 || index_1_based > count) return;
        effect = "custom";
        pixels[index_1_based - 1] = Scale(pr, pg, pb);
        for (int i = 0; i < count; i++) {
            strip->SetSingleColor(i, pixels[i]);
        }
    }

    static StripColor Wheel(uint8_t pos) {
        pos = 255 - pos;
        if (pos < 85) return {(uint8_t)(255 - pos * 3), 0, (uint8_t)(pos * 3)};
        if (pos < 170) { pos -= 85; return {0, (uint8_t)(pos * 3), (uint8_t)(255 - pos * 3)}; }
        pos -= 170;
        return {(uint8_t)(pos * 3), (uint8_t)(255 - pos * 3), 0};
    }

    void TickEffect() {
        if (!strip || effect == "solid" || effect == "custom") return;
        step++;
        if (effect == "rainbow") {
            for (int i = 0; i < count; i++) {
                StripColor w = Wheel((uint8_t)((i * 256 / count + step * 4) & 255));
                strip->SetSingleColor(i, Scale(w.red, w.green, w.blue));
            }
        } else if (effect == "chase") {
            int head = step % count;
            for (int i = 0; i < count; i++) {
                if (i == head) strip->SetSingleColor(i, Scale(r, g, b));
                else if (i == (head + count - 1) % count) strip->SetSingleColor(i, Scale(r / 4, g / 4, b / 4));
                else strip->SetSingleColor(i, {0, 0, 0});
            }
        } else if (effect == "breath") {
            float phase = (sinf(step * 0.12f) + 1.0f) * 0.5f;
            uint8_t br = (uint8_t)(brightness * (0.1f + 0.9f * phase));
            StripColor c = {
                (uint8_t)((r * br) / 100),
                (uint8_t)((g * br) / 100),
                (uint8_t)((b * br) / 100)
            };
            strip->SetAllColor(c);
        }
    }
};

class CompactWifiBoardLcd : public WifiBoard {
private:
    Button boot_button_;
    DummySystemLed dummy_led_;
    KosmonavtEyeDisplay* display_ = nullptr;
    StripChannel helmet_led_;
    StripChannel podium_led_;
    i2c_master_bus_handle_t i2c_bus_ = nullptr;
    SimpleBmp280 bmp280_;
    adc_oneshot_unit_handle_t adc_handle_ = nullptr;
    adc_cali_handle_t adc_cali_ = nullptr;

    void InitPeriphPower() {
        gpio_config_t io_conf = {
            .pin_bit_mask = (1ULL << PERIPH_POWER_PIN),
            .mode = GPIO_MODE_OUTPUT,
            .pull_up_en = GPIO_PULLUP_DISABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type = GPIO_INTR_DISABLE,
        };
        gpio_config(&io_conf);
        gpio_set_level(PERIPH_POWER_PIN, 1);
        vTaskDelay(pdMS_TO_TICKS(50));
    }

    void EnterDeepSleep() {
        ESP_LOGI(TAG, "Подготовка к глубокому сну...");

        if (helmet_led_.strip) helmet_led_.strip->SetAllColor({0, 0, 0});
        if (podium_led_.strip) podium_led_.strip->SetAllColor({0, 0, 0});
        vTaskDelay(pdMS_TO_TICKS(20));

        // Жёстко гасим подсветку дисплея (GPIO 17) и фиксируем уровень на время сна
        if (display_) GetBacklight()->SetBrightness(0);
        gpio_reset_pin(DISPLAY_BACKLIGHT_PIN);
        gpio_set_direction(DISPLAY_BACKLIGHT_PIN, GPIO_MODE_OUTPUT);
        gpio_set_level(DISPLAY_BACKLIGHT_PIN, DISPLAY_BACKLIGHT_OUTPUT_INVERT ? 1 : 0);
        gpio_hold_en(DISPLAY_BACKLIGHT_PIN);

        while (gpio_get_level(BOOT_BUTTON_GPIO) == 0) {
            vTaskDelay(pdMS_TO_TICKS(50));
        }
        vTaskDelay(pdMS_TO_TICKS(120));

        gpio_reset_pin(NEOPIXEL_STRIP_PIN);
        gpio_reset_pin(PODIUM_STRIP_PIN);
        gpio_set_level(PERIPH_POWER_PIN, 0);
        gpio_hold_en(PERIPH_POWER_PIN);

        rtc_gpio_init(BOOT_BUTTON_GPIO);
        rtc_gpio_set_direction(BOOT_BUTTON_GPIO, RTC_GPIO_MODE_INPUT_ONLY);
        rtc_gpio_pullup_en(BOOT_BUTTON_GPIO);
        rtc_gpio_pulldown_dis(BOOT_BUTTON_GPIO);

        esp_sleep_enable_ext0_wakeup(BOOT_BUTTON_GPIO, 0);
        esp_deep_sleep_start();
    }

    void InitI2cAndSensors() {
        i2c_master_bus_config_t bus_cfg = {
            .i2c_port = I2C_NUM_0,
            .sda_io_num = SENSOR_I2C_SDA_PIN,
            .scl_io_num = SENSOR_I2C_SCL_PIN,
            .clk_source = I2C_CLK_SRC_DEFAULT,
            .glitch_ignore_cnt = 7,
            .flags = {.enable_internal_pullup = true},
        };
        if (i2c_new_master_bus(&bus_cfg, &i2c_bus_) == ESP_OK) {
            bmp280_.Init(i2c_bus_);
        }
    }

    void InitBatteryAdc() {
        adc_oneshot_unit_init_cfg_t init_cfg = {.unit_id = ADC_UNIT_1};
        if (adc_oneshot_new_unit(&init_cfg, &adc_handle_) == ESP_OK) {
            adc_oneshot_chan_cfg_t chan_cfg = {.atten = ADC_ATTEN_DB_12, .bitwidth = ADC_BITWIDTH_DEFAULT};
            adc_oneshot_config_channel(adc_handle_, ADC_CHANNEL_0, &chan_cfg);
            adc_cali_curve_fitting_config_t cali_cfg = {
                .unit_id = ADC_UNIT_1, .chan = ADC_CHANNEL_0, .atten = ADC_ATTEN_DB_12, .bitwidth = ADC_BITWIDTH_DEFAULT
            };
            adc_cali_create_scheme_curve_fitting(&cali_cfg, &adc_cali_);
        }
    }

    float ReadBatteryVoltage() {
        if (!adc_handle_) return 4.0f;
        int sum_mv = 0;
        for (int i = 0; i < 16; i++) {
            int raw = 0, mv = 0;
            adc_oneshot_read(adc_handle_, ADC_CHANNEL_0, &raw);
            if (adc_cali_) adc_cali_raw_to_voltage(adc_cali_, raw, &mv);
            else mv = (raw * 3300) / 4095;
            sum_mv += mv;
        }
        return ((sum_mv / 16.0f) * 2.0f) / 1000.0f;
    }

    void InitializeSpi() {
        spi_bus_config_t buscfg = {};
        buscfg.mosi_io_num = DISPLAY_SPI_MOSI_PIN;
        buscfg.miso_io_num = GPIO_NUM_NC;
        buscfg.sclk_io_num = DISPLAY_SPI_SCLK_PIN;
        buscfg.quadwp_io_num = GPIO_NUM_NC;
        buscfg.quadhd_io_num = GPIO_NUM_NC;
        buscfg.max_transfer_sz = DISPLAY_WIDTH * DISPLAY_HEIGHT * sizeof(uint16_t);
        ESP_ERROR_CHECK(spi_bus_initialize(SPI3_HOST, &buscfg, SPI_DMA_CH_AUTO));
    }

    void InitializeLcdDisplay() {
        esp_lcd_panel_io_handle_t panel_io = nullptr;
        esp_lcd_panel_handle_t panel = nullptr;

        esp_lcd_panel_io_spi_config_t io_config = {};
        io_config.cs_gpio_num = DISPLAY_SPI_CS_PIN;
        io_config.dc_gpio_num = DISPLAY_SPI_DC_PIN;
        io_config.spi_mode = DISPLAY_SPI_MODE;
        io_config.pclk_hz = 40 * 1000 * 1000;
        io_config.trans_queue_depth = 10;
        io_config.lcd_cmd_bits = 8;
        io_config.lcd_param_bits = 8;
        ESP_ERROR_CHECK(esp_lcd_new_panel_io_spi(SPI3_HOST, &io_config, &panel_io));

        esp_lcd_panel_dev_config_t panel_config = {};
        panel_config.reset_gpio_num = DISPLAY_SPI_RESET_PIN;
        panel_config.rgb_ele_order = DISPLAY_RGB_ORDER;
        panel_config.bits_per_pixel = 16;
        ESP_ERROR_CHECK(esp_lcd_new_panel_st7789(panel_io, &panel_config, &panel));

        esp_lcd_panel_reset(panel);
        esp_lcd_panel_init(panel);
        esp_lcd_panel_invert_color(panel, DISPLAY_INVERT_COLOR);
        esp_lcd_panel_swap_xy(panel, DISPLAY_SWAP_XY);
        esp_lcd_panel_mirror(panel, DISPLAY_MIRROR_X, DISPLAY_MIRROR_Y);

        display_ = new KosmonavtEyeDisplay(panel_io, panel,
                                           DISPLAY_WIDTH, DISPLAY_HEIGHT,
                                           DISPLAY_OFFSET_X, DISPLAY_OFFSET_Y,
                                           DISPLAY_MIRROR_X, DISPLAY_MIRROR_Y, DISPLAY_SWAP_XY);
    }

    void InitializeButtons() {
        boot_button_.OnClick([this]() {
            Application::GetInstance().ToggleChatState();
        });
        boot_button_.OnDoubleClick([this]() {
            EnterWifiConfigMode();
        });
        boot_button_.OnLongPress([this]() {
            EnterDeepSleep();
        });
    }

    void RegisterStripTools(const std::string& prefix, const std::string& title, StripChannel* ch) {
        auto& mcp = McpServer::GetInstance();

        mcp.AddTool(prefix + ".set_color",
            "Установить сплошной цвет RGB (0-255) для " + title,
            PropertyList({
                Property("red", kPropertyTypeInteger, 0, 255),
                Property("green", kPropertyTypeInteger, 0, 255),
                Property("blue", kPropertyTypeInteger, 0, 255)
            }),
            [ch](const PropertyList& properties) -> ReturnValue {
                for (const auto& p : properties) {
                    if (p.name() == "red") ch->r = p.value<int>();
                    else if (p.name() == "green") ch->g = p.value<int>();
                    else if (p.name() == "blue") ch->b = p.value<int>();
                }
                ch->effect = "solid";
                ch->ApplySolid();
                return true;
            });

        mcp.AddTool(prefix + ".set_brightness",
            "Установить яркость от 0 до 100 процентов для " + title,
            PropertyList({
                Property("brightness", kPropertyTypeInteger, 0, 100)
            }),
            [ch](const PropertyList& properties) -> ReturnValue {
                for (const auto& p : properties) {
                    if (p.name() == "brightness") ch->brightness = p.value<int>();
                }
                if (ch->effect == "solid") ch->ApplySolid();
                return true;
            });

        mcp.AddTool(prefix + ".set_pixel",
            "Установить цвет RGB (0-255) для одного конкретного светодиода (номер от 1 до 12) в " + title,
            PropertyList({
                Property("index", kPropertyTypeInteger, 1, 12),
                Property("red", kPropertyTypeInteger, 0, 255),
                Property("green", kPropertyTypeInteger, 0, 255),
                Property("blue", kPropertyTypeInteger, 0, 255)
            }),
            [ch](const PropertyList& properties) -> ReturnValue {
                int idx = 1, pr = 0, pg = 0, pb = 0;
                for (const auto& p : properties) {
                    if (p.name() == "index") idx = p.value<int>();
                    else if (p.name() == "red") pr = p.value<int>();
                    else if (p.name() == "green") pg = p.value<int>();
                    else if (p.name() == "blue") pb = p.value<int>();
                }
                ch->SetPixel(idx, pr, pg, pb);
                return true;
            });

        mcp.AddTool(prefix + ".set_effect",
            "Включить световой эффект для " + title + ": solid (обычный свет), rainbow (радуга), chase (бегущий огонёк), breath (плавное дыхание)",
            PropertyList({
                Property("effect", kPropertyTypeString)
            }),
            [ch](const PropertyList& properties) -> ReturnValue {
                for (const auto& p : properties) {
                    if (p.name() == "effect") ch->effect = p.value<std::string>();
                }
                if (ch->effect == "solid") ch->ApplySolid();
                return true;
            });
    }

    void InitializeTools() {
        auto& mcp = McpServer::GetInstance();
        mcp.AddTool("self.climate.get_status",
            "Получить температуру воздуха (в градусах Цельсия) и атмосферное давление (мм рт. ст.) в комнате",
            PropertyList(),
            [this](const PropertyList& properties) -> ReturnValue {
                float temp = 0.0f, press = 0.0f;
                if (bmp280_.Read(temp, press)) {
                    std::string res = "{\"temperature_c\": " + std::to_string(temp) +
                                      ", \"pressure_mmhg\": " + std::to_string(press) + "}";
                    return res;
                }
                return std::string("{\"error\": \"Датчик BMP280 не отвечает\"}");
            });

        RegisterStripTools("self.helmet_light", "подсветки шлема космонавта", &helmet_led_);
        RegisterStripTools("self.podium_light", "нижней парящей подсветки подиума", &podium_led_);
    }

    static void LedEffectsTask(void* arg) {
        auto* board = static_cast<CompactWifiBoardLcd*>(arg);
        while (true) {
            vTaskDelay(pdMS_TO_TICKS(50));
            board->helmet_led_.TickEffect();
            board->podium_led_.TickEffect();
        }
    }

    static void BatteryMonitorTask(void* arg) {
        auto* board = static_cast<CompactWifiBoardLcd*>(arg);
        while (true) {
            vTaskDelay(pdMS_TO_TICKS(15000));
            float v = board->ReadBatteryVoltage();
            if (v > 2.5f && v < 3.25f && board->helmet_led_.strip) {
                board->helmet_led_.strip->Blink({255, 0, 0}, 300);
            }
            if (v > 2.5f && v <= 3.12f) {
                board->EnterDeepSleep();
            }
        }
    }

public:
    CompactWifiBoardLcd() : boot_button_(BOOT_BUTTON_GPIO) {
        rtc_gpio_deinit(BOOT_BUTTON_GPIO);
        gpio_set_pull_mode(BOOT_BUTTON_GPIO, GPIO_PULLUP_ONLY);

        gpio_hold_dis(PERIPH_POWER_PIN);
        gpio_hold_dis(DISPLAY_BACKLIGHT_PIN);

        InitPeriphPower();
        InitBatteryAdc();
        InitI2cAndSensors();
        InitializeSpi();
        InitializeLcdDisplay();
        InitializeButtons();

        helmet_led_.Init(NEOPIXEL_STRIP_PIN, NEOPIXEL_STRIP_COUNT, 0, 150, 255);
        podium_led_.Init(PODIUM_STRIP_PIN, PODIUM_STRIP_COUNT, 120, 40, 255);

        InitializeTools();
        if (GetBacklight()) GetBacklight()->RestoreBrightness();

        xTaskCreate(LedEffectsTask, "led_fx", 3072, this, 2, nullptr);
        xTaskCreate(BatteryMonitorTask, "bat_mon", 3072, this, 2, nullptr);
    }

    virtual Led* GetLed() override { return &dummy_led_; }

    virtual AudioCodec* GetAudioCodec() override {
        static NoAudioCodecSimplex audio_codec(
            AUDIO_INPUT_SAMPLE_RATE, AUDIO_OUTPUT_SAMPLE_RATE,
            AUDIO_I2S_SPK_GPIO_BCLK, AUDIO_I2S_SPK_GPIO_LRCK, AUDIO_I2S_SPK_GPIO_DOUT,
            AUDIO_I2S_MIC_GPIO_SCK, AUDIO_I2S_MIC_GPIO_WS, AUDIO_I2S_MIC_GPIO_DIN);
        return &audio_codec;
    }

    virtual Display* GetDisplay() override { return display_; }

    virtual Backlight* GetBacklight() override {
        static PwmBacklight backlight(DISPLAY_BACKLIGHT_PIN, DISPLAY_BACKLIGHT_OUTPUT_INVERT);
        return &backlight;
    }

    virtual bool GetBatteryLevel(int& level, bool& charging, bool& discharging) override {
        float v = ReadBatteryVoltage();
        charging = false;
        discharging = true;
        int pct = (int)((v - 3.20f) / (4.15f - 3.20f) * 100.0f);
        level = (pct < 0) ? 0 : ((pct > 100) ? 100 : pct);
        return true;
    }
};

DECLARE_BOARD(CompactWifiBoardLcd);
