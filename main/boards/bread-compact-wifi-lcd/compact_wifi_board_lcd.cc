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

#define TAG "KosmonavtLcdBoard"

// Кастомный дисплей с крупными анимированными белыми глазами на глухом чёрном фоне
class KosmonavtEyeDisplay : public SpiLcdDisplay {
private:
    lv_obj_t* eye_screen_ = nullptr;
    lv_obj_t* left_eye_ = nullptr;
    lv_obj_t* right_eye_ = nullptr;
    lv_obj_t* status_bar_ = nullptr;
    lv_timer_t* blink_timer_ = nullptr;
    int target_h_ = 96;
    int target_r_ = 28;
    int target_y_ = 62;
    bool is_blinking_ = false;

    static void BlinkTimerCb(lv_timer_t* timer) {
        auto* self = static_cast<KosmonavtEyeDisplay*>(lv_timer_get_user_data(timer));
        if (!self || !self->left_eye_ || !self->right_eye_) return;
        if (!self->is_blinking_) {
            self->is_blinking_ = true;
            lv_obj_set_height(self->left_eye_, 8);
            lv_obj_set_height(self->right_eye_, 8);
            lv_obj_set_y(self->left_eye_, self->target_y_ + self->target_h_ / 2);
            lv_obj_set_y(self->right_eye_, self->target_y_ + self->target_h_ / 2);
            lv_timer_set_period(timer, 140);
        } else {
            self->is_blinking_ = false;
            lv_obj_set_height(self->left_eye_, self->target_h_);
            lv_obj_set_height(self->right_eye_, self->target_h_);
            lv_obj_set_y(self->left_eye_, self->target_y_);
            lv_obj_set_y(self->right_eye_, self->target_y_);
            lv_timer_set_period(timer, 3200);
        }
    }

    void ApplyEyeShape(int h, int r, int y_offset) {
        if (!left_eye_ || !right_eye_) return;
        target_h_ = h;
        target_r_ = r;
        target_y_ = y_offset;
        if (!is_blinking_) {
            lv_obj_set_height(left_eye_, target_h_);
            lv_obj_set_height(right_eye_, target_h_);
            lv_obj_set_style_radius(left_eye_, target_r_, 0);
            lv_obj_set_style_radius(right_eye_, target_r_, 0);
            lv_obj_set_y(left_eye_, target_y_);
            lv_obj_set_y(right_eye_, target_y_);
        }
    }

public:
    KosmonavtEyeDisplay(esp_lcd_panel_io_handle_t io, esp_lcd_panel_handle_t panel,
                        int width, int height, int offset_x, int offset_y,
                        bool mirror_x, bool mirror_y, bool swap_xy)
        : SpiLcdDisplay(io, panel, width, height, offset_x, offset_y, mirror_x, mirror_y, swap_xy) {
        DisplayLockGuard lock(this);
        eye_screen_ = lv_obj_create(nullptr);
        lv_obj_set_style_bg_color(eye_screen_, lv_color_black(), 0);
        lv_obj_set_style_bg_opa(eye_screen_, LV_OPA_COVER, 0);
        lv_obj_clear_flag(eye_screen_, LV_OBJ_FLAG_SCROLLABLE);

        // Левый крупный белый глаз
        left_eye_ = lv_obj_create(eye_screen_);
        lv_obj_set_size(left_eye_, 68, 96);
        lv_obj_set_pos(left_eye_, 36, 62);
        lv_obj_set_style_bg_color(left_eye_, lv_color_white(), 0);
        lv_obj_set_style_bg_opa(left_eye_, LV_OPA_COVER, 0);
        lv_obj_set_style_border_width(left_eye_, 0, 0);
        lv_obj_set_style_radius(left_eye_, 28, 0);
        lv_obj_clear_flag(left_eye_, LV_OBJ_FLAG_SCROLLABLE);

        // Правый крупный белый глаз
        right_eye_ = lv_obj_create(eye_screen_);
        lv_obj_set_size(right_eye_, 68, 96);
        lv_obj_set_pos(right_eye_, 136, 62);
        lv_obj_set_style_bg_color(right_eye_, lv_color_white(), 0);
        lv_obj_set_style_bg_opa(right_eye_, LV_OPA_COVER, 0);
        lv_obj_set_style_border_width(right_eye_, 0, 0);
        lv_obj_set_style_radius(right_eye_, 28, 0);
        lv_obj_clear_flag(right_eye_, LV_OBJ_FLAG_SCROLLABLE);

        // Тонкая полоска состояния внизу визора (белая/голубая/зелёная)
        status_bar_ = lv_obj_create(eye_screen_);
        lv_obj_set_size(status_bar_, 80, 8);
        lv_obj_set_pos(status_bar_, 80, 192);
        lv_obj_set_style_bg_color(status_bar_, lv_color_make(0, 180, 255), 0);
        lv_obj_set_style_bg_opa(status_bar_, LV_OPA_COVER, 0);
        lv_obj_set_style_border_width(status_bar_, 0, 0);
        lv_obj_set_style_radius(status_bar_, 4, 0);

        lv_scr_load(eye_screen_);
        blink_timer_ = lv_timer_create(BlinkTimerCb, 3200, this);
    }

    virtual bool SetEmotion(const char* emotion) override {
        if (!emotion) return false;
        DisplayLockGuard lock(this);
        if (strcmp(emotion, "happy") == 0 || strcmp(emotion, "laughing") == 0 || strcmp(emotion, "loving") == 0) {
            ApplyEyeShape(52, 26, 82); // Весёлый прищур
        } else if (strcmp(emotion, "surprised") == 0 || strcmp(emotion, "shocked") == 0) {
            ApplyEyeShape(114, 34, 52); // Широко раскрытые глаза
        } else if (strcmp(emotion, "sad") == 0 || strcmp(emotion, "crying") == 0 || strcmp(emotion, "sleepy") == 0) {
            ApplyEyeShape(36, 16, 102); // Грустные / сонные прикрытые глаза
        } else if (strcmp(emotion, "thinking") == 0 || strcmp(emotion, "confused") == 0) {
            ApplyEyeShape(64, 20, 74); // Задумчивый прищур
        } else {
            ApplyEyeShape(96, 28, 62); // Обычные крупные открытые глаза
        }
        return true;
    }

    virtual void SetStatus(const char* status) override {
        if (!status || !status_bar_) return;
        DisplayLockGuard lock(this);
        if (strstr(status, "Listening") || strstr(status, "Слушаю")) {
            lv_obj_set_style_bg_color(status_bar_, lv_color_make(0, 255, 100), 0);
            ApplyEyeShape(106, 30, 56);
        } else if (strstr(status, "Speaking") || strstr(status, "Говорю")) {
            lv_obj_set_style_bg_color(status_bar_, lv_color_make(0, 180, 255), 0);
            ApplyEyeShape(88, 28, 66);
        } else {
            lv_obj_set_style_bg_color(status_bar_, lv_color_make(90, 90, 90), 0);
            ApplyEyeShape(96, 28, 62);
        }
    }

    virtual void SetChatMessage(const char* role, const char* content) override {
        // Игнорируем мелкий текст субтитров, чтобы визор оставался чистым
    }
};

class SimpleBmp280 {
private:
    i2c_master_dev_handle_t dev_handle_ = nullptr;
    bool initialized_ = false;
    uint16_t dig_T1_ = 0, dig_P1_ = 0;
    int16_t dig_T2_ = 0, dig_T3_ = 0, dig_P2_ = 0, dig_P3_ = 0, dig_P4_ = 0;
    int16_t dig_P5_ = 0, dig_P6_ = 0, dig_P7_ = 0, dig_P8_ = 0, dig_P9_ = 0;

    bool ReadReg(uint8_t reg, uint8_t* data, size_t len) {
        return i2c_master_transmit_receive(dev_handle_, &reg, 1, data, len, 100) == ESP_OK;
    }
    bool WriteReg(uint8_t reg, uint8_t val) {
        std::vector<uint8_t> buf = {reg, val};
        return i2c_master_transmit(dev_handle_, buf.data(), buf.size(), 100) == ESP_OK;
    }

public:
    bool Init(i2c_master_bus_handle_t bus) {
        std::vector<uint8_t> addrs = {0x76, 0x77};
        for (uint8_t addr : addrs) {
            i2c_device_config_t dev_cfg = {
                .dev_addr_length = I2C_ADDR_BIT_LEN_7,
                .device_address = addr,
                .scl_speed_hz = 100000,
            };
            if (i2c_master_bus_add_device(bus, &dev_cfg, &dev_handle_) == ESP_OK) {
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
        if (!initialized_) return false;
        std::vector<uint8_t> d(6, 0);
        if (!ReadReg(0xF7, d.data(), 6)) return false;

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

class CompactWifiBoardLcd : public WifiBoard {
private:
    Button boot_button_;
    KosmonavtEyeDisplay* display_ = nullptr;
    CircularStrip* led_strip_ = nullptr;
    i2c_master_bus_handle_t i2c_bus_ = nullptr;
    SimpleBmp280 bmp280_;
    adc_oneshot_unit_handle_t adc_handle_ = nullptr;
    adc_cali_handle_t adc_cali_ = nullptr;
    uint8_t helmet_r_ = 0, helmet_g_ = 150, helmet_b_ = 255;
    uint8_t helmet_brightness_ = 80;

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
        vTaskDelay(pdMS_TO_TICKS(20));
    }

    void EnterDeepSleep() {
        if (display_) GetBacklight()->SetBrightness(0);
        if (led_strip_) {
            led_strip_->SetAllColor({0, 0, 0});
            vTaskDelay(pdMS_TO_TICKS(20));
        }
        gpio_reset_pin(NEOPIXEL_STRIP_PIN);
        gpio_set_level(PERIPH_POWER_PIN, 0);
        gpio_hold_en(PERIPH_POWER_PIN);
        esp_sleep_enable_ext0_wakeup(BOOT_BUTTON_GPIO, 0);
        vTaskDelay(pdMS_TO_TICKS(100));
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

    void ApplyHelmetColor() {
        if (!led_strip_) return;
        uint8_t r = (helmet_r_ * helmet_brightness_) / 100;
        uint8_t g = (helmet_g_ * helmet_brightness_) / 100;
        uint8_t b = (helmet_b_ * helmet_brightness_) / 100;
        led_strip_->SetAllColor({r, g, b});
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

        mcp.AddTool("self.helmet_light.set_color",
            "Установить цвет RGB (0-255) неоновой подсветки шлема космонавта",
            PropertyList({
                Property("red", kPropertyTypeInteger, 0, 255),
                Property("green", kPropertyTypeInteger, 0, 255),
                Property("blue", kPropertyTypeInteger, 0, 255)
            }),
            [this](const PropertyList& properties) -> ReturnValue {
                auto props = properties;
                for (const auto& p : props) {
                    if (p.name() == "red") helmet_r_ = p.value<int>();
                    else if (p.name() == "green") helmet_g_ = p.value<int>();
                    else if (p.name() == "blue") helmet_b_ = p.value<int>();
                }
                ApplyHelmetColor();
                return true;
            });

        mcp.AddTool("self.helmet_light.set_brightness",
            "Установить яркость подсветки шлема от 0 до 100 процентов без изменения цвета",
            PropertyList({
                Property("brightness", kPropertyTypeInteger, 0, 100)
            }),
            [this](const PropertyList& properties) -> ReturnValue {
                auto props = properties;
                for (const auto& p : props) {
                    if (p.name() == "brightness") helmet_brightness_ = p.value<int>();
                }
                ApplyHelmetColor();
                return true;
            });
    }

    static void BatteryMonitorTask(void* arg) {
        auto* board = static_cast<CompactWifiBoardLcd*>(arg);
        while (true) {
            vTaskDelay(pdMS_TO_TICKS(15000));
            float v = board->ReadBatteryVoltage();
            if (v > 2.5f && v < 3.25f && board->led_strip_) {
                board->led_strip_->Blink({255, 0, 0}, 300);
            }
            if (v > 2.5f && v <= 3.12f) {
                board->EnterDeepSleep();
            }
        }
    }

public:
    CompactWifiBoardLcd() : boot_button_(BOOT_BUTTON_GPIO) {
        gpio_hold_dis(PERIPH_POWER_PIN);
        InitPeriphPower();
        InitBatteryAdc();
        InitI2cAndSensors();
        InitializeSpi();
        InitializeLcdDisplay();
        InitializeButtons();
        led_strip_ = new CircularStrip(NEOPIXEL_STRIP_PIN, NEOPIXEL_STRIP_COUNT);
        InitializeTools();
        if (GetBacklight()) GetBacklight()->RestoreBrightness();
        xTaskCreate(BatteryMonitorTask, "bat_mon", 3072, this, 2, nullptr);
    }

    virtual Led* GetLed() override { return led_strip_; }

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
