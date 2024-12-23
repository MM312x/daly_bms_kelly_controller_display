//*********** Display setup ************
class LGFX : public lgfx::LGFX_Device
{

  lgfx::Panel_ST7796 _panel_instance;
  lgfx::Bus_SPI _bus_instance;
  lgfx::Light_PWM _light_instance;
  lgfx::Touch_FT5x06 _touch_instance;

public:
  LGFX(void)
  {
    {
      auto cfg = _bus_instance.config();
      cfg.spi_host = VSPI_HOST;
      cfg.spi_mode = 0;
      cfg.freq_write = 40000000;
      cfg.freq_read = 16000000;
      cfg.spi_3wire = true;
      cfg.use_lock = true;
      cfg.dma_channel = 1;
      cfg.pin_sclk = 14;
      cfg.pin_mosi = 13;
      cfg.pin_miso = -1;
      cfg.pin_dc = 21;
      _bus_instance.config(cfg);
      _panel_instance.setBus(&_bus_instance);
    }

    {
      auto cfg = _panel_instance.config();
      cfg.pin_cs = 15;
      cfg.pin_rst = 22;
      cfg.pin_busy = -1;
      cfg.memory_width = 320;
      cfg.memory_height = 480;
      cfg.panel_width = 320;
      cfg.panel_height = 480;
      cfg.offset_x = 0;
      cfg.offset_y = 0;
      cfg.offset_rotation = 0;
      cfg.dummy_read_pixel = 8;
      cfg.dummy_read_bits = 1;
      cfg.readable = true;
      cfg.invert = false;
      cfg.rgb_order = false;
      cfg.dlen_16bit = false;
      cfg.bus_shared = true;
      _panel_instance.config(cfg);
    }

    {
      auto cfg = _light_instance.config();
      cfg.pin_bl = 23;
      cfg.invert = false;
      cfg.freq = 44100;
      cfg.pwm_channel = 7;
      _light_instance.config(cfg);
      _panel_instance.setLight(&_light_instance);
    }

    {
      auto cfg = _touch_instance.config();
      cfg.x_min = 0;
      cfg.x_max = 319;
      cfg.y_min = 0;
      cfg.y_max = 479;
      //cfg.pin_int = 2;
      //cfg.bus_shared = true;
      //cfg.offset_rotation = 0;

      cfg.i2c_port = 1;
      cfg.i2c_addr = 0x38;
      cfg.pin_sda = 18;
      cfg.pin_scl = 19;
      cfg.freq = 400000;

      _touch_instance.config(cfg);
      _panel_instance.setTouch(&_touch_instance);
    }
    setPanel(&_panel_instance);
  }
};

static LGFX tft;

/*Change to your screen resolution*/
static const uint32_t screenWidth = 480;
static const uint32_t screenHeight = 320;
static lv_disp_draw_buf_t draw_buf;
static lv_color_t buf[screenWidth * 10];

/* Display flushing */
void my_disp_flush(lv_disp_drv_t *disp, const lv_area_t *area, lv_color_t *color_p) {
  uint32_t w = (area->x2 - area->x1 + 1);
  uint32_t h = (area->y2 - area->y1 + 1);

  tft.startWrite();
  tft.setAddrWindow(area->x1, area->y1, w, h);
  tft.writePixels((lgfx::rgb565_t *)&color_p->full, w * h);
  tft.endWrite();

  lv_disp_flush_ready(disp);
}

/*Read the touchpad*/
void my_touchpad_read(lv_indev_drv_t *indev_driver, lv_indev_data_t *data) {

  uint16_t x, y;
  if (tft.getTouch(&x, &y)) {
    data->state = LV_INDEV_STATE_PR;
    data->point.x = x;
    data->point.y = y;

  } else {
    data->state = LV_INDEV_STATE_REL;
  }
}

void resetDisplayBMS() {
  lv_slider_set_value(ui_powerRegenSlider, 0, LV_ANIM_ON);
  lv_slider_set_value(ui_powerSlider, 0, LV_ANIM_ON);
  lv_label_set_text_fmt(ui_VoltageLabel, "%.1f V", 0.0);
  lv_label_set_text_fmt(ui_AmpereLabel, "%.1f A", 0.0);
  lv_label_set_text_fmt(ui_WattLabel, "%d W", 0);
  lv_label_set_text_fmt(ui_batteryTempLabel, "%d / %d °C", 0, 0);
  lv_label_set_text_fmt(ui_cellMaxLabel, "%.3f V", 0.0); 
  lv_label_set_text_fmt(ui_cellMinLabel, "%.3f V", 0.0); 
  lv_label_set_text_fmt(ui_cellDiffLabel, "%.3f V", 0.0); 
  lv_label_set_text_fmt(ui_batteryPercentLabel, "%d %%", 0);
  lv_slider_set_value(ui_batteryFillLevelSlider, 0, LV_ANIM_ON);
  lv_label_set_text_fmt(ui_ecoLabel, "%.0f Wh", 0.0);
  lv_label_set_text_fmt(ui_averageAhLabel, "%.2f Wh/Km", 0.0);
  lv_label_set_text_fmt(ui_rangeLabel, "%d Km", 0); 
}

void resetDisplayKelly() {
  lv_label_set_text_fmt(ui_SpeedLabel, "%.0f", 0.0);
  lv_label_set_text_fmt(ui_motorControllerTempLabel, "%d / %d °C", 0, 0);
  lv_label_set_text_fmt(ui_dayKmLabel, "%.1f Km", 0.0);
  lv_label_set_text_fmt(ui_totalKmLabel, "%d Km", 0);
  //lv_label_set_text_fmt(ui_speedModeLabel, "%d ", 0);
}

void initDisplay() {
  tft.begin();
  tft.setRotation(3);
  tft.setBrightness(180);
  tft.fillScreen(TFT_BLACK);
}
