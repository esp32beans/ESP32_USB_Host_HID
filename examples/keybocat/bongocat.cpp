/*
 * MIT License
 *
 * Copyright (c) 2023 esp32beans@gmail.com
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

// This example renders a png file that is stored in a FLASH array
// using the PNGdec library (available via library manager).

/*
 * About half of the code is from a TFT_eSPI example. Then modified to
 * run as an ESP-IDF (FreeRTOS) task to display cat images.
 */

#include <Arduino.h>
#include "./bongocat.h"

// Include the PNG decoder library
#include <PNGdec.h>
#include "./cat_idle_png.h"
#include "./cat_left_png.h"
#include "./cat_right_png.h"

PNG png;  // PNG decoder instance

#define MAX_IMAGE_WIDTH 320   // Adjust for your images

static int16_t xpos = 0;
static int16_t ypos = 0;

// Include the TFT library https://github.com/Bodmer/TFT_eSPI
#include "SPI.h"
#include <TFT_eSPI.h>              // Hardware-specific library
static TFT_eSPI tft = TFT_eSPI();         // Invoke custom library

static const char *TAG = "bongocat";
static TaskHandle_t BONGOCAT_TASK_HANDLE = NULL;

void bongo_wake(void) {
  if (BONGOCAT_TASK_HANDLE == 0) return;
  xTaskNotifyGive(BONGOCAT_TASK_HANDLE);
}

static void bongocat_task(void *pvParameters) {
  while (true) {
    (void)ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    bongo_loop();
  }
}

//====================================================================================
//                                    Setup
//====================================================================================
void bongo_setup() {
  ESP_LOGI(TAG, "\n\n Using the PNGdec library");

  // Initialise the TFT
  tft.begin();
  tft.setRotation(3);
  tft.fillScreen(TFT_WHITE);
  bongo_idle();

  /*
   * Create bongocat task process for animation
   */
  BaseType_t task_created =
      xTaskCreate(bongocat_task, "bongo_task", 4*1024, NULL, 2,
          &BONGOCAT_TASK_HANDLE);
  assert(task_created == pdTRUE);
  configASSERT(BONGOCAT_TASK_HANDLE);

  ESP_LOGI(TAG, "\r\nInitialisation done.");
}

typedef struct png_image_flash {
  const uint8_t *base;
  size_t size;
} png_image_flash_t;

static const png_image_flash_t png_images[] PROGMEM = {
  {CAT_IDLE_PNG, sizeof(CAT_IDLE_PNG)},
  {CAT_LEFT_PNG, sizeof(CAT_LEFT_PNG)},
  {CAT_RIGHT_PNG, sizeof(CAT_RIGHT_PNG)},
};

#define PNG_IMAGES_SIZE sizeof(png_images)/sizeof(png_images[0])

// =========================================v==========================================
//                                       pngDraw
// ====================================================================================
// This next function will be called during decoding of the png file to
// render each image line to the TFT.  If you use a different TFT library
// you will need to adapt this function to suit.
// Callback function to draw pixels to the display
static void pngDraw(PNGDRAW *pDraw) {
  uint16_t lineBuffer[MAX_IMAGE_WIDTH];
  png.getLineAsRGB565(pDraw, lineBuffer, PNG_RGB565_BIG_ENDIAN, 0xffffffff);
  tft.pushImage(xpos, ypos + pDraw->y, pDraw->iWidth, 1, lineBuffer);
}

static void bongo_draw(const uint8_t *image, size_t image_len) {
  int16_t rc;
  rc = png.openFLASH((uint8_t *)image, image_len, pngDraw);
  if (rc == PNG_SUCCESS) {
    ESP_LOGI(TAG, "Successfully opened png file");
    ESP_LOGI(TAG, "image specs: (%d x %d), %d bpp, pixel type: %d\n",
        png.getWidth(), png.getHeight(), png.getBpp(), png.getPixelType());
    tft.startWrite();
    uint32_t dt = millis();
    rc = png.decode(NULL, 0);
    if (rc != PNG_SUCCESS) {
      ESP_LOGI(TAG, "png.decode failed: %u", rc);
    }
    ESP_LOGI(TAG, "%u ms", millis() - dt);
    tft.endWrite();
    // png.close(); // not needed for memory->memory decode
  }
}

void bongo_idle(void) {
  bongo_draw(CAT_IDLE_PNG, sizeof(CAT_IDLE_PNG));
}

void bongo_left(void) {
  bongo_draw(CAT_LEFT_PNG, sizeof(CAT_LEFT_PNG));
}

void bongo_right(void) {
  bongo_draw(CAT_RIGHT_PNG, sizeof(CAT_RIGHT_PNG));
}

//====================================================================================
//                                    Loop
//====================================================================================
void bongo_loop() {
  static size_t png_select = 0;

  bongo_draw((uint8_t *)png_images[png_select].base,
      png_images[png_select].size);
  png_select++;
  if (png_select >= PNG_IMAGES_SIZE) png_select = 0;
}
